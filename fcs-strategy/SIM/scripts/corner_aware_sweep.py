"""
corner_aware_sweep.py — Re-evaluate the best velocity profiles under REAL corner
speed limits derived from the surveyed track geometry (datasheets/sem_2025_eu.csv,
UTMX/UTMY), for a sweep of lateral-grip assumptions a_lat in {0.3, 0.4, 0.5} g.

Why: the production sim (combined_best_profile.py) reads Distance + Elevation from
the track CSV but is otherwise 1-D — it ignores corners. This script folds in the
missing constraint: in a corner of radius R the car cannot exceed
    v_corner = sqrt(a_lat * R).
With no regen, being forced to slow for a corner and re-accelerate costs energy, so
corners can in principle change which velocity-profile family is optimal.

Method (reuses the validated plant exactly):
  1. Build R(s) from the UTM (x,y) geometry (circumscribed-circle curvature).
  2. v_corner(s) = sqrt(a_lat*R(s)), clipped to V_MAX.
  3. Take a base profile's lap (from build_profile / build_profile_cruise), form its
     speed-vs-distance v_base(s), and impose the corner envelope:
        v_lim(s) = min(v_base(s), v_corner(s)),
     then a BACKWARD pass (decel <= A_HARD) so the car can always slow into each
     corner, and a FORWARD pass (accel <= motor F_max/M) so it can re-accelerate.
  4. Forward-integrate a corner-legal v(t) that tracks v_lim(s); tile 11 laps with a
     full stop each lap.
  5. Score with compute_motor_signals (measured motor map) + Strategy G auto-tuned to
     charge-sustaining + km_per_m3 — identical to the main sweep.

Output: per-a_lat table (cornerless vs corner-limited km/m3) and whether the
pulse-and-glide verdict survives, plus a track-geometry figure and the winner's
position(t).
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import pandas as pd
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

import combined_best_profile as C
from combined_best_profile import (
    build_profile, compute_motor_signals, bisect_kp, km_per_m3, verify,
    _net_force, _grade_fn, _elev_fn, D_LAP, DT, V_MAX, A_HARD, N_STOP_STEPS,
    V_COAST_STOP, MASS, ROUTE_CSV, RESULTS_DIR,
)
import profile_optimization_sweep as S

G_ACCEL = 9.81
S_GRID  = np.arange(0.0, D_LAP + 1e-6, 1.0)        # 1 m distance grid over one lap


# ── 1. Track curvature from the surveyed (x,y) geometry ─────────────────────────
def load_geometry():
    df = pd.read_csv(ROUTE_CSV); df.columns = [c.strip() for c in df.columns]
    s = df[[c for c in df.columns if 'Distance' in c][0]].values.astype(float)
    x = df[[c for c in df.columns if c.upper().startswith('UTMX')][0]].values.astype(float)
    y = df[[c for c in df.columns if c.upper().startswith('UTMY')][0]].values.astype(float)
    return s, x, y


def radius_profile(s, x, y, win=6):
    """Local radius of curvature via the circumscribed circle of points spaced
    +/- `win` metres apart (Menger curvature). Large R on straights, small in
    corners. Smoothed and clipped to a sane range."""
    n = len(x); R = np.full(n, 1e4)
    for i in range(n):
        a_i = (i - win) % n; c_i = (i + win) % n
        ax, ay = x[a_i], y[a_i]; bx, by = x[i], y[i]; cx, cy = x[c_i], y[c_i]
        ab = np.hypot(bx-ax, by-ay); bc = np.hypot(cx-bx, cy-by); ca = np.hypot(ax-cx, ay-cy)
        ss = (ab+bc+ca)/2.0
        area2 = ss*(ss-ab)*(ss-bc)*(ss-ca)
        if area2 <= 1e-9:
            R[i] = 1e4
        else:
            R[i] = (ab*bc*ca)/(4.0*np.sqrt(area2))
    R = np.clip(R, 5.0, 1e4)
    # light smoothing over ~5 m
    k = 5; ker = np.ones(k)/k
    R = np.convolve(np.concatenate([R[-k:], R, R[:k]]), ker, 'same')[k:-k]
    return np.interp(S_GRID, s, R)


def v_corner_of_s(R_grid, a_lat_g):
    vc = np.sqrt(a_lat_g * G_ACCEL * R_grid)
    return np.clip(vc, 2.0, V_MAX)


# ── 2. Base profile -> speed-vs-distance over one lap ───────────────────────────
def base_v_of_s(ta, va, la):
    """Reconstruct v(s) for lap 1 using cumulative distance (avoids the s%D_LAP wrap
    at the start/finish line)."""
    m = la == 1
    v1 = va[m]
    s_cum = np.cumsum(v1) * DT
    s_cum = s_cum - s_cum[0] + v1[0]*DT
    # keep the driving arc up to one lap length
    keep = s_cum <= D_LAP + 5.0
    s_u, idx = np.unique(s_cum[keep], return_index=True)
    return np.interp(S_GRID, s_u, v1[keep][idx], left=0.0, right=0.0)


# ── 3. Impose corner envelope with feasible decel/accel passes ──────────────────
def corner_envelope(v_base, v_corner):
    ds = 1.0
    vlim = np.minimum(v_base, v_corner)
    # backward: must be able to decelerate (hard-brake A_HARD) into each corner
    for i in range(len(vlim)-2, -1, -1):
        vlim[i] = min(vlim[i], np.sqrt(vlim[i+1]**2 + 2.0*A_HARD*ds))
    # forward: must be able to accelerate out (motor force cap), launching from rest
    vlim[0] = min(vlim[0], 0.5)
    for i in range(1, len(vlim)):
        grade = float(_grade_fn(S_GRID[i]))
        a_acc = max(0.05, _net_force(max(vlim[i-1], 0.3), 1e6, grade)/MASS)
        vlim[i] = min(vlim[i], np.sqrt(max(vlim[i-1]**2 + 2.0*a_acc*ds, 0.0)))
    return vlim


# ── 4. Corner-legal v(t), tiled over 11 laps ────────────────────────────────────
def build_corner_trace(vlim):
    """Integrate a velocity that tracks the feasible envelope vlim(s); 11 laps,
    full stop each lap. Returns uniform-DT arrays matching the plant interface."""
    AT=[]; AV=[]; ASd=[]; AL=[]; AE=[]; AG=[]; AC=[]; t0=0.0
    for lap in range(1, C.N_LAPS+1):
        vl=[]; sl=[]; s=0.0; v=0.0
        while s < D_LAP:
            grade = float(_grade_fn(s % D_LAP))
            v_env = float(np.interp(min(s + max(v,0.5)*DT, D_LAP), S_GRID, vlim))
            if v_env >= v:
                a_acc = max(0.05, _net_force(max(v,0.3), 1e6, grade)/MASS)
                v = min(v_env, v + a_acc*DT)
            else:
                v = max(v_env, v - A_HARD*DT)
            v = max(0.0, v)
            s += v*DT
            vl.append(v); sl.append(s)
            if len(vl) > 5000: break
        n=len(vl)
        tl=t0+np.arange(n)*DT
        va=np.array(vl); sd=np.array(sl)
        pos=np.mod(sd, D_LAP)
        ea=np.array([float(_elev_fn(p)) for p in pos])
        ga=np.array([float(_grade_fn(p)) for p in pos])
        ca=np.zeros(n, dtype=bool)
        # append the mandatory full stop (motor off) at the lap line
        ts=tl[-1]+DT*np.arange(1, N_STOP_STEPS+1)
        AT.append(tl); AV.append(va); ASd.append(pos); AL.append(np.full(n, lap))
        AE.append(ea); AG.append(ga); AC.append(ca)
        AT.append(ts); AV.append(np.zeros(N_STOP_STEPS)); ASd.append(np.full(N_STOP_STEPS, pos[-1]))
        AL.append(np.full(N_STOP_STEPS, lap)); AE.append(np.full(N_STOP_STEPS, ea[-1]))
        AG.append(np.full(N_STOP_STEPS, ga[-1])); AC.append(np.ones(N_STOP_STEPS, dtype=bool))
        t0=ts[-1]+DT
    cat=lambda L: np.concatenate(L)
    return cat(AT), cat(AV), cat(ASd), cat(AL), cat(AE), cat(AG), cat(AC)


# ── 5. Score (measured map + Strategy G tuned to CS) ────────────────────────────
def score(name, ta, va, sa, la, ea, ga, ca):
    ok, d, dur, stops = verify(ta, va, la)
    P_e, _, _ = compute_motor_signals(va, ga)
    cmask = P_e < 1.0
    Kp, r = bisect_kp(P_e, la, cmask, name)
    km3 = km_per_m3(r); cs = abs(r['dSOC']) <= 0.015
    return dict(name=name, ok=ok, d=d, dur=dur, stops=stops, km3=km3,
                H2=r['m_H2'], dSOC=r['dSOC'], cs=cs, vmax=float(va.max())*3.6)


# ── Candidates (top of the cornerless sweep) ────────────────────────────────────
def base_traces():
    V_DH=10.5
    cands={}
    cands['PnG 9.5/7.0/1200 (best)']   = build_profile(9.5, 7.0, V_DH, 1200., 1200., 0.)
    cands['PnG 10.0/6.0/1400 (deck)']  = build_profile(10.0, 6.0, V_DH, 1400., 1200., 0.)
    cands['PnG 11.0/6.5/1200 (fast)']  = build_profile(11.0, 6.5, V_DH, 1200., 1200., 0.)
    cands['Cruise v8.0 a1.0 (best CR)']= S.build_profile_cruise(8.0, 1.0)
    cands['Cruise v9.0 a1.0']          = S.build_profile_cruise(9.0, 1.0)
    return cands


def main():
    print("Loading surveyed geometry & building curvature ...")
    s, x, y = load_geometry()
    R = radius_profile(s, x, y)
    print(f"  lap={D_LAP:.1f} m  R: min={R.min():.1f} m  p10={np.percentile(R,10):.1f} m  median={np.median(R):.0f} m")
    for g in (0.3, 0.4, 0.5):
        vc=v_corner_of_s(R, g)
        below=np.sum(vc < 11.0)  # < ~40 km/h
        print(f"  a_lat={g}g: tightest corner cap={vc.min()*3.6:.1f} km/h, {below} m of track below 40 km/h")

    cands = base_traces()
    A_LATS = [None, 0.3, 0.4, 0.5]   # None = cornerless control (reproduce main sweep)

    rows={}
    for nm, tr in cands.items():
        ta,va,Pa,sa,la,ea,ga,ca = tr
        vbase = base_v_of_s(ta, va, la)
        rows[nm]={}
        for g in A_LATS:
            if g is None:
                r = score(nm+" [cornerless]", ta, va, sa, la, ea, ga, ca)
            else:
                vlim = corner_envelope(vbase.copy(), v_corner_of_s(R, g))
                t2,v2,s2,l2,e2,g2,c2 = build_corner_trace(vlim)
                r = score(f"{nm} [{g}g]", t2,v2,s2,l2,e2,g2,c2)
            rows[nm][g]=r

    # ── report ──────────────────────────────────────────────────────────────────
    print("\n"+"="*100)
    print("CORNER-AWARE km/m3  (charge-sustaining; '—' = constraint-invalid)")
    print("="*100)
    hdr=f"{'profile':28s} | {'cornerless':>12s} | {'0.3 g':>11s} | {'0.4 g':>11s} | {'0.5 g':>11s}"
    print(hdr); print("-"*len(hdr))
    def cell(r):
        if r is None: return "   n/a"
        v=f"{r['km3']:.1f}"
        if not r['ok']: v+="!"        # constraint (time/dist/stops) violated
        if not r['cs']: v+="x"        # not charge-sustaining
        return f"{v:>11s}"
    for nm in cands:
        rr=rows[nm]
        print(f"{nm:28s} | {cell(rr[None]):>12s} | {cell(rr[0.3])} | {cell(rr[0.4])} | {cell(rr[0.5])}")
    print("\n  flags:  ! = violates 35-min / distance / 11-stop constraint   x = not charge-sustaining")

    # detail dump
    print("\nDetail (dur / vmax / dSOC):")
    for nm in cands:
        for g in A_LATS:
            r=rows[nm][g]
            if r is None: continue
            tag='cornerless' if g is None else f'{g}g'
            print(f"  {nm:28s} {tag:11s}: km/m3={r['km3']:6.1f}  dur={r['dur']:.1f}m  vmax={r['vmax']:.1f}km/h  dSOC={r['dSOC']:+.4f}  {'OK' if r['ok'] else 'INVALID'} {'CS' if r['cs'] else 'noCS'}")

    # ── verdict at 0.4 g (mid assumption) ─────────────────────────────────────────
    def best_valid(g):
        cand=[(nm,rows[nm][g]) for nm in cands if rows[nm][g] and rows[nm][g]['ok'] and rows[nm][g]['cs']]
        return max(cand, key=lambda kv: kv[1]['km3']) if cand else (None,None)
    print("\n"+"="*100); print("VERDICT"); print("="*100)
    for g in A_LATS:
        nm,r=best_valid(g)
        tag='cornerless' if g is None else f'{g}g'
        if r: print(f"  best @ {tag:11s}: {nm:28s}  {r['km3']:.1f} km/m3  (vmax {r['vmax']:.1f} km/h)")
        else: print(f"  best @ {tag:11s}: none valid")

    # ── figure: track map (curvature) + corner caps + winner position(t) ──────────
    fig=plt.figure(figsize=(15,10),dpi=130)
    gs=fig.add_gridspec(2,2,height_ratios=[1.25,1.0],hspace=0.33,wspace=0.25)
    # track map colored by 0.4g corner cap
    axm=fig.add_subplot(gs[0,:])
    vc04=v_corner_of_s(R,0.4)
    xg=np.interp(S_GRID,s,x); yg=np.interp(S_GRID,s,y)
    sc=axm.scatter(xg-xg.min(), yg-yg.min(), c=vc04*3.6, cmap='RdYlGn', s=14)
    axm.set_aspect('equal'); axm.set_title('Surveyed track — colour = corner speed cap at 0.4 g [km/h]',fontweight='bold',fontsize=12)
    axm.set_xlabel('East [m]'); axm.set_ylabel('North [m]'); fig.colorbar(sc,ax=axm,shrink=0.8,label='v_corner [km/h]')
    # radius / cap vs distance
    axr=fig.add_subplot(gs[1,0])
    axr.plot(S_GRID, v_corner_of_s(R,0.3)*3.6,label='0.3 g'); axr.plot(S_GRID, vc04*3.6,label='0.4 g')
    axr.plot(S_GRID, v_corner_of_s(R,0.5)*3.6,label='0.5 g')
    axr.axhline(38,ls='--',c='k',alpha=0.4,label='~best-profile cruise')
    axr.set_xlabel('Distance in lap [m]'); axr.set_ylabel('Corner speed cap [km/h]')
    axr.set_title('Corner speed cap vs position',fontweight='bold',fontsize=11); axr.legend(fontsize=8); axr.grid(alpha=0.3)
    # winner position(t) lap 1
    axp=fig.add_subplot(gs[1,1])
    nm,_=best_valid(0.4)
    if nm:
        vlim=corner_envelope(base_v_of_s(cands[nm][0], cands[nm][1], cands[nm][4]), vc04)
        t2,v2,s2,l2,e2,g2,c2=build_corner_trace(vlim)
        m1=l2==1
        axp.plot(t2[m1]-t2[m1][0], v2[m1]*3.6,c='#D55E00',lw=2,label=f'{nm} @0.4g')
    axp.set_xlabel('Time in lap [s]'); axp.set_ylabel('Velocity [km/h]')
    axp.set_title('Winning corner-legal profile (Lap 1)',fontweight='bold',fontsize=11); axp.legend(fontsize=8); axp.grid(alpha=0.3)
    out=os.path.join(RESULTS_DIR,'corner_aware_profile.png')
    fig.savefig(out,dpi=130,bbox_inches='tight',facecolor='white'); plt.close(fig)
    print(f"\nFigure -> {out}\nDONE.")


if __name__ == '__main__':
    main()
