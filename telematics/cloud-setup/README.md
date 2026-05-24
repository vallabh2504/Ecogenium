# Ecogenium SEM 2026 Telematics Software 3.0

Live-cloud, agent-native telemetry pipeline for validating the SEM 2026 car telemetry path before the real CarSide board is available.

## What This Setup Proves

- A simulator or server-side board can publish car-like CAN telemetry to cloud ingestion.
- Supabase stores raw CAN frames, decoded signals, live metrics, run metadata, and system logs.
- The Next.js/Vercel pit-wall dashboard renders live run state, history, raw-frame drilldown, and an optional AI race engineer.
- Firmware-facing reference code preserves the current 18-byte legacy frame contract so future CarSide work can reuse it.

## Secret Handling

The real `.env` is gitignored. It may contain provider tokens plus generated project secrets. Use `.env.example` for shape only.

## Main Folders

- `supabase/`: migrations, Edge Function, seed data, deployment notes.
- `vercel_dashboard/`: Next.js dashboard.
- `protocol/` and `simulator/`: Python frame codec and cloud ghost car.
- `firmware/`: portable C++ reference harness and Calypso notes.
- `scripts/`: deployment and verification helpers.
