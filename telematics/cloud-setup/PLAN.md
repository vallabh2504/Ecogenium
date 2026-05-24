# 🚀 SEM 2026 Telematics: Software 3.0 Master Implementation Plan

This document serves as the exhaustive master blueprint for migrating the Ecogenium telematics pipeline from the fragile, local-only "Software 1.0/2.0" architecture to the highly secure, self-healing, cloud-native **Software 3.0** architecture. Multiple autonomous worker agents will refer to this document to execute the project in parallel, creating a unified, robust telemetry platform for the Shell Eco-marathon 2026.

---

## 1. Executive Summary

The Shell Eco-marathon is a grueling competition requiring intense energy management (particularly staying strictly under the 60V threshold for Hydrogen Fuel Cell systems). To optimize race strategy, real-time telemetry is critical. 

The legacy telematics setup relies on a local Wi-Fi router, a tripod receiver, a local Qt widget, and local Docker instances (MySQL/Grafana). This system is highly susceptible to range limits on massive tracks (like the 3.6 km Silesia Ring) and complex local port conflicts. 

**Software 3.0** completely demolishes the local Wi-Fi and ground-station hardware paradigms:
1. **Car-Side:** The Calypso Wi-Fi module is switched to **STA Mode** (Station), connecting to a high-speed 4G/5G smartphone hotspot mounted securely inside the cockpit.
2. **Cloud Database:** Telemetry is sent via authenticated HTTP POST over the commercial cellular network directly to a **Supabase PostgreSQL** cloud database.
3. **Global UI:** A **Vercel-hosted React/Next.js dashboard** subscribes to Supabase Realtime WebSocket streams, allowing the team lead in the pit lane, engineers in the garage, and professors back at the university to monitor the car simultaneously at 60 FPS.
4. **Agent Integration:** An integrated AI "Agent Core" acts as a conversational race engineer, connecting directly to the telemetry logs for instant, predictive race diagnostics.

---

## 2. Legacy Architecture Analysis (What We Are Replacing)

To understand the improvements, we must first deeply understand the legacy setups.

### 🛑 Software 1.0: The Real Race "Tripod" Setup
In the physical race setup, the data flows as follows:
1. **Sensors -> CAN-Bus:** Motor, fuel cell, and GPS data are written to the car's CAN-bus.
2. **Atava Board (AP Mode):** The custom telematics board reads the CAN-bus and uses the Calypso Wi-Fi module operating in **Access Point (AP) mode**. It broadcasts raw UDP packets into the air.
3. **The Tripod:** A highly directional antenna mounted on a tripod trackside catches the signal and feeds it to a local router.
4. **Ground Station:** A laptop connects to the router. A C++ Qt Widget receives the UDP packets on port `5001`, decodes them using `messages.dbc`, and writes them to a local Docker MySQL database. Grafana reads the database.
* **The Fatal Flaw:** The track is huge. As the car rounds the back straight, the Wi-Fi signal drops. The UDP packets vanish into the ether, causing fatal blind spots in race strategy. 

### 🧪 Software 2.0: The Mock Lab Setup
Located in `SEM 2026/mock_setup`, this was a brilliant hardware-in-the-loop simulation used to validate the pipeline:
1. Uses `uart_board_injector.py` to inject 5Hz data via USB to an FTDI chip.
2. Employs `align_agent_ip.ps1` to override DHCP and force a local laptop to IP `192.168.43.100`.
3. Validated Big-Endian packing and `0xFF` byte-stuffing (replacing `255` with `254` to prevent C++ memory shifts).
* **The Flaw:** While stable, it still fundamentally validates a local-only pipeline.

---

## 3. The Cloud-Native Architecture: Software 3.0

The new architecture is globally accessible and relies on commercial cellular infrastructure.

### 📡 The Vehicle Tunnel
Instead of broadcasting into the void, the Calypso module connects to the driver's phone hotspot (which has flawless cellular connection to track-side cell towers). The Atava board uses the Calypso's built-in HTTP client to format CAN frames into JSON and `POST` them to a Supabase REST endpoint.

### ☁️ The Supabase Cloud Data Layer
Supabase replaces the local MySQL/Docker stack entirely:
* **PostgreSQL Engine:** High-performance, highly relational database.
* **PostgREST API:** Automatically generates secure REST endpoints for the Atava board to POST to.
* **Realtime WebSockets:** Automatically broadcasts `INSERT` events to all connected Vercel clients.

### 💻 The Vercel Pit-Wall Dashboard
A Next.js (App Router) application that sits on the edge. It uses React Server Components for fast loading and Client Components for the live charts. No software needs to be installed by the pit crew—just scan a QR code or open a URL.

---

## 4. Security & Robustness: Handling Edge Cases

Moving to the cloud requires rigorous defense mechanisms.

### 🛡️ Edge Case 1: The Cellular Dead Zone (Silesia Ring Back Straight)
Cellular networks are reliable, but a 5-second drop is possible.
* **Solution (Double-Buffering):** The C++ firmware will feature a robust queue in RAM. If an `HTTP POST` returns a timeout or `503`, the payload is kept in the buffer. Once the connection re-establishes (e.g., HTTP `200 OK` ping), the board flushes the entire buffered array as a batch `POST` to Supabase, guaranteeing **zero packet loss**.

### 🛡️ Edge Case 2: High Data Volume & Supabase Connection Limits
If the car sends data at 50Hz, it will overwhelm standard API limits and balloon the database size.
* **Solution (On-Board Decimation):** The Atava board calculates high-frequency data locally, but only sends averaged/peak metrics at a stable 5Hz rate to the cloud. 

### 🔒 Security 1: Preventing Data Spoofing
We cannot allow unauthorized HTTP requests to flood the database.
* **Solution (Authentication & HMAC):** The Atava board will hold a secure JWT (Service Role Key) or, ideally, compute an HMAC-SHA256 signature for every payload using a secret key. A Supabase Edge Function will intercept the request, verify the signature, and only then insert it into Postgres.

### 🔒 Security 2: Row Level Security (RLS)
* **Solution:** The Next.js dashboard will use Supabase Auth (e.g., GitHub OAuth or Email/Password for the team). RLS policies will ensure that only `authenticated` users belonging to the `ecogenium_pit_crew` role can `SELECT` from the `telemetry_data` table. The public internet cannot see the race data.

---

## 5. UI/UX Design & Example User Experiences

The dashboard MUST feel exceptionally professional, slick, and data-dense, drawing inspiration from modern motorsport engineering software (like F1 telemetry suites or aerospace control centers). We will pivot away from trendy glassmorphism in favor of high-contrast, flat, precision-oriented design that prioritizes immediate readability under high-stress track conditions.

### 🎨 Aesthetic Guidelines (Ecogenium Brand Identity)
- **Theme:** Deep Navy / Slate dark mode (`#0B1120` to `#0F172A` background) to reduce glare in outdoor track environments.
- **Panels/Cards:** Solid, high-contrast flat panels (`#1E293B`) with razor-sharp borders (`#334155`). No blurring or unnecessary transparency.
- **Ecogenium Core Colors:** 
  - **Ecogenium Blue (`#005B9F` / `#2563EB`):** Used for primary navigation, active state highlights, and primary data traces.
  - **Crisp White (`#FFFFFF`):** High-contrast text for critical numbers and primary labels.
- **Telemetry Semantic Colors:**
  - **Efficiency (Green):** `#10B981` (e.g., optimal H2 consumption, positive regen braking).
  - **Warning (Amber):** `#F59E0B` (e.g., Motor temp approaching limits).
  - **Critical (Red):** `#EF4444` (e.g., Fuel Cell Voltage approaching the strictly enforced 60V SEM limit).
- **Typography:** Google Font `Inter` or `Roboto Mono` for fixed-width data tables to ensure numbers align perfectly vertically, and `Outfit` for section headers.

### 🧑‍💻 Detailed User Navigation & Experience

The application is built as a Single Page Application (SPA) with a persistent left-hand navigation sidebar (Ecogenium Blue).

**View 1: The Pit-Wall Dashboard (Live Run Strategy)**
* **Navigation:** The user clicks "Live Run" from the sidebar.
* **Layout:** The top row features three prominent, flat numerical readouts: **Current Speed**, **Fuel Cell Voltage** (with a red threshold indicator line at 60V), and **H2 Consumption (Joules)**. 
* **Interaction:** The center screen is dominated by a multi-trace timeline chart. The user can hover their mouse over the moving trace to freeze the tooltip, seeing exactly what the values were 2.5 seconds ago. Below the chart is a live-scrolling "Event Log" that auto-tags system flags (e.g., `[14:02:33] REGEN BRAKING ENGAGED`).

**View 2: The Garage Engineer (Historical & Diagnostic Analysis)**
* **Navigation:** The engineer clicks "Run History" from the sidebar.
* **Layout:** A data-table interface displaying previous test runs, searchable by date or driver. Clicking a run opens a detailed post-mortem view.
* **Interaction:** The engineer can drag to highlight a specific time-slice of the graph (e.g., a massive voltage drop). The UI instantly zooms in, and the right-hand panel automatically populates with the exact CAN-bus hexadecimal payloads sent during that millisecond for deep low-level debugging.

**View 3: The AI Race Engineer (Agent Core Sidebar)**
* **Navigation:** A persistent "Ask Agent" button sits in the bottom right corner, opening a slick slide-out panel (`#1E293B`).
* **Interaction:** The user types queries in natural language instead of writing complex SQL.
* **Example Flow:** 
  * **User:** *"Graph our average speed versus motor temperature for the last 5 laps."*
  * **Agent Core:** *(Autonomously queries Supabase, generates a temporary chart component, and displays it in the chat)* *"Here is the correlation. Note that as motor temperature exceeded 65°C on Lap 4, average speed dropped by 8%."*
  * **User:** *"What was the peak voltage during that thermal spike?"*
  * **Agent Core:** *"The peak voltage hit 58.2V, safely below the 60V limit."*
---

## 6. Similar Projects & Inspiration

To ensure architectural success, worker agents should draw inspiration from:
1. **Formula 1 AWS Telemetry:** Utilizing Kinesis (Supabase Realtime is our equivalent) for sub-second global data distribution.
2. **AWS IoT FleetWise:** Specifically how they handle vehicle-to-cloud data ingestion using edge-decimation (sending only what matters).
3. **Open-Source Next.js IoT Dashboards:** Reviewing standard implementations of `@supabase/supabase-js` interacting with `framer-motion` to handle state updates without React re-render lag.

---

## 7. Parallel Execution Plan (Worker Agent Milestones)

The project is divided into strictly independent phases. Multiple agents can be dispatched simultaneously.

### 🚀 Phase A: Cloud & Infrastructure (Agent 1)
*Goal: Provision the backend and API endpoints.*
* **Step 1:** Define the Supabase schema in `schema.sql`.
  * `telemetry_data`: `id`, `timestamp`, `lap_number`, `motor_power_kw`, `fuel_cell_voltage_v`, `fuel_cell_current_a`, `h2_consumption_joules`, `vehicle_speed_kmh`, `situational_flag`.
  * `system_logs`: `id`, `timestamp`, `component`, `error_code`, `message`.
* **Step 2:** Write the Row Level Security (RLS) definitions to lock down the tables.
* **Step 3:** Create an RPC or Edge Function to handle batch inserts (for the car's double-buffering failsafe).
* **Step 4:** Generate a `seed_mock_data.sql` script to flood the database with simulated race data so frontend agents can work immediately.

### 🚀 Phase B: The Vercel Dashboard Frontend (Agent 2)
*Goal: Build the premium Glassmorphic UI.*
* **Step 1:** Initialize the Next.js App Router project in `vercel_dashboard/`.
* **Step 2:** Configure `tailwind.config.ts` with custom colors, backdrop blurs, and Framer Motion utilities.
* **Step 3:** Build the Supabase client wrapper (`lib/supabase.ts`).
* **Step 4:** Implement the UI components: `TelemetryCard`, `LiveChart` (using Recharts/Chart.js), and `StatusIndicator`.
* **Step 5:** Wire the `LiveChart` to `supabase.channel('custom-insert-channel').on('postgres_changes', ...)` for 60FPS updates.

### 🚀 Phase C: AI Agent Integration (Agent 3)
*Goal: Build the Conversational Race Engineer.*
* **Step 1:** Create the chat interface component (`AgentChatbox.tsx`) in the dashboard.
* **Step 2:** Implement an API route (`/api/chat`) using the Vercel AI SDK.
* **Step 3:** Provide the AI with tool-calling capabilities (e.g., `fetchTelemetryHistory(lap: number)`, `getLatestErrors()`) so it can securely query the Supabase database and answer user questions accurately.

### 🚀 Phase D: Firmware & Vehicle Injection Testing (Agent 4)
*Goal: Transition the Atava board and validate end-to-end.*
* **Step 1:** Write `calypso_sta_config.h` detailing AT commands to switch from AP to STA mode and connect to a WPA2 hotspot.
* **Step 2:** Write `http_post_client.cpp` to format raw CAN data into the JSON schema required by Supabase.
* **Step 3:** Implement the double-buffering queue in C++.
* **Step 4:** Write a Python equivalent (`cloud_ghost_car.py`) to simulate the Atava board and continuously POST real `messages.dbc` data to the live Supabase endpoint for final stress testing.

---

## 8. Proposed File Structure (`SEM 2026/New_Setup/`)

```text
SEM 2026/New_Setup/
│
├── PLAN.md                          # This master blueprint
│
├── firmware/                        # C++ updates for Atava board
│   ├── calypso_sta_config.h         # AT commands for Hotspot STA mode
│   ├── http_post_client.cpp         # HTTP client & double-buffering logic
│   └── tests/cloud_ghost_car.py     # Python simulator to POST to Supabase
│
├── supabase/                        # Cloud DB configurations
│   ├── migrations/
│   │   ├── 01_schema.sql            # Table definitions
│   │   ├── 02_rls_policies.sql      # Security lockdowns
│   │   └── 03_edge_functions.sql    # HMAC verification / Batch ingest
│   └── seed/seed_mock_data.sql      # Initial test data generator
│
└── vercel_dashboard/                # Next.js web application
    ├── package.json                 # Dependencies (React, Supabase, Tailwind, Framer Motion, AI SDK)
    ├── src/
    │   ├── app/
    │   │   ├── page.tsx             # Main Pit Wall View
    │   │   ├── api/chat/route.ts    # AI Agent Backend
    │   │   └── layout.tsx
    │   ├── components/
    │   │   ├── LiveChart.tsx        # WebSocket connected Recharts
    │   │   ├── GlassCard.tsx        # UI Wrapper
    │   │   └── AgentChatbox.tsx     # Conversational interface
    │   └── lib/
    │       └── supabase.ts          # Client instantiation
    └── tailwind.config.ts           # Premium aesthetic tokens
```
