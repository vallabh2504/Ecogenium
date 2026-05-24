# Telematics — On-Car Data Acquisition & Pit-Wall System

End-to-end data pipeline: custom PCB on the car → WiFi (Calypso radio) → pit-wall server → optional cloud dashboard.

Full season documentation: [qWiki](https://ecogenium.qwikinow.de/content/cb4fe812-0402-4ff7-80c1-0b5effc30064?title=2425-telematics-board)

## Folder Map

```
telematics/
├── Hardware/            PCB design files (Altium Designer)
│   ├── TelematicsBoardV1/     Full Altium project (schematic, layout, BOM, Gerbers)
│   └── Shematics-Docs/        Exported PDFs and layer renders
│
├── Software/
│   ├── CarSide/         On-car firmware (RP2040 / Arduino C++)
│   │   └── TelematicsComponent/
│   │       ├── TelematicsComponent.cpp  Main firmware entry point
│   │       └── libraries/              GPS, Calypso radio, SD card drivers
│   │
│   └── ServerSide/      Pit-wall software
│       ├── TelematicsBoardWidget/   Qt C++ live-display widget (CAN via Calypso)
│       │   └── dbc/                 CAN message definitions (.dbc / .dbf)
│       ├── docker-compose.yaml      Docker stack for server-side services
│       └── LoggingBusmasterToSdFile.py  Python helper for BusMaster → SD logging
│
├── cloud-setup/         Cloud telemetry validation pipeline (Next.js + Supabase)
│   ├── README.md        Full setup guide
│   └── .env.example     Environment variable template — copy to .env and fill secrets
│
└── docs/
    ├── Calypso.cfg      Calypso radio configuration file
    └── CalypsoSetup.jpg Physical setup photograph
```

## Hardware — TelematicsBoardV1

Custom RP2040-based board logging GPS + CAN + sensors to SD card and streaming live over Calypso WiFi radio.

- **Schematic PDF:** `Hardware/Shematics-Docs/Schematic.pdf`
- **3D renders:** `Hardware/Shematics-Docs/3D-Front.png`, `3D-Back.png`
- **BOM:** `Hardware/TelematicsBoardV1/Project Outputs for TelematicsBoardV1/BOM.csv`
- **Gerbers (zip):** `Hardware/TelematicsBoardV1/Project Outputs for TelematicsBoardV1/TelematicsBoardV1.zip`

## Firmware — Car Side

Flash to the RP2040 on the board. See `Software/CarSide/README.md` for pin map and build steps.

## Pit-Wall Server

The Qt widget decodes live CAN frames via the `.dbc` definition file and displays them.

```bash
cd Software/ServerSide/TelematicsBoardWidget
cmake . && make
./TelematicsBoardWidget
```

Or start the full Docker stack:
```bash
cd Software/ServerSide
docker compose up
```

## Cloud Setup (optional, pre-race validation)

Simulates the full telemetry path before the real board is available.
Requires Supabase + Vercel accounts.

```bash
cd cloud-setup
cp .env.example .env    # fill in provider secrets
npm install
npm run dev
```

See `cloud-setup/README.md` for full deployment instructions.
