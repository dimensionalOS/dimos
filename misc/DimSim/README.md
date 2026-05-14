# DimSim

Standalone 3D simulation runner for SimStudio scenes. Load a scene, spawn AI agents, run tasks — with full sensor support (RGB-D, LiDAR).

## Setup

```bash
npm install    # installs everything (frontend + backend)
```

## Run

Terminal 1:
```bash
npm run server         # Node.js VLM backend on :8000
```

Terminal 2:
```bash
npm run dev            # Frontend on :5173
```

## Architecture

```
DimSim/
├── index.html              ← Sim-mode UI (scene dropdown + full sensor controls)
├── server.js               ← VLM backend (Express + OpenAI SDK)
├── src/
│   ├── main.js             ← Entry point (imports engine.js)
│   ├── engine.js           ← Full SimStudio engine (synced via copy-sources.sh)
│   ├── style.css           ← Synced from SimStudio
│   ├── AiAvatar.js         ← Agent class (synced)
│   └── ai/                 ← VLM modules (synced)
├── public/
│   ├── sims/               ← Scene JSON files + manifest.json
│   └── agent-model/        ← Robot GLB models
├── vlm-server/
│   └── asset-library.json  ← Persisted asset library data
├── copy-sources.sh         ← Sync engine from SimStudio
└── update-sims.sh          ← Rebuild scene manifest
```

## Sync from SimStudio

```bash
npm run sync
```

## Add/remove scenes

Drop `.json` files in `public/sims/`, then:
```bash
npm run update-sims
```
