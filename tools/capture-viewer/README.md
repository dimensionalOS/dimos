# Capture Viewer (demo)

A tiny Bun server that browses a `go2-full-recorder` SQLite capture in the
browser — reads the DB directly (no Python). Demo-grade and disposable.

## 1. Record a capture

```bash
dimos --simulation mujoco run unitree-go2-agentic-gemini go2-full-recorder
# drive the dog (dashboard / `dimos agent-send "..."`), then Ctrl-C
```
→ `assets/output/captures/go2_full.db`

## 2. View it

```bash
bun run tools/capture-viewer/server.ts            # default db path
# or: bun run tools/capture-viewer/server.ts /path/to/go2_full.db
# or: CAPTURE_DB=/path/to.db PORT=3000 bun run tools/capture-viewer/server.ts
```
Open http://localhost:3000 — scrub the timeline to play camera frames; the map
shows the dog's trajectory (from pose columns) with a live heading marker; the
table lists every captured stream + row count.

## Notes
- Run the viewer **after** the capture process stops (clean WAL checkpoint).
- Renders camera frames (JPEG extracted from the LCM blob) + trajectory.
  `lidar`/`global_costmap` (LCM) and `agent` (pickle) show as counts only — their
  blobs need Python decoders (a job for the real Next.js app later).
