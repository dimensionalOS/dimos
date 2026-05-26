# Capture Viewer (demo)

A tiny Bun server that browses a DimOS recording (SQLite) in the browser — reads
the DB directly (no Python). Demo-grade and disposable.

## 1. Record a capture

Append the stock `go2-memory` recorder to the run (records `color_image`,
`lidar`, `odom`):

```bash
dimos --simulation mujoco run unitree-go2-agentic-gemini go2-memory
# drive the dog (dashboard / `dimos agent-send "..."`), then Ctrl-C
```
→ `recording_go2.db` (in the directory you ran from)

## 2. View it

```bash
bun run tools/capture-viewer/server.ts            # default: recording_go2.db
# or: bun run tools/capture-viewer/server.ts /path/to/recording.db
# or: CAPTURE_DB=/path/to.db PORT=3000 bun run tools/capture-viewer/server.ts
```
Open http://localhost:3000 — scrub the timeline to play camera frames; the map
shows the dog's trajectory (from pose columns) with a live heading marker; the
table lists every recorded stream + row count.

## Notes
- Run the viewer **after** the capture process stops (clean WAL checkpoint).
- Renders camera frames (JPEG extracted from the LCM blob) + trajectory.
  `lidar` shows as a count only — its blob is LCM-encoded and needs a Python
  decoder (a job for the real app later).
