# Memory Browser (VR)

WebXR app for Quest 3 that lets you review robot memory recordings (color frames, poses, point-cloud minimap) from a SQLite memory store.

## Running

```bash
python scripts/run_memory_browser.py --db data/go2_bigoffice.db
```

Flags: `--port 8443`, `--stream color_image`, `--n 50` (thumbnail count).

## Viewing in VR

1. On the Quest 3, open the browser and go to `https://<host-ip>:8443/memory_browser`.
2. Accept the self-signed cert, tap **Connect**, then enter VR.
3. Hold right grip + roll your wrist to scrub the timeline ribbon.
4. Lateral hand flick (swipe) clears the focused frame.

Host and headset must be on the same Wi-Fi / LAN.
