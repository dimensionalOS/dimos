# Memory Browser - New Features Plan

## Context

The VR Memory Browser is a WebXR app for Quest 3 that lets operators review robot memory recordings. It already has: a curved thumbnail ribbon with wrist-roll scrub, a large preview pane, a top-down point cloud minimap with pose marker, a dashboard panel (frame metadata), a timeline bar, and a wrist-roll dial. It reads from a SQLite memory store (`go2_bigoffice.db`) with streams: `color_image` (4164), `lidar` (2251), `odom` (5465), `color_image_embedded` (4164 with CLIP vectors).

The goal is to add 5 features that leverage memory2's untapped capabilities (keyframe detection, semantic search, lidar visualization, signal processing).

---

## Feature 1: Keyframe Sampling Mode

**What**: Instead of uniform `throttle(span/N)`, select frames where the robot actually did something (turns, stops, speed changes).

**Server only** - no client changes. The ribbon just gets more interesting thumbnails.

**Changes to `module.py`**:
- Add config: `sampling_mode: Literal["uniform", "keyframe"] = "uniform"` + tuning params (`keyframe_prominence`, `keyframe_distance`, `keyframe_significance_k`)
- In `_build_thumbnails()`, branch on mode:
  - **keyframe**: Run `odom` stream through `speed()` -> `peaks(prominence, distance)` -> `significant(method="mad", k)` to get keyframe timestamps. Then fetch nearest `color_image` for each via `stream.at(ts, tolerance=0.5)`. If fewer than N keyframes found, fill remaining with uniform. If more, subsample evenly.
  - Use `odom` stream (lightweight, no blob decode) not `color_image` (avoids loading 4164 images)

**Existing functions to reuse**:
- `dimos/memory2/transform.py`: `speed()`, `peaks()`, `significant()`, `throttle()`
- `dimos/memory2/stream.py`: `.at(ts, tolerance)`, `.transform()`

---

## Feature 2: Object Detection + Map Overlay

**What**: Detect objects in the active frame, show them in the dashboard, let user select one, then search across ALL frames and plot instances on the minimap.

**Trigger**: Manual - left trigger press sends `detect_request` to server. No auto-detection on scrub (saves GPU when just browsing).

**Data flow**:
1. User scrubs to frame, presses left trigger to request detection
2. Server loads full-res image, runs VLM caption: `model.query(image, "List all visible objects, comma-separated")` (~300ms)
3. Server sends `detections` text msg with object list to client
4. Client shows numbered list in dashboard panel
5. User selects object (left thumbstick up/down + left trigger confirm)
6. Client sends `select_object {name}` to server
7. Server runs `clip_model.embed_text(name)` -> `color_image_embedded.search(embedding, k=50)`
8. Server extracts pose XY from each result, sends `MSG_OBJECT_MARKERS` (0x04) binary msg
9. Client renders colored marker circles on minimap

**Files to modify**:

| File | Changes |
|------|---------|
| `module.py` | Add `_vlm_model`, `_clip_model` (lazy-loaded). Add `_on_detect()`. Add `_on_select_object()` with semantic search. Detection cache per frame index. Search cache per object name. |
| `messages.py` | Add `MSG_OBJECT_MARKERS = 0x04` |
| `scene.js` | Extend `_renderDashboard()` with DETECTIONS section (numbered, highlighted selection). Add `setObjectMarkers(name, color, positions)` to render marker pool on minimap. Add `setDetections(frameIdx, objects)` to store + re-render dashboard. |
| `main.js` | Handle `detections` and `search_complete` text msgs. Handle `MSG_OBJECT_MARKERS` binary msg. Forward `select_object` gesture. |
| `input_adapter.js` | Add left thumbstick up/down -> `object_scroll` gesture. Add left trigger -> `object_select` / `detect_request` gesture. |
| `protocol.js` | Export `MSG_OBJECT_MARKERS` |

**Models to use**:
- Detection: `dimos.models.vl.moondream.MoondreamVlModel` - lazy load on first detection
- Search: CLIP embedding model for `embed_text()`. The `color_image_embedded` stream already has pre-computed CLIP vectors via sqlite-vec.

**New protocol messages**:
- Server->Client text: `{"type": "detections", "frame_index": int, "objects": [{"name": str, "confidence": float}]}`
- Server->Client text: `{"type": "search_complete", "object_name": str, "count": int}`
- Server->Client binary: `MSG_OBJECT_MARKERS (0x04)` - header: `{object_name, color}`, payload: packed float32 `[x, y, similarity, ...]`
- Client->Server text: `{"type": "detect_request"}` (left trigger when not in detection mode)
- Client->Server text: `{"type": "select_object", "name": str}`
- Client->Server text: `{"type": "object_scroll", "direction": "up"|"down"}`

---

## Feature 3: Dashboard - Objects List

This is part of Feature 2's client work. In `scene.js` `_renderDashboard(meta)`:
- After existing rows (id, dims, bright, sharp, pose), add a DETECTIONS section
- Numbered list: `0: chair (0.95)`, `1: table (0.88)`, etc.
- Selected row highlighted in `#ff9944`
- Room for ~4-5 rows in remaining canvas space (current content ends ~y=550, canvas is 720px)
- Store detections in `this._detections = new Map()` keyed by frame index

---

## Feature 4: Video Playback

**What**: Auto-advance cursor from selected frame at configurable FPS. Play/pause toggle.

**Server-side timer** (owns cursor state, broadcasts `active_index`):
- Add `_playing: bool`, `_playback_speed: float`, `_playback_timer: threading.Timer`
- On play: chain `threading.Timer(1/speed, _advance_cursor)` that increments `_cursor_index` by 1 and broadcasts. Each tick schedules the next.
- On pause or end-of-ribbon: cancel timer
- While playing, ignore palm_roll_delta (or auto-pause on scrub)

**Files to modify**:

| File | Changes |
|------|---------|
| `module.py` | Add playback state + timer. Handle `play_toggle` and `playback_speed` messages. Send `playback_state` confirmation. |
| `input_adapter.js` | Detect left Y button press (button index 5) as `play_toggle`. Left thumbstick X-axis while playing for speed adjustment. |
| `main.js` | Forward `play_toggle` gesture. Handle `playback_state` text msg. |
| `scene.js` | Small play/pause indicator on caption strip or timeline bar. |

**Config**: `playback_fps: float = 5.0`, `playback_speed_min/max`

---

## Feature 5: Lidar Integration

### 5a. Build Global Map from Lidar Store

**What**: Add option to build minimap from `lidar` stream in SQLite. Keep pickle as default, lidar as alternative via config flag.

**Server only** (`module.py`):
- Add config: `map_source: Literal["pickle", "lidar"] = "pickle"`, `lidar_stream_name`, `lidar_max_scans: int = 100`
- Add `_build_global_map_from_lidar()`:
  1. Subsample lidar stream: `stream.transform(throttle(span / max_scans))`
  2. For each obs: decode PointCloud2, transform to world frame using obs.pose
  3. Concatenate all points, Z-slab filter, histogram render (reuse existing code)
- ~100 scans x ~2000 pts = 200K points, takes 2-5s on first connect

### 5b. Per-Frame Lidar Overlay

**What**: When viewing a frame, highlight its lidar scan on the minimap.

**Files to modify**:
- `module.py` - on `active_index` change, fetch nearest lidar scan via `lidar_stream.at(ts, tolerance=1.0)`, transform to world frame, project to 2D, send as `MSG_LIDAR_OVERLAY`
- `messages.py` - add `MSG_LIDAR_OVERLAY = 0x05`
- `scene.js` - render overlay as `THREE.Points` (single draw call, handles 2000 pts cheaply). Bright cyan color, fades on `clear_focus`.
- `protocol.js` / `main.js` - handle new binary msg

**Payload**: float32 pairs `[u, v, u, v, ...]` in normalized minimap coords (0-1). ~2000 pts = 16KB, fine over Wi-Fi.

---

## Implementation Order

```
1. Feature 1  (Keyframe Sampling)     - server only, ~50 lines, warm-up
2. Feature 4  (Video Playback)        - server + client, ~100 lines, no new models
3. Feature 5a (Lidar Map from Store)  - server only, ~80 lines, validates lidar access
4. Feature 2+3 (Detection + Map)      - largest feature, ~350 lines, manual trigger
5. Feature 5b (Lidar Overlay)         - server + client, ~160 lines total
```

---

## Verification

For each feature:
1. Launch: `python scripts/run_memory_browser.py --db data/go2_bigoffice.db`
2. Connect from Quest 3 browser to `https://<host>:8443/memory_browser`
3. Check server logs for diagnostic events from client

- **F1**: Verify ribbon thumbnails show varied scenes (turns, stops) not uniform hallway
- **F4**: Y button toggles playback, preview auto-advances, timeline cursor moves
- **F5a**: Set `map_source="lidar"`, verify minimap still renders from store data
- **F2+3**: Scrub to frame, left trigger to detect, check dashboard shows objects, select one, verify markers on minimap
- **F5b**: Scrub to frame, verify cyan point cloud overlay appears on minimap matching robot's position
