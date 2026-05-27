# Live VR Visualization — "Rerun in VR"

Plan for a VR module that subscribes to a **live** robot's streams (odometry,
lidar, camera, TF, annotations) and renders the robot moving + building its map
in real time, viewed from a god view in a Quest 3 headset. The eventual code
lives in this folder.

Status: **planning only — not implemented.**

Sibling/evolution of [`dimos/teleop/memory_world`](../../teleop/memory_world/),
which does the same rendering but from a recorded SQLite store (playback). This
is the **live** counterpart.

---

## Core idea

dimos already feeds Rerun via [`RerunBridgeModule`](../../visualization/rerun/bridge.py):
it subscribes to all LCM topics and converts any message with a `to_rerun()`
method into a Rerun archetype. This feature is the **same pattern with a
different sink** — subscribe to the live streams, and instead of `rr.log(...)`,
push incremental frames over a WebSocket to a Three.js client rendering in the
headset.

The data already flows over LCM. The viz module just listens. memory_world
stops being "a SQLite player" and becomes "a live observer that also supports
playback."

---

## Architecture

### Server — a live module
Subclass `Module` (sharing a WS-plumbing base with memory_world) with `In[]`
ports wired to the robot's `Out[]` streams:

| Port | Type | Stream (Go2) |
|------|------|--------------|
| `odom` | `In[PoseStamped]` | `odom` |
| `lidar` | `In[PointCloud2]` | `lidar` |
| `color_image` | `In[Image]` | `color_image` |
| `tf` | `In[TFMessage]` | `/tf` |
| `markers` | `In[EntityMarkers]` | annotation topic |

- `start()` calls `.subscribe(cb)` on each input (the `RerunBridge._on_message`
  pattern). Each callback transforms the message (TF lookup → world frame) and
  pushes a delta frame to connected headsets.
- Connect to a running robot over **LCM multicast** (same network) — no code on
  the robot side, just blueprint wiring (or `subscribe_all` like RerunBridge).

Relevant existing machinery:
- Robot stream declarations: `dimos/robot/unitree/go2/connection.py`
- `In[T].subscribe(cb)` reactive pattern: `dimos/core/stream.py`, example
  `dimos/navigation/movement_manager/movement_manager.py`
- Transports / LCM multicast: `dimos/core/transport.py`
- TF buffer: `dimos/protocol/tf/tf.py` (`get_transform(parent, child, time_point)`)
- Live voxel accumulation: `dimos/mapping/voxels.py`
  `VoxelMapTransformer(emit_every=1)`

### Client — extend memory_world's `scene.js`
Handle **incremental** updates instead of one-shot payloads.

| Stream | Type | VR rendering | Reuse |
|--------|------|--------------|-------|
| odom | `PoseStamped` | live robot avatar + growing trail | odom trail + markers |
| lidar | `PointCloud2` | voxel map that grows live | cube renderer |
| image | `Image` | floating live-feed panel pinned to the robot | image quads (+ flipY fix) |
| tf | `TFMessage` | coordinate-frame axes; place data in world frame | Z-up→Y-up group |
| markers | `EntityMarkers` | boxes / linestrips in 3D | new (small) |

User stands in god view, scales the scene down (bimanual scale), watches the
robot drive and the map form — the Rerun experience, but from inside it.

---

## The hard part: incremental updates & bandwidth

1. **Voxel map regrows every lidar frame.** Re-sending ~140k cubes at 10 Hz is
   far too much bandwidth. Options (increasing effort):
   - **Throttled full resend** — rebuild map server-side, push at ~1–2 Hz.
     Good enough to *watch* a map form. **Start here.**
   - **Delta voxels** — push only newly-occupied cells; `VoxelGrid`'s hashmap
     knows occupied cells, so diffing is feasible.
   - **Client-side accumulation** — send raw per-scan clouds, accumulate in JS.
     Lighter per-message but reimplements voxelization without Open3D.
2. **Per-stream rates differ** — pose cheap/fast (every update); lidar heavy
   (throttle); images medium (latest, 2–5 Hz). Per-stream throttle policy.
3. **TF timing** — use the TF buffer to place each sensor reading into the map
   frame at *its* timestamp, not "latest", or the map smears as the robot moves.
4. **Performance / thermal** — already our pain point (cube count, Quest 3
   thermal throttle). Live is worse: LOD (cubes near, points far) + the
   points-mode toggle matter more, and update rate must be capped.

---

## Reuse vs new

- **Reused (most of it):** WebSocket plumbing, the whole `scene.js` renderer
  (cubes, image quads, odom trail, HUD minimap, locomotion, scale, teleport),
  voxel accumulation, Z-up→Y-up coordinate group, height coloring.
- **New:** `In[]` subscriptions + LCM wiring, incremental/delta message types,
  a live robot avatar, TF-axis rendering, `EntityMarkers` rendering, per-stream
  throttling.

---

## Phasing

0. **Live pose only** — robot avatar + growing trail. Cheapest; proves the
   live LCM→WS pipe end to end.
1. **Live voxel map** — server accumulates, throttled full resend at 1–2 Hz.
2. **Live camera panel** — latest image pinned near the robot.
3. **TF axes + annotation markers.**
4. **Delta voxel encoding + LOD** — bandwidth/perf hardening.

---

## Decisions to make before building

- **Delta vs throttled-full** for the map (bandwidth vs complexity) — start
  throttled.
- **Observer-only vs also-controlling** — could merge with the Quest *teleop*
  module so one headset drives the robot *and* sees the live map.
- **One robot vs fleet** — subscribe-all naturally extends to a multi-robot
  god view.
