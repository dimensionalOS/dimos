# Viewer Backends

Dimos supports three visualization backends: `rerun` (default), `foxglove`, and `none`.

## Quick Start

Choose your viewer via the CLI:

```bash
# Rerun native viewer (default) - dimos-viewer with built-in teleop + click-to-navigate
dimos run unitree-go2

# Explicitly select the viewer backend:
dimos --viewer rerun run unitree-go2
dimos --viewer foxglove run unitree-go2
dimos --viewer none run unitree-go2
```

Control how the Rerun viewer opens with `--rerun-open` and `--rerun-web`:

```bash
# Open native desktop viewer (default)
dimos --rerun-open native run unitree-go2

# Open web viewer in browser
dimos --rerun-open web run unitree-go2

# Open both native and web
dimos --rerun-open both run unitree-go2

# No viewer (headless) — data still accessible via gRPC
dimos --rerun-open none run unitree-go2

# Serve the web viewer without auto-opening a browser
dimos --rerun-web --rerun-open native run unitree-go2
```

## Viewer Modes Explained

### Rerun Native (`rerun`, `--rerun-open native`) — Default

**What you get:**
- [dimos-viewer](https://github.com/dimensionalOS/dimos-viewer), a custom Dimensional fork of Rerun with built-in keyboard teleop and click-to-navigate
- Native desktop application (opens automatically)
- Better performance with larger maps/higher resolution
- No browser or web server required

---

### Rerun Web (`rerun`, `--rerun-open web`)

**What you get:**
- Browser-based dashboard at http://localhost:7779
- Rerun 3D viewer + command center sidebar in one page
- Teleop controls and goal setting via the web UI
- Works headless (no display required)

---

### Foxglove (`foxglove`)

**What you get:**
- Foxglove bridge on ws://localhost:8765
- No Rerun (saves resources)
- Better performance with larger maps/higher resolution
- Open layout: `assets/foxglove_dashboards/old/foxglove_unitree_lcm_dashboard.json`

---

## Rendering with Custom Blueprints

To enable visualization in your own blueprint, use `vis_module`:

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.visualization.vis_module import vis_module

camera_demo = autoconnect(
    CameraModule.blueprint(),
    vis_module(viewer_backend=global_config.viewer),
)

```

Run the stack locally (this blocks until you stop the process):

```python skip
from dimos.core.coordination.module_coordinator import ModuleCoordinator

if __name__ == "__main__":
    ModuleCoordinator.build(camera_demo).loop()
```

Every LCM stream, such as `color_image` (output by CameraModule), that uses a data type (like `Image`) that has a `.to_rerun` method will get rendered (`rr.log`) using the LCM topic as the rerun entity path. In other words: to render something, simply log it to a stream and it will automatically be available in rerun.

## Performance Tuning

### Symptom: Slow Map Updates

If you notice:
- Robot appears to "walk across empty space"
- Costmap updates lag behind the robot
- Visualization stutters or freezes

This happens on lower-end hardware (NUC, older laptops) with large maps.

### Increase Voxel Size

Edit [`dimos/robot/unitree/go2/blueprints/smart/unitree_go2.py`](/dimos/robot/unitree/go2/blueprints/smart/unitree_go2.py):

```python skip
# Before (high detail, slower on large maps)
voxel_mapper(voxel_size=0.05),  # 5cm voxels

# After (lower detail, 8x faster)
voxel_mapper(voxel_size=0.1),   # 10cm voxels
```

**Trade-off:**
- Larger voxels = fewer voxels = faster updates
- But slightly less detail in the map

---

## Direct Visualization from a Module

If you want to log data to Rerun directly from inside a module (e.g. for debugging or one-off visualizations), use `rerun_init` instead of calling `rr.init()` yourself. It handles colormap registration and can optionally start a gRPC server so a viewer can connect.

```python
import rerun as rr
from dimos.visualization.rerun.init import rerun_init

# Basic init (no gRPC server — use when RerunBridgeModule is already running)
rerun_init()
rr.log("debug/my_points", rr.Points3D(positions=[[1, 2, 3]]))

# Start a gRPC server so you can connect a viewer
rerun_init(start_grpc=True)
# Then connect with: dimos-viewer --connect rerun+http://127.0.0.1:9877/proxy

# Custom gRPC config
rerun_init(
    start_grpc=True,
    grpc_config={
        "connect_url": "rerun+http://127.0.0.1:9999/proxy",
        "server_memory_limit": "4GB",
    },
)
```

When a `RerunBridgeModule` is already part of your blueprint, you typically don't need `start_grpc` — just call `rerun_init()` and log directly with `rr.log()`. The data will appear in the existing viewer.

## Recording demonstrations to `.rrd` for robot learning

The `teleop_quest_piper_data_collection` blueprint pairs the live Rerun viewer
with a standalone **`RerunDataRecorder`** that writes one `.rrd` per
demonstration *episode*. The viewer and the recorder are independent DimOS
modules — either can be omitted without affecting the other — and they share a
single `piper_data_collection_rerun_config()` so the operator sees exactly what
gets recorded. The bridge module is unchanged; the recorder owns its own
`rr.RecordingStream` and subscribes to the same LCM topics in parallel.

### Layout

```
{data_dir}/piper_data_collection/<session_ts>/episode_001.rrd
{data_dir}/piper_data_collection/<session_ts>/episode_002.rrd
…
```

`<session_ts>` is a UTC `YYYYMMDDTHHMMSSZ` stamp. The episode counter is
1-indexed and zero-padded to 3 digits.

### Per-episode entities

Each `.rrd` carries:

- `/observation/camera/usb` — the USB camera frames (`rr.Image`).
- `/observation/state/<joint>` — per-joint measured scalars including the gripper.
- `/action/<joint>` — per-joint commanded scalars including the gripper.
- `/meta/episode_id`, `/meta/episode_index`, `/meta/session_id`, and
  (when `DIMOS_OPERATOR` is set) `/meta/operator` — static text documents.

### Recording on/off toggle

Press the **right-controller B** button (`right_secondary`) to **toggle**
recording. The recorder has two states:

- **RECORDING** — `.rrd` is open, messages flow in.
- **IDLE** — no `.rrd` is open; messages on the bus continue to drive the
  live viewer but are silently discarded by the recorder.

The recorder **starts in IDLE** — nothing lands on disk until you press the
toggle button. This keeps warmup motion, scene setup, and gripper calibration
out of every recording. Each press flips the state:

1. Blueprint starts → recorder is IDLE; no `.rrd` exists.
2. Get the robot in position and ready to demo → press B → recorder opens
   `episode_001.rrd`, enters RECORDING.
3. Demo finishes → press B → recorder flushes and closes `episode_001.rrd`,
   enters IDLE.
4. Take your time resetting the scene — the robot may still be commanded,
   the camera may still stream, none of it lands on disk.
5. Press B again → recorder opens `episode_002.rrd`, enters RECORDING.

Press feedback appears in the log:

```
EpisodeBoundary: recording stopped — waiting for next press to resume
EpisodeBoundary: recording started → /…/episode_002.rrd
```

The toggle is debounced (500 ms by default); bouncing the button twice in
quick succession only counts once. If you toggle off before any payload has
been logged, the empty file is removed; the next toggle-on opens the next
slot number (so filenames may have honest holes if you press twice with no
demo in between).

> **Why B, not A?** A (`right_primary`) is the press-and-hold engage button
> used by the teleop tasks (`TeleopIKTask` / `PinkTeleopTask`) — reusing it
> would toggle the recording on every engage. B is on the same hand but a
> deliberate reach away.

### Reading recordings for training

For ad-hoc reads, post-session, GoP-split and keyframe-tag a captured episode:

```bash
rerun rrd optimize /path/to/episode_007.rrd
```

Then read it from a PyTorch training loop with the experimental dataloader:

```python skip
import rerun.experimental as rrx

dataset = rrx.dataloader(
    rrds=["/path/to/episode_007.rrd"],
    window=(0.0, 1.0),  # 1-second observation/action chunks
)
sample = next(iter(dataset))
# sample contains /observation/camera/usb, /observation/state/*, /action/*
```

> **Note:** `rerun.experimental.dataloader` is experimental. The on-disk `.rrd`
> format is backward-compatible; only the read-side API may shift.

### Browsing collected episodes

Point a Rerun catalog server at the data-collection directory to inspect /
flag episodes in the Dataset Review UI (rerun-sdk 0.32+):

```python skip
import rerun as rr

rr.server.Server(datasets={"piper": "/path/to/piper_data_collection"})
```

Each `.rrd` becomes one addressable row in the resulting dataset; the static
`/meta/*` entities surface as per-row columns.

## How to use Rerun on `dev` (and the TF/entity nuances)

Rerun on `dev` is **module-driven**: modules decide what to log, and `Blueprint.build()` sets up the shared viewer + default layout.
