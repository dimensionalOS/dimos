# Viewer Backends

Dimos supports Rerun as its visualization backend (`rerun` default, or `none` to disable).

## Quick Start

Choose your viewer via the CLI:

```bash
# Rerun native viewer (default) - dimos-viewer with built-in teleop + click-to-navigate
dimos run unitree-go2

# Explicitly select the viewer backend:
dimos --viewer rerun run unitree-go2
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

## Rendering with Custom Blueprints

To enable visualization in your own blueprint, use `vis_module`:

```python skip
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
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

```python skip
import rerun as rr
from dimos.visualization.rerun.init import rerun_init

# Basic init (no gRPC server — use when RerunBridgeModule is already running)
rerun_init()
rr.log("debug/my_points", rr.Points3D(positions=[[1, 2, 3]]))

# Start a gRPC server so a viewer can connect.  `grpc_config` is required
# whenever start_grpc=True; it carries the connect URL and the server memory cap.
rerun_init(
    start_grpc=True,
    grpc_config={
        "connect_url": "rerun+http://127.0.0.1:9999/proxy",
        "server_memory_limit": "4GB",
    },
)
# Then connect with: dimos-viewer --connect rerun+http://127.0.0.1:9999/proxy
```

When a `RerunBridgeModule` is already part of your blueprint, you typically don't need `start_grpc` — just call `rerun_init()` and log directly with `rr.log()`. The data will appear in the existing viewer.

## TF Frames in Rerun

When a module calls `self.tf.publish(...)`, the resulting `TFMessage` is converted to a Rerun `Transform3D` and logged at the entity path `world/tf/<child_frame_id>`. The default `Spatial3DView` is rooted at `world`, so frames appear in the 3D scene with no extra logging code. The conversion lives in `TFMessage.to_rerun()` ([`dimos/msgs/tf2_msgs/TFMessage.py`](/dimos/msgs/tf2_msgs/TFMessage.py#L126)) and runs inside whichever Rerun bridge is active.

> Note: every transform is flattened under `world/tf/` regardless of TF tree depth. Rerun's native convention is hierarchical (each path segment is a frame), but the dimos bridge does not yet reconstruct full ancestry — see the `TODO` in [`dimos/visualization/rerun/bridge.py`](/dimos/visualization/rerun/bridge.py#L79).

### Two ways to run the bridge

| Path | Use |
|------|-----|
| **In-blueprint** | Include `RerunBridgeModule` in your composition. All shipped agentic blueprints do this. |
| **Standalone** | Run `dimos rerun-bridge` in a separate terminal — it subscribes to LCM and picks up any blueprint already running on the host. Useful as a sidecar viewer for debugging. |

The standalone bridge subscribes to LCM at startup, so transforms published before it comes up are missed (the TF buffer is short and Rerun does not backfill). Start `dimos rerun-bridge` first when using it as a sidecar.
