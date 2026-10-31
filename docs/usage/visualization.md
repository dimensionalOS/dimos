# Viewer Backends

Dimos uses Rerun for visualizations. It can be disabled by using
`dimos --viewer none ...`.

Blueprints add Rerun stream visualization with `vis_module(...)`, which renders typed
robot streams according to `GlobalConfig.viewer`.

## Quick Start

Choose your viewer via the CLI:

```bash
# Rerun native viewer - dimos-viewer with built-in teleop + click-to-navigate
dimos run unitree-go2

# Disable visualization:
dimos --viewer none run unitree-go2
```

Control how the Rerun viewer opens with `--rerunbridgemodule.rerun-open` and `--rerunbridgemodule.rerun-web`:

```bash
# Open native desktop viewer (default)
dimos run unitree-go2 --rerunbridgemodule.rerun-open native

# Open web viewer in browser
dimos run unitree-go2 --rerunbridgemodule.rerun-open web

# Open both native and web
dimos run unitree-go2 --rerunbridgemodule.rerun-open both

# No viewer (headless): data still accessible via gRPC
dimos run unitree-go2 --rerunbridgemodule.rerun-open none

# Serve the web viewer without auto-opening a browser
dimos run unitree-go2 --rerunbridgemodule.rerun-web true --rerunbridgemodule.rerun-open native
```

## Viewer Modes Explained

### Rerun Native (`rerun`, `--rerunbridgemodule.rerun-open native`), the default

**What you get:**
- [dimos-viewer](https://github.com/dimensionalOS/dimos-viewer), a custom Dimensional fork of Rerun with built-in keyboard teleop and click-to-navigate
- Native desktop application (opens automatically)
- Better performance with larger maps/higher resolution
- No browser or web server required

---

### Rerun Web (`rerun`, `--rerunbridgemodule.rerun-open web`)

**What you get:**
- The Rerun web viewer in your browser at http://localhost:9878
- Works headless (no display required)

The integrated dashboard at http://localhost:7779 (the Rerun viewer plus the command center sidebar with teleop and goal setting) is the `WebsocketVisModule`, which a blueprint adds with `vis_module(..., websocket_vis=True)`. It opens automatically and embeds Rerun when the bridge's `rerun_open` is `web` or `both`. Override these choices independently with `--websocketvismodule.open-browser false` or `--websocketvismodule.show-rerun-dashboard false`. Without a Rerun bridge, both default to off.

---

## Rendering with Custom Blueprints

To enable visualization in your own blueprint, use `vis_module`:

```python skip
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.visualization.vis_module import vis_module

camera_demo = autoconnect(
    WebcamModule.blueprint(),
    vis_module(viewer_backend=global_config.viewer),
)

```

Run the stack locally (this blocks until you stop the process):

```python skip
from dimos.core.coordination.module_coordinator import ModuleCoordinator

if __name__ == "__main__":
    ModuleCoordinator.build(camera_demo).loop()
```

Every LCM stream, such as `color_image` (output by WebcamModule), that uses a data type (like `Image`) that has a `.to_rerun` method will get rendered (`rr.log`) using the LCM topic as the rerun entity path. In other words: to render something, simply log it to a stream and it will automatically be available in rerun.

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

# Basic init, no gRPC server (use when RerunBridgeModule is already running)
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

When a `RerunBridgeModule` is already part of your blueprint, you typically don't need `start_grpc`. Just call `rerun_init()` and log directly with `rr.log()`. The data will appear in the existing viewer.

## How to use Rerun on `dev` (and the TF/entity nuances)

Rerun on `dev` is **module-driven**: modules decide what to log, and `Blueprint.build()` sets up the shared viewer + default layout.

To use a custom websocket port, set `--rerunwebsocketserver.port`; the bridge in the same blueprint takes the viewer URL from that server. `--rerunbridgemodule.websocket-url` points a bridge at a websocket server elsewhere. Set `host` on each module to override its bind address; both default to the shared `listen_host`.
