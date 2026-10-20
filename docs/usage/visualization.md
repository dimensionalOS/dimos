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

Control how the Rerun viewer opens with `--rerun-open` and `--rerun-web`:

```bash
# Open native desktop viewer (default)
dimos --rerun-open native run unitree-go2

# Open web viewer in browser
dimos --rerun-open web run unitree-go2

# Open both native and web
dimos --rerun-open both run unitree-go2

# No viewer (headless): data still accessible via gRPC
dimos --rerun-open none run unitree-go2

# Serve the web viewer without auto-opening a browser
dimos --rerun-web --rerun-open native run unitree-go2
```

## Viewer Modes Explained

### Rerun Native (`rerun`, `--rerun-open native`), the default

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

## Generated CDR geometry

Generated message classes are plain values. The Rerun bridge supplies the
PoseStamped/PointStamped adapters, including pose axes and a text view of every
field. An explicit CDR subscription is independent of legacy pubsub discovery:

```python
from dimos.core.cdr_transport import CdrTransport
from dimos.messages.geometry_msgs.msg import PoseStamped
from dimos.visualization.rerun.bridge import RerunBridgeModule

viewer = RerunBridgeModule(pubsubs=[], cdr_transports={
    "measured_pose": CdrTransport("measured_pose", PoseStamped, backend="zenoh"),
})
viewer.start()
# Stop the bridge when the application shuts down:
viewer.stop()
```

The producer must use the matching explicit CDR channel and schema. Each
transport instance in `cdr_transports` belongs to the bridge; use a separate
instance for the producer. Header stamps use a source timeline with exact
nanoseconds, including zero. Header frames remain named `tf#/<frame_id>` frames;
they are not silently reinterpreted as world coordinates. The existing TF tree
must locate that frame for a shared spatial view. Other generated message
domains still need their own adapters; this does not change transport defaults.

To see an actual offline exchange, from this checkout with its normal installed
DimOS environment:

```sh
python examples/message-viewer/demo_cdr_rerun.py pose.rrd
rerun pose.rrd
```

The example sends generated PoseStamped bytes over discovery-disabled loopback
Zenoh/CDR, then writes a real Rerun recording. Its layout shows the pose axes and
complete message values side by side. For this demonstration only, it explicitly
places the message's map frame at the viewer origin. It refuses an existing
output, needs no robot, and does not start the module coordinator or change host
configuration. CI runs this exact example and reads the RRD back with the
pinned Rerun CLI to check geometry, axes, fields and nanosecond timestamps.
