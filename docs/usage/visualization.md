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

To use a custom websocket port, set `--rerunwebsocketserver.port`; the bridge in the same blueprint takes the viewer URL from that server. `--rerunbridgemodule.websocket-url` points a bridge at a websocket server elsewhere. Set `host` on each module to override its bind address; both default to the shared `listen_host`.

## Generated CDR viewer demo (WIP)

The CDR viewer is under development. This example runs the real Rerun bridge
with local Zenoh transports. It needs no robot or ROS installation.

From the repository root:

```sh skip
DIMOS_ALLOW_MISSING_COCKPIT=1 uv sync --locked --extra visualization --extra web
uv run --extra visualization --extra web python -I examples/message-viewer/demo_cdr_rerun.py cdr-viewer.rrd
uv run --extra visualization rerun cdr-viewer.rrd
uv run --extra visualization rerun cdr-viewer.mcap
```

Use a new output filename for each run. The example creates the live RRD and
`cdr-viewer.mcap`. Both destinations must be new. The build flag permits
installation without prebuilt browser assets; message generation still runs.

The live recording uses `RerunBridgeModule.start()`, its typed subscriptions and
rendering callbacks. A receipt signal waits for each message before shutdown.
The MCAP contains the source scene encoded as CDR with complete ROS2 schemas.
It is example input for playback, not a capture by the native recorder.

Open the MCAP directly with Rerun's native importer. To save its imported data:

```sh skip
uv run --extra visualization rerun mcap convert cdr-viewer.mcap -o cdr-viewer-native.rrd
```

Rerun 0.32 converts supported standard ROS2 types into visualizations. Other
messages, including this scene's Path and custom EntityMarkers, are available
as reflected fields; they do not automatically acquire the live bridge's path
or labeled-entity display. No dimOS Python renderer is used for this import.
Existing `dimos mem rerun` behavior for legacy stores, including `--seconds`
and `--root`, is unchanged. Use the native Rerun command for CDR MCAP files.

| PR capability | Demo scene | What to check |
| --- | --- | --- |
| TF and source metadata | `tf`, `pose` | Map/camera frames, pose axes at `[1.5, -2.5, 0.25]`, `nanosec: 123456789` in the text tab |
| Geometry adapters | `point`, `odometry`, `path`, `entities` | Point, three-waypoint path, odometry axes, labeled person and object |
| Camera calibration | `image`, `late_image` | Pinhole on each image, whether calibration arrives first or last |
| Sensor buffers | `image`, `depth`, `compressed`, `compressed_depth` | Four RGB pixels, big-endian uint16 depth, PNG image and JXL depth in the sensor tab |
| Cloud rendering | `cloud`, `cloud_boxes`, `cloud_height` | RGB points, boxes and height-colored spheres |
| Legacy rendering | `legacy_pose` | Existing pose rendering alongside generated messages |
| Native MCAP import | `cdr-viewer.mcap` | Standard images, points, calibration and TF; reflected fields for types without a native visualization |

The example anchors its `world` TF frame to the viewer root. Business producers
must supply their own TF tree. It does not demonstrate robot graph migration or
automatic CDR type discovery. The existing integration test runs this example,
checks the live RRD, then converts the MCAP with the official CLI and checks its imported components.

For the live browser SDK and manifest path, run the
[Web bridge demo](/docs/web/bridge.md#generated-cdr-messages-wip).
