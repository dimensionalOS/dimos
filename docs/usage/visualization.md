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

## Topic Monitor

Use `dimos topic monitor` when you want an interactive web console for choosing which live LCM topics are logged to a Rerun view:

```bash
uv run dimos topic monitor
uv run dimos topic monitor --no-open
uv run dimos topic monitor --run latest
```

The monitor is an independent foreground sidecar. It observes the visible LCM bus, owns its own Rerun bridge/viewer and Reflex selector UI, and exits when you press Ctrl-C. If a DimOS run is active, the command prints that run as context; if no run is active, it starts in LCM bus-only mode and can still catalog traffic from manual or external publishers.

The standard `vis_module(...)` path is unchanged: renderable LCM topics are logged automatically by blueprints that include normal visualization. `dimos topic monitor` does not control, disable, or reuse any existing visualization in the running blueprint. Its staged/applied selection affects only the monitor-owned Rerun viewer.

By default the command allocates an isolated set of local ports instead of reusing the fixed Rerun defaults, then prints the actual URLs. It opens the selector page automatically when possible; if browser opening fails, open the printed selector URL manually. Use `--no-open` to skip browser launch.

For a hardware-free demo on a laptop or desktop, run the built-in synthetic publisher:

```bash
uv run dimos run demo-rerun-topic-selector --daemon
uv run dimos topic monitor
```

This demo remains a self-contained hardware-free smoke test for the selector UI. It publishes synthetic typed LCM topics only; `dimos topic monitor` attaches as the independent visualization sidecar. For ordinary robot, simulation, or replay workflows, run the normal stack and then start `dimos topic monitor`.

The monitor provides a Reflex visual console next to the Rerun web viewer. It starts local services for the selector frontend, selector API, Reflex backend websocket/API, Rerun gRPC source, and Rerun web viewer. The exact ports are printed by the CLI because they are allocated per monitor instance.

The selector API is local to the monitor process and forwards UI actions to the monitor-owned selected-only bridge. The Reflex frontend runs as a local `reflex run` subprocess using explicit frontend/backend ports, polls that API, and renders the "DimOS Visual Console" layout:

- a header bar with LCM traffic and Rerun connectivity status chips
- a fixed-width catalog rail with search, `renderable`/`live`/`heavy`/`selected` filter chips, and a `visible/total` counter
- a grouped topic table (Perception, Robot state, Navigation, Control, Text / logs, Other) with per-row render badges (`renderable`, `converter`, `unsupported`), rate, bandwidth (heavy topics highlighted in amber), and live/idle status; unsupported decoded topics are visible but disabled
- current-session staging controls; checking a topic only stages it, and applied topics carry a `LOGGING` badge
- a bottom selection tray showing staged/logging counts, the staged bandwidth estimate, a heavy-topic warning, and explicit **Clear** / **Apply selection** actions
- an embedded Rerun viewer panel with a connection toolbar (**Reconnect**, **Open in tab**), an unreachable-viewer error card, and a bridge footer listing the logged topics as entity chips

Reflex builds a small React/Next frontend at runtime and may install or use Node/Bun/npm-managed web assets. Install it through the visualization extra:

```bash
uv sync --extra visualization
```

### Selected-only logging

In selector mode, decoded DimOS topics are cataloged first. A renderable topic is not converted to Rerun data or logged merely because it is visible or staged. Subsequent messages are logged only after the staged selection is applied. Clearing the staged selection and applying that empty selection stops selector-managed logging for those topics.

This is useful for high-bandwidth streams such as images, maps, and point clouds: browsing the catalog does not automatically pay the Rerun conversion/logging cost. The monitor still subscribes through DimOS' normal decoded LCM path, so non-decodable/non-DimOS LCM traffic is ignored rather than displayed.

### Unsupported and degraded states

The selector catalog is LCM-only in this first version. It discovers live decodable DimOS LCM channels, such as `/camera/color#sensor_msgs.Image`. Untyped, undecodable, or non-DimOS LCM traffic is ignored.

Topic states include:

- **renderable**: the decoded message has `to_rerun()` support or matches a configured `visual_override` converter
- **unsupported**: the message type is known but has no Rerun converter, or a visual override suppresses it
- **live/idle**: traffic freshness based on the catalog freshness window
- **logging**: the topic is in the applied selector-managed logging set

The page also calls out common degraded states: no decodable LCM data yet (pulsing empty state), no search/filter matches, an unreachable selector API, and an unreachable Rerun viewer (error card with retry). If the embedded viewer is blank, use **Open in tab** or verify that the embedded Rerun URL includes an encoded `url=rerun%2Bhttp...%2Fproxy` query parameter that points at the bridge gRPC proxy.

### v1 scope

The monitor discovers live LCM topics only. SHM, ROS, DDS, coordinator metadata, and stored replay stream catalogs are future work unless those streams are actively bridged to LCM during the run.

To make a topic renderable in monitor mode, prefer a typed LCM channel whose message implements `to_rerun()`. Blueprint-specific `visual_override`, static Rerun scene objects, and custom topic-to-entity mappings are not loaded by `dimos topic monitor` in v1. Avoid using `RerunWebSocketServer` as a catalog API; it remains the viewer-to-robot click/teleop websocket path.

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

## How to use Rerun on `dev` (and the TF/entity nuances)

Rerun on `dev` is **module-driven**: modules decide what to log, and `Blueprint.build()` sets up the shared viewer + default layout.
