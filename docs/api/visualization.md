# Viewer Backends

Dimos supports three visualization backends: Rerun (web or native) and Foxglove.

## Quick Start

Choose your viewer backend via the CLI (preferred):

```bash
# Rerun native viewer (default) - native Rerun window + teleop panel at http://localhost:7779
dimos run unitree-go2

# Explicitly select the viewer backend:
dimos --viewer-backend rerun-native run unitree-go2
dimos --viewer-backend rerun-web run unitree-go2
dimos --viewer-backend foxglove run unitree-go2
```

Alternative (environment variable):

```bash
VIEWER_BACKEND=rerun-native dimos run unitree-go2

# Rerun web viewer - Full dashboard in browser
VIEWER_BACKEND=rerun-web dimos run unitree-go2

# Foxglove - Use Foxglove Studio instead of Rerun
VIEWER_BACKEND=foxglove dimos run unitree-go2
```

## Viewer Modes Explained

### Rerun Web (`rerun-web`)

**What you get:**
- Full dashboard at http://localhost:7779
- Rerun 3D viewer + command center sidebar in one page
- Works in browser, no display required (headless-friendly)

---

### Rerun Native (`rerun-native`)

**What you get:**
- Native Rerun application (separate window opens automatically)
- Command center at http://localhost:7779
- Better performance with larger maps/higher resolution

---

### Foxglove (`foxglove`)

**What you get:**
- Foxglove bridge on ws://localhost:8765
- No Rerun (saves resources)
- Better performance with larger maps/higher resolution
- Open layout: `assets/foxglove_dashboards/old/foxglove_unitree_lcm_dashboard.json`

---

## Performance Tuning

### Symptom: Slow Map Updates

If you notice:
- Robot appears to "walk across empty space"
- Costmap updates lag behind the robot
- Visualization stutters or freezes

This happens on lower-end hardware (NUC, older laptops) with large maps.

### Increase Voxel Size

Edit [`dimos/robot/unitree_webrtc/unitree_go2_blueprints.py`](/dimos/robot/unitree_webrtc/unitree_go2_blueprints.py) line 82:

```python
# Before (high detail, slower on large maps)
voxel_mapper(voxel_size=0.05),  # 5cm voxels

# After (lower detail, 8x faster)
voxel_mapper(voxel_size=0.1),   # 10cm voxels
```

**Trade-off:**
- Larger voxels = fewer voxels = faster updates
- But slightly less detail in the map

---

## How to use Rerun on `dev` (and the TF/entity nuances)

Rerun on `dev` is **module-driven**: modules decide what to log, and `ModuleBlueprintSet.build()` sets up the shared viewer + default layout.

### Rerun lifecycle (what happens automatically vs what modules must do)

- **Server/viewer startup happens in the build step**
  - [`dimos/core/blueprints.py`](/dimos/core/blueprints.py#L26) calls `init_rerun_server()` (from [`dimos/dashboard/rerun_init.py`](/dimos/dashboard/rerun_init.py#L18)) when `GlobalConfig.viewer_backend` starts with `rerun` and `rerun_enabled=True`.

- **Worker processes must connect before logging**
  - If a module is going to call `rr.log(...)`, it should call `connect_rerun(global_config=...)` first (see examples in:
    - [`dimos/robot/unitree/connection/go2.py`](/dimos/robot/unitree/connection/go2.py)
    - [`dimos/mapping/costmapper.py`](/dimos/mapping/costmapper.py)
    - [`dimos/mapping/voxels.py`](/dimos/mapping/voxels.py)
    - [`dimos/navigation/replanning_a_star/module.py`](/dimos/navigation/replanning_a_star/module.py)
    ).

### Panels and how to use `rr.log` in DimOS

On `dev`, the default layout is composed from modules’ `rerun_views()` contributions:

- Implement `@classmethod rerun_views()` on your module to add panels (2D/3D/time series).
- The build step aggregates them and sends a composed blueprint (see [`dimos/core/blueprints.py`](/dimos/core/blueprints.py)).

If you `rr.log("some/new/entity", ...)` but don’t see it where you expect:
- it may not be included by any existing view panel, so add a `rerun_views()` panel that points at your entity path.

### TF visualization + scene wiring: the `tf_rerun()` helper

The intended pattern for TF visualization on `dev` is:

- **Publish transforms normally** via `self.tf.publish(...)` (TF is available on every module).
- In blueprints, add `tf_rerun(...)` which composes two modules:
  1. **`TFRerunModule`**: Polls `self.tf.buffers` at a configurable rate and logs the latest transform per edge under `world/tf/{child}` using `Transform.to_rerun()`.
  2. **`RerunSceneWiringModule`**: Logs static scene setup (view coordinates, entity-path attachments, optional URDF, axes, camera pinholes).

The `tf_rerun()` helper accepts scene configuration:

```python
from dimos.dashboard.tf_rerun_module import tf_rerun

# Default: TF polling + minimal scene wiring (world + world/robot attachments)
tf_rerun()

# With URDF and camera:
tf_rerun(
    urdf_path=Path("path/to/robot.urdf"),
    cameras=[
        ("world/robot/camera", "camera_optical", GO2Connection.camera_info_static),
    ],
)

# Multi-camera robot:
tf_rerun(
    cameras=[
        ("world/robot/cameras/front", "cam_front_optical", front_cam_info),
        ("world/robot/cameras/left",  "cam_left_optical",  left_cam_info),
        ("world/robot/cameras/right", "cam_right_optical", right_cam_info),
        ("world/robot/cameras/rear",  "cam_rear_optical",  rear_cam_info),
    ],
)

# Disable scene wiring entirely (TF polling only):
tf_rerun(scene=False)
```

### Entity paths vs TF frames

Rerun has two “spaces” you’re always juggling:

- **Entity paths**: strings like `world/robot/camera/rgb` (how data is organized/browsed).
- **Transform frames**: names like `base_link`, `camera_optical` (how motion is defined).

In DimOS on `dev`:
- TF frames come from the TF system (`Transform.frame_id` / `Transform.child_frame_id`) and are logged via `Transform.to_rerun()` (see [`dimos/msgs/geometry_msgs/Transform.py`](/dimos/msgs/geometry_msgs/Transform.py)).
- Visualization entity paths should be treated as **semantic organization** (`world/**`, `metrics/**`, etc).

**Rule of thumb**:
- Put geometry and sensor data under semantic paths (e.g. `world/robot/**`, `world/nav/**`).
- Drive motion through TF by emitting transforms via `self.tf.publish(...)`.
- If you need a semantic entity to "live in" a particular TF frame (e.g. camera frustum under `camera_optical`), configure it via `tf_rerun(cameras=[...])` which handles the attachment in [`dimos/dashboard/rerun_scene_wiring.py`](/dimos/dashboard/rerun_scene_wiring.py#L20).

### Cameras: pinhole projection vs lens distortion

- The camera frustum/projection comes from `CameraInfo.to_rerun()` → `rr.Pinhole(...)` (see [`dimos/msgs/sensor_msgs/CameraInfo.py`](/dimos/msgs/sensor_msgs/CameraInfo.py)).
- Rerun pinholes model **intrinsics** (focal length / principal point / resolution). Lens distortion coefficients are **not** part of the pinhole archetype. If you need distortion-correct visualization, you must undistort upstream and log the undistorted image.

## Appendix: Where Rerun is used in the codebase

This appendix is an **inventory of every current Rerun touchpoint** in the repository (as of this doc), grouped by role (good for reference).

### Rerun lifecycle (server, viewer, client connections)

- **`GlobalConfig` flags**
  - **File**: [`dimos/core/global_config.py`](/dimos/core/global_config.py)
  - **What**: Defines `rerun_enabled`, `viewer_backend` (`rerun-web`, `rerun-native`, `foxglove`), and `rerun_server_addr`.

- **Rerun process lifecycle**
  - **File**: [`dimos/dashboard/rerun_init.py`](/dimos/dashboard/rerun_init.py)
  - **What**:
    - `init_rerun_server()` starts the gRPC server and optionally the native/web viewer (`rr.spawn`, `rr.serve_grpc`, `rr.serve_web_viewer`).
    - `connect_rerun()` connects a process to the shared recording (`rr.connect_grpc`).
    - `shutdown_rerun()` disconnects (`rr.disconnect`).

- **Dashboard re-exports**
  - **File**: [`dimos/dashboard/__init__.py`](/dimos/dashboard/__init__.py)
  - **What**: Re-exports `connect_rerun`, `init_rerun_server`, `shutdown_rerun`.

### Blueprint/layout composition (Rerun UI)

- **Blueprint composition and server init during build**
  - **File**: [`dimos/core/blueprints.py`](/dimos/core/blueprints.py)
  - **What**:
    - Calls `init_rerun_server()` during `ModuleBlueprintSet.build()` when backend is Rerun.
    - Collects per-module `rerun_views()` panels and composes a default `rrb.Blueprint(...)`.
    - Sends the blueprint via `rr.send_blueprint(...)`.

### TF visualization

- **TF visualization module (polling snapshot)**
  - **File**: [`dimos/dashboard/tf_rerun_module.py`](/dimos/dashboard/tf_rerun_module.py)
  - **What**: Polls `self.tf.buffers` at a configurable rate (`poll_hz`) and logs the latest transform per TF edge to `world/tf/{child}` using `Transform.to_rerun()`.

- **Scene wiring module (static setup)**
  - **File**: [`dimos/dashboard/rerun_scene_wiring.py`](/dimos/dashboard/rerun_scene_wiring.py)
  - **What**: Logs all static Rerun scene setup once at startup:
    - View coordinates (`rr.ViewCoordinates.RIGHT_HAND_Z_UP`)
    - Entity-path attachments under named TF frames (`world` → `world`, `world/robot` → `base_link`)
    - Optional URDF load under `world/robot`
    - Optional axes gizmo at `world/robot/axes`
    - Camera entity-path attachments + pinholes (configurable via tuple list)
  - **Why**: Keeps robot modules focused on I/O + TF publishing; centralizes visualization scene wiring.

- **`tf_rerun()` helper (composed blueprint)**
  - **File**: [`dimos/dashboard/tf_rerun_module.py`](/dimos/dashboard/tf_rerun_module.py)
  - **What**: Returns a `ModuleBlueprintSet` that composes `TFRerunModule` + `RerunSceneWiringModule` via `autoconnect(...)`. Blueprints add one line (`tf_rerun(...)`) to get both TF polling and scene wiring.

- **TF message → Rerun entity mapping**
  - **File**: [`dimos/msgs/tf2_msgs/TFMessage.py`](/dimos/msgs/tf2_msgs/TFMessage.py)
  - **What**: `TFMessage.to_rerun()` returns `(entity_path, rr.Transform3D)` pairs for each transform, currently under `world/tf/{child_frame_id}`.

- **Transform → Rerun transform archetype**
  - **File**: [`dimos/msgs/geometry_msgs/Transform.py`](/dimos/msgs/geometry_msgs/Transform.py)
  - **What**: `Transform.to_rerun()` produces `rr.Transform3D(parent_frame=..., child_frame=...)`.

### Robot/device visualization (GO2)

- **GO2 connection: sensor data logging + TF publishing**
  - **File**: [`dimos/robot/unitree/connection/go2.py`](/dimos/robot/unitree/connection/go2.py)
  - **What**:
    - Connects to Rerun via `connect_rerun()` (only to log sensor data).
    - Publishes TF transforms via `self.tf.publish(...)` (base_link, camera_link, camera_optical).
    - Logs camera images to `world/robot/camera/rgb`.
    - Contributes a camera panel via `rerun_views()` (`rrb.Spatial2DView(origin="world/robot/camera/rgb")`).
  - **Note**: Static scene setup (view coordinates, URDF, axes, camera entity attachments, pinholes) is handled by `RerunSceneWiringModule` via `tf_rerun(...)` in the blueprint. GO2 no longer does this directly.

### Mapping/navigation visualization (modules)

- **Costmap visualization + metrics**
  - **File**: [`dimos/mapping/costmapper.py`](/dimos/mapping/costmapper.py)
  - **What**:
    - Logs 2D costmap image at `world/nav/costmap/image` (`OccupancyGrid.to_rerun(mode="image")`).
    - Logs 3D floor overlay at `world/nav/costmap/floor` (`mode="mesh"`).
    - Logs time series metrics under `metrics/costmap/*` via `rr.Scalars`.
    - Contributes a 2D panel (`rrb.Spatial2DView(origin="world/nav/costmap/image")`) and metrics panel via `rerun_views()`.

- **Voxel map visualization + metrics**
  - **File**: [`dimos/mapping/voxels.py`](/dimos/mapping/voxels.py)
  - **What**:
    - Logs voxel map at `world/map` via `PointCloud2.to_rerun(mode="boxes", ...)`.
    - Logs time series metrics under `metrics/voxel_map/*` via `rr.Scalars`.
    - Contributes metrics panels via `rerun_views()`.

- **Planner debugging path logging**
  - **File**: [`dimos/navigation/replanning_a_star/module.py`](/dimos/navigation/replanning_a_star/module.py)
  - **What**: Logs navigation path at `world/nav/path` using `Path.to_rerun()` when Rerun backend is active.

### Metrics helpers

- **Timing decorator (logs to Rerun scalars)**
  - **File**: [`dimos/utils/metrics.py`](/dimos/utils/metrics.py)
  - **What**: `log_timing_to_rerun(entity_path)` wraps a function and logs its duration to `rr.Scalars(...)` at the given path.

### Message-level `to_rerun()` implementations (conversion layer)

These pull `rerun` into the message layer by returning Rerun archetypes.

- **Camera intrinsics → pinhole**
  - **File**: [`dimos/msgs/sensor_msgs/CameraInfo.py`](/dimos/msgs/sensor_msgs/CameraInfo.py)
  - **What**: `CameraInfo.to_rerun()` returns `rr.Pinhole(...)` for frustum/projection (intrinsics only).

- **PointCloud2 → points/boxes**
  - **File**: [`dimos/msgs/sensor_msgs/PointCloud2.py`](/dimos/msgs/sensor_msgs/PointCloud2.py)
  - **What**: `PointCloud2.to_rerun()` returns `rr.Points3D(...)` or `rr.Boxes3D(...)` depending on mode.

- **Image/DepthImage formatting**
  - **File**: [`dimos/msgs/sensor_msgs/image_impls/AbstractImage.py`](/dimos/msgs/sensor_msgs/image_impls/AbstractImage.py)
  - **What**: Helpers that construct `rr.Image(...)` / `rr.DepthImage(...)` with appropriate color model.

- **OccupancyGrid → image/mesh/points**
  - **File**: [`dimos/msgs/nav_msgs/OccupancyGrid.py`](/dimos/msgs/nav_msgs/OccupancyGrid.py)
  - **What**: `OccupancyGrid.to_rerun(mode="image"|"mesh"|"points")` returns `rr.Image`, `rr.Mesh3D`, or `rr.Points3D`.

- **Path → line strips**
  - **File**: [`dimos/msgs/nav_msgs/Path.py`](/dimos/msgs/nav_msgs/Path.py)
  - **What**: `Path.to_rerun()` returns `rr.LineStrips3D(...)`.

- **PoseStamped → transform (and arrows)**
  - **File**: [`dimos/msgs/geometry_msgs/PoseStamped.py`](/dimos/msgs/geometry_msgs/PoseStamped.py)
  - **What**: `PoseStamped.to_rerun()` returns `rr.Transform3D(...)` (and includes arrow helpers using `rr.Arrows3D`).

### Web UI embedding (split-screen dashboard)

- **Split-screen dashboard HTML**
  - **File**: [`dimos/web/templates/rerun_dashboard.html`](/dimos/web/templates/rerun_dashboard.html)
  - **What**: Embeds:
    - Rerun web viewer iframe (`http://localhost:9090/?url=...9876/proxy`)
    - command center iframe (`http://localhost:7779/command-center`)

- **Websocket visualization server**
  - **File**: [`dimos/web/websocket_vis/websocket_vis_module.py`](/dimos/web/websocket_vis/websocket_vis_module.py)
  - **What**: Serves either the split-screen dashboard or command-center-only depending on `viewer_backend`.

- **Command center client**
  - **File**: [`dimos/web/command-center-extension/src/Connection.ts`](/dimos/web/command-center-extension/src/Connection.ts)
  - **What**: Connects to the websocket server on port `7779` (not Rerun SDK, but part of the Rerun-web dashboard experience).

### Related documentation

- **TF and transforms concepts**
  - **File**: [`docs/api/transforms.md`](/docs/api/transforms.md)
  - **What**: Explains frames/transforms and how `self.tf` is intended to be used.
