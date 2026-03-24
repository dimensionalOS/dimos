# MuJoCo Camera Module Plan

## Goal

Create a `MujocoCamera` module that can drop in for `RealSenseCamera` in manipulation
blueprints, enabling sim-based pick-and-place without real hardware.

## Current Wiring (real hardware)

```
PickAndPlaceModule
    ↑ joint_state (LCM)
RealSenseCamera(base_frame_id="link7", base_transform=...)
    ↓ color_image, depth_image, camera_info
ObjectSceneRegistrationModule(target_frame="world")
```

`ObjectSceneRegistrationModule` consumes exactly three inputs:
- `color_image: In[Image]` — RGB
- `depth_image: In[Image]` — DEPTH16 or float32
- `camera_info: In[CameraInfo]` — intrinsics (K matrix)

## Threading Constraint

MuJoCo's C API is **not thread-safe**. `mujoco.Renderer` must be created and used in
the **same thread** as `mj_step()`. The existing nav camera (mujoco_process.py) solves
this by rendering inside the sim loop. We must follow the same pattern.

`MujocoEngine._sim_loop()` currently only does: apply_control → mj_step → sync_viewer → update_state.
Rendering must be added **inside this loop**.

---

## Basic Plan

### 1. Extend `MujocoEngine` with camera rendering

**File:** `dimos/simulation/engines/mujoco_engine.py`

Add rendering capability to the sim loop:

- New init params: `cameras: list[CameraConfig]` (name, width, height, fps)
- Create `mujoco.Renderer` instances in `_sim_loop()` (same thread as `mj_step`)
- Rate-limit rendering per camera (e.g., 15 FPS vs 500 Hz physics)
- After each render, write RGB + depth arrays into a **locked buffer**
- Expose `read_camera(camera_name) -> (rgb, depth, cam_pos, cam_mat, fovy)` that
  returns the latest buffered frame (thread-safe copy)

```
_sim_loop:
    create renderers (same thread)
    while running:
        apply_control()
        mj_step()
        for each camera (if due):
            renderer.update_scene(data, camera=cam_id)
            rgb = rgb_renderer.render()
            depth = depth_renderer.render()
            cam_pos = data.cam_xpos[cam_id]
            cam_mat = data.cam_xmat[cam_id]
            write to locked buffer
        update_joint_state()
```

### 2. Create `MujocoCamera` module

**File:** `dimos/simulation/mujoco/camera.py`

A `Module` implementing `DepthCameraHardware` + `perception.DepthCamera`:

```python
class MujocoCameraConfig(ModuleConfig, DepthCameraConfig):
    camera_name: str = "wrist_camera"     # MuJoCo camera name in MJCF
    width: int = 640
    height: int = 480
    fps: int = 15
    base_frame_id: str = "link7"
    base_transform: Transform | None = None
    enable_depth: bool = True
    enable_pointcloud: bool = False
    pointcloud_fps: float = 5.0
    camera_info_fps: float = 1.0
    depth_scale: float = 1.0              # MuJoCo depth is in meters

class MujocoCamera(DepthCameraHardware, Module[MujocoCameraConfig], perception.DepthCamera):
    color_image: Out[Image]
    depth_image: Out[Image]
    pointcloud: Out[PointCloud2]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
```

**Lifecycle:**
- `set_engine(engine: MujocoEngine)` — called before `start()`, stores engine ref
- `start()` — compute intrinsics from engine's `model.cam_fovy[cam_id]`, start
  polling thread that calls `engine.read_camera()` at configured FPS
- Polling thread: read buffered frame → wrap as `Image` → publish on output ports
- Publish TF: `base_frame_id → camera_link → optical frames` (same chain as RealSense)

### 3. Refactor `SimMujocoAdapter` to accept external engine

**File:** `dimos/hardware/manipulators/sim/adapter.py`

Currently creates `MujocoEngine` internally. Add option to accept an existing one:

```python
class SimMujocoAdapter(SimManipInterface):
    def __init__(self, dof=7, address=None, headless=True, engine=None, **_):
        if engine is None:
            engine = MujocoEngine(config_path=Path(address), headless=headless)
        # ... rest unchanged
```

### 4. Blueprint wiring

**File:** `dimos/manipulation/blueprints.py`

Create a new `xarm_perception_sim` blueprint:

```python
# Shared engine created once
_engine = MujocoEngine(config_path=Path("path/to/xarm_scene.xml"), headless=False)

xarm_perception_sim = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[_make_xarm7_config(...)],
        ...
    ),
    MujocoCamera.blueprint(
        camera_name="wrist_camera",
        base_frame_id="link7",
        base_transform=_XARM_PERCEPTION_CAMERA_TRANSFORM,
        _engine=_engine,   # injected reference
    ),
    ObjectSceneRegistrationModule.blueprint(target_frame="world"),
    ControlCoordinator.blueprint(
        hardware=[HardwareComponent(adapter_type="sim_mujoco", ...)],
        ...
    ),
)
```

### 5. MJCF scene file

Create `data/sim_assets/xarm7/scene.xml` with:
- xArm7 robot model (arm + gripper)
- `wrist_camera` camera attached to link7 (matching the real RealSense mount)
- Table + objects for pick-and-place
- Lighting

---

## Improvements & Simplifications

### A. Depth format: use meters directly (skip DEPTH16 conversion)

RealSense outputs DEPTH16 (uint16, mm). ObjectSceneRegistrationModule converts it:
```python
depth_meters = depth_data.astype(np.float32) / 1000.0
```

MuJoCo depth is already in meters (float32). We can publish as `ImageFormat.DEPTH32F`
directly. ObjectSceneRegistrationModule already handles both formats (line 281-289):
```python
if depth_image.format == ImageFormat.DEPTH16:
    depth_data = depth_data.astype(np.float32) / 1000.0
```

**Simplification:** No artificial uint16 conversion needed. Just publish float32 depth.

### B. Skip extrinsics — color and depth are co-located

RealSense has physically separate color/depth sensors with extrinsics between them.
MuJoCo renders both from the same virtual camera. So:

- Color and depth share the same intrinsics
- No color-to-depth alignment needed (`align_depth_to_color` is a no-op)
- TF chain simplifies: `camera_link = color_frame = depth_frame` (single camera origin)
- `depth_camera_info` = `color_camera_info` (identical)

### C. Intrinsics from model — no calibration

RealSense reads intrinsics from hardware via `pyrealsense2`. MuJoCo camera params
are known exactly from the model:

```python
fovy = model.cam_fovy[cam_id]  # vertical FOV in degrees
fy = height / (2 * tan(radians(fovy) / 2))
fx = fy  # square pixels
cx, cy = width / 2, height / 2
D = [0, 0, 0, 0, 0]  # no distortion
```

Compute once at `start()`, publish periodically. No runtime calibration.

### D. Consider rendering in `mj_forward()` instead of `mj_step()`

For camera-only updates (no physics needed), `mj_forward()` computes kinematics
without advancing the simulation. This could be useful if we want to render from
a snapshot without waiting for the next physics tick. However, for the shared-engine
approach this isn't needed — rendering is already in the sim loop.

### E. Pointcloud generation — reuse existing utility

`dimos/simulation/mujoco/depth_camera.py` already has `depth_image_to_point_cloud()`.
But for ObjectSceneRegistrationModule, we don't need to generate pointclouds in the
camera module at all — the perception module generates its own per-object pointclouds
from depth + camera_info. Only enable `pointcloud` output if needed for visualization.

### F. Engine injection pattern

Rather than passing engine through Pydantic config (not serializable), use a
post-construction setter:

```python
camera = dimos.deploy(MujocoCamera, camera_name="wrist_camera", ...)
camera.set_engine(engine)  # inject before start()
```

Or use a lightweight engine registry keyed by MJCF path:

```python
# In engine
_engine_registry: dict[str, MujocoEngine] = {}

# Camera config just needs the MJCF path
class MujocoCameraConfig(ModuleConfig):
    address: str  # same MJCF path as adapter — looks up shared engine
```

This keeps blueprints clean and avoids non-serializable config fields.

---

## Summary

| Step | File | What |
|------|------|------|
| 1 | `simulation/engines/mujoco_engine.py` | Add camera rendering to sim loop |
| 2 | `simulation/mujoco/camera.py` | New module (same interface as RealSense) |
| 3 | `hardware/manipulators/sim/adapter.py` | Accept external engine |
| 4 | `manipulation/blueprints.py` | New `xarm_perception_sim` blueprint |
| 5 | `data/sim_assets/xarm7/scene.xml` | MuJoCo scene with camera + objects |

Simplifications vs RealSense:
- No extrinsics / depth alignment
- No distortion
- Float32 depth directly (no DEPTH16 conversion)
- Intrinsics computed from model, not hardware
- Pointcloud generation optional (perception handles it)
