# Depth Camera Integration Guide

This folder contains camera drivers and modules for RGB-D (depth) cameras such as RealSense and ZED.
Use this guide to add a new depth camera, wire TF correctly, and publish the required streams.

## Add a New Depth Camera

1) **Create a new driver module**
   - Path: `dimos/hardware/sensors/camera/<vendor>/camera.py`
   - Export a blueprint in `<vendor>/__init__.py` (match the `realsense` / `zed` pattern).

2) **Define config**
   - Inherit from `ModuleConfig` and `DepthCameraConfig`:
     ```python
     @dataclass
     class MyDepthCameraConfig(ModuleConfig, DepthCameraConfig):
         width: int = 1280
         height: int = 720
         fps: int = 15
         camera_name: str = "camera"
         base_frame_id: str = "base_link"
         base_transform: Transform | None = field(default_factory=default_base_transform)
         align_depth_to_color: bool = True
         enable_depth: bool = True
         enable_pointcloud: bool = False
         pointcloud_fps: float = 5.0
         camera_info_fps: float = 1.0
     ```

3) **Implement the module**
   - Inherit from `DepthCameraHardware` and `Module` (see `RealSenseCamera` / `ZEDCamera`).
   - Provide these outputs (matching `RealSenseCamera` / `ZEDCamera`):
     - `color_image: Out[Image]`
     - `depth_image: Out[Image]`
     - `pointcloud: Out[PointCloud2]` (optional, can be disabled by config)
     - `camera_info: Out[CameraInfo]`
     - `depth_camera_info: Out[CameraInfo]`
   - Implement RPCs:
     - `start()` / `stop()`
     - `get_color_camera_info()` / `get_depth_camera_info()`
     - `get_depth_scale()` (meters per depth unit)

4) **Publish frames**
   - Color images: `Image(format=ImageFormat.RGB, frame_id=_color_optical_frame)`
   - Depth images:
     - If `align_depth_to_color`: use `_color_optical_frame`
     - Else: use `_depth_optical_frame`
   - CameraInfo frame_id must match the image frame_id you publish.

5) **Publish camera info**
   - Build `CameraInfo` from camera intrinsics.
   - Publish at `camera_info_fps`.

6) **Publish pointcloud (optional)**
   - Use `PointCloud2.from_rgbd(color_image, depth_image, camera_info, depth_scale)`.
   - Publish at `pointcloud_fps`.

## TF: Required Frames and Transforms

Frame names are defined by the abstract depth camera spec (`dimos/hardware/sensors/camera/spec.py`).
Use the properties below to ensure consistent naming:

- `_camera_link`: base link for the camera module (usually `{camera_name}_link`)
- `_color_frame`: non-optical color frame
- `_color_optical_frame`: optical color frame
- `_depth_frame`: non-optical depth frame
- `_depth_optical_frame`: optical depth frame

Recommended transform chain (publish every frame or at your preferred TF rate):

1) **Mounting transform** (from config):
   - `base_frame_id -> _camera_link`
   - Use `config.base_transform` if provided

2) **Depth frame**
   - `_camera_link -> _depth_frame` (identity unless the camera provides extrinsics)
   - `_depth_frame -> _depth_optical_frame` using `OPTICAL_ROTATION`

3) **Color frame**
   - `_camera_link -> _color_frame` (from extrinsics, or identity if unavailable)
   - `_color_frame -> _color_optical_frame` using `OPTICAL_ROTATION`

Notes:
- If you align depth to color, keep TFs the same but publish depth images in `_color_optical_frame`.
- Ensure `color_image.frame_id` and `camera_info.frame_id` match. Same for depth.

## Required Streams / Topics

Use these stream names in your module and attach transports as needed.
Default LCM topics in `realsense` / `zed` demos are shown below.

| Stream name        | Type         | Suggested topic         | Frame ID source |
|-------------------|--------------|-------------------------|-----------------|
| `color_image`     | `Image`      | `/camera/color`         | `_color_optical_frame` |
| `depth_image`     | `Image`      | `/camera/depth`         | `_depth_optical_frame` or `_color_optical_frame` |
| `pointcloud`      | `PointCloud2`| `/camera/pointcloud`    | (derived from CameraInfo) |
| `camera_info`     | `CameraInfo` | `/camera/color_info`    | matches `color_image` |
| `depth_camera_info` | `CameraInfo` | `/camera/depth_info`  | matches `depth_image` |

For `ObjectSceneRegistrationModule`, the required inputs are:
- `color_image`
- `depth_image`
- `camera_info`
- TF tree resolving `target_frame` to `color_image.frame_id`

## Object Scene Registration (Brief Overview)

`ObjectSceneRegistrationModule` consumes synchronized RGB + depth + camera intrinsics and produces:
- 2D detections (YOLO‑E)
- 3D detections (projected via depth + intrinsics + TF)
- Overlay annotations and aggregated pointclouds

See:
- `dimos/perception/object_scene_registration.py`
- `dimos/perception/demo_object_scene_registration.py`

Quick wiring example:

```python
from dimos.core.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense import realsense_camera
from dimos.perception.object_scene_registration import object_scene_registration_module

pipeline = autoconnect(
    realsense_camera(enable_pointcloud=False),
    object_scene_registration_module(target_frame="world"),
)
```

Run the demo via CLI:
```bash
dimos run demo-object-scene-registration
```

## Foxglove (Viewer)

Install Foxglove from:
- https://foxglove.dev/download

## Modules and Skills (Short Intro)

- **Modules** are typed components with `In[...]` / `Out[...]` streams and `start()` / `stop()` lifecycles.
- **Skills** are callable methods (decorated with `@skill`) on any `Module`, automatically discovered by agents.

Reference:
- Modules overview: `/docs/usage/modules.md`
- TF fundamentals: `/docs/usage/transforms.md`
