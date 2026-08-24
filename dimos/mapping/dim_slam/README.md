# dimSLAM

cuVSLAM visual odometry, an IMU, and any number of odometry sources, fused by an
error-state Kalman filter in one native process. Publishes `odom_frame` ->
`base_frame` on `odometry` and on `tf`, plus a `depth_cloud` when depth is fed in.

The binary is built from [dimSLAM](https://github.com/dimensionalOS/dimSLAM) by
`build_command`; `sdk_variant()` picks the flake attr for the host (metal, orin,
thor, x86_64-cuda12/13).

## Ports

| Port | Message | Notes |
| --- | --- | --- |
| `image` | `sensor_msgs.Image` | every camera publishes here, told apart by `frame_id` |
| `camera_info` | `sensor_msgs.CameraInfo` | intrinsics per `frame_id`; the rig is built from these |
| `depth_image` | `sensor_msgs.Image` | `rgbd` mode and `depth_cloud` |
| `depth_camera_info` | `sensor_msgs.CameraInfo` | needed when the depth sensor is not the rig camera |
| `imu` | `sensor_msgs.Imu` | with `use_imu` or `cuvslam_enable_imu` |
| `imu_info` | `sensor_msgs.ImuInfo` | noise model and frame for `cuvslam_enable_imu` |
| `sources` | `nav_msgs.Odometry` | all external odometry, told apart by `header.frame_id` |
| `odometry` | `nav_msgs.Odometry` | out: fused pose |
| `depth_cloud` | `sensor_msgs.PointCloud2` | out: cloud cut from `depth_image` |
| `tf` | `tf2_msgs.TFMessage` | in for extrinsics, out for `odom_frame` -> `base_frame` |

## Bring-up on a new robot

### 1. Point the cameras at the module

Both imagers of a stereo pair go onto the same `image` stream. With a RealSense:

```python
.remappings([
    (RealSenseCamera, "infrared_left", "image"),
    (RealSenseCamera, "infrared_right", "image"),
    (RealSenseCamera, "infrared_left_camera_info", "camera_info"),
    (RealSenseCamera, "infrared_right_camera_info", "camera_info"),
    # Keep colour off the stream the rig is read from.
    (RealSenseCamera, "camera_info", "color_camera_info"),
])
```

Leave `camera_frames` empty to auto-discover from `camera_info`, or list the frames
explicitly to fix which ones are on the rig and in what order. Turn the IR emitter
off (`emitter_enabled=False`): its dot pattern is a fake texture that moves with the
robot.

`rectified=True` is an assertion, not a correction — unrectified input is tracked as-is
and will drift.

### 2. Publish the extrinsics

Extrinsics come from tf, looked up against `rig_frame` (empty means `base_frame`).
Anything that puts `base_link` -> the camera's frame on tf works; the RealSense module
takes it directly:

```python
D455_MOUNT = Transform(
    translation=Vector3(-0.2518, -0.2736, 0.42),
    rotation=Quaternion(0.078360, 0.006348, -0.996712, 0.019616),
)
RealSenseCamera.blueprint(base_transform=D455_MOUNT)
```

A wrong mount shows up as a map that shears or tilts as the robot turns, not as an
obviously broken pose, so measure it before blaming the tracker.

### 3. Add the odometry sources

One entry in `source_frames` per source, matched against `header.frame_id`. A source
whose frame equals `odom_frame` is fused absolutely; anything else is fused as
filter-anchored deltas. The tracker's own pose is a source under `visual_odom_frame`
and must be listed.

`source_pose_variances` and `source_twist_variances` take 6 numbers per source,
`[x y z roll pitch yaw]` then `[vx vy vz wx wy wz]`: below zero takes the message
covariance, zero drops the dimension, above zero is a fixed variance. Prefer fixed
variances for sources whose reported covariance is accumulated drift rather than the
error of the delta being fused.

`constraint_twist_variances` is a virtual zero-twist measurement for directions the
platform cannot move in. For a planar holonomic base:

```python
constraint_twist_variances=[0.0, 0.0, 0.01, 0.01, 0.01, 0.0]  # z, roll, pitch pinned
```

Raise `replay_buffer_seconds` when a source crosses a network link and can land late.

### 4. Turn on the IMU

`use_imu=True` propagates the filter on IMU and requires all four datasheet noise
figures, in continuous-time units:

| Field | Unit |
| --- | --- |
| `imu_gyro_noise_density` | rad/s/sqrt(Hz) |
| `imu_gyro_random_walk` | rad/s^2/sqrt(Hz) |
| `imu_accel_noise_density` | m/s^2/sqrt(Hz) |
| `imu_accel_random_walk` | m/s^3/sqrt(Hz) |

They are per-part, not per-robot; the D455's BMI055 is `0.0018 / 2e-5 / 0.02 / 3e-3`.
Config validation rejects `use_imu` with any of them at zero.

The first `imu_init_samples` messages are averaged for the gyro bias, so the robot has
to be **still at startup** — 200 samples is one second at 200 Hz.

### 5. Run it

```bash
dimos run <your-blueprint> --viewer rerun --rerun-host 0.0.0.0 --g.rerun-open none
```

`demo-cuvslam-realsense` is the smallest working example (camera + `DimSlam` +
`OdometryPath`); `alfred-mls-nav` is a full robot.

### 6. Verify

- `odometry` publishes at `publish_rate` and the pose moves when the robot moves.
- Walk a closed loop and check `world/path` in rerun comes back to where it started.
- Turn in place: pure rotation should not translate.
- Cover the cameras — the pose should coast on the IMU and other sources, not jump.

## Per-robot settings

Everything else has a sane default. These do not:

| Setting | Why it is per-robot |
| --- | --- |
| `use_gpu` | `False` runs the CPU path, which needs a libcuvslam built `-DENFORCE_GPU=OFF`. The flake builds `orin` and `x86_64-cuda12` that way; the prebuilt `metal` and `thor` SDKs are GPU-only |
| `depth_units_per_meter` | 1000 for the usual sixteen-bit millimetres |
| `depth_cloud_max_range` | stereo error grows as range squared; a property of the sensor |
| `depth_cloud_decimation` | a full-resolution D455 cloud is ~400k points a frame at 30 Hz |
| `covariance_gate_translation_std` | the CPU tracker's reported std does not separate good frames from bad; 0 disables |
| `base_frame` / `odom_frame` | must match the rest of the robot's tf tree |
| `publish_tf` | off when something downstream owns `odom` -> `base_frame` |
