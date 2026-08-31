# 🧭 Camera-Based Navigation Setup

This guide helps you set up a camera-based navigation system.

---

# ⚠️ What Works and What Does Not

Let me be up front about what works and what does not. All camera-based systems are worse than a good lidar. You can build production-quality navigation with cameras alone, but only if you do it carefully.

A regular phone camera facing forward on a humanoid, with no data from the motors, is basically impossible as of 2026. A wheeled robot with motor encoders and a downward-facing RealSense depth camera can do some pretty cool stuff.

Use these rules of thumb:

- We need good odometry first. A map built on drifting poses will also drift.
- We need a depth camera and good odometry to make a half-decent map.
- Calibration is the difference between incredible and useless. Some cameras, including RealSense cameras, arrive pre-calibrated.
- Make sure everything on your robot is stiff. Flexing and wobble between sensors can ruin performance.
- Stereo cameras are significantly better than mono cameras because they observe metric depth.
- Turn the IR blaster off for better odometry. Its projected pattern moves across the scene as the robot moves.
- Global-shutter cameras, such as the RealSense D455, are much better than rolling-shutter cameras, such as the ZED Mini.
- Motor encoder data is a game changer. It significantly improves the motion estimate.
- An IMU plus a camera does not do much, but an IMU plus motor encoders improves accuracy significantly.
- Multiple cameras are a game changer. They significantly improve coverage and robustness.

---

# 🔌 Software Setup

Assume your calibrated camera is mounted rigidly on the robot. The navigation system needs sensor measurements plus enough metadata to interpret them correctly.

`DimSlam` is a native dimos visual odometry module built on cuVSLAM. It tracks camera images and estimates the robot's motion, while extending stock cuVSLAM with inertial and odometry fusion. Give it an IMU and any wheel, leg, or other odometry your robot already produces, and those sensors improve its estimate.

`DimSlam` is configured from Python, then runs as its own Rust process. The module publishes `odometry` and the `odom -> base_link` transform.

Import its configuration types from:

```python skip
from dimos.mapping.dim_slam.dim_slam import (
    CameraConfig,
    Covariance,
    DimSlam,
    DimSlamConfig,
    ImuConfig,
    InitialStds,
    SourceConfig,
)
```

## The complete list of what you must supply

1. **A `camera_mode`.** This is required and has no default because it determines which inputs are required. Choose `"mono"`, `"stereo"`, `"rgbd"`, or `"multisensor"`.

   `"multisensor"` is experimental in cuVSLAM. It supports a mixture of RGB and RGB-D cameras and requires an explicit `cameras` list. Auto-discovery cannot know when a multisensor rig is complete.

2. **Camera and IMU intrinsics.** Camera intrinsics arrive through `camera_info`. RealSense cameras publish their factory calibration. Non-RealSense cameras must be calibrated first. Configure each IMU with noise figures from its datasheet.

3. **Extrinsics through tf.** Supply a static transform from the robot base to every camera optical frame. An IMU mount must also exist as a frame in the robot URDF because dimSLAM reads IMU mounts from tf.

4. **The `cameras` list in the correct order.** For stereo, the first two entries define the rig, with the left camera first. If the list is empty, cuVSLAM auto-discovers cameras from `camera_info` and sorts their frame names. That produces left-then-right ordering only by luck. Declare the list explicitly for a production robot.

   Entries after the rig can configure settings-only streams, such as an RGB-D depth stream.

5. **Timestamps that agree.** Stereo pairs must satisfy cuVSLAM's 1 ms timestamp contract by default. `max_skew_ms` permits the measured spread of a rig that cannot meet it. All sensors also need compatible clocks.

6. **A second of stillness at startup when using an IMU.** dimSLAM uses stationary samples to estimate gyro bias and level itself.

7. **An `odom_sources` entry for every wheel, leg, or other external odometry source.** Its `parent_frame_id` and `child_frame_id` must exactly match the messages.

8. **Trust values.** Configure `visual_odom_pose_variances`, each source's pose and twist variances, and `per_dimension_error_variance`.

## Shared input streams

`DimSlam` accepts these streams:

| Input | Message type |
| --- | --- |
| `image` | `Image` |
| `depth_image` | `Image` |
| `camera_info` | `CameraInfo` |
| `depth_camera_info` | `CameraInfo` |
| `imu` | `Imu` |
| `odom_sources` | `Odometry` |

It publishes:

| Output | Message type |
| --- | --- |
| `odometry` | `Odometry` |
| `depth_cloud` | `PointCloud2` |
| `tf` | `TFMessage`, bidirectional |

Every camera publishes onto the same `image` and `camera_info` streams. dimSLAM distinguishes cameras using `frame_id`.

Every IMU publishes onto the same `imu` stream and is matched by `frame_id`. Every external odometry source publishes onto `odom_sources` and is matched by `header.frame_id` plus `child_frame_id`.

An unlisted frame is dropped. It does not contribute to the estimate.

## Camera only, with no IMU or wheel odometry

The following blueprint uses the stereo infrared pair from a RealSense camera. It also includes `OdometryHist`, which records the estimated path and publishes it as `odom_hist`.

Run it with:

```sh skip
dimos run demo-dim-slam-realsense --viewer rerun --rerun-host 0.0.0.0
```

```python skip
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.dim_slam.dim_slam import DimSlam
from dimos.mapping.odometry_hist import OdometryHist, path_at_true_height
from dimos.visualization.vis_module import vis_module


def dim_slam_rerun_blueprint() -> Any:
    """Rerun names entities after the topic, which both cameras share."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/image", name="cameras"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5)
                ),
            ),
            column_shares=[1, 3],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


demo_dim_slam_realsense = (
    autoconnect(
        RealSenseCamera.blueprint(
            fps=30,
            enable_infrared=True,
            emitter_enabled=False,
            enable_color=False,
            enable_depth=False,
        ),
        DimSlam.blueprint(camera_mode="stereo"),
        OdometryHist.blueprint(),
        vis_module(
            global_config.viewer,
            rerun_config={
                "blueprint": dim_slam_rerun_blueprint,
                "visual_override": {
                    "world/odom_hist": path_at_true_height
                },
            },
        ),
    )
    .remappings(
        [
            (RealSenseCamera, "infrared_left", "image"),
            (RealSenseCamera, "infrared_right", "image"),
            (
                RealSenseCamera,
                "infrared_left_camera_info",
                "camera_info",
            ),
            (
                RealSenseCamera,
                "infrared_right_camera_info",
                "camera_info",
            ),
        ]
    )
    .global_config(n_workers=4)
)
```

Several details matter:

- `emitter_enabled=False` turns off the IR blaster because it hurts odometry.
- Both infrared images are remapped onto the same `image` stream. Their frame IDs distinguish them.
- Grayscale infrared images are sufficient. Tracking does not require color.
- Pure odometry needs neither the color stream nor the depth stream.
- The RealSense publishes calibrated `camera_info`, so no calibration code appears here.
- `OdometryHist` subscribes to `odometry` and publishes a `Path` on `odom_hist`.
- `path_at_true_height` draws the path at its real z instead of lifting it above a costmap.

In Rerun, inspect `world/odom_hist`. It should retrace the route you walked.

`OdometryHist` defaults to a 0.02 m minimum step, 20,000 poses, and a 0.1 second minimum publish interval. Its empty `frame_id` follows the incoming odometry frame.

## Adding an IMU

Build on the first blueprint by remapping the camera's IMU stream to `imu` and configuring the matching frame. The frame ID must match the samples, and its mount must exist in tf.

The minimum dimSLAM configuration is:

```python skip
from dimos.mapping.dim_slam.dim_slam import DimSlam, ImuConfig

DimSlam.blueprint(
    camera_mode="stereo",
    imus=[
        ImuConfig(
            frame_id="camera_accel_optical_frame",
            gyro_noise_density=2e-4,
            gyro_random_walk=1e-5,
            accel_noise_density=1.8e-3,
            accel_random_walk=1e-4,
        )
    ],
)
```

These are the RealSense D455 datasheet noise figures. Each configured IMU has independent noise values and estimated biases. A good gyro does not lose its influence because another IMU is noisy.

Keep the robot still during initialization. `init_samples=200` corresponds to one second at 200 Hz. Initialization restarts when angular velocity exceeds `init_gyro_limit`, whose default is 0.05 rad/s.

Validation rejects an IMU entry when:

- `frame_id` is empty or duplicated.
- Any noise figure is zero or negative.
- `init_gyro_limit` is zero or negative.

An IMU alone is not the main improvement. The production configuration adds wheel odometry.

## Adding wheel odometry

This configuration comes from Alfred, a wheeled robot with a D455 and wheel encoders:

```python skip
DimSlam.blueprint(
    camera_mode="stereo",
    # Alfred's computer has no GPU, so libcuvslam is built
    # with -DENFORCE_GPU=OFF.
    use_gpu=False,
    # Declare stereo order explicitly. Sorting frame names happens
    # to produce left-then-right here, but that is not a contract.
    cameras=[
        CameraConfig(frame_id="d455_infra1_optical_frame"),
        CameraConfig(frame_id="d455_infra2_optical_frame"),
        CameraConfig(
            # Depth is aligned to color and arrives in this frame.
            frame_id="d455_color_optical_frame",
            depth_cloud_max_range=4.0,
            # A full D455 cloud is about 400k points per frame at
            # 30 Hz. Decimation keeps it from drowning the mapper.
            depth_cloud_decimation=5,
        ),
    ],
    imus=[
        ImuConfig(
            frame_id="d455_accel_optical_frame",
            gyro_noise_density=0.0018,
            gyro_random_walk=2e-5,
            accel_noise_density=0.02,
            accel_random_walk=3e-3,
        )
    ],
    # Use fixed variances. Message covariances report accumulated
    # drift rather than the measured delta. Drop visual z.
    visual_odom_pose_variances=Covariance(
        x=0.01,
        y=0.01,
        roll=0.05,
        pitch=0.05,
        yaw=0.05,
    ),
    # Only the wheels measure velocity. dimSLAM's visual estimate
    # does not include twist. Keep wheel z, roll, and pitch as
    # absolute anchors.
    odom_sources=[
        SourceConfig(
            parent_frame_id="wheel_odom",
            child_frame_id="base_link",
            pose_variances=Covariance(
                x=0.05,
                y=0.05,
                z=0.001,
                roll=0.001,
                pitch=0.001,
            ),
            twist_variances=Covariance(
                x=0.02,
                y=0.02,
                yaw=0.05,
            ),
        ),
    ],
    # CPU visual covariance exceeds this during normal driving,
    # so disable the covariance gate.
    covariance_gate_translation_std=0.0,
    # Alfred is holonomic in the plane.
    per_dimension_error_variance=Covariance(
        z=0.01,
        roll=0.01,
        pitch=0.01,
    ),
    # Wheel odometry can cross Wi-Fi seconds late.
    replay_buffer_seconds=2.0,
).remappings(
    [(DimSlam, "odom_sources", "source_odometry")]
)
```

The measured results explain why these details matter:

- Gyro yaw halved final drift on one drive. Wheel-only estimation ended 2.66 m out. Wheel plus gyro ended 1.33 m out. The lidar reference itself set a 0.59 m floor because of its heading error.
- Skipping stationary gyro-bias initialization changed a 1.6 m final error into 19.8 m over a 517-second drive.
- A 4 m depth range produced better mapping quality than 6 m, with top-down F1 scores of 0.570 and 0.506 against a lidar-raycast reference.
- One uncalibrated IMU mount misaligned gravity by about 2.4 m/s² and caused the estimate to diverge.

---

# 🎚️ Tuning Which Sensors to Trust

Sensors are not equally reliable, and they do not measure the same things. Configure which dimensions of each sensor dimSLAM should trust.

For each variance:

- `0` ignores that dimension.
- A small positive number trusts it more.
- A large positive number trusts it less.
- A negative number uses the covariance carried by the message.

These values are per-axis variances:

- `x`, `y`, and `z` describe translation.
- `roll`, `pitch`, and `yaw` describe rotation.
- Pose translation uses metres and pose rotation uses radians.
- Twist uses m/s and rad/s about the same axes.

## Constrain a ground robot

Start by telling dimSLAM that the robot stays on the ground. An indoor ground robot normally does not move vertically, roll, or pitch.

```python skip
per_dimension_error_variance=Covariance(
    z=1e-6,
    roll=1e-6,
    pitch=1e-6,
)
```

This acts as a virtual zero-twist measurement whenever dimSLAM receives a source message. It prevents sensor error from making the robot sink, rise, or tilt. Use weaker constraints when the robot drives on ramps.

## Visual odometry

A camera can be reliable in some dimensions and unreliable in others:

```python skip
visual_odom_pose_variances=Covariance(
    x=0.01,
    y=0.01,
    z=0.0,
    roll=0.05,
    pitch=0.05,
    yaw=0.05,
)
```

This configuration tells dimSLAM to trust planar translation, ignore visual z, and give the visual part of the rotation estimate less weight. Increase the variances when motion blur or fast rotation makes the visual estimate unreliable. The camera must remain trusted in at least one dimension.

## External odometry

Configure each external source using the exact frames carried by its messages:

```python skip
SourceConfig(
    parent_frame_id="wheel_odom",
    child_frame_id="base_link",
    pose_variances=Covariance(
        x=0.05,
        y=0.05,
        z=0.001,
        roll=0.001,
        pitch=0.001,
    ),
    twist_variances=Covariance(
        x=0.01,
        yaw=0.02,
    ),
)
```

Wheel encoders are good at forward velocity and turn rate. They are weaker as a source of long-term position because small errors accumulate. Trust them less when wheels slip. Skid-steer robots often need a larger yaw variance during turns.

A negative source variance uses the message covariance. A fixed positive variance is usually better for a drifting source because its reported covariance describes accumulated drift, not the measured delta.

A source whose parent is `odom_frame_id` contributes absolute measurements. A source with another parent contributes dimSLAM-anchored deltas because its own pose has drifted.

## IMUs

IMU trust comes from the four noise characteristics in `ImuConfig`, normally taken from the datasheet. Larger noise values give the IMU less influence.

Increase accelerometer noise when robot vibration would otherwise look like real movement. Do not compensate for a bad mount by tuning noise. Fix its transform.

## Four tuning rules

1. Do not use a sensor for dimensions it does not measure. Set those dimensions to zero.
2. Tell the system about physically impossible movement. Constrain z, roll, and pitch on a flat-ground robot.
3. Treat accumulated error carefully. Wheel odometry measures motion well but becomes less reliable as errors accumulate.
4. Tune relative trust. When sensors disagree, the more trusted sensor wins. Focus on the ratio between trust values instead of searching for one perfect number.

---

# 🛠️ Conclusion and Troubleshooting

Before driving, verify:

- The camera mode matches the connected streams.
- Camera intrinsics are published on `camera_info`.
- Every camera and IMU frame has a correct, rigid tf transform.
- Stereo cameras are declared explicitly, left first.
- Sensor timestamps use compatible clocks and meet the configured skew limit.
- The robot remains still for IMU initialization.
- External odometry frame IDs match its `SourceConfig`.
- Sensor variances include only dimensions each sensor measures.
- Ground constraints match the robot's real motion.
- `world/odom_hist` retraces the driven route in Rerun.

| Symptom | Likely cause and fix |
| --- | --- |
| Odometry is fine standing still but wrong while moving | Check timestamps. Stereo pairs must meet cuVSLAM's 1 ms contract. Hardware-triggered D455 pairs do this naturally. Software-triggered rigs may not. Spot images arrive about 15 ms apart and depth can trail by up to 90 ms, causing every frame set to be silently dropped. Set `max_skew_ms=20.0`, or use the measured spread of your rig. |
| Tracking works but the robot moves in the wrong direction | The extrinsics are wrong. A few millimetres of error causes small constant drift. A wrong optical-frame rotation produces complete nonsense. Check tf and the URDF first. |
| Position drifts badly from startup when using an IMU | The robot moved during gyro-bias initialization. Leave it still for about one second. Initialization restarts automatically after it becomes still. |
| Bias initialization never completes | `init_gyro_limit` is below the gyro's noise floor at rest. Raise it above the gyro bias while keeping it below any real rotation. |
| The map has two floors, or the robot appears rolled | z, roll, or pitch are random-walking. Configure `per_dimension_error_variance`. If wheel poses report z, roll, and pitch as zero, keep those dimensions as absolute anchors. |
| The path jumps sharply or resets | The covariance gate is firing. A CPU-only visual estimate's translation standard deviation can start above 1.0 and grow past 9 during normal driving, leaving no useful threshold. Set `covariance_gate_translation_std=0.0` for that case. |
| Gravity is misaligned and the estimate diverges | The IMU mount is uncalibrated. One bad mount produced about 2.4 m/s² of gravity misalignment. Correct the tf transform. |
| Wheel odometry appears to be ignored | Its `parent_frame_id` or `child_frame_id` does not exactly match `header.frame_id` or `child_frame_id` in the messages. Unmatched sources are dropped. |
| Wheel odometry arrives late over Wi-Fi and the estimate stutters | Increase `replay_buffer_seconds`. Alfred uses 2.0 seconds. Late messages make dimSLAM revisit the earlier state and replay subsequent measurements. |
| The depth map looks plausible but has the wrong scale | Check `depth_units_per_meter`. A 16-bit millimetre depth stream uses `1000.0`. |
| The mapper falls behind or drops clouds | A full-resolution depth stream can contain about 400,000 points per frame at 30 Hz. Set `depth_cloud_decimation` and `depth_cloud_max_range`. |
| The rig is mirrored or the stereo baseline sign is wrong | The `cameras` list was empty, so cuVSLAM sorted frame names. Declare the list explicitly with the left camera first. |
| The module warns about the GPU backend | `use_gpu=True` is configured on a machine without a GPU. The CPU path requires `libcuvslam` built with `-DENFORCE_GPU=OFF`. |
