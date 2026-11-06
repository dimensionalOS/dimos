# R1 Pro in the SF555 office

`r1pro-sim2` runs a mobile R1 Pro in the existing SF555 office, with MuJoCo
physics, native Unreal RGB-D, Mid-360 lidar, chassis and torso IMUs, and joint
encoders. DimOS Desktop displays and controls the robot through the standard
streams. The standalone Viser demo has been removed.

## Run in Desktop

Prepare the office assets as described in [the SF555 setup](/docs/development/sim2-sf555.md).
They are shared with the G1 office blueprint; the original office project is
unchanged. From this checkout:

```bash
uv sync --frozen --extra sim-unreal --extra control --extra cpu --extra planning --group dev
source .venv/bin/activate
python -m dimos.robot.galaxea.r1pro.sim.model
```

Set Desktop's DimOS checkout to this checkout, then launch `r1pro-sim2` in its
launcher, or run:

```bash
dimos-desktop dimos launch r1pro-sim2
```

Open [DimOS Desktop Controller](http://127.0.0.1:5555/?app=dim-controller).
Use the wheeled profile, fixed frame `world`, follow frame `r1pro/base_link`,
and drive topic `/cmd_vel`. With the view focused: W/S drives, A/D turns,
Q/E strafes, and Space stops. The simulation controller caps translation at
0.4 m/s and yaw at 0.5 rad/s and brakes if commands stop arriving for 0.3 s.
The robot starts at `(16, 1, 0.005)` in the office's metric Z-up frame.

Stop with `dimos-desktop dimos stop`. The camera module also stops its owned
Unreal process. Initial office compilation can take tens of seconds; the
blueprint gives startup 120 seconds.

## Headset teleoperation

The same blueprint starts the shared DimOS WebXR page. On a Quest on the same
network, open `https://<laptop-wifi-ip>:8443/teleop`, accept the local server's
certificate, and tap **Connect**. On macOS, `ipconfig getifaddr en0` reports the
usual Wi-Fi interface's current address. The network must allow device-to-device
connections. Only one headset control connection is accepted at a time.

- Hold a controller's middle-finger **grip** to move its corresponding arm.
  Movement is relative to the hand and arm poses captured when grip engages.
  Release grip to hold the arm; reposition your hand and grip again to re-clutch.
- **Left stick** drives forward/backward and strafes. **Right stick X** turns.
  Headset speed is capped at 0.2 m/s translation and 0.3 rad/s yaw.
- Releasing the sticks stops driving. Disconnect and stale input also stop the
  base; arm tasks stop following stale poses after 0.3 seconds.
- The headset displays the **Unreal head-left camera on a video panel**. It does
  not load an immersive office model. Desktop continues showing robot + map.
- Fingers remain fixed. Triggers do not actuate grippers in this simulation.

This reuses OpenYAM's WebXR controller protocol, independent grip/clutch sessions,
Pink IK tasks and control coordinator. It uses the standard **Connect** page,
without the OpenYAM learning app's recording or policy controls. Arm targets flow
from the coordinator's connection ports to the existing sim2 upper-body servo;
thumbsticks feed its base velocity input. The torso holds its pose, and each IK
task controls only its own seven arm joints in chassis coordinates. Planning
uses a 2 rad/s² default acceleration limit, matching the existing R1 planning
configuration, and teleop commands are capped at 0.5 rad/s. These are demo settings.

## Sensors and display

| Streams | Source | Requested rate |
|---|---|---|
| `color_image`, `depth_image`, `camera_info` | Head-left optical frame, native Unreal | 15 Hz ceiling |
| `lidar_raw` | Chassis Mid-360 frame, rolling ray acquisition | 10 Hz |
| `pointcloud` | Truth-registered lidar for visualization | 10 Hz |
| `global_map` | VoxelGridMapper accumulating registered lidar | Every 5 scans |
| `imu_chassis`, `imu_torso` | Independent vendor IMU frames | 200 Hz |
| `joint_state` | 24 motor encoders | 50 Hz |
| `tf` | Actual simulated body poses, through UrdfManager | 30 Hz |

The camera is 640x480 with approximate demo pinhole intrinsics. It does not use
factory stereo calibration. Head-right and wrist camera streams are not yet
instantiated. Camera throughput is measured separately from the 15 Hz request;
the existing Unreal performance limitation remains.

The lidar retains timed Mid-360 scan rays at 1/50 density for this desktop demo.
It intersects the office and the moving robot. The sensor's own link is excluded
from its rays by the standard raycaster. `pointcloud` is privileged truth
registration, not an odometry estimate. The IMUs have ideal orientation and no
added noise. Physics steps at 200 Hz; control stays at 50 Hz.

Desktop displays the R1 Pro model and `global_map`, following the original G1
sim2 blueprint. UrdfManager serves the first named model (R1 Pro) for Desktop's
wildcard URDF request. The office remains available to physics, lidar and Unreal,
but its asset meshes are not included in that display response. VoxelGridMapper
accumulates truth-registered scans on CPU with 5 cm voxels and emits every five
scans. Enable `global_map` in Desktop and disable raw lidar/pointcloud layers.
As in the G1 blueprint, robot returns can accumulate in this preview map.

## Robot assumptions

The model generator preserves vendor frames, visuals and inertias. It replaces
wheel collision meshes with cylinders measured from those meshes and fixes the
four gripper joints because their vendor effort and travel limits are zero.
Wheel limits and servo gains are simulation assumptions, not hardware tuning.
Known structural overlaps use the existing R1 Pro collision exclusions.

Base driving uses motor commands and wheel contact, with no root-pose override.
Upper-body commands use `upper_command`. With the headset coordinator running,
it owns and holds those targets; standalone `R1ProSimControl.set_joint` calls
would be overwritten. Desktop's arm-planning panel and gripper actuation are
not implemented by this blueprint.

## Validation

The initial empty-scene test ran at 1.00x real time; that result does not describe
the office configuration. A ten-second office measurement after warmup recorded
1.01x real time before enabling the mapper, 6.7 RGB-D acquisitions/s and 10.1 lidar scans/s, with no new
lidar gaps or late physics steps during that interval. Desktop's drive endpoint
moved the base 0.306 m at a requested 0.15 m/s for two seconds. Both IMU streams
were approximately 200 Hz. Desktop visibly rendered the robot, office, lidar
and camera, but its camera preview reported only 2–3 fps: the 15 Hz target is
not achieved end to end. Full-office measurements and actual camera evidence
are saved locally under `~/.cache/dimos/sf555/validation/r1pro/`.

Validation covers the actual Desktop drive endpoint, a moving free base, RGB-D
camera output, both IMUs, lidar and encoders. The current display uses the robot
URDF and accumulated global map. Unit tests cover steering, command timeout,
limits, collision exclusions and render requests. The blueprint registry, Ruff and targeted mypy
checks are also run.

The headset protocol smoke test exercised the live HTTPS/WebSocket endpoint:
each arm moved independently (maximum joint changes 0.083 and 0.068 rad for
4 cm hand motions), the base drove 0.133 m, and its speed fell below 0.001 m/s
after disconnect. The headset socket delivered 128 decoded 640x480 JPEG frames.
This tests the real WebXR-to-coordinator-to-sim path and camera transport; a
physical headset session and tracking comfort still need an operator check.
The local evidence includes `headset_test.json`, `headset_camera.jpg` and
`demo_headset.py` in the validation directory above. Unit tests also cover
stick release, per-hand input expiry, malformed input and reconnect reset.

The controller can replay an action in response to a repeated lockstep
observation. SimModule counts rejected duplicates; these are distinct from
controller deadline faults. Extreme manual poses can still collide or tip the
robot: joint teleoperation is not collision-aware planning.
