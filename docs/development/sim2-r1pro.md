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

## Sensors and display

| Streams | Source | Requested rate |
|---|---|---|
| `color_image`, `depth_image`, `camera_info` | Head-left optical frame, native Unreal | 15 Hz ceiling |
| `lidar` | Chassis Mid-360 frame, rolling ray acquisition | 10 Hz |
| `pointcloud` | Truth-registered lidar for visualization | 10 Hz |
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

Desktop requests one URDF display tree. UrdfManager now combines named models
for `*.urdf`, with TF placing their roots, while preserving individual models
for named requests and physical consumers. This blueprint omits collision
meshes from that display response: the office has over ten thousand solver
geometries, whereas its visual scene uses sixteen mesh chunks. The physical
collision geometry and lidar query geometry remain intact.

## Robot assumptions

The model generator preserves vendor frames, visuals and inertias. It replaces
wheel collision meshes with cylinders measured from those meshes and fixes the
four gripper joints because their vendor effort and travel limits are zero.
Wheel limits and servo gains are simulation assumptions, not hardware tuning.
Known structural overlaps use the existing R1 Pro collision exclusions.

Base driving uses motor commands and wheel contact, with no root-pose override.
Upper-body joint control remains available through `R1ProSimControl.set_joint`
and `upper_command`; Desktop's arm-planning/VR integration and gripper actuation
are not implemented by this blueprint.

## Validation

The initial empty-scene test ran at 1.00x real time; that result does not describe
the office configuration. A ten-second office measurement after warmup recorded
1.01x real time, 6.7 RGB-D acquisitions/s and 10.1 lidar scans/s, with no new
lidar gaps or late physics steps during that interval. Desktop's drive endpoint
moved the base 0.306 m at a requested 0.15 m/s for two seconds. Both IMU streams
were approximately 200 Hz. Desktop visibly rendered the robot, office, lidar
and camera, but its camera preview reported only 2–3 fps: the 15 Hz target is
not achieved end to end. Full-office measurements and actual camera evidence
are saved locally under `~/.cache/dimos/sf555/validation/r1pro/`.

Validation covers the actual Desktop drive endpoint, a moving free base, RGB-D
camera output, both IMUs, lidar and encoders, and the composed display model.
Unit tests cover steering, command timeout, limits, collision exclusions and
multi-model render requests. The blueprint registry, Ruff and targeted mypy
checks are also run.

The controller can replay an action in response to a repeated lockstep
observation. SimModule counts rejected duplicates; these are distinct from
controller deadline faults. Extreme manual poses can still collide or tip the
robot: joint teleoperation is not collision-aware planning.
