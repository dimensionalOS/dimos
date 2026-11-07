# R1 Pro in the SF555 office

`r1pro-sim2` runs a mobile R1 Pro in the existing SF555 office, with MuJoCo
physics, selectable MuJoCo/Unreal RGB-D, Mid-360 lidar, chassis and torso IMUs, and joint
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

Camera rendering currently defaults to **MuJoCo for testing**. It uses the same
office visual geometry, three camera calibrations, streams and Desktop/headset
views, with basic shading rather than Unreal materials and lighting. To opt
back into Unreal, set `DIMOS_R1PRO_CAMERA_RENDERER=unreal` in this checkout's
`.env` (or the launching process environment) and restart the blueprint. The
accepted choices are `mujoco` and `unreal`.

Stop with `dimos-desktop dimos stop`. The camera module also stops its owned
Unreal process. Initial office compilation can take tens of seconds; the
blueprint gives startup 120 seconds.

## Spatial Desktop and headset teleoperation

Open [Spatial Workspace in Desktop](http://127.0.0.1:5555/?app=dim-spatial-workspace).
This ports the OpenYAM spatial viewer: robot meshes, the observed textured RGB-D
surface, the accumulated global map, camera strip, arm selection/jogs, and shared
Hold/Enable controls. It never sends office asset meshes to the browser.
The static Desktop app discovers `get_web_views` through the gateway. Its local
source is `../dim-spatial-workspace`; install that repository's
`feat/spatial-workspace` branch as `dim-spatial-workspace`.

Use **Headset** for the current LAN URL, `https://<laptop-wifi-ip>:8443/spatial`.
The headset needs a trusted HTTPS connection and a WebXR browser on the same
reachable network. Opening the view only observes. Enter XR facing your intended
forward direction, release the grips, then grip again to engage. This simulation
starts enabled; it does not require a separate Resume after every restart.
An explicit Hold still latches until Resume. Other stations default to paused.

- Controllers: grip clutches the corresponding arm; release holds it. Left stick
  drives/strafes and right stick horizontal turns, capped at 0.2 m/s and 0.3 rad/s.
  Right stick vertical raises/lowers the torso; release holds the reached height.
  It starts fully raised, with about 21 cm of commanded lowering available.
- Face your intended forward direction on XR entry or when pressing **Resume**. Controller
  motion maps to robot-body forward/left/up, at one physical metre per metre,
  independent of scene rotation/scale and robot world position/heading. Looking
  around does not recalibrate. Hold then Resume recalibrates; reconnecting or
  changing input mode also requires a fresh calibration and release/re-grip.
- Hand tracking: pinch a tool handle to move that arm. Two free-space pinches
  reposition/scale the workspace; camera and control panels can be moved/pinned.
- Controller triggers never resize the scene. Use the HUD's −/+ and Actual size
  buttons. In Classic teleop, hold the corresponding grip and squeeze its trigger
  to close the gripper; release the trigger to open it. Tool handles mode provides
  independent opening sliders. Two free hand-tracking pinches still arrange the
  scene in Tool handles mode; arm and base commands are held during arrangement.
- The session HUD starts pinned in front of your view. **Bring here** also
  restores the HUD and camera panels relative to the headset, independent of the
  robot's world position. **UNPIN** parks the HUD in the scene; **HUD** pins it again.
- The lidar global map is displayed as instanced 5 cm cubes matching the mapper's
  voxel size. It remains a sensor-derived map, separate from the RGB-D surfaces.
- Desktop joint jogs pause headset input and command the existing upper servo.
  Hold clears control; resume requires release and re-grip. Base buttons drive
  only while pressed. Lost input stops the base after 0.3 s.
- Both G1Z grippers are driven in simulation, with normalized opening feedback.
  Recording and home planning controls require optional providers and are hidden
  or disabled when absent.

Robot geometry, joints, tool/base frames and camera labels are configured in the
blueprint. Shared code lives under `dimos/teleop/spatial`; R1-specific base driving,
frame conversion and servo jogging live in `r1pro/sim/spatial.py`. This version has
two operator arm slots (`left`/`right`) and three optional camera slots
(`workspace`/`left`/`right`), whose stream inputs can be remapped. It supports zero,
one or two arms with arbitrary joint names; it is not yet an unlimited-group or
arbitrary-camera-count UI. Real robot controllers still need their own bindings.

Arm targets pass through Pink IK and the control coordinator to sim2's upper-body
servo. Each measured body-to-shoulder transform is captured on grip engagement so
driving carries held arms with the base. Each task owns seven arm joints. The
torso servo owns a separate height input; the arm coordinator cannot overwrite it.
Tool handles and desktop keyboard control retain their spatial-world semantics.
The shared protocol validates sessions/epochs and requires a fresh release after
input loss. Joint feedback freshness uses receipt time, so simulation timestamps
are not compared against the wall clock.

## Sensors and display

| Streams | Source | Requested rate |
|---|---|---|
| `color_image`, `depth_image`, `camera_info` | Head-left optical frame, selected renderer | 15 Hz ceiling |
| `left_wrist`, `left_wrist_depth`, `left_wrist_info` | Left D405 mount, shared renderer | 15 Hz ceiling |
| `right_wrist`, `right_wrist_depth`, `right_wrist_info` | Right D405 mount, shared renderer | 15 Hz ceiling |
| `lidar_raw` | Chassis Mid-360 frame, rolling ray acquisition | 10 Hz |
| `pointcloud` | Truth-registered lidar for visualization | 10 Hz |
| `global_map` | VoxelGridMapper accumulating registered lidar | Every 5 scans |
| `imu_chassis`, `imu_torso` | Independent vendor IMU frames | 200 Hz |
| `joint_state` | 28 joint encoders, including four coupled finger coordinates | 50 Hz |
| `tf` | Actual simulated body poses, through UrdfManager | 30 Hz |

The simulated head-left image is 640×512, preserving the physical 1920×1536
aspect ratio. Its approximate 118° horizontal / 92° vertical FOV comes from the
2026 vendor specification. The vendor URDF gives approximately 120 mm stereo
baseline and 20° downward optical pitch. `head_camera(rectified_info=...)` can use
scaled rectified calibration instead of approximate intrinsics; it rejects raw
distorted calibration because Unreal renders an ideal pinhole camera.

The 2026 hardware uses two synchronized GMSL2 Sony ISX031 RGB cameras
(SENSING SG3S-ISX031C-GMSL2F), not a ZED. The local native capture module documents
1920×1536 at 30 Hz and reads factory intrinsics/distortion from
`/opt/galaxea/body/stereo.yaml`. The hardware blueprint disables vendor head-camera
capture in favor of that native driver. Neither this capture path nor the camera
specification establishes a working depth stream: vendor driver documentation
lists registered head depth only **if available**. Optional wrist depth cameras
are separate devices.

This demo selects the **optional wrist depth-camera configuration**, not the
alternative fisheye pair. Each wrist renders 320×180 (physical 1280×720 aspect),
with approximate 87°×58° FOV and valid depth restricted to 0.07–0.50 m, from the
[2026 specification](https://docs.galaxea-dynamics.com/R1Pro/en/docs/2026/).
RGB remains visible beyond that depth range. Both use the pinned URDF's
`left_d405_link` / `right_d405_link` poses as ideal optical frames; factory
intrinsics, distortion and optical-center corrections are not available.
The generator removes the alternative GMSL wrist housings, collisions and masses
while preserving their fixed frames. Upstream includes both alternatives in
nearly the same location, which otherwise occludes the D405 views.

`R1ProSimCameras` accepts the shared `CameraRigRenderer` protocol. MuJoCo shares
one compiled geometry scene and caches image contexts by resolution; Unreal
shares one external process/world across all three cameras. The module's
single worker captures one latest pose at a time, round-robin, with independent
rate ceilings; 15 Hz is requested **per camera**, not measured throughput.
Unreal reuses up to three target resolutions and resets temporal history when
switching optical frames to prevent cross-camera ghosting. Rebuild the prepared
renderer after updating native source (see the SF555 setup).

Desktop and headset expose Head / Left wrist / Right wrist panels. Each RGB-D
pair has its own calibration, timestamped optical-to-world TF and mesh. Wrist
meshes reject depth discontinuities over 3.5 cm. Empty depth clears previous
geometry; stale camera surfaces are hidden rather than attached to a moving arm.
Only robot geometry, sensor-derived surfaces and the global map reach the viewer.
`GET /spatial/surface?camera=workspace|left|right` serves independent surface
updates; omitting `camera` retains the original head-view API. Per-camera ages,
errors and valid point counts are in `/spatial/state` under `surfaces`.

Unreal currently supplies **ideal, registered axial depth in metres**, aligned to
head-left RGB. The surface encoder pairs timestamps, uses CameraInfo and optical
TF, and leaves holes at invalid depths/discontinuities. This is sensor-shaped
simulation output, not a stereo matcher. Physical deployment needs either a
verified registered depth publisher or synchronized stereo rectification and a
validated disparity/depth pipeline with factory extrinsics. No physical stereo
depth pipeline is implemented by this change. Head-right and wrist cameras are
not instantiated in the demo, and factory calibration was not available locally.
The request remains a 15 Hz ceiling; this change does not resolve Unreal's
previous throughput limit.

Sources: [Galaxea 2026 R1 Pro](https://docs.galaxea-dynamics.com/R1Pro/en/docs/2026/),
[driver interfaces](https://docs.galaxea-dynamics.com/R1Pro/en/docs/2026/software_interface/r1pro_driver/),
[SENSING camera](https://sensing-world.com/en/h-pd-286.html), and
`dimos/robot/galaxea/r1pro/head_cameras.py`.

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
wheel collision meshes with cylinders measured from those meshes. The vendor
finger limits are zero placeholders. The simulation uses equal/opposite 52 mm
finger travel, matching the G1Z's [104 mm aperture and 108 mm/s aperture speed](https://galaxea-dynamics.com/products/galaxea-g1z-gripper).
Finger origins, axes and meshes are preserved; measured fingertip geometry fixes
the opening signs. Each finger is limited to 30 N in this simulation. This is
conservative demo tuning, not a calibrated reproduction of the rated grip force.
Wheel limits and servo gains are simulation assumptions, not hardware tuning.
Known structural overlaps use the existing R1 Pro collision exclusions.

Base driving uses motor commands and wheel contact, with no root-pose override.
Upper-body commands use `upper_command`. The spatial joint-jog route pauses the coordinator before calling the servo;
standalone `R1ProSimControl.set_joint` calls while the coordinator owns those
targets can be overwritten. Collision-aware arm planning is not implemented by
this blueprint. Arm IK uses a shoulder-rooted subtree and the measured shoulder
frame at clutch engagement, so torso sag is not replaced with a zero posture.
The R1 simulation accepts up to 0.01 rad of soft-stop feedback overshoot, while
commanding 0.03 rad inside the physical limits. Larger overshoot still rejects
control. This addresses a captured limit-rejection freeze, not every possible
singularity or collision.
R1 uses the quadprog QP backend: a deterministic reproduction
from a stuck measured posture produced a false primal-infeasibility result with
ProxQP, despite zero displacement satisfying every inequality. Disabling its
preconditioner fixes one case but not another. Both arms pass a 1,000-target
deterministic sweep with quadprog; command velocity and tracking-error limits remain.

## JoyLo findings and interface direction

[JoyLo](https://behavior-robot-suite.github.io/docs/sections/brs_ctrl/run_joylo.html)
uses kinematic leader arms and joint-angle commands, with optional bilateral
feedback. Its documented six-joint examples differ from this seven-joint R1 Pro.
The project also publishes a [seven-joint leader design](https://behavior-robot-suite.github.io/docs/sections/joylo/joylo_7dof.html).
Headset hand poses do not provide that joint correspondence or haptic feedback;
our interface still needs IK, sensible limits, clutching and visible reach errors.

The [Joy-Con implementation](https://github.com/behavior-robot-suite/brs-ctrl/blob/main/brs_ctrl/joylo/joycon/joycon_interface.py)
separates four command channels: arm joints, grippers, torso and chassis. Its left
stick translates the base, or turns it when clicked; the right stick changes torso
pitch/yaw, up/down buttons stand/squat, and ZL/ZR close the grippers. It supports
held or toggled gripper input. These mappings are useful design references; its
old torso presets and joint limits must not be copied to the 2026 URDF.

The current model's torso has two folding pitch joints, a third pitch joint and
a yaw joint. Coordinating the folding joints changes shoulder height; it is not
a vertical prismatic lift. The right stick vertical axis now controls height.
For this model's equal 0.35 m links and negative-Y third joint, targets
`(a, -2a, -a, 0)` preserve upper-body orientation and fore/aft position while
lowering it by `0.7 * (1 - cos(a))` metres. The demo bounds `a` to 0–0.8 rad
(also clamped to model limits), with 0.2 rad/s folding speed. Release, Hold or
input expiry stops target integration; no queued destination remains. The
folding knee uses higher simulation stiffness to reduce static sag under load.
Lean/yaw torso controls and whole-body human pose tracking remain unimplemented.
This is a bounded simulation trajectory, not a hardware-calibrated controller
or a collision-aware whole-body planner.

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

The earlier generic headset protocol smoke test exercised the live HTTPS/WebSocket endpoint:
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

The spatial protocol smoke test (`demo_spatial.py` / `spatial_results.json` in the
validation directory) moved both arms through 4 cm hand targets, with maximum
joint changes of 0.072 and 0.066 rad. The base moved 0.133 m, then slowed below
0.004 m/s after disconnect; RGB-D surface payloads were approximately 122 kB.
Focused tests cover calibration scaling, distortion rejection, depth units,
synchronization, input expiry, stale epochs and release-before-grab. A physical
headset tracking/comfort check still requires an operator.

The torso remains owned by the sim servo; repeated Hold/Enable does not replace
its posture target with measured gravity sag. The Desktop joint-jog endpoint was
also exercised against the running simulation, including automatic headset hold.

With all three cameras enabled, a 20-second November 6 measurement after the
wrist integration recorded 2.40 / 2.40 / 2.35 captures per second (head / left /
right), 1.000× simulation speed and zero additional late physics steps. Surface
update rates were 2.45 / 2.45 / 2.40 Hz; p95 age since receipt was approximately
0.48 seconds. These are shared-renderer measurements, not 15 fps per camera.
One Unreal process was running. All three panels and the reconstructed scene
were visually checked in the workspace. Evidence is in `wrist_results.json`,
`wrist_{workspace,left,right}.{jpg,json}` and `demo_wrist_cameras.py` in the local
validation directory. The native calibration test also exercised switching
optical frames and reusing earlier target sizes; its report is under
`wrist_calibration/`. The 74 focused Python/registry checks, 22 JavaScript checks,
and targeted mypy checks passed. The new live motion smoke attempt was refused
because a control client was already connected; the earlier motion results above
are not a rerun of this camera configuration. Physical headset validation remains
outstanding.

The MuJoCo testing renderer measured 5.84 / 5.79 / 5.84 captures/s over 20 seconds
at the same camera resolutions, with 1.000× simulation speed, zero additional
late steps and p95 surface receipt ages below 0.17 seconds. Unreal was stopped.
Evidence is under `mujoco_results.json` and `mujoco_{workspace,left,right}.jpg`
in the validation directory. The office is recognizable with grey shading but
has visual artifacts and none of Unreal's authored materials/lighting. This is
still below the requested 15 Hz per camera. An actual offscreen-render test
verifies unequal focal lengths, principal-point offsets, axial depth and switching
between image sizes without retaining another camera's calibration.

The subsequent teleop fix reproduced the captured left joint-3 soft-stop
overshoot: the old 0.001 rad feedback tolerance rejected it, while the revised
R1 configuration moved it inward at the existing velocity limit and converged
inside the command margin. Separate real-Pink checks verify both shoulder-frame
models against full-body FK with a non-neutral torso. The log also recorded the
same limit rejection on the right arm.

An isolated MuJoCo run using the actual mobile model and servo opened both G1Z
grippers to approximately ±0.052 m per finger, closed the left while the right
stayed open, then closed both to within 0.001 mm of zero. Evidence is in
`demo_gripper_physics.py` and `gripper_physics_results.json` in the validation
directory. Protocol tests verify trigger-to-opening publication and release,
and JavaScript tests verify simultaneous controller triggers cannot resize.
This checks free-space actuation, not grasp force or object retention. The
office blueprint restarted with 28 joint feedback coordinates and three fresh
camera streams. A live WebSocket motion test was refused because the headset
had already reconnected; no new live arm-motion result is claimed.

## Real R1 Pro deployment

`r1pro-spatial` uses the same Desktop/WebXR workstation as `r1pro-sim2`.
It replaces simulation with `R1ProConnection`, the vendor joint tracker and
chassis gatekeeper. Both use the shared arm IK tasks and body-relative clutch.
The hardware binding captures measured posture at engagement; it never homes
on startup. Grippers use the vendor's 0–100 percent opening, not the simulated
finger displacement. Hardware joint jogging and automatic homing are disabled.

On robot `r1-pro-0219` (Tailscale `100.68.53.40`):

```bash
ssh nvidia@100.68.53.40
cd ~/dimos-r1pro-spatial
source /opt/ros/humble/setup.bash
source ~/galaxea-dimos/install/setup.bash
source .venv/bin/activate
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
dimos run r1pro-spatial
```

This environment uses ROS Humble, the installed Galaxea workspace, ROS domain **0**
(the domain actually used by this robot's running drivers), FastDDS, and the
checkout's system-Python-3.10 venv. It requires the vendor stack already running;
it does not boot another vendor profile. Ctrl-C stops DimOS. The optional read-only check
`python -m dimos.robot.galaxea.r1pro.spatial_preflight` checks feedback freshness
and reports other arm, torso, gripper or chassis command publishers before launch.

Open `https://100.68.53.40:8443/spatial` over Tailscale, or
`https://10.10.167.201:8443/spatial` on the robot's current Wi-Fi. WebXR requires
HTTPS accepted/trusted by the headset browser. The checkout uses the robot's
existing Desktop certificate (covers both addresses). The same spatial page is
exposed as a Desktop webview when that Desktop is connected to this run.
Enter XR, release then squeeze a grip to clutch that arm, and use its trigger
for the gripper. Left stick drives/straffes; right stick X turns; right stick Y
changes torso height. Hold is explicit and latches until Resume. View scaling
is on the HUD and does not share the gripper triggers.

Hardware control caps arm commands at 0.25 rad/s, torso joints at 0.15 rad/s,
base translation at 0.15 m/s and rotation at 0.25 rad/s. Torso height follows the
URDF's two 0.35 m links, preserving horizontal reach, pitch and yaw; it advances
only while tracking error remains under five degrees. This is model-based
teleop, not a collision-avoiding whole-body controller. Physical motion has not
been exercised during deployment.

Sensors are read from the already-running vendor ROS camera drivers. Head depth
is computed from the factory `/opt/galaxea/body/stereo.yaml`, with rectification
and a 480×384 SGBM disparity map. Unsupported disparities stay holes. The two
D405 wrist streams use aligned depth converted from ROS millimetres to metres;
color and depth are rectified together using their live CameraInfo. All three
RGB-D feeds target 5 Hz. A read-only on-robot sample measured about 104 ms for
head JPEG decode plus stereo computation; this is not an end-to-end FPS claim.

The chassis Mid-360 and Point-LIO supply world pose and the 5 cm voxel map, using
the existing R1 mount transform. Native lidar sockets use the SDK host ports + 1
to avoid the vendor listener sockets. The native driver configures the lidar
stream when the blueprint starts. This stack does not launch another driver for
any camera. URDF finger meshes remain welded in the hardware planning model;
the interface's gripper opening is measured feedback, but the finger mesh itself
is not animated.

Install/update on the robot with:

```bash
uv sync --python /usr/bin/python3.10 --python-preference only-system \
  --no-default-groups --extra base --extra manipulation --extra cpu
```

The native Mid-360/Point-LIO binaries must also be built for this checkout (Nix
builds in their respective `rust` directories); they are prepared as part of this
deployment. Keep the former `~/tmp/dimos-r1pro-manipulation` checkout separate:
it contains pre-existing uncommitted hardware work.
