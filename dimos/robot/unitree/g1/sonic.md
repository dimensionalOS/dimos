# G1 SONIC controller

SONIC runs the planner, encoder and decoder at a 50 Hz policy rate. It accepts
standard Twist walking commands, selectable gaits, motion clips and optional
upper-body reference targets. No headset or teleop service is required.

The default gait is `SLOW_WALK`. Velocity commands set direction and speed
within that gait; they never select walking or running automatically. With a
walking gait selected, zero input idles while preserving that selection. Use
`set_locomotion_mode` on G1SonicPolicyModule to change modes; `None` restores
`SLOW_WALK`.

For local simulation on macOS:

```bash
uv sync --extra control --extra sim
uv run --no-sync dimos-sonic-models
uv run --no-sync dimos --transport zenoh --simulation mujoco run unitree-g1-sonic-wbc
```

The simulation blueprint selects ONNX CPU inference on macOS, with four threads
per model. Hardware still requires CUDA and rejects `cpu_simulation=true`.
On other hosts simulation uses CUDA by default; CPU-only simulation can be
selected explicitly with `--g1sonicpolicymodule.cpu_simulation=true` after the
blueprint name. Policy timing and motor watchdogs are unchanged.

Use DimOS Desktop's Controller app for visualization and operator controls.
SONIC blueprints do not launch Rerun, its WebSocket server, or scene overlays.
Robot state, camera, point-cloud and navigation streams remain available on Zenoh.

For a CUDA workstation, install the dependencies and NVIDIA assets:

```bash
uv sync --extra control --extra cuda --extra sim \
  --no-install-package onnxruntime --reinstall-package onnxruntime-gpu
source .venv/bin/activate
dimos-sonic-models
dimos-sonic-models --check
dimos --transport zenoh --simulation mujoco run unitree-g1-sonic-wbc
```

The CPU and GPU ONNX Runtime distributions share the same Python package;
excluding the CPU distribution prevents it overwriting the CUDA provider.
Keep these options when syncing this environment again.

Assets live in the DimOS cache (`~/.cache/dimos/sonic` by default), outside
the Git LFS data directory. `SONIC_MODEL_DIR` overrides this location for
both the installer and blueprint. The installer pins the Hugging Face revision and verifies SHA-256 hashes for
the policy, planner and observation configuration. The 13 example motion clips
come from a pinned NVIDIA deployment revision. `--check` performs no downloads.
SONIC v1.1 is the single supported policy bundle.

MuJoCo starts in the SONIC pose and waits for a complete control command before
advancing physics. Simulation arms automatically. Commands from a second shell:

```bash
dimos --transport zenoh shell
```

```python
c = app.G1SonicPolicyModule
c.status()
c.set_locomotion_mode("SLOW_WALK")
c.list_motion_clips()
c.play_motion_clip("macarena_001__A545")
c.stop_motion_clip()
c.set_estop(True)
```

E-stop latches damping. Releasing a control, resetting runtime state, or arming
again cannot clear it; recovery requires restarting the stack. Hardware also
checks feedback and command freshness in its independent motor publisher.

For JetPack 5 or 6, use `bin/hardware/g1/setup-sonic`. It detects the release and
installs the matching ONNX Runtime and model assets; `--check` checks system
prerequisites only. JetPack 5 uses its native CUDA 11.4 and cuDNN 8; an existing
CUDA 11.8 installation with its compatibility driver is also supported.
The wheel version `1.18.1.11.8` means ONNX Runtime 1.18.1, CUDA major 11,
cuDNN major 8, not a requirement for CUDA 11.8. JetPack 6 requires CUDA 12.6
and cuDNN 9.
Use the setup script's CUDA-only `LD_LIBRARY_PATH` as printed, without appending
`/usr/lib/aarch64-linux-gnu` or an inherited path: those can override the newer
libc and libstdc++ used by Nix-built lidar and mapping modules.
Hardware starts unarmed with policy outputs in dry-run.

Setup also runs a no-motor inference preflight. Run
`bin/hardware/g1/setup-sonic --verify` to repeat it without reinstalling packages.
Stop the blueprint before running this check. It checks model hashes,
loads the production pipeline on CUDA, and compares 30 planner calls against
an unoptimized CPU reference. Failure exits setup before it reports readiness.
This checks numerical consistency, not physical stability or control deadlines.

The JP5 ORT 1.18.1 planner can return corrupted finite values after its first
call with memory patterns enabled. SONIC disables that optimization for the
planner and TF32 for Jetson inference. Startup checks two planner calls, and
every returned plan is checked for valid frame count, finite values and unit
root quaternions before use. A failed check follows the existing control-fault
path; it is not silently normalized into a motion reference.

On the Jetson, install the clock service once from the checkout:

```bash
sudo bash bin/hardware/g1/install-jetson-clocks
```

This applies NVIDIA's `jetson_clocks` now and at each boot, after
`nvpmodel.service` restores the selected power mode. It locks CPU, GPU and EMC
clocks to the maximum allowed by that mode. It preserves the selected power mode
and fan policy. The service runs as root; DimOS runs as your normal user. It does
not start or restart the robot stack and needs no passwordless sudo rule.

Power mode and clock locking are separate: selecting MAXN alone still permits
dynamic clock scaling. A manual `sudo jetson_clocks` lasts until reboot (or a
clock-setting change).

In the 2026-10-18 Orin NX G1 deployment trial, MAXN was already selected. A
publisher-free inference harness used synthetic standing feedback for 250 steps,
with ZED HD720 at 15 FPS, NEURAL LIGHT depth and DimOS Desktop running; Rerun was
disabled. The same harness before and after locking clocks measured:

| Metric | Dynamic clocks | Locked clocks |
| --- | ---: | ---: |
| Effective rate | 45.58 Hz | 50.00 Hz |
| Mean inference step | 15.54 ms | 8.29 ms |
| p95 inference step | 28.19 ms | 13.75 ms |
| Maximum inference step | 33.93 ms | 18.65 ms |

GPU clocks were mostly 510 MHz before and fixed at 918 MHz afterward; CPU limits
were locked to 1,984 MHz. The policy's 50 Hz target gives each step 20 ms. These
numbers measure the deployed inference workload, not this PR's complete blueprint
or physical motion. The harness sent no motor commands. After locking, the
maximum standing start-to-start interval was still 27.36 ms, and a separate
synthetic turning workload had a 36.31 ms maximum inference step. Clock locking
does not establish walking stability or guarantee every control deadline.

Check the service after a reboot with:

```bash
systemctl status dimos-jetson-clocks.service --no-pager
```

`active (exited)` is expected for this one-shot service. SONIC independently reads
the CPU/GPU frequency limits before hardware startup, arming, reactivation and
switching off dry-run. Unlocked or unreadable limits reject the operation with
the command to fix it; simulation and non-Jetson hosts skip the check. The check
does not run in the policy loop. It verifies configured CPU/GPU locks, not the
power budget, EMC, thermal throttling or inference deadlines. A lower power mode
can still be too slow with locked clocks; choose and validate the mode for the
specific robot. An `active` service alone does not prove clocks were left locked.

After checking startup and feedback, the hardware activation RPCs are:

```python
c = app.G1SonicPolicyModule
c.set_dry_run(False)
c.arm()
```

This standalone wiring has not been validated on physical hardware.

Squat and kneeling request zero translation. Centered sticks stop crawling
while retaining its posture. Face-down mode 7 is unavailable, matching NVIDIA's
selectable motion menu. Floor transitions remain timed; crawl stability and
the earlier hardware instability remain open validation issues.

## Module interface

The blueprint runs `G1SonicPolicyModule` directly, without ControlCoordinator,
coordinator tasks or a task-compatibility wrapper. Command flow is:

```text
navigation / teleop -> MovementManager -> G1SonicPolicyModule -> SonicController -> motor IO
                      selects cmd_vel                        runs SONIC at 50 Hz
```

`MovementManager` combines `nav_cmd_vel` and `tele_cmd_vel`, with teleoperation
cancelling the active navigation goal. Its `cmd_vel` output feeds SONIC's
`base_command`. Applications should use those upstream inputs instead of
competing publishers on `cmd_vel`. There is no general task-priority scheduler
or timed arm-trajectory API in this stack. SONIC owns all 29 motor targets;
arm inputs condition the policy rather than independently controlling motors.

The policy module exposes streams and RPCs and delegates to `SonicController` in
`sonic_controller.py`. That controller owns the 50 Hz loop, pose ramp and fault
latch; `SonicPipeline` handles model inference. `G1WholeBodyConnection` owns real
motor IO and its independent 500 Hz publisher. MuJoCo uses the existing
shared-memory whole-body adapter. SONIC sends no motor commands before its models load.

SONIC has no sim2 dependency. Migrating its simulated motor backend belongs in
the later sim2 integration, not this policy PR. The driver's
`controller_fault: In[String]` accepts faults from any motor controller;
`G1SonicPolicyModule.controller_fault` supplies that signal for SONIC.

- `base_command: In[Twist]`: `linear.x`, `linear.y`, and `angular.z`, remapped
  to `cmd_vel`. After one second without fresh input, SONIC commands zero
  walking velocity while continuing to balance.
- `position_command: In[JointState]`: named arm references, on
  `/g1/position_command` in this blueprint. Use the arm names from measured
  `joint_state`, for example `g1/left_shoulder_pitch`. Partial arm updates hold
  the other reference values. Unknown names, leg targets, duplicate names,
  missing positions and non-finite values are rejected. These are encoder
  references, not independent joint servos.
- `joint_state: Out[JointState]`: measured state for all 29 joints, remapped to
  `g1_joints` (`/g1/joints`) for existing visualization.
- `imu: Out[Imu]`: measured IMU state. Raw hardware feedback uses the separate
  `low_level_imu` input. Odometry still comes from simulation or localization.

For example, send `JointState(name=["g1/left_shoulder_pitch"], position=[0.2])`
to `/g1/position_command`. `c.set_upper_body(positions)` sets a complete
14-element arm reference in DDS order. The controller limits these encoder
reference changes to 1 rad/s per joint, starting from measured arm positions.
`c.clear_upper_body()` returns to the default arm references at the same rate.
`set_vr_3point()` retains its existing wrist/head reference
contract; a generic Cartesian command stream is not implemented here.

Topic paths above are logical names. Typed Zenoh keys include the `dimos/`
prefix and message-type suffix, for example `dimos/cmd_vel/geometry_msgs.Twist`.

`arm()`, `disarm()`, `set_dry_run()`, `set_locomotion_mode()`, motion-clip
commands, `reset_runtime_state()` and `status()` are direct module RPCs.
`disarm()` returns to measured-pose hold; it does not keep the balancing policy
running. `set_estop(True)` and `halt_robot()` latch damping. Ordinary zero
walking commands and command expiry keep balancing and do not clear faults.

`set_velocity_command(vx, vy, yaw_rate)` is also available for direct shell
control. It bypasses `MovementManager`, so do not use it alongside active
navigation/teleoperation. Repeat commands to sustain motion; they expire after
the same one-second timeout. Zero velocity means stop walking and keep balancing,
not disarm. Disarming, resetting, and completing the activation ramp clear old
walking commands; send fresh commands after activation.

Stopping the complete stack stops SONIC and latches damping. This uses the
existing `Module` base and does not depend on the new `ConnectionModule` or
ControlCoordinator refactor. `task_invoke`, `set_activated`, `execute_trajectory`
and `/g1/joint_command` are not part of the SONIC interface.

## Focused validation

```bash
uv run pytest dimos/robot/unitree/g1/test_sonic_policy.py \
  dimos/robot/unitree/g1/test_wholebody_connection.py dimos/control/sonic \
  -m 'not mujoco'
```

These exercise standalone command routing, expiry, re-arm/reset, fault latching,
motor-publisher safeguards, inference semantics and blueprint composition.
Models and hardware IO are mocked. The blueprint tests import optional simulation
dependencies but do not load assets or start hardware. They do not establish
physical walking stability or hardware safety.
