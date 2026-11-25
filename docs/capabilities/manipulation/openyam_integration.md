# OpenYAM integration

OpenYAM and Dual OpenYAM use `ConnectionModule` implementations that own the
native `can-motor-control` robot. The coordinator receives `JointState` feedback
and sends position commands through streams. It does not construct hardware
adapters. `openyam-teach-collection` uses the same path with
`OpenYamConnection` in teach mode (`teach=True`): zero stiffness, teaching damping, gravity compensation
and passive grippers, declared as zero-stiffness joints. Seeed B601-DM also uses
its own direct connection.

## Run without hardware

Use `--simulation` to compose a separate `MockJointConnection`. Its initial
arm positions come from the planning model's home posture, or zero when none
is configured. Grippers start closed. Native startup requires CAN selectors.

```bash
dimos --simulation run coordinator-openyam
dimos --simulation run openyam-planner-coordinator
dimos --simulation run keyboard-teleop-openyam
dimos --simulation run keyboard-teleop-openyam-planner
dimos --simulation run teleop-webxr-openyam
dimos --simulation run coordinator-dual-openyam
dimos --simulation run dual-openyam-planner-coordinator
dimos --simulation run teleop-webxr-dual-openyam
```

`openyam-connection-control` exposes trajectory and normalized-gripper control.

## CAN transport

List the selectors accepted on the current host:

```bash
dimos hardware can list
```

Linux uses SocketCAN interface names. Configure each interface for classical
CAN at 1 Mbit/s before starting:

```bash
dimos hardware can setup can0
dimos hardware can setup can1
dimos run teleop-webxr-openyam --can-port can0
dimos run teleop-webxr-dual-openyam --left-can-port can0 --right-can-port can1
```

macOS uses native Rust IOKit access to gs_usb/candleLight adapters with USB ID
`1d50:606f`. Use their USB serial numbers as selectors. No PyUSB, libusb,
`python-can`, or Python `gs_usb` package is required. The native transport sets
the bitrate. Connect one adapter at a time, record its serial, and label its
side; adapters used together must have unique serial numbers.

```bash
dimos run teleop-webxr-openyam --can-port <USB-SERIAL>
dimos run teleop-webxr-dual-openyam \
  --left-can-port <LEFT-USB-SERIAL> \
  --right-can-port <RIGHT-USB-SERIAL>
```

Dual YAM requires both selectors, and they must differ. Blank selectors are
rejected before connection. `--simulation` composes a separate
`MockJointConnection` without starting a native CAN connection. Module-qualified overrides are also available,
for example `--openyamconnection.can-port can0` or
`--dualopenyamconnection.left-can-port can0`.

A selector starting with `/dev/` uses `DmSerialBus` on either Linux or macOS,
preserving the native Damiao serial USB-CAN transport.

## Models and joint names

The canonical planning model is the managed OpenYAM single-arm description from
`dimensionalOS/dim_description` (`openyam_single`, pinned by commit). Its arm
joints are mapped onto `yam_joint1` through `yam_joint6`. Shipped deployments
use connection source `arm`, with `arm/yam_joint1` through `arm/yam_joint6` and
`arm/joint7` for a single YAM. `joint7` is the managed model's driven finger
joint, in metres: 0.0594641 is closed and 0.01081 fully open, and `joint8`
copies it. The gripper task maps its 0.0 (closed) to 1.0 (open) command onto
that range. Dual YAM uses
`arm/left_joint1` through `arm/left_joint6`, `arm/right_joint1` through
`arm/right_joint6`, then `arm/left_gripper` and `arm/right_gripper`. Its URDF has
its finger joints fixed, so these two are openings from 0.0 closed to 1.0 open.

The deployed planner model, planning groups, task claims and recorded schema
all use those exact names. Link and target-frame names remain unchanged.
Single YAM retains a separate physical model for gravity compensation. Dual
YAM uses its arm-only combined model for planning and gravity compensation.

`OpenYamConnection()` and `DualOpenYamConnection()` default to sources `yam` and
`dual_openyam` when used directly. For a custom stack, declare the connection on
the coordinator with `connections=[arm_declaration(OpenYamConnection)]` or
`connections=[arm_declaration(DualOpenYamConnection)]`. The `arm_declaration` helper
in `dimos/robot/manipulators/damiao/damiao_blueprint.py` sets source `arm` for both.

Quest recordings, policy rollout and hand-guided teaching all use the deployed
seven-coordinate names. Hand-guided teaching keeps its zero-stiffness gains and
passive-jaw behavior. Its CAN override is `--can-port`, as for the other single-arm stacks.

Planner gripper routing is explicit through `ManipulationModuleConfig.gripper_tasks`:
single YAM binds `manipulator` to `arm_gripper`; dual YAM binds
`left_manipulator` and `right_manipulator` to their respective `*_arm_gripper`
tasks for both command dispatch and feedback.

## Teleoperation

Single YAM WebXR uses the right controller for arm pose and a separate gripper
task. Keyboard twist control and gripper control remain independent. Manual
control takes priority over a running planner trajectory.

Dual YAM has one pose task per arm. Each controller's grip engages only its own
arm; releasing it leaves the other arm's task independent. Each trigger controls
its matching gripper. Both arms share one connection, with one CAN bus per arm.

### Experimental tri-mode policy (right follower)

This opt-in blueprint keeps Viser planning and execute/cancel/status, the stock
OpenYAM SDK and calibrated jaw. The coordinator sends one of three typed
references to a dedicated connection worker: six ordered joint positions, base
frame `arm_tcp` pose, or base frame `arm_tcp` position. Only that worker runs the
50 Hz actor and submits arm commands. Grip takes over from a plan without
resuming it afterward; after a Quest input gap, release grip with both raw input
streams live before re-engaging. Trigger still operates the jaw.

**Operator only.** Launching this blueprint opens CAN, enables the arm, and may
move the jaw during the stock calibration. Use only a commissioned right OpenYAM
follower with a supported arm, a clear workspace and a tested physical stop.
Stop any other writer on that CAN bus first. Install the usual dimOS manipulation dependencies, including the
`cpu` extra and stock `can-motor-control==0.0.12`. Put your authorized copy of
`policy_trimode_dual_enc_10ms_13500.onnx` outside the checkout. No download or
older checkpoint is used. Verify it before launch:

```sh
MODEL="$HOME/.local/share/dimos/models/policy_trimode_dual_enc_10ms_13500.onnx"
printf '%s  %s\n' 8ac35d59dc94d04361286e7ac07679da2e623c60ea458d4ef6e0513482cea98f "$MODEL" | sha256sum -c -
```

From the reviewed checkout, with the right bus configured as `follower_r`, the
foreground command is:

```sh
dimos run openyam-trimode --can-port follower_r --model-path "$MODEL" --quest-mode pose
```

Use `--quest-mode position` for position-only input. Open the native Quest
HTTPS `/teleop` URL shown in startup logs and the ManipulationModule Viser URL.
The workers and their private control streams must be on one host and on an
isolated, trusted dimOS transport. Use the physical stop for commissioning;
software e-stop revokes policy and requests a measured hold, not a hardware
power cut. Support the arm before foreground shutdown, which disables torque.

Valid reference deadlines are monotonic and at most 100 ms ahead of issuance.
Normal jaw traffic never renews arm authority or resends arm MIT rows. The stock
command deadman and write-failure halt still revoke arm authority and request a
measured arm and jaw hold, even before the first policy write. If the hold
cannot be sampled or sent, the connection attempts to disable the device.
A completed plan or a fresh grip release revokes external authority and keeps
tracking the last submitted goal from fresh feedback until takeover. Cancellation,
expiry, faults and E-stop revoke policy and fall back to a measured hardware
hold when possible. Shutdown revokes policy before device teardown. A native
send already in progress cannot be undone. This host-side protection is **not**
an independent firmware watchdog and cannot protect against host death. SDK q/dq polls can contain cached
values: plan completion checks a subsequent SDK poll against the final target
and velocity tolerance, not a motor execution receipt or a verified per-axis
acquisition timestamp.

## Motor topology

| Motor | Send ID | Reply ID | Type |
|---|---|---|---|
| joint1-3 | `0x01`-`0x03` | send ID + `0x10` | DM4340 |
| joint4-6 | `0x04`-`0x06` | send ID + `0x10` | DM4310 |
| gripper | `0x08` | `0x18` | DM4310 |

Each arm uses the same IDs on its own bus. Gripper opening decreases motor
position. The connection retains the YAM MIT gains and gravity compensation.
Commands are position-only, and invalid target messages are rejected as a
whole. The connection holds measured positions when its command deadline expires.

Support the arm before stopping because deactivation removes torque. Verify
motor types, IDs, CAN-H/CAN-L wiring, power and termination before physical use.

## Verification

See the [Damiao connection test guide](/docs/capabilities/manipulation/damiao-connections.md)
for focused tests, software-only checks and physical bring-up. The YAM tests
cover deployed joint names, gripper routing, planner priority, Dual YAM hand
isolation, model limits and the Quest profile. The native recorder test verifies
Zenoh serialization and episode extraction with synthetic input streams.

- Dual OpenYAM leader teleop: [Dual OpenYAM Leader Teleop](/docs/capabilities/manipulation/dual_openyam_leader_teleop.md)
