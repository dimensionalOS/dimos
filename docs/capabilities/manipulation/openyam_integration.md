# OpenYAM integration

OpenYAM and Dual OpenYAM use `ConnectionModule` implementations that own the
native `can-motor-control` robot. The coordinator receives `JointState` feedback
and sends position commands through streams. It does not construct hardware
adapters. `openyam-teach-collection` uses the same path with
`OpenYamTeachConnection`: zero stiffness, teaching damping, gravity compensation
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
`dual_openyam` when used directly. For a custom stack, the coordinator's model is
`make_openyam_model_config(robot="arm").model` or
`dual_openyam_control_model(robot="arm")`.

New Quest recordings and policy rollout use the deployed seven-coordinate names.
Hand-guided teaching retains its original bare names, PD gains and passive-jaw
behavior; its CAN override remains `--controlcoordinator.can-port`.

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

## Pick-and-place benchmark

The xArm grasp stack on the workcell, both grippers bound to their planning groups.

The three D405 serials are rig configuration, not source. Put them in a
config file outside the checkout and pass it with `--config`:

```json
{
  "realsensecamera": {"serial_number": "<overhead D405 serial>"},
  "left_wrist_camera": {"serial_number": "<left wrist D405 serial>"},
  "right_wrist_camera": {"serial_number": "<right wrist D405 serial>"}
}
```

```bash
dimos run dual-openyam-grasp-agent --config ~/workcell.json --left-can-port follower_l --right-can-port follower_r
dimos humancli      # second terminal: "scan for a soup can", "pick up the soup can", "put it in the tray"
```

`dual-openyam-grasp` is the same stack without the agent. Without the config
file each camera module opens the first D405 it finds, so set it before the
first run on a new host.

Every skill takes a planning group, `left_manipulator` or `right_manipulator`;
the agent picks the arm on the object's side and the manipulation module routes
gripper commands and readback to that group's gripper task.
Every run records joint states, the three cameras and TF to sqlite.

What the stack does not have on the workcell, and therefore what the benchmark
measures as is: grasps are one top-down heuristic candidate per object; a pick
executes as soon as it is planned, so the operator keeps a hand on the e-stop;
grasp success is judged from the jaw readback alone. The planner knows the
arms, the wrist cameras, the post and the overhead camera from the workcell
model; the table, the bin and everything else on the bench reach it from the
overhead D405 through the same self filter, voxel map and box filter chain the
xArm stack runs (`manipulation_mapping`). Two settings differ from the xArm
because its wrist camera never sees the rest of the robot while the overhead
camera sees all of it: the self filter model is the whole rig with the fingers
fixed, not the arm alone, since the grippers' and wrist cameras' own cells
otherwise pin the arm at its start pose, and the filter padding is 8 cm, not
1: a 2.5 cm table cell next to the post or an arm base straddles that static
link unless the returns within a cell of it are gone, on top of this camera's
1 to 2 cm of slop, and the wrist cameras' plugs and cables are not in the
model, so returns off them during a descent otherwise become cells the lifted
arm sits against. The planner is blind to anything within that padding of an
arm link; a modelled cable box would let it drop to 4 cm. 1 cm cells were
tried and made the planner's world too heavy on this machine. The place
target is a taped square on the table on the arm's side (0.26, -0.38 for the
right arm, 0.26, 0.38 for the left, tcp released at z 0.08): the pick-and-place
module places top-down with the grasp orientation and a pre-place 0.05 above
the target, and the arm's fingers-down reach ends near z 0.16 at the bin's x,
so the bin rim cannot be cleared on this stack.

### Rebuilding a scene

The grasp stacks carry a scene overlay module, pass-through until a scene is named. Point it at
the folder of saved overhead captures and the Rerun "Scene overlay" tile shows the live image
with that capture blended over it, so objects can be put back where they were:

```bash
dimos run dual-openyam-grasp-agent --sceneoverlaymodule.scenes-dir ~/scenes --sceneoverlaymodule.scene Scene_10
```

From `dimos shell`, `SceneOverlayModule.set_scene("Scene_25")` switches scenes,
`capture_reference("Scene_25")` saves the current frame as one, `set_opacity(0.5)` and
`set_mode("edges")` change the drawing, and after `capture_background()` of the empty table
`get_alignment()` scores how well each object sits on its ghost.

## Model assets

`dual_openyam_workcell_v7`, exported as `DUAL_OPENYAM_MODEL`, is the physical
workcell: the i2rt YAM arm geometry and meshes, the two arm bases and the
joint 3 zero offsets from the rig calibration of 2026-10-15, the link masses
and centres of mass from the gravity identification of 2026-10-16 (the Damiao
adapter's gravity feed-forward is built from this model), joint 1 limits
inside the measured stops, a convex hull collision mesh on every link, boxes
for each wrist D405 and bracket, and a fixed `workcell` link with the camera
post and the overhead camera. No table: like every model on main, the planner
takes the table from the camera. Its zero configuration is the folded rest pose
on the supports. The hardware gains in `config.py` are the ones chosen by the
step sweeps of the same calibration. The package's `SOURCE.md` records where
every number comes from. v5 restored the i2rt model's prismatic fingertip joints
(zero closed, 47 mm open each) so the pick's grasp contact check can open and
close the fingers; the planner lists the arm joints only and keeps them closed.
The model's `GripperConfig` (fingertip links as the pads) and `robot=` on
`PickAndPlaceModule` turn the grasp contact check on, and the GraspGenX stack
hands it all 500 samples, as the xArm stack does. One thing differs from the
xArm: its wrist camera never sees the arm, while the overhead D405 sees the
arms' bases and links, and the raw depth frame the check reads as its scene
put the arm on its own image at every grasp. So
the grasp stacks register the scene through `SelfClearedSceneRegistrationModule`
(robot folder): the registration module with the robot's returns removed from
the scene cloud by main's `RoboPlanBodyFilter`, the filter the planner's map
chain already runs, padded 2 cm for this camera. v6 dropped the table slab from
the model, which the same replay found swallowing the camera's table points.
v7 widens the finger stroke to the measured jaws: calipers read 98 mm between
the nose faces at full open where the i2rt travel gave 93.9, and the check had
been denying that 2 mm per side to every jaw-width object. A revised model
ships under a new package name, because LFS extraction caches by directory name.

## Safety
