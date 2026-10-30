# OpenYAM integration

OpenYAM uses the generic Damiao whole-body adapter backed by
`can-motor-control`. The adapter owns one upstream robot containing the six-joint
arm and its gripper; motor topology is independent of the host operating system.

## CAN transport

The generic Damiao layer selects the native transport provided by
`can-motor-control>=0.0.8`:

| Host | Default | `runtime_config.bus_devices["openyam"]` override |
|------|---------|-----------------------|
| Linux | `SocketCanBus("can0")` | SocketCAN interface, such as `can1` |
| macOS | first `1d50:606f` gs_usb adapter | USB serial number |

The macOS transport is implemented in Rust with native IOKit access. It does
not require PyUSB, libusb, `python-can`, or the Python `gs_usb` package.

Logical buses are ordered. Without explicit selectors, a future two-bus Damiao
robot maps to `can0`/`can1` on Linux and gs_usb indices 0/1 on macOS. Production
multi-arm deployments should configure USB serial numbers because enumeration
order is not a stable device identity.

List the selectors accepted on the current host:

```bash
dimos hardware can list
```

On Linux this prints SocketCAN network-interface names. Configure each physical
interface before use:

```bash
dimos hardware can setup can0
dimos hardware can setup can1
```

On macOS, connect a `gs_usb`/candleLight-compatible adapter with USB ID
`1d50:606f` for each arm. No SocketCAN interface or extra USB library is needed.
The motor buses still require CAN-H/CAN-L wiring, motor power, and correct bus
termination. To assign sides reliably, connect one adapter at a time, run the
list command, record and label its USB serial, then repeat for the other arm.
Adapters used together must expose unique serial numbers.

## Run

Use the default CAN device:

```bash
dimos run coordinator-openyam
```

Select a Linux SocketCAN interface or a macOS adapter serial in a blueprint of your own with `openyam_hardware(bus_devices={"openyam": "can1"})`. The device lives in the adapter's `runtime_config`, not in `controlcoordinator.addresses`; see [Hardware configuration](/docs/usage/configuration.md#hardware-configuration).

The dual-arm WebXR blueprint is identical on both operating systems; only the
selector values differ:

```bash
# Linux
dimos run teleop-webxr-dual-openyam --left-can-port can0 --right-can-port can1

# macOS
dimos run teleop-webxr-dual-openyam \
  --left-can-port <LEFT-USB-SERIAL> \
  --right-can-port <RIGHT-USB-SERIAL>
```

Linux interfaces must already be configured for classical CAN at 1 Mbit/s.
The native macOS transport configures the adapter bitrate directly.

## Hardware topology

| Motor | Send ID | Reply ID | Type |
|-------|---------|----------|------|
| joint1–3 | `0x01`–`0x03` | send ID + `0x10` | DM4340 |
| joint4–6 | `0x04`–`0x06` | send ID + `0x10` | DM4310 |
| gripper | `0x08` | `0x18` | DM4310 |

The bus uses classical CAN at 1 Mbit/s. Opening the gripper decreases motor
position. The arm and gripper are exposed together as one whole-body hardware
component.

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
otherwise pin the arm at its start pose, and the filter padding is 4 cm, not
1, since a 2.5 cm table cell next to the post or an arm base straddles that
static link unless the returns within a cell of it are gone, on top of this
camera's 1 to 2 cm of slop (scene 56 replays, 2026-10-29). 1 cm cells were
tried and made the planner's world too heavy on this machine. The
place target is a taped tray square on the table on the arm's side (0.26,
-0.20 for the right arm, 0.26, 0.20 for the left, tcp released at z 0.05):
the pick-and-place module places top-down with the grasp orientation and a
pre-place 0.05 above the target, and the arm's fingers-down reach ends near
z 0.16 at the bin's x, so the bin rim cannot be cleared on this stack.

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

`dual_openyam_workcell_v6`, exported as `DUAL_OPENYAM_MODEL`, is the physical
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
put the arm on its own image at every grasp (scene 56 replay, 2026-10-29). So
the grasp stacks register the scene through `SelfClearedSceneRegistrationModule`
(robot folder): the registration module with the robot's returns removed from
the scene cloud by main's `RoboPlanBodyFilter`, the filter the planner's map
chain already runs, padded 2 cm for this camera. v6 dropped the table slab from
the model, which the same replay found swallowing the camera's table points.
A revised model ships under a new package name, because LFS extraction caches
by directory name.

## Safety

- Keep the workspace clear and the emergency stop reachable during first
  activation.
- Verify motor IDs and types against the physical unit before commanding large
  motions; an incorrect type scales velocity and torque incorrectly.
- Deactivation removes motor torque, so support the arm before stopping it.
