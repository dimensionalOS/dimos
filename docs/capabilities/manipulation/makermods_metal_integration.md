# MakerMods Metal connections

Single and dual Metal use direct connection modules. The coordinator sends
position commands over streams; each native connection owns its CAN buses,
arm views, calibrated grippers, and shutdown. CAN selectors are required
strings. Mock operation uses a separate module selected by `--simulation`.

## Run without hardware

```bash
uv run dimos --simulation run makermods-metal-planner-coordinator
uv run dimos --simulation run dual-makermods-metal-planner-coordinator
```

The existing main entry names are retained: `coordinator-makermods-metal`,
`makermods-metal-planner-coordinator`, and
`dual-makermods-metal-planner-coordinator`. Single-arm trajectory and gripper
control without a planner is also available as `makermods-metal-connection-control`.

Mock feedback echoes accepted joint positions. It does not simulate dynamics,
gripper contact, or calibration. See the [connection test runbook](/docs/capabilities/manipulation/damiao-connections.md)
for repository setup, command checks, and deadman checks.

## Select native hardware

Configure the existing CAN interfaces before starting the connection. Use a
SocketCAN interface name on Linux, a gs_usb serial number on macOS, or a
`/dev/...` Damiao serial USB-CAN bridge on either platform.

```bash
uv run dimos run makermods-metal-planner-coordinator --can-port can0
uv run dimos run dual-makermods-metal-planner-coordinator \
  --left-can-port can0 --right-can-port can1
```

Dual Metal requires two distinct, nonempty selectors. Missing selectors fail
configuration validation. Physical startup enables the motors and calibrates
both gripper endpoints before publishing readiness or state.

Both arms use the single-Metal profiles, addresses, MIT gains, and gripper
settings. Motor IDs are 1 through 6 and gripper ID 7 on each separate bus.
Gripper direction is increasing position, with current 0.1. Gravity compensation
remains disabled, as in the main setup.

## Configure the mounting geometry

The model reuses main's fixed attachments of the pinned official Metal model.
Its default bench layout puts the left base at `(0, 0.3, 0)` and the right base
at `(0, -0.3, 0)`, with zero rotation. These are example transforms, not a
measurement of your rig. Set the actual mounting transforms before planning.

```bash
uv run dimos run dual-makermods-metal-planner-coordinator \
  --left-can-port can0 --right-can-port can1 \
  --left-base-xyz '[0,0.25,0]' --right-base-xyz '[0,-0.25,0]' \
  --left-base-rpy '[0,0,0]' --right-base-rpy '[0,0,0]'
```

XYZ values are meters; RPY values are radians. Both arm trees share one collision
world, with the `left_arm` and `right_arm` planning groups. Deployed arm joints
are `arm/left_joint1` through `arm/left_joint6`, and the corresponding right
names. Each jaw is its URDF finger joint, `arm/left_joint7` and
`arm/right_joint7`, in metres: 0 is closed and -0.05 fully open. `joint8` copies
it.

The coordinator retains the tasks `left_arm_gripper` and `right_arm_gripper`.
The grippers can be commanded independently through those tasks or their
left/right gripper streams. `ManipulationModuleConfig.gripper_tasks` explicitly
binds `left_arm` and `right_arm` to those task names for both commands and
feedback. The single-Metal planner binds `arm` to `metal_gripper`.

## Quest teleop and collection

`teleop-webxr-metal` drives the arm from the right Quest controller: hold the
grip to engage, the trigger closes the jaw, B plans back to the zero pose. The
IK target is `gripper_base`, the jaw mount; the URDF has no fingertip frame.
`metal-quest-collection` adds the episode monitor, a wrist webcam and the
recorder. It records `observation.images.wrist` (480x640 RGB),
`observation.state` from `coordinator_joint_state` and `action` from
`coordinator_joint_command`, both over `arm/joint1` to `arm/joint6` and
`arm/joint7`.

```bash
dimos run teleop-webxr-metal --can-port can0
dimos run metal-quest-collection --can-port can0 \
  --recorder.recording recordings/session-001 \
  --episodes.task "pick up the cube"
dimos imitation control
```

`--wrist.hardware.camera-index /dev/video2` selects another camera.
`--simulation` swaps the connection for the echo mock; the camera is still a
real device. There is no hand-guided teaching entry: the Metal runs without
gravity compensation, so a zero-stiffness arm drops.

## Bimanual Quest teleop and collection

`teleop-webxr-dual-metal` drives both arms, the left controller on the left
arm and the right on the right, each targeting its own `gripper_base`; grip,
trigger and B work as on one Metal. It takes the dual planner's mount flags.
`dual-metal-quest-collection` adds the episode monitor, two wrist webcams and
the recorder. It records `observation.images.left_wrist` and
`observation.images.right_wrist` (480x640 RGB), `observation.state` and
`action` over `arm/left_joint1` to `arm/left_joint7`, then `arm/right_joint1`
to `arm/right_joint7`.

```bash
dimos run teleop-webxr-dual-metal --left-can-port can0 --right-can-port can1
dimos run dual-metal-quest-collection --left-can-port can0 --right-can-port can1 \
  --left-wrist.hardware.camera-index 0 --right-wrist.hardware.camera-index 1 \
  --recorder.recording recordings/session-001 \
  --episodes.task "hand over the cube"
dimos imitation control
```

## Validation scope

Software tests cover required selectors, mounted models, independent gripper
commands, SDK motor declarations and payloads, startup readiness, rollback,
and shutdown with fake native devices. Hardware acceptance and measured mount
calibration remain separate.
