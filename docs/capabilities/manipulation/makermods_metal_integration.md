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

## Validation scope

Software tests cover required selectors, mounted models, independent gripper
commands, SDK motor declarations and payloads, startup readiness, rollback,
and shutdown with fake native devices. Hardware acceptance and measured mount
calibration remain separate.
