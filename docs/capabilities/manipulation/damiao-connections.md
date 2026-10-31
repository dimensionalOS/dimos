# Run YAM, OpenArm, or Metal through a connection module

The connection modules expose `position_command: In[JointState]` and
`joint_state: Out[JointState]`. The coordinator sends commands over streams,
while the connection owns the existing Damiao adapter and its CAN lifecycle.
The existing adapter-based blueprints and learning setups keep their current behavior.

## Start with the in-memory adapter

Run one of these blueprints without CAN selectors:

```bash
dimos run openyam-connection-control
dimos run openarm-connection-control
dimos run makermods-metal-connection-control
```

Each blueprint contains a coordinator, a connection, and `UrdfManager`.
The coordinator provides a trajectory task and one gripper task per gripper.
`--simulation mujoco` selects the in-memory adapter even when CAN selectors are supplied.
This adapter echoes targets as state. It does not simulate robot dynamics.

## Select physical CAN interfaces

Use the connection module's parameters to select existing, configured CAN interfaces:

```bash
dimos run openyam-connection-control --openyamconnection.can-port can0
dimos run makermods-metal-connection-control --metalconnection.can-port can0
dimos run openarm-connection-control --openarmconnection.left-can-port can1 --openarmconnection.right-can-port can0
```

OpenArm requires both selectors, and they must differ. On macOS, selectors
are USB serial numbers as supported by the existing adapter.
These modules do not configure interfaces, install drivers, or request sudo.
Startup retains the adapter's activation and gripper calibration behavior.
It does not add a home motion.

## Send joint and gripper targets

The new blueprints name their robot `arm`. Metal joints are `arm/joint1`
through `arm/joint6`. YAM joints are `arm/yam_joint1` through `arm/yam_joint6`.
OpenArm joints are `arm/openarm_left_joint1` through `arm/openarm_left_joint7`,
followed by the corresponding right joints.

Single-arm grippers are `arm/gripper`; OpenArm grippers are
`arm/left_gripper` and `arm/right_gripper`. Their values retain the adapter's
calibrated opening scale: 0 closed, 1 open. They are not distances or motor angles.
The control model adds empty links for these logical coordinates and keeps
physical finger geometry fixed. It does not animate the jaws.

Send named positions in `JointState.position`. Partial messages retain the
other targets, initially read from motor feedback. Velocity and torque
command ports are not exposed. The existing MIT gains, motor profiles,
current settings, and gravity compensation remain in the vendor adapter configuration.

The coordinator's single-arm gripper task is `arm_gripper` and consumes
`right_gripper_command`. OpenArm tasks `arm_left_gripper` and
`arm_right_gripper` consume the corresponding side's command stream.
The existing `task_invoke` RPC can call their `set_normalized` method.

## Stop the connection

The shared deadman calls `halt()` after commands stop. A halt samples the
measured pose and sends a position hold. Shutdown joins the state reader, sends a hold, disables the adapter,
and disconnects CAN. A feedback exception disables
the adapter and blocks further commands until a new connection session.

## Validate without hardware

Run the focused tests:

```bash
pytest dimos/robot/manipulators/common/test_damiao_connection.py
```

These tests use the real connection and coordinator with an in-memory motor
adapter. They cover command mapping, gains, limits, grippers, deadman handling,
estop, feedback faults, and cleanup. They do not establish hardware acceptance.
