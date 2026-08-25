# DimOS Robotics

Canonical language for robot capabilities and operator interaction in DimOS.

## Teleoperation

**Quest arm teleoperation**:
Arm teleoperation in which tracked Quest controllers provide clutched, relative Cartesian pose commands and gripper input.
_Avoid_: Keyboard teleoperation, leader-follower teleoperation

**Operator hand**:
The left or right Quest controller as a source of teleoperation intent, independent of the robot arm it controls.
_Avoid_: Hand, side

**Mixed-arm setup**:
Two independent manipulators teleoperated through separate control tasks, even when presented in one operator session.
_Avoid_: Bimanual robot, dual-arm robot

**Bimanual robot**:
One coupled robot with two manipulator groups represented by a single kinematic model and controlled by one bimanual task.
_Avoid_: Mixed-arm setup

**Bimanual engagement**:
A two-hand deadman condition in which a bimanual teleoperation task is active only while both operator hands are engaged; releasing either hand disengages the whole task.
_Avoid_: Partial engagement, independent hand engagement

## Joint-limit safety

**Feedback limit tolerance**:
The bounded discrepancy beyond a nominal joint limit that is accepted only when interpreting measured hardware state.
_Avoid_: Command tolerance, expanded joint limit

**Command limit margin**:
The inward distance from each nominal joint limit within which generated position commands must remain.
_Avoid_: Feedback tolerance, relaxed joint limit

## Inverse kinematics

**Pink task stack**:
The ordered, named set of kinematic objectives used by Pink to produce a robot command. Its structure is composed once per IK control context; reserved frame objectives ensure every commanded end effector participates, while subclasses may compose or replace named auxiliary objectives.
_Avoid_: Teleoperation behavior, solver implementation

**IK control context**:
The persistent inverse-kinematics state owned by one control-task instance for one robot model, controlled-joint selection, and target-frame selection, including its Pink task stack. Stateful Pink tasks are never shared between control-task instances.
_Avoid_: Planning group, teleoperation session

## Robot connections

**Connection module**:
The boundary that owns access to a real or simulated robot and presents the same command and observation contract to the rest of DimOS.
_Avoid_: Simulator adapter, SHM adapter, transport-specific connection

**Held motor command**:
The latest complete `MotorCommandArray`, atomically replaced when a command arrives and applied by the connection module on every control or physics step until superseded. The held command is ordinary module state, not an IPC mechanism.
_Avoid_: One-shot command, SHM command slot

**Latest motor observation**:
The most recent complete `JointState` published by a connection module. Producers publish at their native control or physics rate, consumers atomically replace their local snapshot, and slow consumers do not backpressure or accumulate observations.
_Avoid_: Motor-state queue, lossless state history

**Port-backed whole-body adapter**:
A transport-agnostic adapter constructed by a `ControlCoordinator` subclass from that module's typed command and observation ports. It preserves the coordinator's synchronous `WholeBodyAdapter` interface while leaving stream wiring and transport selection to the blueprint.
_Avoid_: Zenoh adapter, LCM adapter, topic-owning adapter
