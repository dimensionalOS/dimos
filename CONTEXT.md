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

## Recording

**Source observation**:
An observation whose publisher call completed successfully.
_Avoid_: Frame, sent message

**Received observation**:
A source observation emitted by the Recorder's input transport before Recorder scheduling or storage.
_Avoid_: Recorded observation

**Persisted observation**:
An observation committed to the recording and readable through its configured codec.
_Avoid_: Received observation, saved frame

**Recording fidelity**:
Exact correspondence between source observations and persisted observations, including membership, order, timestamps, and codec-defined payload content.
_Avoid_: Frame rate, throughput

**Recorder fidelity**:
Exact correspondence between received observations and persisted observations. Recording fidelity also includes the transport path before the Recorder.
_Avoid_: Recording fidelity

**Shared loss window**:
A source-time interval in which every recorded data stream is missing observations.
_Avoid_: Freeze, lag spike

**Tail loss**:
Observations lost between the start of graceful shutdown and the recording's final commit.
_Avoid_: Shared loss window
