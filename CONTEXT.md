# DimOS Module Runtime

This context names the parts that let DimOS compose modules while their work runs in different processes or environments.

## Language

**Module**:
A typed, independently managed capability that participates in a Blueprint through streams and RPCs.
_Avoid_: Service, node

**Native Module**:
A Module whose computation runs in a managed external process while DimOS retains its composition and lifecycle identity.
_Avoid_: Foreign module, binary wrapper

**Python-Native Module**:
A Native Module whose concrete implementation runs in an isolated Python environment.
_Avoid_: External Python module, external-python deployment

**Contract**:
The host-visible Module class that defines a Python-Native Module's streams, configuration, RPCs, and skills.
_Avoid_: Declaration, interface module

**Runtime Subclass**:
The concrete subclass that implements a Contract inside its isolated Python environment.
_Avoid_: Worker class, external implementation

**Internal RPC Endpoint**:
The private identity through which a Contract reaches its Runtime Subclass without changing the Module's public identity.
_Avoid_: Public child RPC, socket bridge

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
