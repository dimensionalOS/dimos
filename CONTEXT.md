# Dim Robot Control

This glossary defines the shared language for coordinating tasks, robot joints, and device connections.

## Language

**Robot**:
The single controllable unit presented to an upper-level caller. A robot may contain several arms, grippers, bases, or other parts.
_Avoid_: Robot ID, sub-robot

**Joint Resource**:
A canonical joint that one task may control at a time. Arbitration assigns the whole joint resource, regardless of which command interface the winner uses.
_Avoid_: Scalar resource, mode-specific resource

**Joint Claim**:
A task's request to control a set of joint resources at a stated priority. A claim establishes contention; arbitration establishes ownership.
_Avoid_: Interface claim, hardware claim

**Joint Owner**:
The task selected by arbitration to control a joint resource. The owner alone supplies command values for that joint.
_Avoid_: Interface owner

**Command Interface**:
A writable control quantity for a joint, such as position, velocity, effort, proportional gain, or derivative gain. A joint owner may select one or several supported command interfaces.
_Avoid_: Control mode, resource

**State Interface**:
A readable quantity describing a joint or another control-relevant signal. State interfaces may have many readers and are not arbitrated.

**Canonical Interface Name**:
The complete wire identity of one scalar interface, such as `left/j1/position`. Every `ControlValues` frame carries these names explicitly; a description validates them but does not replace them with implicit indices.
_Avoid_: Interface index, description slot

**Native Control Mode**:
A device-specific operating state derived from incoming command semantics. It belongs to the connection boundary and is not part of task or coordinator vocabulary.
_Avoid_: Global control mode

**Connection**:
The owner of device communication, native control behavior, conversion, limits, watchdogs, and safe stop for its declared joint resources.
_Avoid_: Coordinator hardware, adapter registry entry

**Module Readiness**:
A live framework-level indication that a module can safely accept responsibility for its declared function. Successful startup establishes readiness; a module can withdraw readiness after startup and provide a reason. Readiness is distinct from worker-process liveness and from whether one operation has completed instantaneously.
_Avoid_: Startup complete, command ready, connection ready

**Arm Preparation**:
The first phase of robot-wide arming. Each connection establishes its device-specific operating preconditions while the coordinator blocks ordinary task commands. Preparation may include bounded connection-owned motion and ends in a stable `PREPARED` state.
_Avoid_: Task arm, native control mode

**Arm Confirmation**:
The explicit commit of one prepared operation. It activates a new control epoch and opens the coordinator command gate only after every required connection acknowledges. Connections may declare direct activation or require operator confirmation.
_Avoid_: Enable motors, resume

**Safe Stop**:
A latched robot-level request that makes every connection stop task-directed motion through its own device-specific stable behavior. A safe stop may retain actuator power, position hold, damping, balance, or braking. It is distinct from connection shutdown and emergency stop.
_Avoid_: Stop publishing, zero everything, disable motors

**Safe-Stop Clearance**:
A robot-wide transaction that clears the safe-stop latch only after every required connection succeeds. Partial clearance rolls back to `SAFE_STOPPED`; it never arms the robot.
_Avoid_: Clear one connection, resume

**Task Cancellation**:
An ordinary end to one task's command production. It releases that task's joint claims and lets fresh armed heartbeats apply each connection's declared omission or idle policy. It does not clear or create a safety latch.
_Avoid_: Safe stop

**Standby**:
The operational connection state whose command gate is closed. A connection starts in standby and returns there after an acknowledged arm-preparation abort or successful safe-stop or emergency-stop clearance. Standby does not imply actuator power-off.
_Avoid_: Disarmed

**Emergency Stop**:
A latched emergency action using the strongest device-specific stop available. Clearing emergency stop never arms the robot or resumes commands.

**Control Epoch**:
The identifier created by a successful robot arm transaction. Connections accept commands only while armed and only from the current epoch; safe stop and emergency stop invalidate it.

**Process-Loss Protection**:
The verified mechanism that puts a physical device into its declared safe state when its connection process or native command link disappears. A connection classifies it as a native watchdog, external supervisor, intrinsically safe behavior, or unprotected. Production arming rejects unprotected physical connections.
_Avoid_: Software watchdog when the same process can die
