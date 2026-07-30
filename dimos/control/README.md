# Control Coordinator

Centralized control system for multi-arm robots with per-joint arbitration.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   ControlCoordinator                        │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐   │
│  │                    TickLoop (100Hz)                  │   │
│  │                                                      │   │
│  │   READ ──► COMPUTE ──► ARBITRATE ──► ROUTE ──► WRITE │   │
│  └──────────────────────────────────────────────────────┘   │
│         │           │           │              │            │
│         ▼           ▼           ▼              ▼            │
│    ┌─────────┐  ┌───────┐  ┌─────────┐   ┌──────────┐       │
│    │Connected│  │ Tasks │  │Priority │   │ Adapters │       │
│    │Hardware │  │       │  │ Winners │   │          │       │
│    └─────────┘  └───────┘  └─────────┘   └──────────┘       │
└─────────────────────────────────────────────────────────────┘
```

## Quick Start

```bash
# Terminal 1: Run coordinator
dimos run coordinator-mock          # Single 7-DOF mock arm
dimos run coordinator-dual-mock     # Dual arms (7+6 DOF)
dimos run coordinator-piper-xarm    # Real hardware

# Terminal 2: Control via CLI
python -m dimos.manipulation.control.coordinator_client
```

## Core Concepts

### Tick Loop
Single deterministic loop at 100Hz:
1. **Read** - Get joint positions from all hardware
2. **Compute** - Each task calculates desired output
3. **Arbitrate** - Per-joint, highest priority wins
4. **Route** - Group commands by hardware
5. **Write** - Send commands to adapters

### Tasks (Controllers)
Tasks are passive controllers called by the coordinator:

```python
from dimos.control.task import BaseControlTask

class MyController(BaseControlTask):
    def claim(self) -> ResourceClaim:
        return ResourceClaim(joints={"joint1", "joint2"}, priority=10)

    def compute(self, state: CoordinatorState) -> JointCommandOutput:
        # Your control law here (PID, impedance, etc.)
        return JointCommandOutput(
            joint_names=["joint1", "joint2"],
            positions=[0.5, 0.3],
            mode=ControlMode.POSITION,
        )
```

Registering a `BaseControlTask` grants it a frozen, stateless
`ControlTaskContext` for the lifetime of that registration. A command handler
can transparently call `self.context.get_state()` to read the coordinator's
latest complete tick observation. Removing the task revokes access; stopping
and restarting the coordinator preserves it. The read returns `None` before
the first tick and after stop or runtime reset; it does not perform a hardware
read or a freshness check. `CoordinatorState`, `JointStateSnapshot`, and their
mappings are read-only.

### Priority & Arbitration
Higher priority always wins. Arbitration happens every tick:

```
joint_trajectory (priority=10) wants joint1 = 0.5
safety           (priority=100) wants joint1 = 0.0
                              ↓
                    safety wins, joint_trajectory preempted
```

### Preemption
When a task loses a joint to higher priority, it gets notified:

```python
def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
    self._state = TrajectoryState.PREEMPTED
```

## Files

```
dimos/control/
├── coordinator.py       # Module + RPC interface
├── tick_loop.py         # 100Hz control loop
├── task.py              # ControlTask protocol + types
├── hardware_interface.py # ConnectedHardware wrapper
├── components.py        # HardwareComponent config + type aliases
├── blueprints.py        # Pre-configured setups
└── tasks/
    └── trajectory_task.py  # Joint trajectory controller
```

## Configuration

```python
from dimos.control import ControlCoordinator, HardwareComponent, TaskConfig

my_robot = ControlCoordinator.blueprint(
    tick_rate=100.0,
    hardware=[
        HardwareComponent(
            hardware_id="left_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("left_arm", 7),
            adapter_type="xarm",
            address="192.168.1.100",
        ),
        HardwareComponent(
            hardware_id="right_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("right_arm", 6),
            adapter_type="piper",
            address="can0",
        ),
    ],
    tasks=[
        TaskConfig(
            name="joint_trajectory",
            type="trajectory",
            joint_names=[...],  # union of both arms
            priority=10,
            params={"start_position_tolerance": 0.05},
        ),
    ],
)
```

## RPC Methods

| Method | Description |
|--------|-------------|
| `list_hardware()` | List hardware IDs |
| `list_joints()` | List all joint names |
| `list_tasks()` | List task names |
| `get_joint_positions()` | Get current positions |
| `task_invoke(task, command, kwargs)` | Invoke a command declared by a task card |

Trajectory execution is an optional task capability, not a coordinator RPC.
The trajectory package enforces the canonical task name `joint_trajectory`:

```python
result = coordinator.task_invoke(
    "joint_trajectory",
    "execute",
    {"trajectory": trajectory},
)
coordinator.task_invoke("joint_trajectory", "cancel")
```

`TASK_EXPOSES` in a task package's `_registry.py` is a strict remote-command
allowlist. Missing tasks raise an error, undeclared commands raise
`AttributeError`, invalid arguments raise `TypeError` before the handler runs,
and handler results or exceptions pass through unchanged.

## Control Modes

Tasks output commands in one of three modes:

| Mode | Output | Use Case |
|------|--------|----------|
| POSITION | `q` | Trajectory following |
| VELOCITY | `q_dot` | Joystick teleoperation |
| TORQUE | `tau` | Force control, impedance |

## Writing a Custom Task

```python
from dimos.control.task import BaseControlTask, ResourceClaim, JointCommandOutput, ControlMode

class PIDController(BaseControlTask):
    def __init__(self, joints: list[str], priority: int = 10):
        self._name = "pid_controller"
        self._claim = ResourceClaim(joints=frozenset(joints), priority=priority)
        self._joints = joints
        self.Kp, self.Ki, self.Kd = 10.0, 0.1, 1.0
        self._integral = [0.0] * len(joints)
        self._last_error = [0.0] * len(joints)
        self.target = [0.0] * len(joints)

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return self._claim

    def is_active(self) -> bool:
        return True

    def compute(self, state) -> JointCommandOutput:
        positions = [state.joints.joint_positions[j] for j in self._joints]
        error = [t - p for t, p in zip(self.target, positions)]

        # PID
        self._integral = [i + e * state.dt for i, e in zip(self._integral, error)]
        derivative = [(e - le) / state.dt for e, le in zip(error, self._last_error)]
        output = [self.Kp*e + self.Ki*i + self.Kd*d
                  for e, i, d in zip(error, self._integral, derivative)]
        self._last_error = error

        return JointCommandOutput(
            joint_names=self._joints,
            positions=output,
            mode=ControlMode.POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        pass  # Handle preemption
```

## Joint State Output

The coordinator publishes one aggregated `JointState` message containing all joints:

```python
JointState(
    name=["left_arm_joint1", ..., "right_arm_joint1", ...],  # All joints
    position=[...],
    velocity=[...],
    effort=[...],
)
```

Subscribe via: `/coordinator_joint_state`
