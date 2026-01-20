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
class MyController:
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

### Priority & Arbitration
Higher priority always wins. Arbitration happens every tick:

```
traj_arm (priority=10) wants joint1 = 0.5
safety   (priority=100) wants joint1 = 0.0
                              ↓
                    safety wins, traj_arm preempted
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
from dimos.control import control_coordinator, HardwareComponent, TaskConfig

my_robot = control_coordinator(
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
        TaskConfig(name="traj_left", type="trajectory", joint_names=[...], priority=10),
        TaskConfig(name="traj_right", type="trajectory", joint_names=[...], priority=10),
        TaskConfig(name="safety", type="trajectory", joint_names=[...], priority=100),
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
| `execute_trajectory(task, traj)` | Execute trajectory |
| `get_trajectory_status(task)` | Get task status |
| `cancel_trajectory(task)` | Cancel active trajectory |

## Control Modes

Tasks output commands in one of three modes:

| Mode | Output | Use Case |
|------|--------|----------|
| POSITION | `q` | Trajectory following |
| VELOCITY | `q_dot` | Joystick teleoperation |
| TORQUE | `tau` | Force control, impedance |

## Writing a Custom Task

```python
from dimos.control.task import ControlTask, ResourceClaim, JointCommandOutput, ControlMode

class PIDController:
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

Subscribe via: `/coordinator/joint_state`
