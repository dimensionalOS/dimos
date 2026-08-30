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
├── routing.py           # Routing rules + card types
├── hardware_interface.py # ConnectedHardware wrapper
├── components.py        # HardwareComponent config + type aliases
├── blueprints/          # Pre-configured setups
└── tasks/
    ├── registry.py         # Task-card discovery + lazy factories
    └── trajectory_task/    # Joint trajectory controller
```

## Configuration

```python
from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import ControlCoordinator
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task

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
        joint_trajectory_task([...]),  # union of both arms
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
| `execute_trajectory(traj)` | Execute through the sole trajectory task |
| `cancel_trajectory()` | Cancel the sole trajectory task |

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

## Task Cards

Each task type ships a manifest at `dimos/control/tasks/<task>/_registry.py`.
`tasks/registry.py` discovers these without importing the tasks, so heavy deps
(Pinocchio, ONNX Runtime) load only when a task is actually created.

```python
TASK_FACTORIES = {
    "trajectory": "dimos.control.tasks.trajectory_task.trajectory_task:create_task"
}
TASK_CONSUMES = {"trajectory": {"joint_command": ("on_joint_command", "claim_overlap")}}
TASK_EXPOSES = {"trajectory": ["execute", "cancel", "get_state", "get_status"]}
```

`TASK_CONSUMES` maps a coordinator input to `(handler, routing rule)`. The
handler gets the raw message; the coordinator only routes.

`TASK_EXPOSES` lists what `task_invoke` may call. The method's own signature is
the argument schema, so `task_invoke` binds kwargs against it and a typo raises
back to the caller. `describe_task()` reports those signatures live.

### Routing rules

| Rule | Delivers when | Used by |
|------|---------------|---------|
| `claim_overlap` | the message names a joint the task currently claims | `joint_command` |
| `broadcast` | always, to every task on the port | `teleop_buttons`, `gripper_command` |
| `direct` | always, but the port is meant for one task (a second logs a warning) | `path`, `speed`, `cartesian_command`, `ee_twist_command` |

Addressing is topology: which task gets a message is decided by which port it
reads (wired once at startup), never by message content. Multi-instance
deployments give each instance its own port via `stream_bind`.

## Deployment I/O

`ControlCoordinator` declares the shared streams (`joint_command`,
`twist_command`, `teleop_buttons`, `gripper_command`). Per-instance inputs —
`cartesian_command`, `ee_twist_command` — are deployment I/O: a deployment
subclasses the coordinator and annotates one port per consuming task instance:

`gripper_command` carries a normalized `Float32` opening: `0.0` is fully
closed and `1.0` is fully open. Input devices translate buttons and triggers
to this scale before publishing.

```python
class _Go2Coordinator(PathFollowingCoordinator):
    go2_joints: Out[JointState]

blueprint = _Go2Coordinator.blueprint(
    instance_name="ControlCoordinator",  # RPC clients look the coordinator up by class name
    publish_robot_joint_states=True,
)
```

Card-bound ports are checked against the live instance at startup, not the
registry, since a subclass can declare ports the registry never sees. A missing
port fails `add_task()` with the annotation to add, before any route registers.

`TaskConfig.stream_bind` remaps a card input per instance, so two tasks of the
same type can read different ports (task-level remapping, ROS sense). The
dual-arm teleop coordinator declares `left_cartesian` / `right_cartesian` and
binds each arm's `teleop_ik` task to its side:

```python
TaskConfig(name="teleop_xarm", type="teleop_ik",
           stream_bind={"cartesian_command": "left_cartesian"})
```

## Joint State Views

The coordinator publishes two views of the same per-tick read. Both are
permanent; neither is a downgrade from the other.

| View | Stream | Carries | Enabled by |
|------|--------|---------|------------|
| Aggregate | `coordinator_joint_state` | every joint, `frame_id="coordinator"` | `publish_joint_state` (on by default) |
| Per-robot | `{hardware_id}_joints` | one robot, `frame_id=hardware_id` | `publish_robot_joint_states` plus a matching `Out[JointState]` annotation |

Both carry canonical `{hardware_id}/{joint}` names sampled once per tick, so
they line up with the commands sent on that tick. That is the *control* view.
Connection modules (`GO2Connection`, `G1WholeBodyConnection`) publish the
*device* view instead: raw state at the device's own rate, units and ordering.

Choose by what the consumer is, not by how many robots the deployment has.

Consumers that capture or model whatever is present read the aggregate:
`CollectionRecorder`, `WorldBeliefRecorder`, `ManipulationModule`.

Consumers about one named robot read that robot's stream: `scripts/g1_replay.py`
reads `/g1/joints`, and the go2 controller blueprints pin `go2_joints`.
