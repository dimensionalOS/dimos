# External Robot Packages

This guide shows how an installed package can provide the ControlCoordinator pieces its blueprints reference without putting adapter or task files inside the DimOS source tree.

Use this path when you are building a robot package outside the DimOS repository, such as `dimos_mydog`, `dimos_myarm`, or `dimos_myhumanoid`.

## What this supports

This guide covers explicit registration for:

- hardware adapters used by `HardwareComponent(adapter_type=...)`;
- control tasks used by `TaskConfig(type=...)`.

It does not add package auto-discovery, Python entry point discovery, blueprint discovery changes, template generation, or `available_*` diagnostics helpers. Your package still needs to be imported by user code or by a separate blueprint-discovery mechanism before its registered names can be used.

## Package layout

A small external robot package usually has this shape:

```text
dimos_mydog/
├── adapters.py
├── blueprints.py        # calls register_extensions()
├── tasks/
│   └── gait.py          # lazy control task factory
├── skills.py            # optional
└── prompts.py           # optional
```

The important part is that the module defining the blueprint also performs extension registration before the blueprint is built.

For a runnable no-hardware version of this pattern, see `examples/external_control_extension/`:

```bash
uv run python examples/external_control_extension/demo_external_control.py
```

The demo logs registration, adapter construction, task creation, task ticks, and adapter writes so you can see that the externally registered pieces are actually running.

## Supported embodiments

Register hardware adapters for the same hardware categories that `ControlCoordinator` already understands:

| Embodiment | Hardware type | Example adapter name |
|------------|---------------|----------------------|
| External arm | `HardwareType.MANIPULATOR` | `"myarm"` |
| Robot dog or mobile base | `HardwareType.BASE` | `"mydog"` |
| Humanoid or whole-body robot | `HardwareType.WHOLE_BODY` | `"myhumanoid"` |

Each registration routes to the matching built-in registry, but external packages should use only the public facade in `dimos.control.extensions`.

## Full example: robot dog / mobile base

In `dimos_mydog/adapters.py`, define a base adapter that implements the twist-base adapter protocol used by `HardwareType.BASE`:

```python skip
class MyDogAdapter:
    def __init__(
        self,
        dof: int,
        address: str | None = None,
        hardware_id: str = "base",
        **_: object,
    ) -> None:
        self._dof = dof
        self._address = address
        self._hardware_id = hardware_id
        self._connected = False
        self._enabled = False

    def connect(self) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def get_dof(self) -> int:
        return self._dof

    def read_velocities(self) -> list[float]:
        return [0.0] * self._dof

    def read_odometry(self) -> list[float] | None:
        return None

    def write_velocities(self, velocities: list[float]) -> bool:
        return True

    def write_stop(self) -> bool:
        return True

    def write_enable(self, enable: bool) -> bool:
        self._enabled = enable
        return True

    def read_enabled(self) -> bool:
        return self._enabled
```

In `dimos_mydog/tasks/gait.py`, define the task factory. The task module is referenced by import path, so it is imported only when a coordinator actually creates this task:

```python skip
from dimos.control.task import BaseControlTask, CoordinatorState, JointCommandOutput, ResourceClaim


class MyDogGaitTask(BaseControlTask):
    def __init__(self, name: str) -> None:
        self._name = name

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(joints=frozenset())

    def is_active(self) -> bool:
        return False

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        return None

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        return None


def create_task(cfg, hardware):
    return MyDogGaitTask(cfg.name)
```

In `dimos_mydog/blueprints.py`, group registration in `register_extensions()` and call it at module import time:

```python skip
from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.extensions import register_control_task, register_hardware_adapter
from dimos.core.coordination.blueprints import autoconnect

from dimos_mydog.adapters import MyDogAdapter


def register_extensions() -> None:
    register_hardware_adapter(HardwareType.BASE, "mydog", MyDogAdapter)
    register_control_task("mydog_gait", "dimos_mydog.tasks.gait:create_task")


register_extensions()

mydog = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[
            HardwareComponent(
                hardware_id="base",
                hardware_type=HardwareType.BASE,
                joints=make_twist_base_joints("base"),
                adapter_type="mydog",
            ),
        ],
        tasks=[
            TaskConfig(
                name="gait",
                type="mydog_gait",
                joint_names=[],
                auto_start=True,
            ),
        ],
    ),
)
```

The blueprint still uses the normal DimOS configuration objects:

```python skip
HardwareComponent(hardware_type=HardwareType.BASE, adapter_type="mydog")
TaskConfig(type="mydog_gait")
```

The only new requirement is that `register_extensions()` runs before the coordinator resolves those names.

## Other embodiments

The same pattern works for arms and whole-body robots:

```python skip
from dimos.control.components import HardwareType
from dimos.control.extensions import register_hardware_adapter

register_hardware_adapter(HardwareType.MANIPULATOR, "myarm", MyArmAdapter)
register_hardware_adapter(HardwareType.BASE, "mydog", MyDogAdapter)
register_hardware_adapter(HardwareType.WHOLE_BODY, "myhumanoid", MyHumanoidAdapter)
```

Choose the hardware type that matches the protocol your adapter implements:

- `MANIPULATOR`: arm-style joint, Cartesian, and gripper IO.
- `BASE`: velocity-commanded platforms that consume twist-like virtual joints.
- `WHOLE_BODY`: joint-level whole-body motor IO.

## Duplicate registration policy

Registration is deterministic:

- registering the same hardware type, adapter name, and same factory object again is idempotent;
- registering the same hardware type and adapter name with a different factory raises an error;
- registering the same task type and same factory path again is idempotent;
- registering the same task type with a different factory path raises an error.

Use stable top-level classes or functions for hardware factories. Fresh wrappers, lambdas, or `functools.partial(...)` objects created on each call are different objects and are not treated as the same factory.

## Boundaries

This registration API handles only the ControlCoordinator layer. It does not make the external package discoverable by `dimos run` on its own. External blueprint discovery, package auto-discovery, entry points, richer diagnostics, and robot package templates are separate work.

Low-level registry singletons such as `adapter_registry`, `twist_base_adapter_registry`, and `whole_body_adapter_registry` remain internal implementation details for DimOS itself. External robot packages should import from `dimos.control.extensions`.
