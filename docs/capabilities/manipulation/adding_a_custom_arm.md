---
title: "How to Integrate a New Manipulator Arm"
---
This guide walks through integrating a new robot arm with DimOS: writing the hardware adapter, declaring the robot's config, and wiring up blueprints for control and motion planning.

The fastest path is to copy an arm that already works. The A-750 (`dimos/robot/manipulators/a750/`, `dimos/hardware/manipulators/a750/`) is the smallest complete example. The xArm (`dimos/robot/manipulators/xarm/`) is the most featureful, and is the one to read if you also want perception or pick-and-place.

## Architecture

Three layers, each owning one job:

```
┌──────────────────────────────────────────────────────────────┐
│              ManipulationModule (planning)                    │
│  - Plans collision-free trajectories from a URDF             │
│  - Sends trajectories to the coordinator via RPC             │
└───────────────────────┬──────────────────────────────────────┘
                        │ RPC: execute trajectory
┌───────────────────────▼──────────────────────────────────────┐
│              ControlCoordinator (100Hz tick loop)             │
│  - Reads state from every adapter                            │
│  - Runs tasks (trajectory, servo, velocity, eef_twist, ...)  │
│  - Arbitrates per-joint conflicts by priority                │
│  - Publishes aggregated joint state                          │
└───────────────────────┬──────────────────────────────────────┘
                        │ calls Protocol methods
┌───────────────────────▼──────────────────────────────────────┐
│              Your adapter (implements the Protocol)           │
│  - Wraps the vendor SDK (TCP/IP, CAN, serial, ...)           │
│  - Converts vendor units to SI                               │
│  - Handles the connection lifecycle                          │
└──────────────────────────────────────────────────────────────┘
```

Your code lands in two places, and the split matters:

| Tree | Holds | You create |
|------|-------|------------|
| `dimos/hardware/manipulators/<arm>/` | Hardware IO. Arm-specific, framework-agnostic. | `adapter.py`, `_registry.py` |
| `dimos/robot/manipulators/<arm>/` | Robot config and blueprints. | `config.py`, `blueprints/` |

Generic planning, IK, and world code lives in `dimos/manipulation/` and you should not need to touch it.

There are **no `__init__.py` files** anywhere in these trees. DimOS uses PEP 420 namespace packages. Do not add one, and import submodules directly (`from dimos.hardware.manipulators.yourarm.adapter import YourArmAdapter`), never from the package.

## Prerequisites

- The vendor Python SDK for the arm, if it has one (`xarm-python-sdk`, `piper-sdk`). OpenArm has none and talks raw SocketCAN, so this is optional.
- A URDF or xacro, only if you want motion planning. Control and teleop work without one.
- Connection info: IP address, CAN interface, serial device.

## Step 1: Write the adapter

Create `dimos/hardware/manipulators/yourarm/adapter.py`. The adapter is duck-typed against the `ManipulatorAdapter` Protocol in `dimos/hardware/manipulators/spec.py`. There is no base class and nothing to inherit from: match the method signatures and you are done.

Every value crossing the adapter boundary must be SI.

| Quantity | SI unit |
|------------------|---------|
| Angles | radians |
| Angular velocity | rad/s |
| Torque | Nm |
| Position | meters |
| Force | Newtons |

```python skip
"""YourArm adapter — implements the ManipulatorAdapter protocol.

SDK units: <describe the vendor's native units here>
DimOS units: radians, meters, rad/s, Nm.
"""

from __future__ import annotations

from dimos.hardware.manipulators.spec import ControlMode, JointLimits, ManipulatorInfo
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class YourArmAdapter:
    def __init__(
        self,
        address: str = "192.168.1.100",
        dof: int = 6,
        initial_positions: list[float] | None = None,
        **_: object,
    ) -> None:
        self._address = address
        self._dof = dof
        self._positions = list(initial_positions) if initial_positions else [0.0] * dof
        self._sdk = None
        self._control_mode = ControlMode.POSITION
```

The trailing `**_: object` is not optional decoration. The coordinator constructs your adapter with `address`, `dof`, and whatever you put in `adapter_kwargs`, so the constructor has to tolerate keys it does not care about. Every shipped adapter does this.

### Connection lifecycle

```python skip
    def connect(self) -> bool:
        """Connect to hardware. True on success."""
        try:
            from yourarm_sdk import YourArmSDK  # lazy: SDK is an optional dep

            self._sdk = YourArmSDK(self._address)
            self._sdk.connect()
            return bool(self._sdk.is_alive())
        except ImportError:
            logger.error("yourarm-sdk not installed. Run: pip install yourarm-sdk")
            return False
        except Exception as e:
            logger.error(f"Failed to connect to arm at {self._address}: {e}")
            return False

    def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...
    def activate(self) -> bool:
        """Prepare for commanded motion after connect()."""
        return self.write_enable(True)

    def deactivate(self) -> bool:
        """Gracefully stop commanded motion before disconnect()."""
        return self.write_stop()
```

Import the vendor SDK inside `connect()` rather than at module scope when the SDK is an optional dependency. That keeps `import dimos` working on a machine that has never seen your arm.

### The rest of the Protocol

Read `dimos/hardware/manipulators/spec.py` for the authoritative signatures. There are 26 methods in five groups:

| Group | Methods |
|-------|---------|
| Connection | `connect`, `disconnect`, `is_connected`, `activate`, `deactivate` |
| Info | `get_info`, `get_dof`, `get_limits` |
| Mode | `set_control_mode`, `get_control_mode` |
| Read | `read_joint_positions`, `read_joint_velocities`, `read_joint_efforts`, `read_state`, `read_error`, `read_enabled` |
| Write | `write_joint_positions`, `write_joint_velocities`, `write_stop`, `write_enable`, `write_clear_errors` |
| Optional | `read_cartesian_position`, `write_cartesian_position`, `read_gripper_position`, `write_gripper_position`, `read_force_torque` |

For anything your arm does not support, return `None` from a read and `False` from a write. Never raise. If the SDK gives you no velocity or effort feedback, return zeros: the coordinator copes.

Use `dimos/hardware/manipulators/mock/adapter.py` as a complete, readable reference implementation of all 26.

## Step 2: Declare the adapter in a manifest

Create `dimos/hardware/manipulators/yourarm/_registry.py`:

```python skip
ADAPTER_FACTORIES = {
    "yourarm": "dimos.hardware.manipulators.yourarm.adapter:YourArmAdapter",
}
```

DimOS has two unrelated families of file called `_registry.py`, and only the first is yours:

| File | Declares | Written by |
|------|----------|------------|
| `dimos/hardware/**/_registry.py` | `ADAPTER_FACTORIES` | you, adding an arm |
| `dimos/control/tasks/**/_registry.py` | `TASK_FACTORIES`, and the task's stream/command cards | someone adding a new control **task type** |

Adding an arm never means touching the second. You reuse the task types that already ship (`trajectory`, `eef_twist`, and so on) through the blueprint helpers in Step 4, and those already declare whatever they need.

The registry (`dimos/hardware/manipulators/registry.py`) discovers adapters lazily. It walks the subpackages under `dimos.hardware.manipulators`, loads each `_registry.py`, and records the name-to-import-path mapping. Your adapter module itself is imported only when someone calls `create("yourarm")`.

This is why the manifest must import nothing outside the stdlib. It gets loaded even on a machine without your vendor SDK, so the name still appears in `available()`, and a missing SDK fails loudly at `create()` instead of silently dropping the arm.

Now update the golden set in `dimos/hardware/test_adapter_registries.py`. Adding an adapter is a deliberate act, so CI pins the exact set of registered names and fails until you say so explicitly:

```python skip
EXPECTED_NAMES = {
    "manipulators": {"a750", "mock", "openarm", "piper", "sim_mujoco", "xarm", "yourarm"},
    ...
}
```

If your vendor SDK is an optional dependency, add its top-level module name to `OPTIONAL_VENDOR_MODULES` in the same file. That tells CI a factory failing to import is acceptable when the SDK is genuinely absent, while still failing on a typo in your manifest or a broken internal import.

That test also fails if an adapter directory has no manifest, or if a manifest points at something that does not resolve.

Verify:

```python skip
from dimos.hardware.manipulators.registry import adapter_registry

print(adapter_registry.available())
# ['a750', 'mock', 'openarm', 'piper', 'sim_mujoco', 'xarm', 'yourarm']
```

## Step 3: Write the robot config

Create `dimos/robot/manipulators/yourarm/config.py`. This holds two factory functions: one describing the arm to the **coordinator**, one describing it to the **planner**.

### 3a. Hardware component (for the coordinator)

```python skip
from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.core.global_config import global_config

YOURARM_HOME_JOINTS = [0.0, 0.0, -math.radians(90), 0.0, 0.0, 0.0]


def make_yourarm_hardware(
    hw_id: str = "arm",
    *,
    adapter_type: str = "yourarm",
    address: str | None = None,
    gripper: bool = True,
    auto_enable: bool = True,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hw_id, 6),                    # ["arm/joint1", ... "arm/joint6"]
        adapter_type=adapter_type,                        # must match ADAPTER_FACTORIES key
        address=address,                                  # passed to adapter __init__
        auto_enable=auto_enable,
        gripper_joints=[f"{hw_id}/gripper"] if gripper else [],
        adapter_kwargs={"initial_positions": home_joints or YOURARM_HOME_JOINTS},
    )


def yourarm_hardware(hw_id: str = "arm", *, mock_without_address: bool = False):
    """Real hardware when an address is configured, mock otherwise."""
    address = global_config.device_path
    if mock_without_address and not address:
        return make_yourarm_hardware(hw_id, adapter_type="mock", address=None)
    return make_yourarm_hardware(hw_id, address=address or "/dev/ttyACM0")
```

**Joint names are slash-separated.** `make_joints("arm", 6)` returns `["arm/joint1", ..., "arm/joint6"]`, not `arm_joint1`. The coordinator splits on that `/` to route a command back to the owning hardware component, and `split_joint_name` raises `ValueError` on a name without one. Always call `make_joints`; never hand-write the list.

Two `HardwareComponent` fields are easy to miss and you will need both for a gripper:

- `gripper_joints` — joints driven through the adapter's `read/write_gripper_position` methods rather than the normal joint path. Conventionally `["arm/gripper"]`.
- `adapter_kwargs` — an untyped passthrough to your adapter's constructor, for knobs that do not belong in the shared schema.

The `mock_without_address` pattern is worth copying. It lets the same blueprint run on a laptop with no arm attached, which is how you should develop.

### 3b. Planning model (for the planner)

Only needed if you want motion planning. Ship the URDF through Git LFS and reference it with `LfsPath`, a `Path` subclass that defers the download until the path is actually read, so importing a blueprint does not pull data.

```python skip
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators._modeling import base_pose, coordinator_joint_mapping, joint_names
from dimos.utils.data import LfsPath

YOURARM_MODEL_PATH = LfsPath("yourarm_description/urdf/yourarm.urdf")
YOURARM_PACKAGE_PATHS = {"yourarm_description": LfsPath("yourarm_description")}


def make_yourarm_model_config(
    name: str = "arm",
    *,
    coordinator_task_name: str | None = None,
) -> RobotModelConfig:
    dof = 6
    return RobotModelConfig(
        name=name,
        model_path=YOURARM_MODEL_PATH,
        base_pose=base_pose(),                    # base_pose(y=0.5) to offset a second arm
        joint_names=joint_names(dof),             # URDF names: ["joint1", ... "joint6"]
        end_effector_link="gripper_base",
        base_link="base_link",
        package_paths=YOURARM_PACKAGE_PATHS,
        auto_convert_meshes=True,
        collision_exclusion_pairs=YOURARM_GRIPPER_COLLISION_EXCLUSIONS,
        joint_name_mapping=coordinator_joint_mapping(name, dof),  # "arm/joint1" -> "joint1"
        coordinator_task_name=coordinator_task_name or f"traj_{name}",
        gripper_hardware_id=name,
        home_joints=YOURARM_HOME_JOINTS,
    )
```

`joint_name_mapping` is the bridge between the two namespaces: the coordinator knows `arm/joint1`, the URDF knows `joint1`. `coordinator_joint_mapping` builds it for you.

Fields worth knowing:

| Field | Description |
|-------|-------------|
| `model_path` | `.urdf`, `.xacro`, or `.xml` (MJCF) |
| `joint_names` | Ordered controlled joints, in the **URDF** namespace |
| `end_effector_link` | Link used as the end-effector for FK and IK |
| `base_link` | Root link of the model |
| `package_paths` | Maps `package://` URIs to filesystem paths, for xacro |
| `joint_name_mapping` | Coordinator name to URDF name |
| `coordinator_task_name` | Must match a `TaskConfig.name` in your coordinator blueprint |
| `collision_exclusion_pairs` | Link pairs allowed to touch, e.g. gripper fingers |
| `gripper_hardware_id` | `hardware_id` of the component owning the gripper |
| `home_joints` | Target for the `go_home` skill |
| `pre_grasp_offset` | Approach standoff in meters, default `0.10` |
| `auto_convert_meshes` | Convert DAE/STL meshes to OBJ for Drake |
| `max_velocity` / `max_acceleration` | Trajectory generation limits |

Getting `collision_exclusion_pairs` right takes iteration. Start empty, plan a motion, and add pairs the planner reports as self-colliding when they physically cannot. The A-750 and xArm configs have realistic lists to crib from.

## Step 4: Write the blueprints

Create `dimos/robot/manipulators/yourarm/blueprints/basic.py`. Note that `blueprints` is a **package**, not a single module: real arms split it into `basic.py`, `teleop.py`, `planner.py`, `perception.py`.

Rather than calling `ControlCoordinator.blueprint(...)` by hand, use the shared helpers in `dimos/robot/manipulators/common/blueprints.py`. They encode the naming conventions for you.

```python skip
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.blueprints import coordinator, planner, trajectory_task
from dimos.robot.manipulators.yourarm.config import (
    make_yourarm_model_config,
    yourarm_hardware,
)

_hw = yourarm_hardware("arm", mock_without_address=True)

# Coordinator only: control, no planning.
coordinator_yourarm = autoconnect(
    coordinator(hardware=[_hw], tasks=[trajectory_task(_hw)]),  # names the task "traj_arm"
)

# Planner + coordinator, wired together.
yourarm_planner_coordinator = autoconnect(
    planner(robots=[make_yourarm_model_config(name="arm")]),
    coordinator(hardware=[_hw], tasks=[trajectory_task(_hw)]),
)
```

`autoconnect` matches each module's output ports to the others' input ports by type and name, so the planner's `coordinator_joint_state` input finds the coordinator's output with no explicit topic wiring.

The `autoconnect(...)` wrapper around the coordinator-only blueprint looks redundant with a single argument, and it is not. The blueprint scanner in Step 5 only recognises a module-level assignment whose right-hand side is `autoconnect(...)` or a `.blueprint(...)` call. A bare `coordinator_yourarm = coordinator(...)` builds a perfectly valid blueprint that never gets a CLI name, and `dimos run coordinator-yourarm` will not find it. Wrap it, or call `ControlCoordinator.blueprint(...)` directly.

The two names must line up: `trajectory_task(_hw)` produces a task called `traj_arm`, and `make_yourarm_model_config` sets `coordinator_task_name="traj_arm"`. That string is how `ManipulationModule` finds the task to hand a trajectory to. If they disagree, planning succeeds and execution silently does nothing.

Helpers available for other control modes:

| Helper | Task type | Use for |
|--------|-----------|---------|
| `trajectory_task` | `trajectory` | Executing planned joint trajectories |
| `cartesian_ik_task` | `cartesian_ik` | Cartesian pose commands |
| `eef_twist_task` | `eef_twist` | End-effector twist, e.g. keyboard teleop |
| `teleop_ik_task` | `teleop_ik` | VR / Quest teleop |

The Cartesian and twist tasks run IK inside the coordinator and need `model_path` and `ee_joint_id`. See `dimos/robot/manipulators/a750/blueprints/teleop.py` for a worked example.

## Step 5: Register and run

`dimos/robot/all_blueprints.py` maps CLI names to blueprint symbols, and it is **auto-generated**. Never hand-edit it. Regenerate:

```bash
pytest dimos/robot/test_all_blueprints_generation.py
```

Expect this to fail the first time, by design. The test rewrites `all_blueprints.py`, notices the file now has uncommitted changes, and fails to make you commit them. Commit the regenerated file and it goes green.

The scanner only picks up module-level variables assigned from `autoconnect(...)` or a `.blueprint(...)` call. A bare `x = some_helper(...)` is invisible to it, so if your blueprint does not show up, that is the first thing to check.

Your symbol name becomes the CLI name, underscores to dashes:

```bash
dimos run coordinator-yourarm
dimos run yourarm-planner-coordinator
```

## Step 6: Test

Start with the registry, which catches a broken manifest immediately:

```python skip
from dimos.hardware.manipulators.registry import adapter_registry

assert "yourarm" in adapter_registry.available()
adapter = adapter_registry.create("yourarm", address="192.168.1.100", dof=6)
```

Unit-test coordinator logic against a mocked Protocol, no hardware needed:

```python skip
from unittest.mock import MagicMock

import pytest

from dimos.hardware.manipulators.spec import ManipulatorAdapter


@pytest.fixture
def mock_adapter():
    adapter = MagicMock(spec=ManipulatorAdapter)
    adapter.get_dof.return_value = 6
    adapter.read_joint_positions.return_value = [0.0] * 6
    adapter.write_joint_positions.return_value = True
    adapter.is_connected.return_value = True
    return adapter
```

`MagicMock(spec=ManipulatorAdapter)` will reject any method not on the Protocol, so this also guards against typos in your method names.

Then exercise the real adapter standalone, importing the module directly since there is no `__init__.py`:

```python skip
from dimos.hardware.manipulators.yourarm.adapter import YourArmAdapter

adapter = YourArmAdapter(address="192.168.1.100", dof=6)
assert adapter.connect()

print(f"joint positions (rad): {adapter.read_joint_positions()}")

adapter.activate()
adapter.write_joint_positions([0.0] * 6)
adapter.deactivate()
adapter.disconnect()
```

For an end-to-end check without hardware, point the blueprint at the mock adapter by flipping `adapter_type="yourarm"` to `adapter_type="mock"`. That is exactly what `mock_without_address=True` does, and it exercises the whole coordinator and planner path.

## Gotchas

Joint names use `/`, not `_`. `make_joints("arm", 6)` gives `arm/joint1`. Hand-writing `arm_joint1` raises `ValueError: Joint name 'arm_joint1' missing separator '/'` at runtime.

A blueprint the scanner cannot see gets no CLI name. Assign from `autoconnect(...)` or `.blueprint(...)`, never from a bare helper call.

Do not add `__init__.py`. These are PEP 420 namespace packages. Import submodules directly instead of relying on package re-exports.

`coordinator_task_name` must match your `TaskConfig.name`. A mismatch fails silently: the plan succeeds, the arm does not move.

Adapter constructors need `**_: object`. The coordinator passes kwargs your adapter may not know about.

Unsupported features return `None` or `False`. They never raise.

`_registry.py` imports stdlib only. It is loaded on machines that do not have your vendor SDK.

## Checklist

Files to create:

- [ ] `dimos/hardware/manipulators/yourarm/adapter.py` — implements the Protocol
- [ ] `dimos/hardware/manipulators/yourarm/_registry.py` — declares `ADAPTER_FACTORIES`
- [ ] `dimos/robot/manipulators/yourarm/config.py` — `HardwareComponent` + `RobotModelConfig` factories
- [ ] `dimos/robot/manipulators/yourarm/blueprints/basic.py` — coordinator and planner blueprints

Files to modify:

- [ ] `dimos/hardware/test_adapter_registries.py` — add your name to `EXPECTED_NAMES["manipulators"]`, and your SDK to `OPTIONAL_VENDOR_MODULES` if it is an optional dep
- [ ] `dimos/robot/all_blueprints.py` — **regenerated, never hand-edited**. Commit the result.
- [ ] `pyproject.toml` — add the vendor SDK as an optional dependency, if it has one

Do not create any `__init__.py`.

Verification:

- [ ] `adapter_registry.available()` includes `"yourarm"`
- [ ] `isinstance(adapter, ManipulatorAdapter)` is `True` (the Protocol is runtime-checkable, so this catches a missing or misnamed method)
- [ ] `pytest dimos/hardware/test_adapter_registries.py` passes
- [ ] `pytest dimos/robot/test_all_blueprints_generation.py` passes once the regenerated `all_blueprints.py` is committed
- [ ] `dimos run coordinator-yourarm` starts against the mock adapter
- [ ] `dimos run coordinator-yourarm` starts against real hardware
