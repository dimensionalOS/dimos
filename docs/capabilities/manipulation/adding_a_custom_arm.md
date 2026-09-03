# How to Integrate a New Manipulator Arm

This guide walks through integrating a new robot arm with dimOS, from writing the hardware adapter to creating blueprints for planning and control.

## Architecture Overview

dimOS uses a **Protocol-based adapter pattern**. No base class inheritance is required. Your adapter wraps the vendor SDK and exposes a standard interface that the rest of the system consumes:

```
┌──────────────────────────────────────────────────────────────┐
│              ManipulationModule (Planning)                    │
│  - Plans collision-free trajectories using Drake             │
│  - Sends trajectories to coordinator via RPC                 │
└───────────────────────┬──────────────────────────────────────┘
                        │ RPC: execute trajectory
┌───────────────────────▼──────────────────────────────────────┐
│              ControlCoordinator (100Hz control loop)          │
│  - Reads state from all adapters                             │
│  - Runs tasks (trajectory, servo, velocity)                  │
│  - Arbitrates per-joint conflicts (priority-based)           │
│  - Routes commands to the correct adapter                    │
│  - Publishes aggregated joint state                          │
└───────────────────────┬──────────────────────────────────────┘
                        │ uses
┌───────────────────────▼──────────────────────────────────────┐
│              Your Adapter (implements Protocol)               │
│  - Wraps vendor SDK (TCP/IP, CAN, serial, etc.)             │
│  - Converts between vendor units and SI units                │
│  - Handles connection lifecycle                              │
└──────────────────────────────────────────────────────────────┘
```

> See also: `dimos/hardware/manipulators/README.md` for a quick reference.

## Prerequisites

1. **Vendor SDK**: The Python SDK for your robot arm (e.g., `xarm-python-sdk`, `piper-sdk`)
2. **URDF/xacro**: A robot description file (only needed if you want motion planning)
3. **Connection info**: IP address, CAN port, serial device, etc.

## Step 1: Create the Adapter

Create a new directory for your arm under `dimos/hardware/manipulators/`:

```
dimos/hardware/manipulators/
├── spec.py              # ManipulatorAdapter Protocol (don't modify)
├── registry.py          # Auto-discovery registry (don't modify)
├── mock/
├── xarm/
├── piper/
└── yourarm/             # ← New directory
    ├── _registry.py  # Declares your adapter (name → import path)
    └── adapter.py
```

### adapter.py: Full Skeleton

Below is a complete annotated adapter. Implement each method by wrapping your vendor SDK calls. All values crossing the adapter boundary **must use SI units**.

| Quantity         | SI Unit  |
|------------------|----------|
| Angles           | radians  |
| Angular velocity | rad/s    |
| Torque           | Nm       |
| Position         | meters   |
| Force            | Newtons  |

```python skip
"""YourArm adapter: implements ManipulatorAdapter protocol.

SDK Units: <describe your SDK's native units here>
dimOS Units: angles=radians, distance=meters, velocity=rad/s
"""

from __future__ import annotations

import math

# Import your vendor SDK
from yourarm_sdk import YourArmSDK

from dimos.hardware.manipulators.spec import (
    ControlMode,
    JointLimits,
    ManipulatorInfo,
)

# Unit conversion constants (if your SDK doesn't use SI units)
MM_TO_M = 0.001
M_TO_MM = 1000.0


class YourArmAdapter:
    """YourArm hardware adapter.

    Implements ManipulatorAdapter protocol via duck typing.
    No inheritance required. Just match the method signatures in spec.py.
    """

    def __init__(self, address: str, dof: int = 6) -> None:
        """Initialize the adapter.

        Args:
            address: Connection address (IP, CAN port, serial device, etc.)
            dof: Degrees of freedom.
        """
        if not address:
            raise ValueError("address is required for YourArmAdapter")
        self._address = address
        self._dof = dof
        self._sdk: YourArmSDK | None = None
        self._control_mode: ControlMode = ControlMode.POSITION


    def connect(self) -> bool:
        """Connect to hardware. Returns True on success."""
        try:
            self._sdk = YourArmSDK(self._address)
            self._sdk.connect()
            # Verify connection succeeded
            if not self._sdk.is_alive():
                print(f"ERROR: Arm at {self._address} not reachable")
                return False
            return True
        except Exception as e:
            print(f"ERROR: Failed to connect to arm at {self._address}: {e}")
            return False

    def disconnect(self) -> None:
        """Disconnect from hardware."""
        if self._sdk:
            self._sdk.disconnect()
            self._sdk = None

    def is_connected(self) -> bool:
        """Check if connected."""
        return self._sdk is not None and self._sdk.is_alive()

    def activate(self) -> bool:
        """Prepare hardware for commanded motion after connect()."""
        return self.write_enable(True)

    def deactivate(self) -> bool:
        """Gracefully stop commanded motion before disconnect()."""
        return self.write_stop()


    def get_info(self) -> ManipulatorInfo:
        """Get manipulator info (vendor, model, DOF)."""
        return ManipulatorInfo(
            vendor="YourVendor",
            model="YourModel",
            dof=self._dof,
            firmware_version=None,  # Optional: query from SDK if available
            serial_number=None,     # Optional: query from SDK if available
        )

    def get_dof(self) -> int:
        """Get degrees of freedom."""
        return self._dof

    def get_limits(self) -> JointLimits:
        """Get joint position and velocity limits in SI units.

        Either hardcode known limits or query them from the SDK.
        """
        return JointLimits(
            position_lower=[-math.pi] * self._dof,     # radians
            position_upper=[math.pi] * self._dof,       # radians
            velocity_max=[math.pi] * self._dof,          # rad/s
        )


    def set_control_mode(self, mode: ControlMode) -> bool:
        """Set control mode.

        Map dimOS ControlMode enum values to your SDK's mode codes.
        Return False for modes your arm doesn't support.
        """
        if not self._sdk:
            return False

        mode_map = {
            ControlMode.POSITION: 0,        # Your SDK's position mode code
            ControlMode.SERVO_POSITION: 1,   # High-frequency servo mode
            ControlMode.VELOCITY: 4,         # Velocity mode
            # Add other supported modes...
        }

        sdk_mode = mode_map.get(mode)
        if sdk_mode is None:
            return False  # Unsupported mode

        success = self._sdk.set_mode(sdk_mode)
        if success:
            self._control_mode = mode
        return success

    def get_control_mode(self) -> ControlMode:
        """Get current control mode."""
        return self._control_mode


    def read_joint_positions(self) -> list[float]:
        """Read current joint positions in radians.

        Convert from SDK units to radians.
        """
        if not self._sdk:
            raise RuntimeError("Not connected")
        raw_positions = self._sdk.get_joint_positions()
        return [math.radians(p) for p in raw_positions[:self._dof]]

    def read_joint_velocities(self) -> list[float]:
        """Read current joint velocities in rad/s.

        If your SDK doesn't provide velocity feedback, return zeros.
        The coordinator can estimate velocity via finite differences.
        """
        if not self._sdk:
            return [0.0] * self._dof
        # If SDK supports velocity reading:
        # raw_velocities = self._sdk.get_joint_velocities()
        # return [math.radians(v) for v in raw_velocities[:self._dof]]
        return [0.0] * self._dof

    def read_joint_efforts(self) -> list[float]:
        """Read current joint torques in Nm.

        If your SDK doesn't provide torque feedback, return zeros.
        """
        if not self._sdk:
            return [0.0] * self._dof
        # If SDK supports torque reading:
        # return list(self._sdk.get_joint_torques()[:self._dof])
        return [0.0] * self._dof

    def read_state(self) -> dict[str, int]:
        """Read robot state (mode, state code, etc)."""
        if not self._sdk:
            return {"state": 0, "mode": 0}
        return {
            "state": self._sdk.get_state(),
            "mode": self._sdk.get_mode(),
        }

    def read_error(self) -> tuple[int, str]:
        """Read error code and message. (0, '') means no error."""
        if not self._sdk:
            return 0, ""
        code = self._sdk.get_error_code()
        if code == 0:
            return 0, ""
        return code, f"YourArm error {code}"


    def write_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
    ) -> bool:
        """Command joint positions in radians.

        Args:
            positions: Target positions in radians.
            velocity: Speed as fraction of max (0-1).

        Convert from radians to SDK units before sending.
        """
        if not self._sdk:
            return False
        sdk_positions = [math.degrees(p) for p in positions]
        return self._sdk.set_joint_positions(sdk_positions)

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        """Command joint velocities in rad/s.

        Return False if velocity control is not supported.
        """
        if not self._sdk:
            return False
        sdk_velocities = [math.degrees(v) for v in velocities]
        return self._sdk.set_joint_velocities(sdk_velocities)

    def write_stop(self) -> bool:
        """Stop all motion immediately."""
        if not self._sdk:
            return False
        return self._sdk.emergency_stop()


    def write_enable(self, enable: bool) -> bool:
        """Enable or disable servos."""
        if not self._sdk:
            return False
        return self._sdk.enable_motors(enable)

    def read_enabled(self) -> bool:
        """Check if servos are enabled."""
        if not self._sdk:
            return False
        return self._sdk.motors_enabled()

    def write_clear_errors(self) -> bool:
        """Clear error state."""
        if not self._sdk:
            return False
        return self._sdk.clear_errors()


    def read_cartesian_position(self) -> dict[str, float] | None:
        """Read end-effector pose.

        Returns dict with keys: x, y, z (meters), roll, pitch, yaw (radians).
        Return None if not supported.
        """
        return None  # Or implement if your SDK supports it

    def write_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
    ) -> bool:
        """Command end-effector pose. Return False if not supported."""
        return False


    def read_gripper_position(self) -> float | None:
        """Read gripper position in meters. Return None if no gripper."""
        return None

    def write_gripper_position(self, position: float) -> bool:
        """Command gripper position in meters. Return False if no gripper."""
        return False


    def read_force_torque(self) -> list[float] | None:
        """Read F/T sensor data [fx, fy, fz, tx, ty, tz]. None if no sensor."""
        return None
```

Then declare the adapter in a `_registry.py` manifest next to it:

```py
# dimos/hardware/manipulators/yourarm/_registry.py
ADAPTER_FACTORIES = {
    "yourarm": "dimos.hardware.manipulators.yourarm.adapter:YourArmAdapter",
}
```

### Key implementation notes

- **Unsupported features**: Return `None` for reads and `False` for writes. Never raise exceptions for optional features.
- **Velocity/effort feedback**: If your SDK doesn't provide these, return zeros. The coordinator handles this gracefully.
- **Lazy SDK import**: If the vendor SDK is an optional dependency, you can import it inside `connect()` instead of at module level (see Piper adapter for this pattern):

  ```py
  def connect(self) -> bool:
      try:
          from yourarm_sdk import YourArmSDK
          self._sdk = YourArmSDK(self._address)
          ...
      except ImportError:
          print("ERROR: yourarm-sdk not installed. Run: pip install yourarm-sdk")
          return False
  ```

## Step 2: Create Package Files

### How discovery works

The `AdapterRegistry` in `dimos/hardware/manipulators/registry.py` discovers adapters from `_registry.py` manifests at import time:

1. It iterates over all subpackages under `dimos/hardware/manipulators/`
2. For each subpackage, it loads `<subpackage>._registry` and records each `ADAPTER_FACTORIES` entry (name → `"module:attr"` import path)
3. Your adapter module is imported only when `create("yourarm")` is first called

The manifest must import nothing beyond stdlib. It is loaded even when your vendor SDK is missing, so the name always shows up in `available()` and a missing SDK fails loudly at `create()` instead of silently dropping the adapter. A CI test (`dimos/hardware/test_adapter_registries.py`) fails if an adapter directory has no manifest or a manifest path doesn't resolve.

You can verify discovery works:

```python skip
from dimos.hardware.manipulators.registry import adapter_registry
print(adapter_registry.available())  # Should include "yourarm"
```

## Step 3: Create Your Robot Folder and Blueprints

Each robot in dimOS gets its own folder under `dimos/robot/`. This is where you define all blueprints for your arm: coordinator, planning, perception, etc. This follows the same pattern as Unitree robots (`dimos/robot/unitree/`).

### 3a. Create the robot directory

```
dimos/robot/
├── unitree/                 # Unitree robots (reference example)
│   ├── go2/
│   │   └── blueprints/
│   └── g1/
│       └── blueprints/
└── yourarm/                 # ← New directory for your robot
    └── blueprints.py
```

### 3b. Define your blueprints

Create `dimos/robot/yourarm/blueprints.py` with your coordinator and (optionally) planning blueprints:

```python skip
"""Blueprints for YourArm robot.

Usage:
    # Run via CLI:
    dimos run coordinator-yourarm          # Start coordinator with real hardware
    dimos run yourarm-planner              # Start planner (optional, for motion planning)

    # Or programmatically:
    from dimos.robot.yourarm.blueprints import coordinator_yourarm
    coordinator = coordinator_yourarm.build()
    coordinator.loop()
"""

from __future__ import annotations

from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task


# YourArm (6-DOF), real hardware
coordinator_yourarm = ControlCoordinator.blueprint(
    tick_rate=100.0,                    # Control loop frequency (Hz)
    publish_joint_state=True,           # Publish aggregated joint state
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",                        # Unique ID for this hardware
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 6),             # Creates ["arm/joint1", ..., "arm/joint6"]
            adapter_type="yourarm",                   # Must match registry name
            address="192.168.1.100",                  # Passed to adapter __init__
            auto_enable=True,                         # Auto-enable servos on start
        ),
    ],
    tasks=[
        joint_trajectory_task([f"arm_joint{i+1}" for i in range(6)]),
    ],
)


```

### Blueprint field reference

| Field | Description |
|-------|-------------|
| `hardware_id` | Unique name for this hardware component. Used to route commands. |
| `adapter_type` | Name registered with `adapter_registry` (e.g., `"yourarm"`). |
| `address` | Connection info passed to adapter's `__init__` as `address` kwarg. |
| `joints` | List of joint names. `make_joints("arm", 6)` creates `["arm/joint1", ..., "arm/joint6"]`. |
| `auto_enable` | If `True`, servos are enabled automatically when the coordinator starts. |
| `task.name` | Name used by the ManipulationModule to invoke trajectory execution via RPC. |
| `task.type` | Task type: `"trajectory"`, `"servo"`, `"velocity"`, or `"cartesian_ik"`. |
| `task.priority` | Priority for per-joint arbitration. Higher number wins. |

## Step 4: Add URDF and Planning Integration (Optional)

If you want motion planning, you need a URDF and a planning blueprint. Add these
to your robot's own `blueprints.py`.

### 4a. Add your URDF

Prefer an upstream `RobotDescriptionSource` beside your robot adapter. Joining
paths from the source is lazy: importing the catalog does not clone or update
the repository, while the first concrete path access resolves it into the
robot asset cache. Use `LfsPath` only when the description is intentionally
vendored, locally modified, or has no suitable upstream source.

If the planning blueprint selects the RoboPlan TOPP-RA trajectory
parametrizer, dimOS currently pins RoboPlan to `0.5.1`. Every movable joint in
each selected planning group must provide finite, positive velocity limits.
Authored extended acceleration limits take precedence; when absent, dimOS
temporarily inserts a global `2.0 rad/s²` acceleration fallback during RoboPlan
model composition:

```xml
<joint name="joint1" type="revolute">
  <!-- parent, child, origin, and axis omitted -->
  <limit
    lower="-3.14"
    upper="3.14"
    effort="100"
    velocity="2.0"
    acceleration="4.0"
  />
</joint>
```

RoboPlan loads both limits from its scene model. If either is absent, zero,
negative, or non-finite, plan materialization fails before preview or execution
and identifies the affected joint. dimOS does not substitute
`RobotModelConfig.max_velocity`, `velocity_limits`, or `max_acceleration` for
this backend. Formal per-joint dimOS overrides will be added separately.

```python skip
from dimos.manipulation.manipulation_module import manipulation_module
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource

_YOURARM_REPO = RobotDescriptionSource(
    url="https://github.com/example/yourarm_description",
    ref="main",
)
_YOURARM_URDF_PATH = _YOURARM_REPO / "urdf" / "yourarm.urdf.xacro"
_YOURARM_PACKAGE_PATHS = {"yourarm_description": _YOURARM_REPO / "."}


def _make_base_pose(x=0.0, y=0.0, z=0.0) -> PoseStamped:
    return PoseStamped(
        frame_id="map",
        position=Vector3(x=x, y=y, z=z),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )
```

### 4b. Create a robot model config helper

```python skip
def _make_yourarm_config(y_offset: float = 0.0) -> RobotModelConfig:
    """Create YourArm robot config for planning.

    Args:
        y_offset: Y-axis offset for model placement.
    """
    # These must match the joint names in your URDF
    joint_names = [f"arm/joint{i}" for i in range(1, 7)]

    return RobotModelConfig(
        model=RobotModel.from_file(
            _YOURARM_URDF_PATH,
            package_paths=_YOURARM_PACKAGE_PATHS,
        ),
        joint_names=joint_names,
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(joint_names),
                base_link="base_link",
                tip_link="link6",
            )
        ],
        base_pose=_make_base_pose(y=y_offset),  # world -> base_link placement
        base_link="base_link",                 # Robot-scoped placement/weld/strip link
        collision_exclusion_pairs=[],   # Pairs of links that can touch (e.g., gripper fingers)
        auto_convert_meshes=True,       # Convert DAE/STL meshes for Drake
        max_velocity=1.0,               # Max velocity scaling factor
        max_acceleration=2.0,           # Max acceleration scaling factor
    )
```

### 4c. Create a planning blueprint

Add this to your `dimos/robot/yourarm/blueprints.py` alongside the coordinator blueprint:

```python skip

yourarm_planner = manipulation_module(
    model=_make_yourarm_config(),
    planning_timeout=10.0,
    visualization={"backend": "meshcat"},
    trajectory_parametrization={"backend": "simple_trapezoid"},
)
# The planner's `coordinator_joint_state` input auto-connects to the
# ControlCoordinator's output on the default `/coordinator_joint_state`
# topic, so no `.transports(...)` override is needed.
```

You may omit `trajectory_parametrization` when the world-based default is
appropriate: `world_backend="roboplan"` selects `roboplan_toppra`, while
`world_backend="drake"` selects `simple_trapezoid`.

To configure TOPP-RA tuning explicitly, select RoboPlan for the world and
parametrizer after adding the URDF limits described above:

```python skip
yourarm_planner = manipulation_module(
    model=_make_yourarm_config(),
    world_backend="roboplan",
    trajectory_parametrization={
        "backend": "roboplan_toppra",
        "velocity_scale": 0.8,
        "acceleration_scale": 0.8,
    },
    visualization={"backend": "viser"},
)
```

### Key config fields

| Field | Description |
|-------|-------------|
| `model` | Lazy `RobotModel` created from a `.urdf` or `.xacro` source |
| `joint_names` | Ordered canonical model joint set (must match the URDF and coordinator); not itself a planning group |
| `planning_groups` / `srdf_path` | Explicit planning groups or SRDF source; direct `RobotModelConfig(...)` helpers should pass explicit groups, while shared config helpers can discover groups from SRDF/fallback |
| `base_pose` / `base_link` | Optional robot placement: `base_pose` places `base_link` in the world for weld/strip behavior |
| `collision_exclusion_pairs` | List of `(link_a, link_b)` tuples for links that may legitimately touch (e.g., gripper fingers) |

Coordinator-facing joint states and trajectories use the model's canonical
joint names unchanged. Keep hardware-native name translation inside the
hardware adapter.

Planning-group `base_link`/`tip_link` values define kinematic chains and pose
target frames. `base_link` is only the robot-scoped link placed by
`base_pose`; do not use it as a substitute for planning-group chain metadata.
See [Planning Groups](/docs/capabilities/manipulation/planning_groups.md).

### 4d. Add Cartesian and teleoperation IK

Cartesian, EEF-twist, and engagement-relative teleoperation tasks share the
Pink backend. Install its dependencies with the manipulation extra:

```bash skip
uv sync --extra manipulation --inexact
```

Use one control task for each robot model that Pink should solve as one system:

| Robot setup | Task layout |
| --- | --- |
| One arm | One task with one target frame |
| Two independent robot models | One task per arm |
| One bimanual robot model | One task with both arm joint sets and two target frames |

The task uses coordinator-facing joint names and URDF frame names. Give it the
same `RobotModelConfig` used by planning; hardware-native naming remains inside
the adapter.

```python skip
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.manipulators.common.blueprints import teleop_ik_task

pink = PinkKinematicsConfig()
robot_model = _make_yourarm_config()
teleop_task = teleop_ik_task(
    hardware,
    name="teleop_arm",
    robot_model=robot_model,
    bindings=[{"hand": "right", "target_frame": "link6"}],
    params={"pink": pink},
)
```

For a bimanual model, pass both joint sets and bind each hand to its own frame:

```python skip
bimanual_task = teleop_ik_task(
    dual_arm_hardware,
    name="teleop_dual_arm",
    robot_model=dual_arm_model,
    joint_names=[*left_arm_joint_names, *right_arm_joint_names],
    bindings=[
        {"hand": "left", "target_frame": "left_tool_frame"},
        {"hand": "right", "target_frame": "right_tool_frame"},
    ],
    params={"pink": PinkKinematicsConfig()},
)
```

Do not pass planning groups into a control task. Planning groups select planning
requests; control IK needs only the controlled joints and target frames. A
single-hand task requires that hand's primary button. A bimanual task requires
both buttons, and releasing either clears both controller references.

Before connecting hardware, follow [Pink IK Configuration and Tuning](/docs/capabilities/manipulation/pink_ik_tuning.md)
to tune task weights, robot-specific objectives, velocity limits, feedback
tolerance, and command tracking.

## Step 5: Register Blueprints

The blueprint registry in `dimos/robot/all_blueprints.py` is **auto-generated** by scanning the codebase for blueprint declarations. After adding your blueprints:

1. Run the generation test to update the registry:
   ```bash
   pytest dimos/robot/test_all_blueprints_generation.py
   ```
2. Now you can run your arm via CLI:
   ```bash
   dimos run coordinator-yourarm
   dimos run yourarm-planner        # If you added a planning blueprint
   ```

## Step 6: Testing

### Verify adapter registration

```python skip
from dimos.hardware.manipulators.registry import adapter_registry

# Check your adapter shows up
assert "yourarm" in adapter_registry.available()

# Create an instance via registry (same path the coordinator uses)
adapter = adapter_registry.create("yourarm", address="192.168.1.100", dof=6)
```

### Unit test with mock

You can test coordinator logic without hardware by using `unittest.mock`:

```python skip
import pytest
from unittest.mock import MagicMock
from dimos.hardware.manipulators.spec import ManipulatorAdapter

@pytest.fixture
def mock_adapter():
    adapter = MagicMock(spec=ManipulatorAdapter)
    adapter.get_dof.return_value = 6
    adapter.read_joint_positions.return_value = [0.0] * 6
    adapter.read_joint_velocities.return_value = [0.0] * 6
    adapter.read_joint_efforts.return_value = [0.0] * 6
    adapter.write_joint_positions.return_value = True
    adapter.activate.return_value = True
    adapter.deactivate.return_value = True
    adapter.read_enabled.return_value = True
    adapter.is_connected.return_value = True
    return adapter

def test_read_positions(mock_adapter):
    assert mock_adapter.read_joint_positions() == [0.0] * 6

def test_write_positions(mock_adapter):
    target = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert mock_adapter.write_joint_positions(target) is True
```

### Integration test with coordinator

```python skip
from dimos.robot.manipulators.common.mock import coordinator_mock
from dimos.core.coordination.module_coordinator import ModuleCoordinator

# Build and start coordinator with mock hardware
coordinator = ModuleCoordinator.build(coordinator_mock)
coordinator.start()

# Your adapter is tested through the same coordinator interface
# Just swap adapter_type="mock" to adapter_type="yourarm" in a blueprint

coordinator.stop()
```

### Test the real adapter standalone

```python skip
from dimos.hardware.manipulators.yourarm import YourArmAdapter

adapter = YourArmAdapter(address="192.168.1.100", dof=6)
assert adapter.connect() is True
assert adapter.is_connected() is True

# Read state
positions = adapter.read_joint_positions()
assert len(positions) == 6
print(f"Joint positions (rad): {positions}")

# Activate and move
adapter.activate()
adapter.write_joint_positions([0.0] * 6)

# Cleanup
adapter.deactivate()
adapter.disconnect()
```

## Quick Reference Checklist

Files to create:

- [ ] `dimos/hardware/manipulators/yourarm/adapter.py` (implements Protocol)
- [ ] `dimos/hardware/manipulators/yourarm/_registry.py` (declares `ADAPTER_FACTORIES`)
- [ ] `dimos/robot/yourarm/blueprints.py` (coordinator + planning blueprints)

Files to modify:

- [ ] `pyproject.toml`: Add vendor SDK to optional dependencies *(if applicable)*

Verification:

- [ ] `adapter_registry.available()` includes `"yourarm"`
- [ ] `pytest dimos/robot/test_all_blueprints_generation.py` passes (regenerates `all_blueprints.py`)
- [ ] `dimos run coordinator-yourarm` starts successfully
