# How to Integrate a New Manipulator Arm

This guide walks through integrating a new robot arm with DimOS, from writing the hardware adapter to creating blueprints for planning and control.

## Architecture Overview

DimOS uses a **Protocol-based adapter pattern** — no base class inheritance required. Your adapter wraps the vendor SDK and exposes a standard interface that the rest of the system consumes:

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

1. **Vendor SDK** — The Python SDK for your robot arm (e.g., `xarm-python-sdk`, `piper-sdk`)
2. **URDF/xacro** — A robot description file (only needed if you want motion planning)
3. **Connection info** — IP address, CAN port, serial device, etc.

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
    ├── __init__.py
    └── adapter.py
```

### adapter.py — Full Skeleton

Below is a complete annotated adapter. Implement each method by wrapping your vendor SDK calls. All values crossing the adapter boundary **must use SI units**.

| Quantity         | SI Unit  |
|------------------|----------|
| Angles           | radians  |
| Angular velocity | rad/s    |
| Torque           | Nm       |
| Position         | meters   |
| Force            | Newtons  |

```python
"""YourArm adapter — implements ManipulatorAdapter protocol.

SDK Units: <describe your SDK's native units here>
DimOS Units: angles=radians, distance=meters, velocity=rad/s
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

# Import your vendor SDK
from yourarm_sdk import YourArmSDK

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry

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
    No inheritance required — just match the method signatures in spec.py.
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

    # =========================================================================
    # Connection
    # =========================================================================

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

    # =========================================================================
    # Info
    # =========================================================================

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

    # =========================================================================
    # Control Mode
    # =========================================================================

    def set_control_mode(self, mode: ControlMode) -> bool:
        """Set control mode.

        Map DimOS ControlMode enum values to your SDK's mode codes.
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

    # =========================================================================
    # State Reading
    # =========================================================================

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

    # =========================================================================
    # Motion Control (Joint Space)
    # =========================================================================

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

    # =========================================================================
    # Servo Control
    # =========================================================================

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

    # =========================================================================
    # Optional: Cartesian Control
    # Return None/False if not supported by your arm.
    # =========================================================================

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

    # =========================================================================
    # Optional: Gripper
    # =========================================================================

    def read_gripper_position(self) -> float | None:
        """Read gripper position in meters. Return None if no gripper."""
        return None

    def write_gripper_position(self, position: float) -> bool:
        """Command gripper position in meters. Return False if no gripper."""
        return False

    # =========================================================================
    # Optional: Force/Torque Sensor
    # =========================================================================

    def read_force_torque(self) -> list[float] | None:
        """Read F/T sensor data [fx, fy, fz, tx, ty, tz]. None if no sensor."""
        return None


# ── Registry hook (required for auto-discovery) ───────────────────
def register(registry: AdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("yourarm", YourArmAdapter)


__all__ = ["YourArmAdapter"]
```

### Key implementation notes

- **Unsupported features** — Return `None` for reads and `False` for writes. Never raise exceptions for optional features.
- **Velocity/effort feedback** — If your SDK doesn't provide these, return zeros. The coordinator handles this gracefully.
- **Lazy SDK import** — If the vendor SDK is an optional dependency, you can import it inside `connect()` instead of at module level (see Piper adapter for this pattern):
  ```python
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

### \_\_init\_\_.py

```python
"""YourArm manipulator hardware adapter.

Usage:
    >>> from dimos.hardware.manipulators.yourarm import YourArmAdapter
    >>> adapter = YourArmAdapter(address="192.168.1.100", dof=6)
    >>> adapter.connect()
    >>> positions = adapter.read_joint_positions()
"""

from dimos.hardware.manipulators.yourarm.adapter import YourArmAdapter

__all__ = ["YourArmAdapter"]
```

### How auto-discovery works

The `AdapterRegistry` in `dimos/hardware/manipulators/registry.py` automatically discovers your adapter at import time:

1. It iterates over all subpackages under `dimos/hardware/manipulators/`
2. For each subpackage, it tries to import `<subpackage>.adapter`
3. If that module has a `register()` function, it calls it

This means **no manual registration is needed** — just having the `register()` function in your `adapter.py` is sufficient.

You can verify discovery works:

```python
from dimos.hardware.manipulators.registry import adapter_registry
print(adapter_registry.available())  # Should include "yourarm"
```

## Step 3: Create Your Robot Folder and Blueprints

Each robot in DimOS gets its own folder under `dimos/robot/`. This is where you define all blueprints for your arm — coordinator, planning, perception, etc. This follows the same pattern as Unitree robots (`dimos/robot/unitree/`).

### 3a. Create the robot directory

```
dimos/robot/
├── unitree/                 # Unitree robots (reference example)
│   ├── go2/
│   │   └── blueprints/
│   └── g1/
│       └── blueprints/
└── yourarm/                 # ← New directory for your robot
    ├── __init__.py
    └── blueprints.py
```

### 3b. Define your blueprints

Create `dimos/robot/yourarm/blueprints.py` with your coordinator and (optionally) planning blueprints:

```python
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
from dimos.control.coordinator import TaskConfig, control_coordinator
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs import JointState

# =============================================================================
# Coordinator Blueprints
# =============================================================================

# YourArm (6-DOF) — real hardware
coordinator_yourarm = control_coordinator(
    tick_rate=100.0,                    # Control loop frequency (Hz)
    publish_joint_state=True,           # Publish aggregated joint state
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",                        # Unique ID for this hardware
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 6),             # Creates ["arm_joint1", ..., "arm_joint6"]
            adapter_type="yourarm",                   # Must match registry name
            address="192.168.1.100",                  # Passed to adapter __init__
            auto_enable=True,                         # Auto-enable servos on start
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_arm",                          # Task name (used by ManipulationModule RPC)
            type="trajectory",                        # Trajectory execution task
            joint_names=[f"arm_joint{i+1}" for i in range(6)],
            priority=10,                              # Higher priority wins arbitration
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


```

### Blueprint field reference

| Field | Description |
|-------|-------------|
| `hardware_id` | Unique name for this hardware component. Used to route commands. |
| `adapter_type` | Name registered with `adapter_registry` (e.g., `"yourarm"`). |
| `address` | Connection info passed to adapter's `__init__` as `address` kwarg. |
| `joints` | List of joint names. `make_joints("arm", 6)` creates `["arm_joint1", ..., "arm_joint6"]`. |
| `auto_enable` | If `True`, servos are enabled automatically when the coordinator starts. |
| `task.name` | Name used by the ManipulationModule to invoke trajectory execution via RPC. |
| `task.type` | Task type: `"trajectory"`, `"servo"`, `"velocity"`, or `"cartesian_ik"`. |
| `task.priority` | Priority for per-joint arbitration. Higher number wins. |

## Step 4: Add URDF and Planning Integration (Optional)

If you want motion planning (collision-free trajectories via Drake), you need a URDF and a planning blueprint. Add these to your robot's own `blueprints.py`.

### 4a. Add your URDF

Place your URDF/xacro files under LFS data so they can be resolved via `LfsPath`. `LfsPath` is a `Path` subclass that lazily downloads LFS data on first access — this avoids downloading at import time when the blueprint module is loaded.

```python
from dimos.utils.data import LfsPath
from dimos.manipulation.manipulation_module import manipulation_module
from dimos.manipulation.planning.spec import RobotModelConfig
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3

# LfsPath defers download until the path is actually accessed
_YOURARM_URDF_PATH = LfsPath("yourarm_description/urdf/yourarm.urdf")
_YOURARM_PACKAGE_PATH = LfsPath("yourarm_description")


def _make_base_pose(x=0.0, y=0.0, z=0.0) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x=x, y=y, z=z),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )
```

### 4b. Create a robot model config helper

```python
def _make_yourarm_config(
    name: str = "arm",
    y_offset: float = 0.0,
    joint_prefix: str = "",
    coordinator_task: str | None = None,
) -> RobotModelConfig:
    """Create YourArm robot config for planning.

    Args:
        name: Robot name in the Drake planning world.
        y_offset: Y-axis offset for multi-arm setups.
        joint_prefix: Prefix for joint name mapping to coordinator namespace.
        coordinator_task: Coordinator task name for trajectory execution via RPC.
    """
    # These must match the joint names in your URDF
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    joint_mapping = {f"{joint_prefix}{j}": j for j in joint_names} if joint_prefix else {}

    return RobotModelConfig(
        name=name,
        urdf_path=_YOURARM_URDF_PATH,
        base_pose=_make_base_pose(y=y_offset),
        joint_names=joint_names,
        end_effector_link="link6",      # Last link in your URDF's kinematic chain
        base_link="base_link",          # Root link of your URDF
        package_paths={"yourarm_description": _YOURARM_PACKAGE_PATH},
        xacro_args={},                  # Xacro arguments if using .xacro files
        collision_exclusion_pairs=[],   # Pairs of links that can touch (e.g., gripper fingers)
        auto_convert_meshes=True,       # Convert DAE/STL meshes for Drake
        max_velocity=1.0,               # Max velocity scaling factor
        max_acceleration=2.0,           # Max acceleration scaling factor
        joint_name_mapping=joint_mapping,
        coordinator_task_name=coordinator_task,
    )
```

### 4c. Create a planning blueprint

Add this to your `dimos/robot/yourarm/blueprints.py` alongside the coordinator blueprint:

```python
# =============================================================================
# Planner Blueprints (requires URDF)
# =============================================================================

yourarm_planner = manipulation_module(
    robots=[_make_yourarm_config("arm", joint_prefix="arm_", coordinator_task="traj_arm")],
    planning_timeout=10.0,
    enable_viz=True,
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)
```

### Key config fields

| Field | Description |
|-------|-------------|
| `urdf_path` | Path to `.urdf` or `.xacro` file |
| `joint_names` | Ordered list of controlled joints (must match URDF) |
| `end_effector_link` | Link to use as the end-effector for IK |
| `base_link` | Root link of the robot model |
| `package_paths` | Maps `package://` URIs to filesystem paths (for xacro) |
| `joint_name_mapping` | Maps coordinator names (e.g., `"arm_joint1"`) to URDF names (e.g., `"joint1"`) |
| `coordinator_task_name` | Must match the `TaskConfig.name` in your coordinator blueprint |
| `collision_exclusion_pairs` | List of `(link_a, link_b)` tuples for links that may legitimately touch (e.g., gripper fingers) |

## Step 5: Register Blueprints

The blueprint registry in `dimos/robot/all_blueprints.py` is **auto-generated** by scanning the codebase for blueprint declarations. After adding your blueprints:

1. Run the generation test to update the registry:
   ```bash
   pytest dimos/robot/test_all_blueprints_generation.py
   ```
3. Now you can run your arm via CLI:
   ```bash
   dimos run coordinator-yourarm
   dimos run yourarm-planner        # If you added a planning blueprint
   ```

## Step 6: Testing

### Verify adapter registration

```python
from dimos.hardware.manipulators.registry import adapter_registry

# Check your adapter shows up
assert "yourarm" in adapter_registry.available()

# Create an instance via registry (same path the coordinator uses)
adapter = adapter_registry.create("yourarm", address="192.168.1.100", dof=6)
```

### Unit test with mock

You can test coordinator logic without hardware by using `unittest.mock`:

```python
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

```python
from dimos.control.blueprints import coordinator_mock

# Build and start coordinator with mock hardware
coordinator = coordinator_mock.build()
coordinator.start()

# Your adapter is tested through the same coordinator interface
# Just swap adapter_type="mock" to adapter_type="yourarm" in a blueprint
```

### Test the real adapter standalone

```python
from dimos.hardware.manipulators.yourarm import YourArmAdapter

adapter = YourArmAdapter(address="192.168.1.100", dof=6)
assert adapter.connect() is True
assert adapter.is_connected() is True

# Read state
positions = adapter.read_joint_positions()
assert len(positions) == 6
print(f"Joint positions (rad): {positions}")

# Enable and move
adapter.write_enable(True)
adapter.write_joint_positions([0.0] * 6)

# Cleanup
adapter.write_stop()
adapter.disconnect()
```

## Quick Reference Checklist

Files to create:

- [ ] `dimos/hardware/manipulators/yourarm/__init__.py`
- [ ] `dimos/hardware/manipulators/yourarm/adapter.py` (implements Protocol + `register()`)
- [ ] `dimos/robot/yourarm/__init__.py`
- [ ] `dimos/robot/yourarm/blueprints.py` (coordinator + planning blueprints)

Files to modify:

- [ ] `pyproject.toml` — Add vendor SDK to optional dependencies *(if applicable)*

Verification:

- [ ] `adapter_registry.available()` includes `"yourarm"`
- [ ] `pytest dimos/robot/test_all_blueprints_generation.py` passes (regenerates `all_blueprints.py`)
- [ ] `dimos run coordinator-yourarm` starts successfully
