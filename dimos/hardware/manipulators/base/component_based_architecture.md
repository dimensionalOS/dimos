# Component-Based Architecture for Manipulator Drivers

## Overview

The Component-Based Architecture provides a highly modular, scalable framework for integrating 100+ different manipulator types into a unified system. This architecture emphasizes maximum code reuse through standardized components and SDK abstraction layers.

## Architecture Layers

```
┌─────────────────────────────────────────────────────┐
│                   RPC Interface                      │
│              (Standardized across all arms)          │
└─────────────────────────────────────────────────────┘
                           ▲
                           │
┌─────────────────────────────────────────────────────┐
│                  Driver Instance                     │
│                   (XArmDriver)                       │
│                 - Assembles components               │
│                 - Declares capabilities              │
└─────────────────────────────────────────────────────┘
                           ▲
                           │
┌─────────────────────────────────────────────────────┐
│              Standard Components                     │
│   (Motion, Servo, Status, Trajectory, Gripper)      │
│          - Work with ANY SDK wrapper                 │
│          - Tested once, used everywhere              │
└─────────────────────────────────────────────────────┘
                           ▲
                           │
┌─────────────────────────────────────────────────────┐
│                SDK Abstraction Layer                 │
│                 (BaseManipulatorSDK)                 │
│           - Standardized interface for all           │
│           - Unit conversions                         │
│           - Error translation                        │
└─────────────────────────────────────────────────────┘
                           ▲
                           │
┌─────────────────────────────────────────────────────┐
│                  SDK Wrapper                         │
│               (XArmSDKWrapper)                       │
│         - Implements standard interface              │
│         - Translates vendor SDK to standard          │
└─────────────────────────────────────────────────────┘
                           ▲
                           │
┌─────────────────────────────────────────────────────┐
│                 Native Vendor SDK                    │
│              (XArmAPI, PiperSDK, etc.)              │
└─────────────────────────────────────────────────────┘
```

## Core Components

### 1. BaseManipulatorSDK (Abstract Interface)

```python
from abc import ABC, abstractmethod
from typing import Optional

class BaseManipulatorSDK(ABC):
    """Standard interface that all SDK wrappers must implement."""

    @abstractmethod
    def connect(self, config: dict) -> bool:
        """Establish connection to hardware."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Close connection to hardware."""
        pass

    @abstractmethod
    def get_joint_positions(self) -> list[float]:
        """Get current joint positions in radians."""
        pass

    @abstractmethod
    def get_joint_velocities(self) -> list[float]:
        """Get current joint velocities in rad/s."""
        pass

    @abstractmethod
    def set_joint_positions(self, positions: list[float], velocity: float) -> bool:
        """Move to target positions (radians) at specified velocity (rad/s)."""
        pass

    @abstractmethod
    def set_joint_velocities(self, velocities: list[float]) -> bool:
        """Set joint velocities in rad/s."""
        pass

    @abstractmethod
    def enable_servos(self) -> bool:
        """Enable motor control."""
        pass

    @abstractmethod
    def disable_servos(self) -> bool:
        """Disable motor control."""
        pass

    @abstractmethod
    def emergency_stop(self) -> bool:
        """Execute emergency stop."""
        pass

    @abstractmethod
    def get_error_code(self) -> int:
        """Get current error code (0 = no error)."""
        pass

    @abstractmethod
    def reset_errors(self) -> bool:
        """Clear error states."""
        pass

    # Optional methods (return None if not supported)
    def get_force_torque(self) -> Optional[list[float]]:
        """Get force/torque sensor readings if available."""
        return None

    def set_impedance_parameters(self, stiffness: list[float], damping: list[float]) -> bool:
        """Set impedance control parameters if supported."""
        return False
```

### 2. SDK Wrapper Implementation

```python
# sdk_wrappers/xarm_wrapper.py
import math
from typing import Optional
from xarm import XArmAPI
from ..sdk_interface import BaseManipulatorSDK

class XArmSDKWrapper(BaseManipulatorSDK):
    """Translates XArm's SDK to standard interface."""

    def __init__(self):
        self.native_sdk: Optional[XArmAPI] = None
        self.dof: int = 7

    def connect(self, config: dict) -> bool:
        """Connect to XArm controller."""
        ip = config.get('ip', '192.168.1.100')
        self.dof = config.get('dof', 7)

        self.native_sdk = XArmAPI(ip)
        ret = self.native_sdk.connect()

        if ret == 0:
            # XArm-specific initialization
            self.native_sdk.motion_enable(True)
            self.native_sdk.set_mode(1)  # Servo mode
            self.native_sdk.set_state(0)  # Ready state
            return True
        return False

    def disconnect(self) -> None:
        """Disconnect from XArm."""
        if self.native_sdk:
            self.native_sdk.disconnect()
            self.native_sdk = None

    def get_joint_positions(self) -> list[float]:
        """Get positions in radians (XArm returns degrees)."""
        code, angles = self.native_sdk.get_servo_angle()
        if code != 0:
            raise RuntimeError(f"XArm error getting positions: {code}")

        # Convert degrees to radians
        positions = [math.radians(angle) for angle in angles[:self.dof]]
        return positions

    def get_joint_velocities(self) -> list[float]:
        """Get velocities in rad/s (XArm returns deg/s)."""
        # XArm doesn't directly provide velocities, estimate from positions
        # Or use get_joint_speeds if available
        code, speeds = self.native_sdk.get_joint_speeds()
        if code != 0:
            return [0.0] * self.dof

        # Convert deg/s to rad/s
        velocities = [math.radians(speed) for speed in speeds[:self.dof]]
        return velocities

    def set_joint_positions(self, positions: list[float], velocity: float) -> bool:
        """Set positions (convert radians to degrees for XArm)."""
        # Convert radians to degrees
        degrees = [math.degrees(pos) for pos in positions]

        # Convert rad/s to deg/s
        speed = math.degrees(velocity)

        # XArm set_servo_angle API
        ret = self.native_sdk.set_servo_angle(
            angle=degrees,
            speed=speed,
            wait=False,
            timeout=10
        )
        return ret == 0

    def set_joint_velocities(self, velocities: list[float]) -> bool:
        """Set velocities (convert rad/s to deg/s for XArm)."""
        # Convert rad/s to deg/s
        deg_velocities = [math.degrees(vel) for vel in velocities]

        # XArm velocity control
        ret = self.native_sdk.vc_set_joint_velocity(deg_velocities)
        return ret == 0

    def enable_servos(self) -> bool:
        """Enable XArm servos."""
        ret1 = self.native_sdk.motion_enable(True)
        ret2 = self.native_sdk.set_state(0)  # Ready state
        return ret1 == 0 and ret2 == 0

    def disable_servos(self) -> bool:
        """Disable XArm servos."""
        ret = self.native_sdk.motion_enable(False)
        return ret == 0

    def emergency_stop(self) -> bool:
        """Emergency stop."""
        ret = self.native_sdk.emergency_stop()
        return ret == 0

    def get_error_code(self) -> int:
        """Get XArm error code."""
        return self.native_sdk.error_code

    def reset_errors(self) -> bool:
        """Clear XArm errors."""
        ret = self.native_sdk.clean_error()
        return ret == 0

    def get_force_torque(self) -> Optional[list[float]]:
        """Get F/T sensor if available."""
        if hasattr(self.native_sdk, 'get_ft_sensor_data'):
            code, ft_data = self.native_sdk.get_ft_sensor_data()
            if code == 0:
                return ft_data
        return None
```

### 3. Standard Components

```python
# components/motion.py
from ..sdk_interface import BaseManipulatorSDK

class StandardMotionComponent:
    """Motion control component that works with ANY SDK wrapper."""

    def __init__(self, sdk: BaseManipulatorSDK, shared_state):
        self.sdk = sdk
        self.shared_state = shared_state
        self.joint_limits = None

    def set_joint_limits(self, lower: list[float], upper: list[float]):
        """Configure software joint limits."""
        self.joint_limits = (lower, upper)

    def validate_positions(self, positions: list[float]) -> bool:
        """Validate positions are within limits."""
        if not self.joint_limits:
            return True

        lower, upper = self.joint_limits
        for i, pos in enumerate(positions):
            if pos < lower[i] or pos > upper[i]:
                return False
        return True

    def rpc_move_joint(self, positions: list[float], velocity: float = 1.0) -> dict:
        """RPC method for joint movement."""
        # Validate
        if not self.validate_positions(positions):
            return {"success": False, "error": "Joint limits exceeded"}

        # Execute using standardized SDK method
        success = self.sdk.set_joint_positions(positions, velocity)

        if success:
            # Update shared state
            with self.shared_state.lock:
                self.shared_state.target_positions = positions

        return {"success": success}

    def rpc_move_joint_velocity(self, velocities: list[float]) -> dict:
        """RPC method for velocity control."""
        success = self.sdk.set_joint_velocities(velocities)
        return {"success": success}

    def rpc_get_joint_state(self) -> dict:
        """RPC method to get current state."""
        positions = self.sdk.get_joint_positions()
        velocities = self.sdk.get_joint_velocities()

        return {
            "positions": positions,
            "velocities": velocities,
            "timestamp": time.time()
        }

    def rpc_stop_motion(self) -> dict:
        """Stop all motion."""
        # Set zero velocities
        zeros = [0.0] * len(self.sdk.get_joint_positions())
        success = self.sdk.set_joint_velocities(zeros)
        return {"success": success}
```

### 4. BaseManipulatorDriver

```python
# base/driver.py
from abc import ABC
from threading import Thread, Event, Lock
import time

class BaseManipulatorDriver(ABC):
    """Base driver with threading and component management."""

    def __init__(self, sdk: BaseManipulatorSDK, components: list, config: dict):
        self.sdk = sdk
        self.components = components
        self.config = config

        # Threading infrastructure
        self.shared_state = SharedState()
        self.stop_event = Event()
        self.threads = []

        # Register RPC methods from components
        self._register_component_methods()

        # Connect to hardware
        if not self.sdk.connect(config):
            raise RuntimeError("Failed to connect to manipulator")

    def _register_component_methods(self):
        """Auto-register all RPC methods from components."""
        for component in self.components:
            for method_name in dir(component):
                if method_name.startswith('rpc_'):
                    method = getattr(component, method_name)
                    # Register with blueprint system
                    self.register_rpc(method_name, method)

    def start(self):
        """Start all threads."""
        self.threads = [
            Thread(target=self._state_reader_thread, daemon=True),
            Thread(target=self._command_sender_thread, daemon=True),
            Thread(target=self._state_publisher_thread, daemon=True),
        ]
        for thread in self.threads:
            thread.start()

    def _state_reader_thread(self):
        """Read hardware state at 100Hz."""
        while not self.stop_event.is_set():
            try:
                # Read from SDK
                positions = self.sdk.get_joint_positions()
                velocities = self.sdk.get_joint_velocities()
                error_code = self.sdk.get_error_code()

                # Update shared state
                with self.shared_state.lock:
                    self.shared_state.joint_positions = positions
                    self.shared_state.joint_velocities = velocities
                    self.shared_state.error_code = error_code

            except Exception as e:
                print(f"State reader error: {e}")

            time.sleep(0.01)  # 100Hz

    def stop(self):
        """Stop all threads and disconnect."""
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=1.0)
        self.sdk.disconnect()
```

### 5. Final Driver Assembly

```python
# xarm/driver.py
from dimos.hardware.manipulators.base import BaseManipulatorDriver
from dimos.hardware.manipulators.base.sdk_wrappers import XArmSDKWrapper
from dimos.hardware.manipulators.base.components import (
    StandardMotionComponent,
    StandardServoComponent,
    StandardStatusComponent,
    StandardTrajectoryComponent,
)

class XArmDriver(BaseManipulatorDriver):
    """XArm driver using component-based architecture."""

    def __init__(self, config: dict):
        # Create SDK wrapper
        sdk = XArmSDKWrapper()

        # Create shared state
        shared_state = SharedState()

        # Assemble standard components
        components = [
            StandardMotionComponent(sdk, shared_state),
            StandardServoComponent(sdk, shared_state),
            StandardStatusComponent(sdk, shared_state),
            StandardTrajectoryComponent(sdk, shared_state),
        ]

        # Add XArm-specific components if needed
        if config.get('has_gripper'):
            components.append(StandardGripperComponent(sdk, shared_state))

        # Initialize base driver
        super().__init__(sdk, components, config)
```

## Implementation Steps

### Step 1: Create Base Framework (Week 1)
1. Create `base/sdk_interface.py` with `BaseManipulatorSDK`
2. Create `base/driver.py` with `BaseManipulatorDriver`
3. Create `base/utils/` with shared utilities
4. Set up threading and state management

### Step 2: Build Standard Components (Week 2)
1. Create `components/motion.py`
2. Create `components/servo.py`
3. Create `components/status.py`
4. Create `components/trajectory.py`
5. Create comprehensive tests for each

### Step 3: Implement SDK Wrappers (Week 3)
1. Create `sdk_wrappers/xarm_wrapper.py`
2. Create `sdk_wrappers/piper_wrapper.py`
3. Test wrappers with hardware

### Step 4: Assemble Drivers (Week 4)
1. Create `xarm/driver.py` (assembly only)
2. Create `piper/driver.py` (assembly only)
3. Integration testing

## Adding New Manipulators

To add a new manipulator (e.g., Universal Robots UR5):

### 1. Create SDK Wrapper (200 lines)
```python
# sdk_wrappers/ur_wrapper.py
class URSDKWrapper(BaseManipulatorSDK):
    def get_joint_positions(self) -> list[float]:
        # UR already uses radians!
        return self.native_sdk.getj()

    def set_joint_positions(self, positions: list[float], velocity: float) -> bool:
        self.native_sdk.movej(positions, v=velocity)
        return True
    # ... implement other required methods
```

### 2. Create Driver (20 lines)
```python
# ur/driver.py
class UR5Driver(BaseManipulatorDriver):
    def __init__(self, config):
        sdk = URSDKWrapper()
        components = [
            StandardMotionComponent(sdk),  # Reuse!
            StandardServoComponent(sdk),   # Reuse!
            StandardStatusComponent(sdk),  # Reuse!
        ]
        super().__init__(sdk, components, config)
```

## Advantages

1. **Maximum Code Reuse**: Components tested once, used by 100+ arms
2. **Consistent Behavior**: All arms behave identically at RPC level
3. **Centralized Bug Fixes**: Fix once in component, all arms benefit
4. **Strong Testing**: Components can be thoroughly tested in isolation
5. **Clear Contracts**: SDK interface defines exact requirements
6. **Team Scalability**: Different developers can work on wrappers independently
7. **Maintainability**: Clear separation of concerns

## Disadvantages

1. **Initial Complexity**: More abstractions to understand
2. **More Files**: SDK wrapper + driver for each arm
3. **Rigid Structure**: Must fit into the abstraction
4. **Performance Overhead**: Extra function call layer
5. **Learning Curve**: New developers need to understand the framework

## When to Use This Architecture

Choose this architecture when:
- Building for 50+ manipulator types
- Multiple team members
- Need strict quality control
- Components have complex logic
- Building a commercial product
- Long-term maintenance is critical

## Testing Strategy

```python
# Test SDK wrapper in isolation
def test_xarm_wrapper_positions():
    mock_sdk = Mock()
    mock_sdk.get_servo_angle.return_value = (0, [0, 90, 180])

    wrapper = XArmSDKWrapper()
    wrapper.native_sdk = mock_sdk

    positions = wrapper.get_joint_positions()
    assert positions == [0, math.pi/2, math.pi]  # Converted to radians

# Test component with any SDK
def test_motion_component():
    mock_sdk = Mock(spec=BaseManipulatorSDK)
    mock_sdk.set_joint_positions.return_value = True

    component = StandardMotionComponent(mock_sdk, SharedState())
    result = component.rpc_move_joint([1, 2, 3])

    assert result["success"] == True
    mock_sdk.set_joint_positions.assert_called_once()
```

## Configuration Example

```yaml
# config/xarm7.yaml
manipulator:
  type: xarm
  sdk_wrapper: XArmSDKWrapper
  ip: 192.168.1.100
  dof: 7
  has_gripper: true
  has_force_torque: true
  components:
    - StandardMotionComponent
    - StandardServoComponent
    - StandardStatusComponent
    - StandardTrajectoryComponent
    - StandardGripperComponent
  joint_limits:
    lower: [-6.28, -2.09, -6.28, -3.14, -6.28, -3.14, -6.28]
    upper: [6.28, 2.09, 6.28, 3.14, 6.28, 3.14, 6.28]
```
