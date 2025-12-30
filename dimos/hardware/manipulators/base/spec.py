# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass
from typing import Any, Protocol

from dimos.core import In, Out
from dimos.msgs.geometry_msgs import WrenchStamped
from dimos.msgs.sensor_msgs import JointCommand, JointState


@dataclass
class RobotState:
    """Universal robot state compatible with all manipulators."""

    # Core state fields (all manipulators must provide these)
    state: int = 0  # 0: idle, 1: moving, 2: error, 3: e-stop
    mode: int = 0  # 0: position, 1: velocity, 2: torque, 3: impedance
    error_code: int = 0  # Standardized error codes across all arms
    warn_code: int = 0  # Standardized warning codes

    # Extended state (optional, arm-specific)
    is_connected: bool = False
    is_enabled: bool = False
    is_moving: bool = False
    is_collision: bool = False

    # Vendor-specific data (if needed)
    vendor_data: dict[str, Any] | None = None


@dataclass
class ManipulatorCapabilities:
    """Describes what a manipulator can do."""

    dof: int  # Degrees of freedom
    has_gripper: bool = False
    has_force_torque: bool = False
    has_impedance_control: bool = False
    has_cartesian_control: bool = False
    max_joint_velocity: list[float] | None = None  # rad/s
    max_joint_acceleration: list[float] | None = None  # rad/s²
    joint_limits_lower: list[float] | None = None  # rad
    joint_limits_upper: list[float] | None = None  # rad
    payload_mass: float = 0.0  # kg
    reach: float = 0.0  # meters


class ManipulatorDriverSpec(Protocol):
    """Universal protocol specification for ALL manipulator drivers.

    This defines the standard interface that every manipulator driver
    must implement, regardless of the underlying hardware (XArm, Piper,
    UR, Franka, etc.).

    ## Component-Based Architecture

    Drivers use a **component-based architecture** where functionality is provided
    by composable components:

    - **StandardMotionComponent**: Joint/cartesian motion, trajectory execution
    - **StandardServoComponent**: Servo control, modes, emergency stop, error handling
    - **StandardStatusComponent**: State monitoring, capabilities, diagnostics

    RPC methods are provided by components and registered with the driver.
    Access them via:

    ```python
    # Method 1: Via component (direct access)
    motion = driver.get_component(StandardMotionComponent)
    motion.rpc_move_joint(positions=[0, 0, 0, 0, 0, 0])

    # Method 2: Via driver's RPC registry
    move_fn = driver.get_rpc_method('rpc_move_joint')
    move_fn(positions=[0, 0, 0, 0, 0, 0])

    # Method 3: Via blueprints (recommended - automatic routing)
    # Commands sent to input topics are automatically routed to components
    driver.joint_position_command.publish(JointCommand(positions=[0, 0, 0, 0, 0, 0]))
    ```

    ## Required Components

    Every driver must include these standard components:
    - `StandardMotionComponent` - Provides motion control RPC methods
    - `StandardServoComponent` - Provides servo control RPC methods
    - `StandardStatusComponent` - Provides status monitoring RPC methods

    ## Available RPC Methods (via Components)

    ### Motion Control (StandardMotionComponent)
    - `rpc_move_joint()` - Move to joint positions
    - `rpc_move_joint_velocity()` - Set joint velocities
    - `rpc_move_joint_effort()` - Set joint efforts (optional)
    - `rpc_stop_motion()` - Stop all motion
    - `rpc_get_joint_state()` - Get current joint state
    - `rpc_get_joint_limits()` - Get joint limits
    - `rpc_move_cartesian()` - Cartesian motion (optional)
    - `rpc_execute_trajectory()` - Execute trajectory (optional)

    ### Servo Control (StandardServoComponent)
    - `rpc_enable_servo()` - Enable motor control
    - `rpc_disable_servo()` - Disable motor control
    - `rpc_set_control_mode()` - Set control mode
    - `rpc_emergency_stop()` - Execute emergency stop
    - `rpc_clear_errors()` - Clear error states
    - `rpc_home_robot()` - Home the robot

    ### Status Monitoring (StandardStatusComponent)
    - `rpc_get_robot_state()` - Get robot state
    - `rpc_get_capabilities()` - Get capabilities
    - `rpc_get_system_info()` - Get system information
    - `rpc_check_connection()` - Check connection status

    ## Standardized Units

    All units are standardized:
    - Angles: radians
    - Angular velocity: rad/s
    - Linear position: meters
    - Linear velocity: m/s
    - Force: Newtons
    - Torque: Nm
    - Time: seconds
    """

    # ============= Capabilities Declaration =============
    capabilities: ManipulatorCapabilities

    # ============= Input Topics (Commands) =============
    # Core control inputs (all manipulators must support these)
    joint_position_command: In[JointCommand]  # Target joint positions (rad)
    joint_velocity_command: In[JointCommand]  # Target joint velocities (rad/s)

    # ============= Output Topics (Feedback) =============
    # Core feedback (all manipulators must provide these)
    joint_state: Out[JointState]  # Current positions, velocities, efforts
    robot_state: Out[RobotState]  # System state and health

    # Optional feedback (capability-dependent)
    ft_sensor: Out[WrenchStamped] | None  # Force/torque sensor data

    # ============= Component Access =============
    def get_component(self, component_type: type) -> Any:
        """Get a component by type.

        Args:
            component_type: Type of component to retrieve

        Returns:
            Component instance if found, None otherwise

        Example:
            motion = driver.get_component(StandardMotionComponent)
            motion.rpc_move_joint([0, 0, 0, 0, 0, 0])
        """
        pass

    def get_rpc_method(self, method_name: str) -> Any:
        """Get an RPC method by name.

        Args:
            method_name: Name of the RPC method (e.g., 'rpc_move_joint')

        Returns:
            Callable method if found, None otherwise

        Example:
            move_fn = driver.get_rpc_method('rpc_move_joint')
            result = move_fn(positions=[0, 0, 0, 0, 0, 0])
        """
        ...

    def list_rpc_methods(self) -> list[str]:
        """List all available RPC methods from all components.

        Returns:
            List of RPC method names

        Example:
            methods = driver.list_rpc_methods()
            # ['rpc_move_joint', 'rpc_enable_servo', 'rpc_get_robot_state', ...]
        """
        ...
