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

"""Base manipulator driver with threading and component management."""

from dataclasses import dataclass
import logging
from queue import Empty, Queue
from threading import Event, Thread
import time
from typing import Any

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import WrenchStamped
from dimos.msgs.sensor_msgs import JointCommand, JointState, RobotState

from .sdk_interface import BaseManipulatorSDK
from .spec import ManipulatorCapabilities
from .utils import SharedState


@dataclass
class Command:
    """Command to be sent to the manipulator."""

    type: str  # 'position', 'velocity', 'effort', 'cartesian', etc.
    data: Any
    timestamp: float = 0.0


class BaseManipulatorDriver(Module):
    """Base driver providing threading and component management.

    This class handles:
    - Thread management (state reader, command sender, state publisher)
    - Component registration and lifecycle
    - RPC method registration
    - Shared state management
    - Error handling and recovery
    - Pub/Sub with LCM transport for real-time control
    """

    # Input topics (commands from controllers - initialized by Module)
    joint_position_command: In[JointCommand] = None  # type: ignore[assignment]
    joint_velocity_command: In[JointCommand] = None  # type: ignore[assignment]

    # Output topics (state publishing - initialized by Module)
    joint_state: Out[JointState] = None  # type: ignore[assignment]
    robot_state: Out[RobotState] = None  # type: ignore[assignment]
    ft_sensor: Out[WrenchStamped] = None  # type: ignore[assignment]

    def __init__(
        self,
        sdk: BaseManipulatorSDK,
        components: list[Any],
        config: dict[str, Any],
        name: str | None = None,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        """Initialize the base manipulator driver.

        Args:
            sdk: SDK wrapper instance
            components: List of component instances
            config: Configuration dictionary
            name: Optional driver name for logging
            *args, **kwargs: Additional arguments for Module
        """
        # Initialize Module parent class
        super().__init__(*args, **kwargs)

        self.sdk = sdk
        self.components = components
        self.config: Any = config  # Config dict accessed as object
        self.name = name or self.__class__.__name__

        # Logging
        self.logger = logging.getLogger(self.name)

        # Shared state
        self.shared_state = SharedState()

        # Threading
        self.stop_event = Event()
        self.threads: list[Thread] = []
        self.command_queue: Queue[Any] = Queue(maxsize=10)

        # RPC registry
        self.rpc_methods: dict[str, Any] = {}
        self._exposed_component_apis: set[str] = set()  # Track auto-exposed method names

        # Capabilities
        self.capabilities = self._get_capabilities()

        # Rate control
        self.control_rate = config.get("control_rate", 100)  # Hz - control loop + joint feedback
        self.monitor_rate = config.get("monitor_rate", 10)  # Hz - robot state monitoring

        # Pre-allocate reusable objects (optimization: avoid per-cycle allocation)
        # Note: _joint_names is populated after _get_capabilities() sets self.capabilities
        self._joint_names: list[str] = [f"joint{i + 1}" for i in range(self.capabilities.dof)]

        # Initialize components with shared resources
        self._initialize_components()

        # Auto-expose component API methods as RPCs on the driver
        self._auto_expose_component_apis()

        # Connect to hardware
        self._connect()

    def _get_capabilities(self) -> ManipulatorCapabilities:
        """Get manipulator capabilities from config or SDK.

        Returns:
            ManipulatorCapabilities instance
        """
        # Try to get from SDK info
        info = self.sdk.get_info()

        # Get joint limits
        lower_limits, upper_limits = self.sdk.get_joint_limits()
        velocity_limits = self.sdk.get_velocity_limits()
        acceleration_limits = self.sdk.get_acceleration_limits()

        return ManipulatorCapabilities(
            dof=info.dof,
            has_gripper=self.config.get("has_gripper", False),
            has_force_torque=self.config.get("has_force_torque", False),
            has_impedance_control=self.config.get("has_impedance_control", False),
            has_cartesian_control=self.config.get("has_cartesian_control", False),
            max_joint_velocity=velocity_limits,
            max_joint_acceleration=acceleration_limits,
            joint_limits_lower=lower_limits,
            joint_limits_upper=upper_limits,
            payload_mass=self.config.get("payload_mass", 0.0),
            reach=self.config.get("reach", 0.0),
        )

    def _initialize_components(self) -> None:
        """Initialize components with shared resources."""
        for component in self.components:
            # Provide access to shared state
            if hasattr(component, "set_shared_state"):
                component.set_shared_state(self.shared_state)

            # Provide access to SDK
            if hasattr(component, "set_sdk"):
                component.set_sdk(self.sdk)

            # Provide access to command queue
            if hasattr(component, "set_command_queue"):
                component.set_command_queue(self.command_queue)

            # Provide access to capabilities
            if hasattr(component, "set_capabilities"):
                component.set_capabilities(self.capabilities)

            # Initialize component
            if hasattr(component, "initialize"):
                component.initialize()

    def _auto_expose_component_apis(self) -> None:
        """Auto-expose @component_api methods from components as RPC methods on the driver.

        This scans all components for methods decorated with @component_api and creates
        corresponding @rpc wrapper methods on the driver instance. This allows external
        code to call these methods via the standard Module RPC system.

        Example:
            # Component defines:
            @component_api
            def enable_servo(self): ...

            # Driver auto-generates an RPC wrapper, so external code can call:
            driver.enable_servo()

            # And the method is discoverable via:
            driver.rpcs  # Lists 'enable_servo' among available RPCs
        """
        for component in self.components:
            for method_name in dir(component):
                if method_name.startswith("_"):
                    continue

                method = getattr(component, method_name, None)
                if not callable(method) or not getattr(method, "__component_api__", False):
                    continue

                # Skip if driver already has a non-wrapper method with this name
                existing = getattr(self, method_name, None)
                if existing is not None and not getattr(
                    existing, "__component_api_wrapper__", False
                ):
                    self.logger.warning(
                        f"Driver already has method '{method_name}', skipping component API"
                    )
                    continue

                # Create RPC wrapper - use factory to properly capture method reference
                wrapper = self._create_component_api_wrapper(method)

                # Attach to driver instance
                setattr(self, method_name, wrapper)

                # Store in rpc_methods dict for backward compatibility
                self.rpc_methods[method_name] = wrapper

                # Track exposed method name for cleanup
                self._exposed_component_apis.add(method_name)

                self.logger.debug(f"Exposed component API as RPC: {method_name}")

    def _create_component_api_wrapper(self, component_method: Any) -> Any:
        """Create an RPC wrapper for a component API method.

        Args:
            component_method: The component method to wrap

        Returns:
            RPC-decorated wrapper function
        """
        import functools

        @rpc
        @functools.wraps(component_method)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            return component_method(*args, **kwargs)

        wrapper.__component_api_wrapper__ = True  # type: ignore[attr-defined]
        return wrapper

    def _connect(self) -> None:
        """Connect to the manipulator hardware."""
        self.logger.info(f"Connecting to {self.name}...")

        # Connect via SDK
        if not self.sdk.connect(self.config):
            raise RuntimeError(f"Failed to connect to {self.name}")

        self.shared_state.is_connected = True
        self.logger.info(f"Successfully connected to {self.name}")

        # Get initial state
        self._update_joint_state()
        self._update_robot_state()

    def _update_joint_state(self) -> None:
        """Update joint state from hardware (high frequency - 100Hz).

        Reads joint positions, velocities, efforts and publishes to LCM immediately.
        """
        try:
            # Get joint state feedback
            positions = self.sdk.get_joint_positions()
            velocities = self.sdk.get_joint_velocities()
            efforts = self.sdk.get_joint_efforts()

            self.shared_state.update_joint_state(
                positions=positions, velocities=velocities, efforts=efforts
            )

            # Publish joint state immediately at control rate
            if self.joint_state and hasattr(self.joint_state, "publish"):
                joint_state_msg = JointState(
                    ts=time.time(),
                    frame_id="joint-state",
                    name=self._joint_names,  # Pre-allocated list (optimization)
                    position=positions or [0.0] * self.capabilities.dof,
                    velocity=velocities or [0.0] * self.capabilities.dof,
                    effort=efforts or [0.0] * self.capabilities.dof,
                )
                self.joint_state.publish(joint_state_msg)

        except Exception as e:
            self.logger.error(f"Error updating joint state: {e}")

    def _update_robot_state(self) -> None:
        """Update robot state from hardware (low frequency - 10Hz).

        Reads robot mode, errors, warnings, optional states and publishes to LCM immediately.
        """
        try:
            # Get robot state (mode, errors, warnings)
            robot_state = self.sdk.get_robot_state()
            self.shared_state.update_robot_state(
                state=robot_state.get("state", 0),
                mode=robot_state.get("mode", 0),
                error_code=robot_state.get("error_code", 0),
                error_message=self.sdk.get_error_message(),
            )

            # Update status flags
            self.shared_state.is_moving = robot_state.get("is_moving", False)
            self.shared_state.is_enabled = self.sdk.are_servos_enabled()

            # Get optional states (cartesian, force/torque, gripper)
            if self.capabilities.has_cartesian_control:
                cart_pos = self.sdk.get_cartesian_position()
                if cart_pos:
                    self.shared_state.cartesian_position = cart_pos

            if self.capabilities.has_force_torque:
                ft = self.sdk.get_force_torque()
                if ft:
                    self.shared_state.force_torque = ft

            if self.capabilities.has_gripper:
                gripper_pos = self.sdk.get_gripper_position()
                if gripper_pos is not None:
                    self.shared_state.gripper_position = gripper_pos

            # Publish robot state immediately at monitor rate
            if self.robot_state and hasattr(self.robot_state, "publish"):
                robot_state_msg = RobotState(
                    state=self.shared_state.robot_state,
                    mode=self.shared_state.control_mode,
                    error_code=self.shared_state.error_code,
                    warn_code=0,
                )
                self.robot_state.publish(robot_state_msg)

            # Publish force/torque if available
            if (
                self.ft_sensor
                and hasattr(self.ft_sensor, "publish")
                and self.capabilities.has_force_torque
            ):
                if self.shared_state.force_torque:
                    ft_msg = WrenchStamped.from_force_torque_array(
                        ft_data=self.shared_state.force_torque,
                        frame_id="ft_sensor",
                        ts=time.time(),
                    )
                    self.ft_sensor.publish(ft_msg)

        except Exception as e:
            self.logger.error(f"Error updating robot state: {e}")
            self.shared_state.update_robot_state(error_code=999, error_message=str(e))

    # ============= Threading =============

    @rpc
    def start(self) -> None:
        """Start all driver threads and subscribe to input topics."""
        super().start()
        self.logger.info(f"Starting {self.name} driver threads...")

        # Subscribe to input topics if they have transports
        try:
            if self.joint_position_command and hasattr(self.joint_position_command, "subscribe"):
                self.joint_position_command.subscribe(self._on_joint_position_command)
                self.logger.debug("Subscribed to joint_position_command")
        except (AttributeError, ValueError) as e:
            self.logger.debug(f"joint_position_command transport not configured: {e}")

        try:
            if self.joint_velocity_command and hasattr(self.joint_velocity_command, "subscribe"):
                self.joint_velocity_command.subscribe(self._on_joint_velocity_command)
                self.logger.debug("Subscribed to joint_velocity_command")
        except (AttributeError, ValueError) as e:
            self.logger.debug(f"joint_velocity_command transport not configured: {e}")

        self.threads = [
            Thread(target=self._control_loop_thread, name=f"{self.name}-ControlLoop", daemon=True),
            Thread(
                target=self._robot_state_monitor_thread,
                name=f"{self.name}-StateMonitor",
                daemon=True,
            ),
        ]

        for thread in self.threads:
            thread.start()
            self.logger.debug(f"Started thread: {thread.name}")

        self.logger.info(f"{self.name} driver started successfully")

    def _control_loop_thread(self) -> None:
        """Control loop: send commands AND read joint feedback (100Hz).

        This tight loop ensures synchronized command/feedback for real-time control.
        """
        self.logger.debug("Control loop thread started")
        period = 1.0 / self.control_rate
        next_time = time.perf_counter() + period  # perf_counter for precise timing

        while not self.stop_event.is_set():
            try:
                # 1. Process all pending commands (non-blocking)
                while True:
                    try:
                        command = self.command_queue.get_nowait()  # Non-blocking (optimization)
                        self._process_command(command)
                    except Empty:
                        break  # No more commands

                # 2. Read joint state feedback (critical for control)
                self._update_joint_state()

            except Exception as e:
                self.logger.error(f"Control loop error: {e}")

            # Rate control - maintain precise timing
            next_time += period
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Fell behind - reset timing
                next_time = time.perf_counter() + period
                if sleep_time < -period:
                    self.logger.warning(f"Control loop fell behind by {-sleep_time:.3f}s")

        self.logger.debug("Control loop thread stopped")

    def _robot_state_monitor_thread(self) -> None:
        """Monitor robot state: mode, errors, warnings (10-20Hz).

        Lower frequency monitoring for high-level planning and error handling.
        """
        self.logger.debug("Robot state monitor thread started")
        period = 1.0 / self.monitor_rate
        next_time = time.perf_counter() + period  # perf_counter for precise timing

        while not self.stop_event.is_set():
            try:
                # Read robot state, mode, errors, optional states
                self._update_robot_state()
            except Exception as e:
                self.logger.error(f"Robot state monitor error: {e}")

            # Rate control
            next_time += period
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_time = time.perf_counter() + period

        self.logger.debug("Robot state monitor thread stopped")

    def _process_command(self, command: Command) -> None:
        """Process a command from the queue.

        Args:
            command: Command to process
        """
        try:
            if command.type == "position":
                success = self.sdk.set_joint_positions(
                    command.data["positions"],
                    command.data.get("velocity", 1.0),
                    command.data.get("acceleration", 1.0),
                    command.data.get("wait", False),
                )
                if success:
                    self.shared_state.target_positions = command.data["positions"]

            elif command.type == "velocity":
                success = self.sdk.set_joint_velocities(command.data["velocities"])
                if success:
                    self.shared_state.target_velocities = command.data["velocities"]

            elif command.type == "effort":
                success = self.sdk.set_joint_efforts(command.data["efforts"])
                if success:
                    self.shared_state.target_efforts = command.data["efforts"]

            elif command.type == "cartesian":
                success = self.sdk.set_cartesian_position(
                    command.data["pose"],
                    command.data.get("velocity", 1.0),
                    command.data.get("acceleration", 1.0),
                    command.data.get("wait", False),
                )
                if success:
                    self.shared_state.target_cartesian_position = command.data["pose"]

            elif command.type == "stop":
                self.sdk.stop_motion()

            else:
                self.logger.warning(f"Unknown command type: {command.type}")

        except Exception as e:
            self.logger.error(f"Error processing command {command.type}: {e}")

    # ============= Input Callbacks =============

    def _on_joint_position_command(self, cmd_msg: JointCommand) -> None:
        """Callback when joint position command is received.

        Args:
            cmd_msg: JointCommand message containing positions
        """
        command = Command(
            type="position", data={"positions": list(cmd_msg.positions)}, timestamp=time.time()
        )
        try:
            self.command_queue.put_nowait(command)
        except:
            self.logger.warning("Command queue full, dropping position command")

    def _on_joint_velocity_command(self, cmd_msg: JointCommand) -> None:
        """Callback when joint velocity command is received.

        Args:
            cmd_msg: JointCommand message containing velocities
        """
        command = Command(
            type="velocity",
            data={"velocities": list(cmd_msg.positions)},  # JointCommand uses 'positions' field
            timestamp=time.time(),
        )
        try:
            self.command_queue.put_nowait(command)
        except:
            self.logger.warning("Command queue full, dropping velocity command")

    # ============= Lifecycle Management =============

    @rpc
    def stop(self) -> None:
        """Stop all threads and disconnect from hardware."""
        self.logger.info(f"Stopping {self.name} driver...")

        # Signal threads to stop
        self.stop_event.set()

        # Stop any ongoing motion
        try:
            self.sdk.stop_motion()
        except:
            pass

        # Wait for threads to stop
        for thread in self.threads:
            thread.join(timeout=2.0)
            if thread.is_alive():
                self.logger.warning(f"Thread {thread.name} did not stop cleanly")

        # Disconnect from hardware
        try:
            self.sdk.disconnect()
        except:
            pass

        self.shared_state.is_connected = False
        self.logger.info(f"{self.name} driver stopped")

        # Call Module's stop
        super().stop()

    def __del__(self) -> None:
        """Cleanup on deletion."""
        if self.shared_state.is_connected:
            self.stop()

    # ============= RPC Method Access =============

    def get_rpc_method(self, method_name: str) -> Any:
        """Get an RPC method by name.

        Args:
            method_name: Name of the RPC method

        Returns:
            The method if found, None otherwise
        """
        return self.rpc_methods.get(method_name)

    def list_rpc_methods(self) -> list[str]:
        """List all available RPC methods.

        Returns:
            List of RPC method names
        """
        return list(self.rpc_methods.keys())

    # ============= Component Access =============

    def get_component(self, component_type: type[Any]) -> Any:
        """Get a component by type.

        Args:
            component_type: Type of component to find

        Returns:
            The component if found, None otherwise
        """
        for component in self.components:
            if isinstance(component, component_type):
                return component
        return None

    def add_component(self, component: Any) -> None:
        """Add a component at runtime.

        Args:
            component: Component instance to add
        """
        self.components.append(component)
        self._initialize_components()
        self._auto_expose_component_apis()

    def remove_component(self, component: Any) -> None:
        """Remove a component at runtime.

        Args:
            component: Component instance to remove
        """
        if component in self.components:
            self.components.remove(component)
            # Clean up old exposed methods and re-expose for remaining components
            self._cleanup_exposed_component_apis()
            self._auto_expose_component_apis()

    def _cleanup_exposed_component_apis(self) -> None:
        """Remove all auto-exposed component API methods from the driver."""
        for method_name in self._exposed_component_apis:
            if hasattr(self, method_name):
                delattr(self, method_name)
        self._exposed_component_apis.clear()
        self.rpc_methods.clear()
