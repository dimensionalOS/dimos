# Copyright 2025-2026 Dimensional Inc.
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

"""ControlCoordinator module.

Centralized control coordinator that replaces per-driver/per-controller
loops with a single deterministic tick-based system.

Features:
- Single tick loop (read -> compute -> arbitrate -> route -> write)
- Per-joint arbitration (highest priority wins)
- Mode conflict detection
- Partial command support (hold last value)
- Aggregated preemption notifications
"""

from dataclasses import dataclass, field
import threading
import time
from typing import TYPE_CHECKING, Any

from dimos.control.components import (
    TWIST_SUFFIX_MAP,
    HardwareComponent,
    HardwareId,
    HardwareType,
    JointName,
    TaskName,
    split_joint_name,
)
from dimos.control.hardware_interface import (
    ConnectedHardware,
    ConnectedTwistBase,
    ConnectedWholeBody,
)
from dimos.control.task import ControlTask
from dimos.control.tick_loop import TickLoop
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.drive_trains.spec import (
    TwistBaseAdapter,
)
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.hardware.whole_body.spec import WholeBodyAdapter
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.quest.quest_types import (
    Buttons,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

logger = setup_logger()


@dataclass
class TaskConfig:
    """Configuration for a registered control task."""

    name: str
    type: str = "trajectory"
    joint_names: list[str] = field(default_factory=lambda: [])
    priority: int = 10
    auto_start: bool = False
    params: dict[str, Any] = field(default_factory=dict)


class ControlCoordinatorConfig(ModuleConfig):
    """Configuration for the ControlCoordinator."""

    tick_rate: float = 100.0
    publish_joint_state: bool = True
    joint_state_frame_id: str = "coordinator"
    log_ticks: bool = False
    hardware: list[HardwareComponent] = field(default_factory=lambda: [])
    tasks: list[TaskConfig] = field(default_factory=lambda: [])


class ControlCoordinator(Module):
    """Centralized control coordinator with per-joint arbitration.

    The coordinator is normally used as a DimOS blueprint module. Hardware
    adapters and control tasks are described declaratively in
    ``ControlCoordinatorConfig`` and instantiated when the module starts.

    Per tick, the coordinator:
    1. Reads state from configured hardware
    2. Runs active tasks
    3. Arbitrates conflicting commands per joint (highest priority wins)
    4. Routes commands to the owning hardware adapter
    5. Publishes the aggregated canonical joint state

    Key design decisions:
    - Joint-centric commands (not hardware-centric)
    - Per-joint arbitration (not per-hardware)
    - Centralized time (tasks use state.t_now, never time.time())
    - Partial commands OK (hardware holds last value)
    - Aggregated preemption (one notification per task per tick)

    Example:
        >>> from dimos.control.components import HardwareComponent, HardwareType
        >>> from dimos.control.components import make_joints
        >>> from dimos.control.coordinator import ControlCoordinator, TaskConfig
        >>>
        >>> coordinator = ControlCoordinator.blueprint(
        ...     tick_rate=100.0,
        ...     hardware=[
        ...         HardwareComponent(
        ...             hardware_id="arm",
        ...             hardware_type=HardwareType.MANIPULATOR,
        ...             joints=make_joints("arm", 7),
        ...             adapter_type="xarm",
        ...             address="192.168.1.185",
        ...         ),
        ...     ],
        ...     tasks=[
        ...         TaskConfig(
        ...             name="traj_arm",
        ...             type="trajectory",
        ...             joint_names=make_joints("arm", 7),
        ...             priority=10,
        ...         ),
        ...     ],
        ... )
    """

    config: ControlCoordinatorConfig

    # Output: Aggregated joint state for external consumers
    coordinator_joint_state: Out[JointState]

    # Input: Streaming joint commands for real-time control
    joint_command: In[JointState]

    # Input: Streaming cartesian commands for CartesianIKTask
    # Uses frame_id as task name for routing
    coordinator_cartesian_command: In[PoseStamped]

    # Input: Streaming twist commands for velocity-commanded platforms
    twist_command: In[Twist]

    # Input: Teleop buttons for engage/disengage signaling
    teleop_buttons: In[Buttons]

    # Arming and dry-run are one-shot RPCs, not streams.

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        # Connected hardware (keyed by hardware_id)
        self._hardware: dict[HardwareId, ConnectedHardware | ConnectedWholeBody] = {}
        self._hardware_lock = threading.Lock()

        # Joint -> hardware mapping (built when hardware added)
        self._joint_to_hardware: dict[JointName, HardwareId] = {}

        # Registered tasks
        self._tasks: dict[TaskName, ControlTask] = {}
        self._task_lock = threading.Lock()

        # Tick loop (created on start)
        self._tick_loop: TickLoop | None = None

        # Subscription handles for streaming commands
        self._joint_command_unsub: Callable[[], None] | None = None
        self._cartesian_command_unsub: Callable[[], None] | None = None
        self._twist_command_unsub: Callable[[], None] | None = None
        self._buttons_unsub: Callable[[], None] | None = None

        logger.info(f"ControlCoordinator initialized at {self.config.tick_rate}Hz")

    def _setup_from_config(self) -> None:
        """Create hardware and tasks from config (called on start)."""
        hardware_added: list[str] = []

        try:
            for component in self.config.hardware:
                self._setup_hardware(component)
                hardware_added.append(component.hardware_id)

            for task_cfg in self.config.tasks:
                task = self._create_task_from_config(task_cfg)
                self.add_task(task)
                if task_cfg.auto_start:
                    start = getattr(task, "start", None)
                    if callable(start):
                        start()

        except Exception:
            # Rollback: clean up all successfully added hardware
            for hw_id in hardware_added:
                try:
                    self.remove_hardware(hw_id)
                except Exception:
                    pass
            raise

    def _setup_hardware(self, component: HardwareComponent) -> None:
        """Connect and add a single hardware adapter."""
        adapter: ManipulatorAdapter | TwistBaseAdapter | WholeBodyAdapter
        if component.hardware_type == HardwareType.WHOLE_BODY:
            adapter = self._create_whole_body_adapter(component)
        elif component.hardware_type == HardwareType.BASE:
            adapter = self._create_twist_base_adapter(component)
        else:
            adapter = self._create_adapter(component)

        if not adapter.connect():
            raise RuntimeError(f"Failed to connect to {component.adapter_type} adapter")

        try:
            if component.auto_enable:
                activate = getattr(adapter, "activate", None)
                if callable(activate):
                    if activate() is False:
                        raise RuntimeError(f"Failed to activate hardware {component.hardware_id}")
                elif hasattr(adapter, "write_enable"):
                    adapter.write_enable(True)

            self.add_hardware(adapter, component)
        except Exception:
            adapter.disconnect()
            raise

    def _create_adapter(self, component: HardwareComponent) -> ManipulatorAdapter:
        """Create a manipulator adapter from component config."""
        from dimos.hardware.manipulators.registry import adapter_registry

        return adapter_registry.create(
            component.adapter_type,
            dof=len(component.joints),
            address=component.address,
            hardware_id=component.hardware_id,
            **component.adapter_kwargs,
        )

    def _create_twist_base_adapter(self, component: HardwareComponent) -> TwistBaseAdapter:
        """Create a twist base adapter from component config."""
        from dimos.hardware.drive_trains.registry import twist_base_adapter_registry

        return twist_base_adapter_registry.create(
            component.adapter_type,
            dof=len(component.joints),
            address=component.address,
            hardware_id=component.hardware_id,
            **component.adapter_kwargs,
        )

    def _create_whole_body_adapter(self, component: HardwareComponent) -> WholeBodyAdapter:
        """Create a whole-body adapter from component config."""
        from dimos.hardware.whole_body.registry import whole_body_adapter_registry

        return whole_body_adapter_registry.create(
            component.adapter_type,
            dof=len(component.joints),
            hardware_id=component.hardware_id,
            address=component.address,
            domain_id=component.domain_id,
            **component.adapter_kwargs,
        )

    def _create_task_from_config(self, cfg: TaskConfig) -> ControlTask:
        """Create a control task from config via the task registry."""
        from dimos.control.tasks.registry import control_task_registry

        return control_task_registry.create(cfg.type, cfg, hardware=self._hardware)

    @rpc
    def add_hardware(
        self,
        adapter: ManipulatorAdapter | TwistBaseAdapter | WholeBodyAdapter,
        component: HardwareComponent,
    ) -> bool:
        """Register a hardware adapter with the coordinator."""
        is_base = component.hardware_type == HardwareType.BASE
        is_whole_body = component.hardware_type == HardwareType.WHOLE_BODY

        if is_base and not isinstance(adapter, TwistBaseAdapter):
            raise TypeError(
                f"Hardware type / adapter mismatch for '{component.hardware_id}': "
                f"hardware_type={component.hardware_type.value} but got "
                f"{type(adapter).__name__}"
            )

        if is_whole_body and not isinstance(adapter, WholeBodyAdapter):
            raise TypeError(
                f"Hardware type / adapter mismatch for '{component.hardware_id}': "
                f"hardware_type={component.hardware_type.value} but got "
                f"{type(adapter).__name__}"
            )

        with self._hardware_lock:
            if component.hardware_id in self._hardware:
                logger.warning(f"Hardware {component.hardware_id} already registered")
                return False

            if isinstance(adapter, WholeBodyAdapter):
                connected: ConnectedHardware = ConnectedWholeBody(
                    adapter=adapter,
                    component=component,
                )
            elif isinstance(adapter, TwistBaseAdapter):
                connected = ConnectedTwistBase(
                    adapter=adapter,
                    component=component,
                )
            else:
                connected = ConnectedHardware(
                    adapter=adapter,
                    component=component,
                )

            self._hardware[component.hardware_id] = connected

            for joint_name in connected.joint_names:
                self._joint_to_hardware[joint_name] = component.hardware_id

            logger.info(
                f"Added hardware {component.hardware_id} with joints: {connected.joint_names}"
            )
            return True

    @rpc
    def remove_hardware(self, hardware_id: str) -> bool:
        """Remove a hardware interface.

        Note: For safety, call this only when no tasks are actively using this
        hardware. Consider stopping the coordinator before removing hardware.
        """
        with self._hardware_lock:
            if hardware_id not in self._hardware:
                return False

            interface = self._hardware[hardware_id]
            hw_joints = set(interface.joint_names)

            with self._task_lock:
                for task in self._tasks.values():
                    if task.is_active():
                        claimed_joints = task.claim().joints
                        overlap = hw_joints & claimed_joints
                        if overlap:
                            logger.error(
                                f"Cannot remove hardware {hardware_id}: "
                                f"task '{task.name}' is actively using joints {overlap}"
                            )
                            return False

            for joint_name in interface.joint_names:
                del self._joint_to_hardware[joint_name]

            interface.disconnect()
            del self._hardware[hardware_id]
            logger.info(f"Removed hardware {hardware_id}")
            return True

    @rpc
    def list_hardware(self) -> list[str]:
        """List registered hardware IDs."""
        with self._hardware_lock:
            return list(self._hardware.keys())

    @rpc
    def list_joints(self) -> list[str]:
        """List all joint names across all hardware."""
        with self._hardware_lock:
            return list(self._joint_to_hardware.keys())

    @rpc
    def get_joint_positions(self) -> dict[str, float]:
        """Get current joint positions for all joints."""
        with self._hardware_lock:
            positions: dict[str, float] = {}
            for hw in self._hardware.values():
                state = hw.read_state()  # {joint_name: JointState}
                for joint_name, joint_state in state.items():
                    positions[joint_name] = joint_state.position
            return positions

    @rpc
    def add_task(self, task: ControlTask) -> bool:
        """Register a task with the coordinator."""
        if not isinstance(task, ControlTask):
            raise TypeError("task must implement ControlTask")

        with self._task_lock:
            if task.name in self._tasks:
                logger.warning(f"Task {task.name} already registered")
                return False
            self._tasks[task.name] = task
            logger.info(f"Added task {task.name}")
            return True

    @rpc
    def remove_task(self, task_name: TaskName) -> bool:
        """Remove a task by name."""
        with self._task_lock:
            if task_name in self._tasks:
                del self._tasks[task_name]
                logger.info(f"Removed task {task_name}")
                return True
            return False

    @rpc
    def get_task(self, task_name: TaskName) -> ControlTask | None:
        """Get a task by name."""
        with self._task_lock:
            return self._tasks.get(task_name)

    @rpc
    def list_tasks(self) -> list[str]:
        """List registered task names."""
        with self._task_lock:
            return list(self._tasks.keys())

    @rpc
    def get_active_tasks(self) -> list[str]:
        """List currently active task names."""
        with self._task_lock:
            return [name for name, task in self._tasks.items() if task.is_active()]

    def _on_joint_command(self, msg: JointState) -> None:
        """Route incoming JointState to streaming tasks by joint name.

        Routes position data to servo tasks and velocity data to velocity tasks.
        Each task only receives data for joints it claims.
        """
        if not msg.name:
            return

        t_now = time.perf_counter()
        incoming_joints = set(msg.name)

        with self._task_lock:
            for task in self._tasks.values():
                claimed_joints = task.claim().joints

                # Skip if no overlap between incoming and claimed joints
                if not (claimed_joints & incoming_joints):
                    continue

                # Route to servo tasks (position control)
                if msg.position:
                    positions_by_name = dict(zip(msg.name, msg.position, strict=False))
                    task.set_target_by_name(positions_by_name, t_now)

                # Route to velocity tasks (velocity control)
                elif msg.velocity:
                    velocities_by_name = dict(zip(msg.name, msg.velocity, strict=False))
                    task.set_velocities_by_name(velocities_by_name, t_now)

    def _on_cartesian_command(self, msg: PoseStamped) -> None:
        """Route incoming PoseStamped to CartesianIKTask by task name.

        Uses frame_id as the target task name for routing.
        """
        task_name = msg.frame_id
        if not task_name:
            logger.warning("Received cartesian_command with empty frame_id (task name)")
            return

        t_now = time.perf_counter()

        with self._task_lock:
            task = self._tasks.get(task_name)
            if task is None:
                logger.warning(f"Cartesian command for unknown task: {task_name}")
                return

            task.on_cartesian_command(msg, t_now)

    def _on_twist_command(self, msg: Twist) -> None:
        """Convert Twist → virtual joint velocities and route via _on_joint_command.

        Maps Twist fields to virtual joints using suffix convention:
        base_vx ← linear.x, base_vy ← linear.y, base_wz ← angular.z, etc.
        """
        names: list[str] = []
        velocities: list[float] = []

        with self._hardware_lock:
            for hw in self._hardware.values():
                if hw.component.hardware_type != HardwareType.BASE:
                    continue
                for joint_name in hw.joint_names:
                    # Extract suffix (e.g., "base/vx" → "vx")
                    _, suffix = split_joint_name(joint_name)
                    mapping = TWIST_SUFFIX_MAP.get(suffix)
                    if mapping is None:
                        continue
                    group, axis = mapping
                    value = getattr(getattr(msg, group), axis)
                    names.append(joint_name)
                    velocities.append(value)

        if names:
            joint_state = JointState(name=names, velocity=velocities)
            self._on_joint_command(joint_state)

        # Velocity-capable tasks opt in with set_velocity_command().
        t_now = time.perf_counter()
        with self._task_lock:
            for task in self._tasks.values():
                set_vel = getattr(task, "set_velocity_command", None)
                if set_vel is not None:
                    set_vel(msg.linear.x, msg.linear.y, msg.angular.z, t_now)

    def _on_buttons(self, msg: Buttons) -> None:
        """Forward button state to all tasks."""
        with self._task_lock:
            for task in self._tasks.values():
                task.on_buttons(msg)

    @rpc
    def set_activated(self, engaged: bool) -> None:
        """Arm/disarm every task exposing ``arm()`` / ``disarm()``."""
        with self._task_lock:
            for task in self._tasks.values():
                method_name = "arm" if engaged else "disarm"
                handler = getattr(task, method_name, None)
                if callable(handler):
                    try:
                        handler()
                    except Exception:
                        logger.exception(f"{method_name}() raised on task {task.name!r}")

    @rpc
    def set_dry_run(self, enabled: bool) -> None:
        """Toggle dry-run on every task exposing ``set_dry_run``."""
        with self._task_lock:
            for task in self._tasks.values():
                handler = getattr(task, "set_dry_run", None)
                if callable(handler):
                    try:
                        handler(enabled)
                    except Exception:
                        logger.exception(f"set_dry_run() raised on task {task.name!r}")

    @rpc
    def reset_runtime_state(self, reactivate: bool | None = None) -> dict[str, bool]:
        """Reset transient state on tasks that expose ``reset_runtime_state``.

        This is meant for simulation/runtime discontinuities such as MuJoCo
        respawn, where task histories and latched commands must be cleared
        without tearing down the coordinator.
        """
        results: dict[str, bool] = {}
        with self._task_lock:
            for task in self._tasks.values():
                try:
                    results[task.name] = bool(task.reset_runtime_state(reactivate=reactivate))
                except Exception:
                    logger.exception(f"reset_runtime_state() raised on task {task.name!r}")
                    results[task.name] = False
        return results

    @rpc
    def task_invoke(
        self, task_name: TaskName, method: str, kwargs: dict[str, Any] | None = None
    ) -> Any:
        """Invoke a method on a task. Pass t_now=None to auto-inject current time."""
        with self._task_lock:
            task = self._tasks.get(task_name)
            if task is None:
                logger.warning(f"Task {task_name} not found")
                return None

            if not hasattr(task, method):
                logger.warning(f"Task {task_name} has no method {method}")
                return None

            kwargs = kwargs or {}

            # Auto-inject t_now if requested (None means "use current time")
            if "t_now" in kwargs and kwargs["t_now"] is None:
                kwargs["t_now"] = time.perf_counter()

            return getattr(task, method)(**kwargs)

    @rpc
    def set_gripper_position(self, hardware_id: str, position: float) -> bool:
        """Set gripper position on a specific hardware device.

        Args:
            hardware_id: ID of the hardware with the gripper
            position: Gripper position in meters
        """
        with self._hardware_lock:
            hw = self._hardware.get(hardware_id)
            if hw is None:
                logger.warning(f"Hardware '{hardware_id}' not found for gripper command")
                return False
            if isinstance(hw, ConnectedTwistBase):
                logger.warning(f"Hardware '{hardware_id}' is a twist base, no gripper support")
                return False
            return hw.adapter.write_gripper_position(position)

    @rpc
    def get_gripper_position(self, hardware_id: str) -> float | None:
        """Get gripper position from a specific hardware device.

        Args:
            hardware_id: ID of the hardware with the gripper
        """
        with self._hardware_lock:
            hw = self._hardware.get(hardware_id)
            if hw is None:
                return None
            if isinstance(hw, ConnectedTwistBase):
                return None
            return hw.adapter.read_gripper_position()

    @rpc
    def start(self) -> None:
        """Start the coordinator control loop."""
        if self._tick_loop and self._tick_loop.is_running:
            logger.warning("Coordinator already running")
            return

        super().start()

        # Setup hardware and tasks from config (if any)
        if self.config.hardware or self.config.tasks:
            self._setup_from_config()

        # Create and start tick loop
        publish_cb = (
            self.coordinator_joint_state.publish if self.config.publish_joint_state else None
        )
        self._tick_loop = TickLoop(
            tick_rate=self.config.tick_rate,
            hardware=self._hardware,
            hardware_lock=self._hardware_lock,
            tasks=self._tasks,
            task_lock=self._task_lock,
            joint_to_hardware=self._joint_to_hardware,
            publish_callback=publish_cb,
            frame_id=self.config.joint_state_frame_id,
            log_ticks=self.config.log_ticks,
        )
        self._tick_loop.start()

        # Subscribe to joint commands if any streaming tasks configured
        streaming_types = ("servo", "velocity")
        has_streaming = any(t.type in streaming_types for t in self.config.tasks)
        if has_streaming:
            try:
                self._joint_command_unsub = self.joint_command.subscribe(self._on_joint_command)
                logger.info("Subscribed to joint_command for streaming tasks")
            except Exception:
                logger.warning(
                    "Streaming tasks configured but could not subscribe to joint_command. "
                    "Use task_invoke RPC or set transport via blueprint."
                )

        # Subscribe to cartesian commands if any cartesian_ik tasks configured
        has_cartesian_ik = any(t.type in ("cartesian_ik", "teleop_ik") for t in self.config.tasks)
        if has_cartesian_ik:
            try:
                self._cartesian_command_unsub = self.coordinator_cartesian_command.subscribe(
                    self._on_cartesian_command
                )
                logger.info("Subscribed to cartesian_command for CartesianIK/TeleopIK tasks")
            except Exception:
                logger.warning(
                    "CartesianIK/TeleopIK tasks configured but could not subscribe to cartesian_command. "
                    "Use task_invoke RPC or set transport via blueprint."
                )

        # Twist commands drive either base hardware or velocity-capable tasks.
        has_twist_base = any(c.hardware_type == HardwareType.BASE for c in self.config.hardware)
        with self._task_lock:
            has_velocity_task = any(
                callable(getattr(task, "set_velocity_command", None))
                for task in self._tasks.values()
            )
        if has_twist_base or has_velocity_task:
            try:
                self._twist_command_unsub = self.twist_command.subscribe(self._on_twist_command)
                logger.info("Subscribed to twist_command for twist base / velocity-capable tasks")
            except Exception:
                logger.warning(
                    "Twist base or velocity-capable task configured but could not subscribe "
                    "to twist_command. Use task_invoke RPC or set transport via blueprint."
                )

        # Subscribe to buttons if any teleop_ik tasks configured (engage/disengage)
        has_teleop_ik = any(t.type == "teleop_ik" for t in self.config.tasks)
        if has_teleop_ik:
            self._buttons_unsub = self.teleop_buttons.subscribe(self._on_buttons)
            logger.info("Subscribed to buttons for engage/disengage")

        # Arming + dry-run are RPC-only; no stream subscription here.

        logger.info(f"ControlCoordinator started at {self.config.tick_rate}Hz")

    @rpc
    def stop(self) -> None:
        """Stop the coordinator."""
        logger.info("Stopping ControlCoordinator...")

        # Unsubscribe from streaming commands
        if self._joint_command_unsub:
            self._joint_command_unsub()
            self._joint_command_unsub = None
        if self._cartesian_command_unsub:
            self._cartesian_command_unsub()
            self._cartesian_command_unsub = None
        if self._twist_command_unsub:
            self._twist_command_unsub()
            self._twist_command_unsub = None
        if self._buttons_unsub:
            self._buttons_unsub()
            self._buttons_unsub = None

        if self._tick_loop:
            self._tick_loop.stop()

        with self._hardware_lock:
            for hw_id, interface in self._hardware.items():
                deactivate = getattr(interface.adapter, "deactivate", None)
                if not callable(deactivate):
                    continue
                try:
                    if deactivate() is False:
                        logger.error(f"Hardware {hw_id} deactivate returned False")
                except Exception as e:
                    logger.error(f"Error deactivating hardware {hw_id}: {e}")

        # Disconnect all hardware adapters
        with self._hardware_lock:
            for hw_id, interface in self._hardware.items():
                try:
                    interface.disconnect()
                    logger.info(f"Disconnected hardware {hw_id}")
                except Exception as e:
                    logger.error(f"Error disconnecting hardware {hw_id}: {e}")

        super().stop()
        logger.info("ControlCoordinator stopped")

    @rpc
    def get_tick_count(self) -> int:
        """Get the number of ticks since start."""
        return self._tick_loop.tick_count if self._tick_loop else 0
