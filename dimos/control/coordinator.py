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

from __future__ import annotations

from dataclasses import dataclass, field
import threading
import time
from typing import TYPE_CHECKING, Any

from dimos.control.components import HardwareComponent, HardwareId, JointName, TaskName
from dimos.control.hardware_interface import ConnectedHardware
from dimos.control.task import ControlTask
from dimos.control.tick_loop import TickLoop
from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs import (
    PoseStamped,  # noqa: TC001 - needed at runtime for In[PoseStamped]
)
from dimos.msgs.sensor_msgs import (
    JointState,  # noqa: TC001 - needed at runtime for Out[JointState]
)
from dimos.teleop.quest.quest_types import Buttons  # noqa: TC001 - needed for teleop buttons
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable
    from pathlib import Path

    from dimos.hardware.manipulators.spec import ManipulatorAdapter

logger = setup_logger()


# =============================================================================
# Configuration
# =============================================================================


@dataclass
class TaskConfig:
    """Configuration for a control task.

    Attributes:
        name: Task name (e.g., "traj_arm")
        type: Task type ("trajectory", "servo", "velocity", "cartesian_ik", "teleop_ik")
        joint_names: List of joint names this task controls
        priority: Task priority (higher wins arbitration)
        model_path: Path to URDF/MJCF for IK solver (cartesian_ik/teleop_ik only)
        ee_joint_id: End-effector joint ID in model (cartesian_ik/teleop_ik only)
    """

    name: str
    type: str = "trajectory"
    joint_names: list[str] = field(default_factory=lambda: [])
    priority: int = 10
    # Cartesian IK / Teleop IK specific
    model_path: str | Path | None = None
    ee_joint_id: int = 6
    hand: str = ""  # teleop_ik only: "left" or "right" controller


@dataclass
class ControlCoordinatorConfig(ModuleConfig):
    """Configuration for the ControlCoordinator.

    Attributes:
        tick_rate: Control loop frequency in Hz (default: 100)
        publish_joint_state: Whether to publish aggregated JointState
        joint_state_frame_id: Frame ID for published JointState
        log_ticks: Whether to log tick information (verbose)
        hardware: List of hardware configurations to create on start
        tasks: List of task configurations to create on start
    """

    tick_rate: float = 100.0
    publish_joint_state: bool = True
    joint_state_frame_id: str = "coordinator"
    log_ticks: bool = False
    hardware: list[HardwareComponent] = field(default_factory=lambda: [])
    tasks: list[TaskConfig] = field(default_factory=lambda: [])


# =============================================================================
# ControlCoordinator Module
# =============================================================================


class ControlCoordinator(Module[ControlCoordinatorConfig]):
    """Centralized control coordinator with per-joint arbitration.

    Single tick loop that:
    1. Reads state from all hardware
    2. Runs all active tasks
    3. Arbitrates conflicts per-joint (highest priority wins)
    4. Routes commands to hardware
    5. Publishes aggregated joint state

    Key design decisions:
    - Joint-centric commands (not hardware-centric)
    - Per-joint arbitration (not per-hardware)
    - Centralized time (tasks use state.t_now, never time.time())
    - Partial commands OK (hardware holds last value)
    - Aggregated preemption (one notification per task per tick)

    Example:
        >>> from dimos.control import ControlCoordinator
        >>> from dimos.hardware.manipulators.xarm import XArmAdapter
        >>>
        >>> orch = ControlCoordinator(tick_rate=100.0)
        >>> adapter = XArmAdapter(ip="192.168.1.185", dof=7)
        >>> adapter.connect()
        >>> orch.add_hardware("left_arm", adapter, joint_prefix="left")
        >>> orch.start()
    """

    # Output: Aggregated joint state for external consumers
    joint_state: Out[JointState]

    # Input: Streaming joint commands for real-time control
    joint_command: In[JointState]

    # Input: Streaming cartesian commands for CartesianIKTask
    # Uses frame_id as task name for routing
    cartesian_command: In[PoseStamped]

    # Input: Teleop buttons for engage/disengage signaling
    buttons: In[Buttons]

    config: ControlCoordinatorConfig
    default_config = ControlCoordinatorConfig

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        # Connected hardware (keyed by hardware_id)
        self._hardware: dict[HardwareId, ConnectedHardware] = {}
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
        self._buttons_unsub: Callable[[], None] | None = None

        logger.info(f"ControlCoordinator initialized at {self.config.tick_rate}Hz")

    # =========================================================================
    # Config-based Setup
    # =========================================================================

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
        adapter = self._create_adapter(component)

        if not adapter.connect():
            raise RuntimeError(f"Failed to connect to {component.adapter_type} adapter")

        try:
            if component.auto_enable and hasattr(adapter, "write_enable"):
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
        )

    def _create_task_from_config(self, cfg: TaskConfig) -> ControlTask:
        """Create a control task from config."""
        task_type = cfg.type.lower()

        if task_type == "trajectory":
            from dimos.control.tasks import JointTrajectoryTask, JointTrajectoryTaskConfig

            return JointTrajectoryTask(
                cfg.name,
                JointTrajectoryTaskConfig(
                    joint_names=cfg.joint_names,
                    priority=cfg.priority,
                ),
            )

        elif task_type == "servo":
            from dimos.control.tasks import JointServoTask, JointServoTaskConfig

            return JointServoTask(
                cfg.name,
                JointServoTaskConfig(
                    joint_names=cfg.joint_names,
                    priority=cfg.priority,
                ),
            )

        elif task_type == "velocity":
            from dimos.control.tasks import JointVelocityTask, JointVelocityTaskConfig

            return JointVelocityTask(
                cfg.name,
                JointVelocityTaskConfig(
                    joint_names=cfg.joint_names,
                    priority=cfg.priority,
                ),
            )

        elif task_type == "cartesian_ik":
            from dimos.control.tasks import CartesianIKTask, CartesianIKTaskConfig

            if cfg.model_path is None:
                raise ValueError(f"CartesianIKTask '{cfg.name}' requires model_path in TaskConfig")

            return CartesianIKTask(
                cfg.name,
                CartesianIKTaskConfig(
                    joint_names=cfg.joint_names,
                    model_path=cfg.model_path,
                    ee_joint_id=cfg.ee_joint_id,
                    priority=cfg.priority,
                ),
            )

        elif task_type == "teleop_ik":
            from dimos.control.tasks.teleop_task import TeleopIKTask, TeleopIKTaskConfig

            if cfg.model_path is None:
                raise ValueError(f"TeleopIKTask '{cfg.name}' requires model_path in TaskConfig")

            return TeleopIKTask(
                cfg.name,
                TeleopIKTaskConfig(
                    joint_names=cfg.joint_names,
                    model_path=cfg.model_path,
                    ee_joint_id=cfg.ee_joint_id,
                    priority=cfg.priority,
                    hand=cfg.hand,
                ),
            )

        else:
            raise ValueError(f"Unknown task type: {task_type}")

    # =========================================================================
    # Hardware Management (RPC)
    # =========================================================================

    @rpc
    def add_hardware(
        self,
        adapter: ManipulatorAdapter,
        component: HardwareComponent,
    ) -> bool:
        """Register a hardware adapter with the coordinator."""
        with self._hardware_lock:
            if component.hardware_id in self._hardware:
                logger.warning(f"Hardware {component.hardware_id} already registered")
                return False

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

    # =========================================================================
    # Task Management (RPC)
    # =========================================================================

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

    # =========================================================================
    # Streaming Control
    # =========================================================================

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

    def _on_buttons(self, msg: Buttons) -> None:
        """Forward button state to all tasks."""
        with self._task_lock:
            for task in self._tasks.values():
                task.on_buttons(msg)

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

    # =========================================================================
    # Gripper
    # =========================================================================

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
            return hw.adapter.read_gripper_position()

    # =========================================================================
    # Lifecycle
    # =========================================================================

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
        publish_cb = self.joint_state.publish if self.config.publish_joint_state else None
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
                self._cartesian_command_unsub = self.cartesian_command.subscribe(
                    self._on_cartesian_command
                )
                logger.info("Subscribed to cartesian_command for CartesianIK/TeleopIK tasks")
            except Exception:
                logger.warning(
                    "CartesianIK/TeleopIK tasks configured but could not subscribe to cartesian_command. "
                    "Use task_invoke RPC or set transport via blueprint."
                )

        # Subscribe to buttons if any teleop_ik tasks configured (engage/disengage)
        has_teleop_ik = any(t.type == "teleop_ik" for t in self.config.tasks)
        if has_teleop_ik:
            self._buttons_unsub = self.buttons.subscribe(self._on_buttons)
            logger.info("Subscribed to buttons for engage/disengage")

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
        if self._buttons_unsub:
            self._buttons_unsub()
            self._buttons_unsub = None

        if self._tick_loop:
            self._tick_loop.stop()

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


# Blueprint export
control_coordinator = ControlCoordinator.blueprint


__all__ = [
    "ControlCoordinator",
    "ControlCoordinatorConfig",
    "HardwareComponent",
    "TaskConfig",
    "control_coordinator",
]
