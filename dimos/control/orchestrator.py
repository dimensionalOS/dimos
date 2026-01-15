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

"""ControlOrchestrator module.

Centralized control orchestrator that replaces per-driver/per-controller
loops with a single deterministic tick-based system.

Features:
- Single tick loop (read → compute → arbitrate → route → write)
- Per-joint arbitration (highest priority wins)
- Mode conflict detection
- Partial command support (hold last value)
- Aggregated preemption notifications
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import threading
import time
from typing import TYPE_CHECKING, Any

from dimos.control.hardware_interface import BackendHardwareInterface, HardwareInterface
from dimos.control.task import ControlTask
from dimos.control.tick_loop import TickLoop
from dimos.core import Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.sensor_msgs import (
    JointState,  # noqa: TC001 - needed at runtime for Out[JointState]
)
from dimos.msgs.trajectory_msgs import JointTrajectory, TrajectoryState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hardware.manipulators.spec import ManipulatorBackend

logger = setup_logger()


# =============================================================================
# Configuration
# =============================================================================


@dataclass
class HardwareConfig:
    """Configuration for a hardware backend.

    Attributes:
        id: Unique hardware identifier (e.g., "arm", "left_arm")
        type: Backend type ("mock", "xarm", "piper")
        dof: Degrees of freedom
        joint_prefix: Prefix for joint names (defaults to id)
        ip: IP address (required for xarm)
        can_port: CAN port (for piper, default "can0")
        auto_enable: Whether to auto-enable servos (default True)
    """

    id: str
    type: str = "mock"
    dof: int = 7
    joint_prefix: str | None = None
    ip: str | None = None
    can_port: str | None = None
    auto_enable: bool = True


@dataclass
class TaskConfig:
    """Configuration for a control task.

    Attributes:
        name: Task name (e.g., "traj_arm")
        type: Task type ("trajectory")
        joint_names: List of joint names this task controls
        priority: Task priority (higher wins arbitration)
    """

    name: str
    type: str = "trajectory"
    joint_names: list[str] = field(default_factory=lambda: [])
    priority: int = 10


@dataclass
class TaskStatus:
    """Status of a control task.

    Attributes:
        active: Whether the task is currently active
        state: Task state name (e.g., "IDLE", "RUNNING", "DONE")
        progress: Task progress from 0.0 to 1.0
    """

    active: bool
    state: str | None = None
    progress: float | None = None


@dataclass
class ControlOrchestratorConfig(ModuleConfig):
    """Configuration for the ControlOrchestrator.

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
    joint_state_frame_id: str = "orchestrator"
    log_ticks: bool = False
    hardware: list[HardwareConfig] = field(default_factory=lambda: [])
    tasks: list[TaskConfig] = field(default_factory=lambda: [])


# =============================================================================
# ControlOrchestrator Module
# =============================================================================


class ControlOrchestrator(Module[ControlOrchestratorConfig]):
    """Centralized control orchestrator with per-joint arbitration.

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
        >>> from dimos.control import ControlOrchestrator
        >>> from dimos.hardware.manipulators.xarm import XArmBackend
        >>>
        >>> orch = ControlOrchestrator(tick_rate=100.0)
        >>> backend = XArmBackend(ip="192.168.1.185", dof=7)
        >>> backend.connect()
        >>> orch.add_hardware("left_arm", backend, joint_prefix="left")
        >>> orch.start()
    """

    # Output: Aggregated joint state for external consumers
    joint_state: Out[JointState]

    config: ControlOrchestratorConfig
    default_config = ControlOrchestratorConfig

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        # Hardware interfaces (keyed by hardware_id)
        self._hardware: dict[str, HardwareInterface] = {}
        self._hardware_lock = threading.Lock()

        # Joint -> hardware mapping (built when hardware added)
        self._joint_to_hardware: dict[str, str] = {}

        # Registered tasks
        self._tasks: dict[str, ControlTask] = {}
        self._task_lock = threading.Lock()

        # Tick loop (created on start)
        self._tick_loop: TickLoop | None = None

        logger.info(f"ControlOrchestrator initialized at {self.config.tick_rate}Hz")

    # =========================================================================
    # Config-based Setup
    # =========================================================================

    def _setup_from_config(self) -> None:
        """Create hardware and tasks from config (called on start)."""
        hardware_added: list[str] = []

        try:
            for hw_cfg in self.config.hardware:
                self._setup_hardware(hw_cfg)
                hardware_added.append(hw_cfg.id)

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

    def _setup_hardware(self, hw_cfg: HardwareConfig) -> None:
        """Connect and add a single hardware backend."""
        backend = self._create_backend_from_config(hw_cfg)

        if not backend.connect():
            raise RuntimeError(f"Failed to connect to {hw_cfg.type} backend")

        try:
            if hw_cfg.auto_enable and hasattr(backend, "write_enable"):
                backend.write_enable(True)
            self.add_hardware(
                hw_cfg.id,
                backend,
                joint_prefix=hw_cfg.joint_prefix or hw_cfg.id,
            )
        except Exception:
            backend.disconnect()
            raise

    def _create_backend_from_config(self, cfg: HardwareConfig) -> ManipulatorBackend:
        """Create a manipulator backend from config."""
        match cfg.type.lower():
            case "mock":
                from dimos.hardware.manipulators.mock import MockBackend

                return MockBackend(dof=cfg.dof)
            case "xarm":
                if cfg.ip is None:
                    raise ValueError("ip is required for xarm backend")
                from dimos.hardware.manipulators.xarm import XArmBackend

                return XArmBackend(ip=cfg.ip, dof=cfg.dof)
            case "piper":
                from dimos.hardware.manipulators.piper import PiperBackend

                return PiperBackend(can_port=cfg.can_port or "can0", dof=cfg.dof)
            case _:
                raise ValueError(f"Unknown backend type: {cfg.type}")

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

        else:
            raise ValueError(f"Unknown task type: {task_type}")

    # =========================================================================
    # Hardware Management (RPC)
    # =========================================================================

    @rpc
    def add_hardware(
        self,
        hardware_id: str,
        backend: ManipulatorBackend,
        joint_prefix: str | None = None,
    ) -> bool:
        """Register a hardware backend with the orchestrator."""
        with self._hardware_lock:
            if hardware_id in self._hardware:
                logger.warning(f"Hardware {hardware_id} already registered")
                return False

            interface = BackendHardwareInterface(
                backend=backend,
                hardware_id=hardware_id,
                joint_prefix=joint_prefix,
            )
            self._hardware[hardware_id] = interface

            for joint_name in interface.joint_names:
                self._joint_to_hardware[joint_name] = hardware_id

            logger.info(f"Added hardware {hardware_id} with joints: {interface.joint_names}")
            return True

    @rpc
    def remove_hardware(self, hardware_id: str) -> bool:
        """Remove a hardware interface.

        Note: For safety, call this only when no tasks are actively using this
        hardware. Consider stopping the orchestrator before removing hardware.
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
                state = hw.read_state()  # {joint_name: (pos, vel, effort)}
                for joint_name, (pos, _vel, _effort) in state.items():
                    positions[joint_name] = pos
            return positions

    # =========================================================================
    # Task Management (RPC)
    # =========================================================================

    @rpc
    def add_task(self, task: ControlTask) -> bool:
        """Register a task with the orchestrator."""
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
    def remove_task(self, task_name: str) -> bool:
        """Remove a task by name."""
        with self._task_lock:
            if task_name in self._tasks:
                del self._tasks[task_name]
                logger.info(f"Removed task {task_name}")
                return True
            return False

    @rpc
    def get_task(self, task_name: str) -> ControlTask | None:
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
    # Trajectory Execution (RPC)
    # =========================================================================

    @rpc
    def execute_trajectory(self, task_name: str, trajectory: JointTrajectory) -> bool:
        """Execute a trajectory on a named task."""
        with self._task_lock:
            task = self._tasks.get(task_name)
            if task is None:
                logger.warning(f"Task {task_name} not found")
                return False

            if not hasattr(task, "execute"):
                logger.warning(f"Task {task_name} doesn't support execute()")
                return False

            logger.info(
                f"Executing trajectory on {task_name}: "
                f"{len(trajectory.points)} points, duration={trajectory.duration:.3f}s"
            )
            return task.execute(trajectory)  # type: ignore[attr-defined,no-any-return]

    @rpc
    def get_trajectory_status(self, task_name: str) -> TaskStatus | None:
        """Get the status of a trajectory task."""
        with self._task_lock:
            task = self._tasks.get(task_name)
            if task is None:
                return None

            state: str | None = None
            if hasattr(task, "get_state"):
                task_state: TrajectoryState = task.get_state()  # type: ignore[attr-defined]
                state = (
                    task_state.name if isinstance(task_state, TrajectoryState) else str(task_state)
                )

            progress: float | None = None
            if hasattr(task, "get_progress"):
                t_now = time.perf_counter()
                progress = task.get_progress(t_now)  # type: ignore[attr-defined]

            return TaskStatus(active=task.is_active(), state=state, progress=progress)

    @rpc
    def cancel_trajectory(self, task_name: str) -> bool:
        """Cancel an active trajectory on a task."""
        with self._task_lock:
            task = self._tasks.get(task_name)
            if task is None:
                logger.warning(f"Task {task_name} not found")
                return False

            if not hasattr(task, "cancel"):
                logger.warning(f"Task {task_name} doesn't support cancel()")
                return False

            logger.info(f"Cancelling trajectory on {task_name}")
            return task.cancel()  # type: ignore[attr-defined,no-any-return]

    # =========================================================================
    # Lifecycle
    # =========================================================================

    @rpc
    def start(self) -> None:
        """Start the orchestrator control loop."""
        if self._tick_loop and self._tick_loop.is_running:
            logger.warning("Orchestrator already running")
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

        logger.info(f"ControlOrchestrator started at {self.config.tick_rate}Hz")

    @rpc
    def stop(self) -> None:
        """Stop the orchestrator."""
        logger.info("Stopping ControlOrchestrator...")

        if self._tick_loop:
            self._tick_loop.stop()

        # Disconnect all hardware backends
        with self._hardware_lock:
            for hw_id, interface in self._hardware.items():
                try:
                    interface.disconnect()
                    logger.info(f"Disconnected hardware {hw_id}")
                except Exception as e:
                    logger.error(f"Error disconnecting hardware {hw_id}: {e}")

        super().stop()
        logger.info("ControlOrchestrator stopped")

    @rpc
    def get_tick_count(self) -> int:
        """Get the number of ticks since start."""
        return self._tick_loop.tick_count if self._tick_loop else 0


# Blueprint export
control_orchestrator = ControlOrchestrator.blueprint


__all__ = [
    "ControlOrchestrator",
    "ControlOrchestratorConfig",
    "HardwareConfig",
    "TaskConfig",
    "control_orchestrator",
]
