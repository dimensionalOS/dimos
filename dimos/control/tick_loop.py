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

"""Tick loop for the ControlCoordinator.

This module contains the core control loop logic:
- Read state from all hardware
- Compute outputs from all active tasks
- Arbitrate conflicts per-joint (highest priority wins)
- Route commands to hardware
- Publish aggregated joint state

Separated from coordinator.py following the DimOS pattern of
splitting coordination logic from module wrapper.
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING, NamedTuple

from dimos.control.task import (
    ControlTask,
    CoordinatorState,
    JointCommandOutput,
    JointStateSnapshot,
    ResourceClaim,
)
from dimos.msgs.sensor_msgs import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.control.hardware_interface import HardwareInterface
    from dimos.hardware.manipulators.spec import ControlMode

logger = setup_logger()


class JointWinner(NamedTuple):
    """Tracks the winning task for a joint during arbitration."""

    priority: int
    value: float
    mode: ControlMode
    task_name: str


class TickLoop:
    """Core tick loop for the control coordinator.

    Runs the deterministic control cycle:
    1. READ: Collect joint state from all hardware
    2. COMPUTE: Run all active tasks
    3. ARBITRATE: Per-joint conflict resolution (highest priority wins)
    4. NOTIFY: Send preemption notifications to affected tasks
    5. ROUTE: Convert joint-centric commands to hardware-centric
    6. WRITE: Send commands to hardware
    7. PUBLISH: Output aggregated JointState

    Args:
        tick_rate: Control loop frequency in Hz
        hardware: Dict of hardware_id -> HardwareInterface
        hardware_lock: Lock protecting hardware dict
        tasks: Dict of task_name -> ControlTask
        task_lock: Lock protecting tasks dict
        joint_to_hardware: Dict mapping joint_name -> hardware_id
        publish_callback: Optional callback to publish JointState
        frame_id: Frame ID for published JointState
        log_ticks: Whether to log tick information
    """

    def __init__(
        self,
        tick_rate: float,
        hardware: dict[str, HardwareInterface],
        hardware_lock: threading.Lock,
        tasks: dict[str, ControlTask],
        task_lock: threading.Lock,
        joint_to_hardware: dict[str, str],
        publish_callback: Callable[[JointState], None] | None = None,
        frame_id: str = "coordinator",
        log_ticks: bool = False,
    ) -> None:
        self._tick_rate = tick_rate
        self._hardware = hardware
        self._hardware_lock = hardware_lock
        self._tasks = tasks
        self._task_lock = task_lock
        self._joint_to_hardware = joint_to_hardware
        self._publish_callback = publish_callback
        self._frame_id = frame_id
        self._log_ticks = log_ticks

        self._stop_event = threading.Event()
        self._stop_event.set()  # Initially stopped
        self._tick_thread: threading.Thread | None = None
        self._last_tick_time: float = 0.0
        self._tick_count: int = 0

    @property
    def tick_count(self) -> int:
        """Number of ticks since start."""
        return self._tick_count

    @property
    def is_running(self) -> bool:
        """Whether the tick loop is currently running."""
        return not self._stop_event.is_set()

    def start(self) -> None:
        """Start the tick loop in a daemon thread."""
        if not self._stop_event.is_set():
            logger.warning("TickLoop already running")
            return

        self._stop_event.clear()
        self._last_tick_time = time.perf_counter()
        self._tick_count = 0

        self._tick_thread = threading.Thread(
            target=self._loop,
            name="ControlCoordinator-Tick",
            daemon=True,
        )
        self._tick_thread.start()
        logger.info(f"TickLoop started at {self._tick_rate}Hz")

    def stop(self) -> None:
        """Stop the tick loop."""
        self._stop_event.set()
        if self._tick_thread and self._tick_thread.is_alive():
            self._tick_thread.join(timeout=2.0)
        logger.info("TickLoop stopped")

    def _loop(self) -> None:
        """Main control loop - deterministic read → compute → arbitrate → write."""
        period = 1.0 / self._tick_rate

        while not self._stop_event.is_set():
            tick_start = time.perf_counter()

            try:
                self._tick()
            except Exception as e:
                logger.error(f"TickLoop tick error: {e}")

            # Rate control - recalculate sleep time to account for overhead
            next_tick_time = tick_start + period
            sleep_time = next_tick_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _tick(self) -> None:
        """Single tick: read → compute → arbitrate → route → write."""
        t_now = time.perf_counter()
        dt = t_now - self._last_tick_time
        self._last_tick_time = t_now
        self._tick_count += 1

        # === PHASE 1: READ ALL HARDWARE ===
        joint_states = self._read_all_hardware()
        state = CoordinatorState(joints=joint_states, t_now=t_now, dt=dt)

        # === PHASE 2: COMPUTE ALL ACTIVE TASKS ===
        commands = self._compute_all_tasks(state)

        # === PHASE 3: ARBITRATE (with mode validation) ===
        joint_commands, preemptions = self._arbitrate(commands)

        # === PHASE 4: NOTIFY PREEMPTIONS (once per task) ===
        self._notify_preemptions(preemptions)

        # === PHASE 5: ROUTE TO HARDWARE ===
        hw_commands = self._route_to_hardware(joint_commands)

        # === PHASE 6: WRITE TO HARDWARE ===
        self._write_all_hardware(hw_commands)

        # === PHASE 7: PUBLISH AGGREGATED STATE ===
        if self._publish_callback:
            self._publish_joint_state(joint_states)

        # Optional logging
        if self._log_ticks:
            active = len([c for c in commands if c[2] is not None])
            logger.debug(
                f"Tick {self._tick_count}: dt={dt:.4f}s, "
                f"{len(joint_states.joint_positions)} joints, "
                f"{active} active tasks"
            )

    def _read_all_hardware(self) -> JointStateSnapshot:
        """Read state from all hardware interfaces."""
        joint_positions: dict[str, float] = {}
        joint_velocities: dict[str, float] = {}
        joint_efforts: dict[str, float] = {}

        with self._hardware_lock:
            for hw in self._hardware.values():
                try:
                    state = hw.read_state()
                    for joint_name, joint_state in state.items():
                        joint_positions[joint_name] = joint_state.position
                        joint_velocities[joint_name] = joint_state.velocity
                        joint_efforts[joint_name] = joint_state.effort
                except Exception as e:
                    logger.error(f"Failed to read {hw.hardware_id}: {e}")

        return JointStateSnapshot(
            joint_positions=joint_positions,
            joint_velocities=joint_velocities,
            joint_efforts=joint_efforts,
            timestamp=time.time(),
        )

    def _compute_all_tasks(
        self, state: CoordinatorState
    ) -> list[tuple[ControlTask, ResourceClaim, JointCommandOutput | None]]:
        """Compute outputs from all active tasks."""
        results: list[tuple[ControlTask, ResourceClaim, JointCommandOutput | None]] = []

        with self._task_lock:
            for task in self._tasks.values():
                if not task.is_active():
                    continue

                try:
                    claim = task.claim()
                    output = task.compute(state)
                    results.append((task, claim, output))
                except Exception as e:
                    logger.error(f"Task {task.name} compute error: {e}")

        return results

    def _arbitrate(
        self,
        commands: list[tuple[ControlTask, ResourceClaim, JointCommandOutput | None]],
    ) -> tuple[
        dict[str, tuple[float, ControlMode, str]],
        dict[str, dict[str, str]],
    ]:
        """Per-joint arbitration with mode conflict detection.

        Returns:
            Tuple of:
            - joint_commands: {joint_name: (value, mode, task_name)}
            - preemptions: {preempted_task: {joint: winning_task}}
        """
        winners: dict[str, JointWinner] = {}  # joint_name -> current winner
        preemptions: dict[str, dict[str, str]] = {}  # loser_task -> {joint: winner_task}

        for task, claim, output in commands:
            if output is None:
                continue

            values = output.get_values()
            if values is None:
                continue

            for i, joint_name in enumerate(output.joint_names):
                candidate = JointWinner(claim.priority, values[i], output.mode, task.name)

                # First claim on this joint
                if joint_name not in winners:
                    winners[joint_name] = candidate
                    continue

                current = winners[joint_name]

                # Lower priority loses - notify preemption
                if candidate.priority < current.priority:
                    preemptions.setdefault(task.name, {})[joint_name] = current.task_name
                    continue

                # Higher priority - take over
                if candidate.priority > current.priority:
                    preemptions.setdefault(current.task_name, {})[joint_name] = task.name
                    winners[joint_name] = candidate
                    continue

                # Same priority - check for mode conflict
                if candidate.mode != current.mode:
                    logger.warning(
                        f"Mode conflict on {joint_name}: {task.name} wants "
                        f"{candidate.mode.name}, but {current.task_name} wants "
                        f"{current.mode.name}. Dropping {task.name}."
                    )
                    preemptions.setdefault(task.name, {})[joint_name] = current.task_name
                # Same priority + same mode: first wins (keep current)

        # Convert to output format: joint -> (value, mode, task_name)
        joint_commands = {joint: (w.value, w.mode, w.task_name) for joint, w in winners.items()}

        return joint_commands, preemptions

    def _notify_preemptions(self, preemptions: dict[str, dict[str, str]]) -> None:
        """Notify each preempted task with affected joints, grouped by winning task."""
        with self._task_lock:
            for task_name, joint_winners in preemptions.items():
                task = self._tasks.get(task_name)
                if not task:
                    continue

                # Group joints by winning task
                by_winner: dict[str, set[str]] = {}
                for joint, winner in joint_winners.items():
                    if winner not in by_winner:
                        by_winner[winner] = set()
                    by_winner[winner].add(joint)

                # Notify once per distinct winning task
                for winner, joints in by_winner.items():
                    try:
                        task.on_preempted(
                            by_task=winner,
                            joints=frozenset(joints),
                        )
                    except Exception as e:
                        logger.error(f"Error notifying {task_name} of preemption: {e}")

    def _route_to_hardware(
        self,
        joint_commands: dict[str, tuple[float, ControlMode, str]],
    ) -> dict[str, tuple[dict[str, float], ControlMode]]:
        """Route joint-centric commands to hardware.

        Returns:
            {hardware_id: ({joint: value}, mode)}
        """
        hw_commands: dict[str, tuple[dict[str, float], ControlMode]] = {}

        with self._hardware_lock:
            for joint_name, (value, mode, _) in joint_commands.items():
                hw_id = self._joint_to_hardware.get(joint_name)
                if hw_id is None:
                    logger.warning(f"Unknown joint {joint_name}, cannot route")
                    continue

                if hw_id not in hw_commands:
                    hw_commands[hw_id] = ({}, mode)
                else:
                    # Check for mode conflict across joints on same hardware
                    existing_mode = hw_commands[hw_id][1]
                    if mode != existing_mode:
                        logger.error(
                            f"Mode conflict for hardware {hw_id}: joint {joint_name} wants "
                            f"{mode.name} but hardware already has {existing_mode.name}. "
                            f"Dropping command for {joint_name}."
                        )
                        continue

                hw_commands[hw_id][0][joint_name] = value

        return hw_commands

    def _write_all_hardware(
        self,
        hw_commands: dict[str, tuple[dict[str, float], ControlMode]],
    ) -> None:
        """Write commands to all hardware interfaces."""
        with self._hardware_lock:
            for hw_id, (positions, mode) in hw_commands.items():
                if hw_id in self._hardware:
                    try:
                        self._hardware[hw_id].write_command(positions, mode)
                    except Exception as e:
                        logger.error(f"Failed to write to {hw_id}: {e}")

    def _publish_joint_state(self, snapshot: JointStateSnapshot) -> None:
        """Publish aggregated JointState for external consumers."""
        names = list(snapshot.joint_positions.keys())
        msg = JointState(
            ts=snapshot.timestamp,
            frame_id=self._frame_id,
            name=names,
            position=[snapshot.joint_positions[n] for n in names],
            velocity=[snapshot.joint_velocities.get(n, 0.0) for n in names],
            effort=[snapshot.joint_efforts.get(n, 0.0) for n in names],
        )
        if self._publish_callback:
            self._publish_callback(msg)


__all__ = ["TickLoop"]
