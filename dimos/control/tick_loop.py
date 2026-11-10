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


from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING, NamedTuple

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.components import HardwareType
from dimos.control.task import (
    ControlTask,
    CoordinatorState,
    JointCommandOutput,
    JointStateSnapshot,
    ResourceClaim,
    SensorStateSnapshot,
)
from dimos.hardware.manipulators.spec import ControlMode
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.control.components import HardwareId, JointName, JointState as JointReading, TaskName
    from dimos.control.connection_source import ConnectionSource
    from dimos.control.hardware_interface import ConnectedHardware
    from dimos.hardware.whole_body.spec import IMUState
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

logger = setup_logger()


class JointWinner(NamedTuple):
    priority: int
    value: float
    mode: ControlMode
    task_name: str


class TickLoop:
    def __init__(
        self,
        tick_rate: float,
        hardware: dict[HardwareId, ConnectedHardware],
        hardware_lock: threading.Lock,
        tasks: dict[TaskName, ControlTask],
        task_lock: threading.Lock,
        joint_to_hardware: dict[JointName, HardwareId],
        publish_callback: Callable[[JointState], None] | None = None,
        publish_robot_callback: Callable[[HardwareId, JointState], None] | None = None,
        publish_tf_callback: Callable[[TFMessage], None] | None = None,
        frame_pose_hz: float = 30.0,
        frame_id: str = "coordinator",
        log_ticks: bool = False,
        publish_command_callback: Callable[[JointState], None] | None = None,
        connections: dict[str, ConnectionSource] | None = None,
        connection_command_callback: Callable[[str, dict[str, float]], None] | None = None,
    ) -> None:
        self._tick_rate = tick_rate
        self._hardware = hardware
        self._connections: dict[str, ConnectionSource] = (
            connections if connections is not None else {}
        )
        self._connection_command_callback = connection_command_callback
        self._silence_logged: set[str] = set()
        self._started_at = time.monotonic()
        self._hardware_lock = hardware_lock
        self._tasks = tasks
        self._task_lock = task_lock
        self._joint_to_hardware = joint_to_hardware
        self._publish_callback = publish_callback
        self._publish_command_callback = publish_command_callback
        self._publish_robot_callback = publish_robot_callback
        self._publish_tf_callback = publish_tf_callback
        self._frame_pose_period = 1.0 / frame_pose_hz if publish_tf_callback else 0.0
        self._last_frame_pose_time = float("-inf")
        self._frame_id = frame_id
        self._log_ticks = log_ticks

        self._stop_event = threading.Event()
        self._stop_event.set()
        self._tick_thread: threading.Thread | None = None
        self._last_tick_time: float = 0.0
        self._tick_count: int = 0
        self._joint_positions: dict[str, float] = {}

    @property
    def joint_positions(self) -> dict[str, float]:
        return self._joint_positions.copy()

    @property
    def tick_count(self) -> int:
        return self._tick_count

    @property
    def is_running(self) -> bool:
        return not self._stop_event.is_set()

    def start(self) -> None:
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
        self._stop_event.set()
        if self._tick_thread and self._tick_thread.is_alive():
            self._tick_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        logger.info("TickLoop stopped")

    def _loop(self) -> None:
        period_ns = round(1_000_000_000 / self._tick_rate)
        next_tick_time = time.perf_counter_ns()

        while not self._stop_event.is_set():
            try:
                self._tick()
            except Exception as e:
                logger.error(f"TickLoop tick error: {e}")

            next_tick_time += period_ns
            sleep_ns = next_tick_time - time.perf_counter_ns()
            if sleep_ns > 0:
                time.sleep(sleep_ns / 1_000_000_000)
            else:
                next_tick_time = time.perf_counter_ns()

    def _tick(self) -> None:
        t_now = time.perf_counter()
        dt = t_now - self._last_tick_time
        self._last_tick_time = t_now
        self._tick_count += 1

        joint_states, per_hardware = self._read_all_hardware()
        imu_states = self._read_all_imu()
        state = CoordinatorState(
            joints=joint_states,
            imu=imu_states,
            t_now=t_now,
            dt=dt,
            sensors=self._read_all_sensors(),
        )

        commands = self._compute_all_tasks(state)

        joint_commands, preemptions = self._arbitrate(commands)

        self._notify_preemptions(preemptions)

        hw_commands = self._route_to_hardware(joint_commands)

        self._write_all_hardware(hw_commands)

        self._write_all_connections(hw_commands)

        if self._publish_callback:
            self._publish_joint_state(joint_states)

        if self._publish_robot_callback:
            self._publish_robot_joint_states(per_hardware, joint_states.timestamp)

        if (
            self._publish_tf_callback
            and t_now - self._last_frame_pose_time >= self._frame_pose_period
        ):
            self._last_frame_pose_time = t_now
            self._publish_frame_poses(state)

        if self._log_ticks:
            active = len([c for c in commands if c[2] is not None])
            logger.debug(
                f"Tick {self._tick_count}: dt={dt:.4f}s, "
                f"{len(joint_states.joint_positions)} joints, "
                f"{active} active tasks"
            )

    def _read_all_hardware(
        self,
    ) -> tuple[JointStateSnapshot, dict[HardwareId, dict[JointName, JointReading]]]:
        joint_positions: dict[str, float] = {}
        joint_velocities: dict[str, float] = {}
        joint_efforts: dict[str, float] = {}
        per_hardware: dict[HardwareId, dict[JointName, JointReading]] = {}
        read_at: list[float] = []

        with self._hardware_lock:
            for hw in self._hardware.values():
                if not hw.ready_for_control():
                    continue
                try:
                    state = hw.read_state()
                    for joint_name, joint_state in state.items():
                        joint_positions[joint_name] = joint_state.position
                        joint_velocities[joint_name] = joint_state.velocity
                        joint_efforts[joint_name] = joint_state.effort
                    if self._publish_robot_callback:
                        per_hardware[hw.hardware_id] = state
                    taken = getattr(hw, "read_state_ts", None)
                    stamp = taken() if callable(taken) else None
                    if stamp:
                        read_at.append(stamp)
                except Exception as e:
                    logger.error(f"Failed to read {hw.hardware_id}: {e}")
            for source in self._connections.values():
                joints = source.read_joints()
                joint_positions.update(joints.joint_positions)
                joint_velocities.update(joints.joint_velocities)
                joint_efforts.update(joints.joint_efforts)
                if joints.timestamp:
                    read_at.append(joints.timestamp)

        snapshot = JointStateSnapshot(
            joint_positions=joint_positions,
            joint_velocities=joint_velocities,
            joint_efforts=joint_efforts,
            timestamp=max(read_at) if read_at else time.time(),
        )
        return snapshot, per_hardware

    def _read_all_imu(self) -> dict[str, IMUState]:
        from dimos.control.hardware_interface import ConnectedWholeBody

        out: dict[str, IMUState] = {}
        with self._hardware_lock:
            for hw_id, hw in self._hardware.items():
                if not isinstance(hw, ConnectedWholeBody):
                    continue
                if not hw.ready_for_control():
                    continue
                read_imu = getattr(hw.adapter, "read_imu", None)
                if not callable(read_imu):
                    continue
                try:
                    out[hw_id] = read_imu()
                except Exception as e:
                    logger.error(f"Failed to read IMU from {hw_id}: {e}")
        return out

    def _read_all_sensors(self) -> SensorStateSnapshot:
        readings: dict[str, dict[str, float]] = {}
        with self._hardware_lock:
            for source in self._connections.values():
                readings.update(source.read_sensors())
        return SensorStateSnapshot(readings=readings)

    def _compute_all_tasks(
        self, state: CoordinatorState
    ) -> list[tuple[ControlTask, ResourceClaim, JointCommandOutput | None]]:
        results: list[tuple[ControlTask, ResourceClaim, JointCommandOutput | None]] = []

        with self._task_lock:
            self._joint_positions = state.joints.joint_positions.copy()
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
        winners: dict[str, JointWinner] = {}
        preemptions: dict[str, dict[str, str]] = {}

        for task, claim, output in commands:
            if output is None:
                continue

            values = output.get_values()
            if values is None:
                continue

            for i, joint_name in enumerate(output.joint_names):
                candidate = JointWinner(claim.priority, values[i], output.mode, task.name)

                if joint_name not in winners:
                    winners[joint_name] = candidate
                    continue

                current = winners[joint_name]

                if candidate.priority < current.priority:
                    preemptions.setdefault(task.name, {})[joint_name] = current.task_name
                    continue

                if candidate.priority > current.priority:
                    preemptions.setdefault(current.task_name, {})[joint_name] = task.name
                    winners[joint_name] = candidate
                    continue

                if candidate.mode != current.mode:
                    logger.warning(
                        f"Mode conflict on {joint_name}: {task.name} wants "
                        f"{candidate.mode.name}, but {current.task_name} wants "
                        f"{current.mode.name}. Dropping {task.name}."
                    )
                    preemptions.setdefault(task.name, {})[joint_name] = current.task_name

        joint_commands = {joint: (w.value, w.mode, w.task_name) for joint, w in winners.items()}

        return joint_commands, preemptions

    def _notify_preemptions(self, preemptions: dict[str, dict[str, str]]) -> None:
        with self._task_lock:
            for task_name, joint_winners in preemptions.items():
                task = self._tasks.get(task_name)
                if not task:
                    continue

                by_winner: dict[str, set[str]] = {}
                for joint, winner in joint_winners.items():
                    if winner not in by_winner:
                        by_winner[winner] = set()
                    by_winner[winner].add(joint)

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
        hardware = self._hardware
        joint_commands: dict[str, float] = {}
        with self._hardware_lock:
            for hw_id, (positions, mode) in hw_commands.items():
                if hw_id in hardware:
                    if not hardware[hw_id].ready_for_control():
                        continue
                    if (
                        self._publish_command_callback is not None
                        and mode in (ControlMode.POSITION, ControlMode.SERVO_POSITION)
                        and hardware[hw_id].component.hardware_type != HardwareType.BASE
                    ):
                        joint_commands.update(
                            (name, positions[name])
                            for name in hardware[hw_id].joint_names
                            if name in positions
                        )
                    try:
                        accepted = hardware[hw_id].write_command(positions, mode)
                        if not accepted:
                            logger.error(
                                f"Hardware {hw_id} rejected {mode.name} command from control task"
                            )
                    except Exception as e:
                        logger.error(f"Failed to write to {hw_id}: {e}")
        if joint_commands and self._publish_command_callback is not None:
            try:
                self._publish_command_callback(
                    JointState(
                        ts=time.time(),
                        frame_id=self._frame_id,
                        name=list(joint_commands),
                        position=list(joint_commands.values()),
                        velocity=[],
                        effort=[],
                    )
                )
            except Exception:
                logger.exception("Failed to publish joint commands")

    def _write_all_connections(
        self,
        hw_commands: dict[str, tuple[dict[str, float], ControlMode]],
    ) -> None:
        publish = self._connection_command_callback
        if publish is None:
            return
        now = time.monotonic()
        with self._hardware_lock:
            for name, source in self._connections.items():
                if not source.ready_for_control(now):
                    self._note_silence(name, source, now)
                    continue
                winners: dict[str, float] = {}
                mode: ControlMode | None = None
                if name in hw_commands:
                    winners, mode = hw_commands[name]
                try:
                    publish(name, source.command(winners, mode))
                except Exception as e:
                    logger.error(f"Failed to send a command to {name}: {e}")

    def _note_silence(self, name: str, source: ConnectionSource, now: float) -> None:
        if name in self._silence_logged or now - self._started_at < 10 * source.reading_timeout_s:
            return
        silent = source.unreported()
        if not silent:
            return
        self._silence_logged.add(name)
        ports = ", ".join(f"{name}_{kind}" for kind in silent)
        theirs = ", ".join(silent)
        onto = "it" if len(silent) == 1 else "them"
        logger.error(
            f"robot {name!r} has not reported on {ports}; is the connection module's {theirs} "
            f"remapped onto {onto}?"
        )

    def _publish_joint_state(self, snapshot: JointStateSnapshot) -> None:
        if any(source.unreported() for source in self._connections.values()):
            return
        reported = (
            snapshot.joint_positions.keys()
            | snapshot.joint_velocities.keys()
            | snapshot.joint_efforts.keys()
        )
        if any(
            not reported.issuperset(source.joint_names) for source in self._connections.values()
        ):
            return
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

    def _publish_frame_poses(self, state: CoordinatorState) -> None:
        with self._task_lock:
            tasks = list(self._tasks.values())
        poses: dict[str, PoseStamped] = {}
        for task in tasks:
            poses.update(task.measured_frame_poses(state))
        if not poses or self._publish_tf_callback is None:
            return
        ts = state.joints.timestamp
        self._publish_tf_callback(
            TFMessage(
                *(
                    Transform(
                        translation=pose.position,
                        rotation=pose.orientation,
                        frame_id="world",
                        child_frame_id=frame,
                        ts=ts,
                    )
                    for frame, pose in poses.items()
                )
            )
        )

    def _publish_robot_joint_states(
        self,
        per_hardware: dict[HardwareId, dict[JointName, JointReading]],
        timestamp: float,
    ) -> None:
        publish = self._publish_robot_callback
        if publish is None:
            return

        for hw_id, state in per_hardware.items():
            names = list(state.keys())
            msg = JointState(
                ts=timestamp,
                frame_id=hw_id,
                name=names,
                position=[state[n].position for n in names],
                velocity=[state[n].velocity for n in names],
                effort=[state[n].effort for n in names],
            )
            try:
                publish(hw_id, msg)
            except Exception as e:
                logger.error(f"Failed to publish joint state for {hw_id}: {e}")
