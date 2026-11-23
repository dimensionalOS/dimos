# Copyright 2026 Dimensional Inc.
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

"""Opt-in coordinator with a co-located connection and no legacy arm output."""

from __future__ import annotations

from collections.abc import Callable
from contextlib import suppress
from pathlib import Path
import threading
from typing import Literal, Protocol

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.connection_source import JOINT_STATE, POSITION_COMMAND, ConnectionSource
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import movable_joints
from dimos.control.coordinator import ControlCoordinatorConfig, TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.core import rpc
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.openyam.joints import OPENYAM_ARM_JOINTS, OPENYAM_GRIPPER_JOINT
from dimos.robot.manipulators.openyam.trimode_tasks import (
    JOINT,
    JOINTS,
    POSE,
    XYZ,
    Feedback,
    PlannedReference,
    QuestReference,
)
from dimos.robot.manipulators.openyam.trimode_tracker import (
    Tracker,
    Vector,
    load_actor,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
_JAW_KEY = f"{OPENYAM_GRIPPER_JOINT}/position"


class LocalPlant(Protocol):
    """Bounded local calls; a jaw write must not resend any arm commands.

    Readings must contain newly acquired per-axis feedback, not cached state
    re-stamped on a poll. After a bounded interval without a successful arm
    submission, the plant must take the arm to its commissioned safe state
    independently of the coordinator, including on host failure. Jaw writes,
    feedback polling, and other bus traffic must not renew this arm watchdog.
    The caller must explicitly commission/authorize any physical adapter;
    no stock Damiao connection implements this contract.
    """

    def read(self) -> JointState | None: ...
    def submit_arm(self, q_des: Vector) -> None: ...
    def submit_jaw(self, position: float) -> None: ...


def trimode_connection(model: RobotModel) -> ConnectionConfig:
    joints = frozenset(OPENYAM_ARM_JOINTS)
    return ConnectionConfig(
        source="arm",
        model=model,
        joint_commands={OPENYAM_GRIPPER_JOINT.split("/")[-1]: (Interface.POSITION,)},
        group_commands=dict.fromkeys(("tcp_pose", "tcp_position", "joint_reference"), joints),
    )


class TrimodeCoordinatorConfig(ControlCoordinatorConfig):
    model_path: Path
    quest_mode: Literal["pose", "position"] = "pose"
    reading_timeout_s: float = 0.1
    plant_factory: Callable[[], LocalPlant] | None = None


class TrimodeCoordinator(TeleopControlCoordinator):
    """Run in the same process as the plant, with a single arm sink.

    Bind a local plant before start. There is no arm position output port, no
    stock connection startup, and no arm hold fallback. The gripper remains
    the ordinary scalar task/limits/hold path with an independent local sink.
    """

    config: TrimodeCoordinatorConfig  # type: ignore[assignment]
    _plant: LocalPlant | None = None
    _tracker: Tracker | None = None
    _reading_thread: threading.Thread | None = None

    def bind_local_plant(self, plant: LocalPlant) -> None:
        if self._plant is not None or self._tick_loop is not None:
            raise RuntimeError("bind exactly one local plant before starting")
        self._plant = plant

    def _read_group(self, interface: str) -> Feedback | None:
        reading = self._connections["arm"].group_feedback(interface)
        if reading is None:
            return None
        epoch, received, values = reading
        return Feedback(
            epoch,
            received,
            received + self.config.reading_timeout_s,
            tuple(values[f"{j}/position"] for j in JOINTS),
            tuple(values[f"{j}/velocity"] for j in JOINTS),
        )

    def _connect_robots(self) -> None:
        plant = self._plant
        if plant is None:
            raise RuntimeError("tri-mode requires a co-located plant bound before start")
        if (
            len(self.config.connections) != 1
            or self.config.hardware
            or self.config.tasks
            or self.config.zero_stiffness_joints
        ):
            raise ValueError("tri-mode requires one local declaration and built-in tasks only")
        declaration = self.config.connections[0]
        model = declaration.model
        if model is None:
            raise ValueError("tri-mode needs a canonical OpenYAM model")
        expected = trimode_connection(model)
        if (
            declaration.source != expected.source
            or declaration.joint_commands != expected.joint_commands
            or declaration.group_commands != expected.group_commands
        ):
            raise ValueError("tri-mode forbids alternative arm command routes")
        joints = {j.name: j for j in movable_joints(model)}
        limits = []
        for name in OPENYAM_ARM_JOINTS:
            joint = joints[name]
            if joint.lower is None or joint.upper is None:
                raise ValueError("tri-mode requires position bounds for each arm joint")
            limits.append((joint.lower, joint.upper))
        bounds = tuple(limits)
        tracker = Tracker(load_actor(self.config.model_path), bounds, plant.submit_arm)
        self._tracker = tracker
        source = ConnectionSource(
            declaration,
            [POSITION_COMMAND],
            [JOINT_STATE],
            self.config.reading_timeout_s,
            group_lock=self._hardware_lock,
            group_feedback_fields=frozenset({Interface.POSITION, Interface.VELOCITY}),
            group_handlers={i: tracker.offer for i in (POSE, XYZ, JOINT)},
        )
        self._connections["arm"] = source
        self._joint_to_hardware.update(dict.fromkeys(source.joint_names, "arm"))

        def takeover() -> None:
            plan.cancel()

        quest = QuestReference(
            POSE if self.config.quest_mode == "pose" else XYZ, self._read_group, takeover
        )
        plan = PlannedReference(bounds, self._read_group, lambda: quest.blocks_plan)
        self.add_task(quest, task_type="teleop_ik")
        self.add_task(plan, task_type="trajectory")
        gripper = TaskConfig(
            name="arm_gripper",
            type="gripper",
            joint_names=[OPENYAM_GRIPPER_JOINT],
            priority=20,
            params={"hold_duration": 0.1, "open_at": "lower"},
            stream_bind={"gripper_command": "right_gripper_command"},
        )
        self.add_task(
            self._create_task_from_config(gripper),
            task_type=gripper.type,
            stream_bind=gripper.stream_bind,
        )
        self._reading_stop = threading.Event()

        def read() -> None:
            while not self._reading_stop.is_set():
                try:
                    msg = plant.read()
                    if msg is not None:
                        source.on_message(JOINT_STATE, msg)
                except Exception:
                    logger.exception("Local OpenYAM acquisition failed")
                    # Silence expires all leases, rather than fabricating a sample.
                    return
                self._reading_stop.wait(0.01)

        self._reading_thread = threading.Thread(target=read, name="openyam-feedback", daemon=True)
        self._reading_thread.start()

    def _publish_command(self, robot: str, values: dict[str, float]) -> None:
        if robot != "arm" or set(values) != {_JAW_KEY} or self._plant is None:
            raise ValueError("only the jaw may be commanded outside the tracker")
        if not self._estopped:
            self._plant.submit_jaw(values[_JAW_KEY])

    @rpc
    def reset_runtime_state(self, reactivate: bool | None = None) -> dict[str, bool]:
        results = super().reset_runtime_state(reactivate)
        with self._task_lock:
            for task in self._tasks.values():
                if isinstance(task, (QuestReference, PlannedReference)):
                    self._revoke_task_groups(task.name)
                    results[task.name] = task.reset_runtime_state(reactivate)
        return results

    @rpc
    def start(self) -> None:
        if self._plant is None and self.config.plant_factory is not None:
            self.bind_local_plant(self.config.plant_factory())
        if self._plant is None:
            raise RuntimeError("bind a local plant before starting tri-mode")
        if self._tick_loop is not None:
            if self._tick_loop.is_running:
                return
            raise RuntimeError("create a fresh tri-mode coordinator after stop")
        try:
            super().start()
        except Exception:
            with suppress(Exception):
                self.stop()
            raise

    @rpc
    def stop(self) -> None:
        try:
            super().stop()  # Revoke leases before joining any local worker.
        finally:
            with self._task_lock:
                for task in self._tasks.values():
                    if isinstance(task, (QuestReference, PlannedReference)):
                        task.stop()
            if self._reading_thread is not None:
                self._reading_stop.set()
                self._reading_thread.join(timeout=2.0)
            if self._tracker is not None:
                self._tracker.close()
            if self._reading_thread is not None and self._reading_thread.is_alive():
                raise RuntimeError("local OpenYAM acquisition did not stop")
