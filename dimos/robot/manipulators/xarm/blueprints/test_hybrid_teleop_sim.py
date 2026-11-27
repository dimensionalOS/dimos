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

"""OperatorHoldCoordinator: the hold and e-stop status streams.

A rig drives one arm through connection ports, with the trajectory task and
the hold task, on a real OperatorHoldCoordinator, then ticks it by hand so
every assertion is deterministic.

The collection tests write a small session folder by hand and export it, to
check that a camera running at the blueprint's frame rate passes the
recording profile's checks.
"""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
from typing import Any, cast

import numpy as np
import pytest

from dimos.control._control_test_helpers import LocalTransport, Recorder, fake_arm_model
from dimos.control.components import make_joints
from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.contract.keys import Interface
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OPERATOR_HOLD_TASK_NAME,
    OperatorHoldStatus,
    OperatorHoldTask,
    operator_hold_task,
)
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
    TrajectoryExecutionStatus,
)
from dimos.control.tick_loop import TickLoop
from dimos.core.coordination.blueprint_config.fields import module_config_cls
from dimos.core.coordination.module_coordinator import resolve_module_refs
from dimos.core.stream import In, Out
from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.dataprep.build import run_dataprep
from dimos.imitation.dataprep.schema import OutputConfig, RecordingSchema, Sample
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.String import String
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.xarm.blueprints.hybrid_teleop_sim import (
    _CAMERA_HEIGHT,
    _CAMERA_WIDTH,
    OperatorHoldCoordinator,
    _collection_profile,
    hybrid_teleop_sim,
    hybrid_teleop_sim_collect,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

ARM_JOINTS = make_joints("arm", 3)
ARM_POSITIONS = [0.1, 0.2, 0.3]


def _trajectory() -> JointTrajectory:
    """One second from the arm's current pose to +0.5 rad on every joint."""
    return JointTrajectory(
        joint_names=list(ARM_JOINTS),
        points=[
            TrajectoryPoint(positions=list(ARM_POSITIONS), time_from_start=0.0),
            TrajectoryPoint(positions=[p + 0.5 for p in ARM_POSITIONS], time_from_start=1.0),
        ],
    )


class ArmHoldCoordinator(OperatorHoldCoordinator):
    """Ports for one arm behind a connection module."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


class Rig:
    def __init__(self) -> None:
        self.coordinator = ArmHoldCoordinator(
            publish_joint_state=False,
            reading_timeout_s=60.0,
            connections=[
                ConnectionConfig(
                    source="arm",
                    model=fake_arm_model(("joint1", "joint2", "joint3")),
                    joint_commands={
                        name: (Interface.POSITION,) for name in ("joint1", "joint2", "joint3")
                    },
                )
            ],
        )
        reading = LocalTransport()
        self.coordinator.set_transport("arm_joint_state", reading)  # type: ignore[arg-type]
        self.sent: list[tuple[str, JointState]] = []
        self.coordinator.set_transport(
            "arm_position_command",
            Recorder("arm_position_command", self.sent),  # type: ignore[arg-type]
        )
        self.coordinator.start()
        assert self.coordinator._tick_loop is not None
        self._loop = self.coordinator._tick_loop
        still = [0.0] * len(ARM_JOINTS)
        reading.publish(
            JointState(name=ARM_JOINTS, position=list(ARM_POSITIONS), velocity=still, effort=still)
        )

        self.trajectory_task = JointTrajectoryTask(
            JointTrajectoryTaskConfig(
                joint_names=ARM_JOINTS,
                velocity_limits=dict.fromkeys(ARM_JOINTS, 1000.0),
            )
        )
        self.coordinator.add_task(self.trajectory_task, task_type="trajectory")
        hold = self.coordinator._create_task_from_config(operator_hold_task())
        assert isinstance(hold, OperatorHoldTask)
        self.hold_task = hold
        self.coordinator.add_task(self.hold_task, task_type="operator_hold")

        self.hold_statuses: list[Bool] = []
        self.coordinator.operator_hold_status.subscribe(self.hold_statuses.append)
        self.estop_statuses: list[Bool] = []
        self.coordinator.operator_estop_status.subscribe(self.estop_statuses.append)

    def tick(self) -> None:
        self._loop._tick()

    def arm_writes(self) -> list[list[float]]:
        return [list(msg.position) for _, msg in self.sent]


@pytest.fixture
def rig(mocker) -> Iterator[Rig]:
    mocker.patch.object(TickLoop, "start")
    rig = Rig()
    try:
        yield rig
    finally:
        rig.coordinator.stop()


class TestHoldStatus:
    def test_request_holds_the_arm_and_sends_true(self, rig):
        status = rig.coordinator.task_invoke(
            OPERATOR_HOLD_TASK_NAME, "request", {"route": "manual", "reason": "arm stuck"}
        )
        assert isinstance(status, OperatorHoldStatus) and status.on
        assert rig.hold_statuses == []

        rig.tick()

        assert rig.arm_writes()[-1] == pytest.approx(ARM_POSITIONS)
        assert [type(msg) for msg in rig.hold_statuses] == [Bool]
        assert rig.hold_statuses[-1].data is True

    def test_acknowledge_releases_and_sends_false(self, rig):
        rig.coordinator.task_invoke(OPERATOR_HOLD_TASK_NAME, "request", {"route": "manual"})
        rig.tick()

        rig.coordinator.task_invoke(OPERATOR_HOLD_TASK_NAME, "acknowledge")

        assert [msg.data for msg in rig.hold_statuses] == [True, False]
        assert not rig.hold_task.is_active()
        assert (
            rig.coordinator.execute_trajectory(_trajectory()).status
            is TrajectoryExecutionStatus.ACCEPTED
        )


class TestEstopStatus:
    def test_set_estop_sends_true_then_false(self, rig):
        rig.coordinator.set_estop(True)

        assert [type(msg) for msg in rig.estop_statuses] == [Bool]
        assert rig.estop_statuses[-1].data is True
        assert rig.hold_task.get_status().estopped
        assert rig.hold_statuses[-1].data is True  # the e-stop turned the hold on

        rig.coordinator.set_estop(False)

        assert [msg.data for msg in rig.estop_statuses] == [True, False]
        assert rig.hold_statuses[-1].data is True  # on until acknowledged

    def test_start_sends_the_current_state(self):
        coordinator = OperatorHoldCoordinator(publish_joint_state=False)
        statuses: list[Bool] = []
        coordinator.operator_estop_status.subscribe(statuses.append)
        try:
            coordinator.start()
        finally:
            coordinator.stop()

        assert [msg.data for msg in statuses] == [False]


def test_request_operator_reaches_the_coordinator() -> None:
    assert resolve_module_refs(hybrid_teleop_sim.solidify())["teleopskills", "_coordinator"] == (
        "ControlCoordinator"
    )


def test_blueprint_ranks_hold_over_jog_over_autonomy():
    tasks = next(
        cast("list[TaskConfig]", cast("dict[str, Any]", atom.kwargs)["tasks"])
        for atom in hybrid_teleop_sim.solidify().blueprints
        if issubclass(atom.module, ControlCoordinator)
    )
    priority = {task.type: task.priority for task in tasks}

    assert priority["operator_hold"] > priority["eef_twist"] > priority["trajectory"]


def _camera_fps() -> float:
    """The frame rate (frames per second) the collection stack's sim camera is set to."""
    sim = next(
        atom
        for atom in hybrid_teleop_sim_collect.solidify().active_blueprints
        if issubclass(atom.module, MujocoSimModule)
    )
    return float(module_config_cls(sim)(**sim.kwargs).fps)


def _session(directory: Path, camera_fps: float) -> Path:
    """Write a session folder like the collection stack's, with one saved one-second episode.

    Args:
        directory: folder to create.
        camera_fps: how many camera frames per second the episode holds. The
            joint positions come at 100 per second.
    """
    schema = _collection_profile.to_schema()
    schema.payload = "recording.db"
    directory.mkdir()
    (directory / "schema.json").write_text(schema.model_dump_json())
    joints = list(schema.observation["state"].names)
    image = np.zeros((_CAMERA_HEIGHT, _CAMERA_WIDTH, 3), dtype=np.uint8)
    with SqliteStore(path=str(directory / schema.payload)) as store:
        status = store.stream("status", String, codec="json")
        camera = store.stream("color_image", Image, codec="lcm")
        state = store.stream("coordinator_joint_state", JointState, codec="lcm")
        for ts, event in [(10.0, "start"), (11.0, "save")]:
            document = EpisodeStatus(
                ts=ts,
                state="recording" if event == "start" else "idle",
                last_event=event,
                episodes_saved=0,
                episodes_discarded=0,
                task_label="teleop",
            )
            status.append(String(document.to_json()), ts=ts)
        for k in range(round(camera_fps) + 1):
            ts = 10.0 + k / camera_fps
            camera.append(Image(ts=ts, format=ImageFormat.RGB, data=image), ts=ts)
        for k in range(101):
            ts = 10.0 + k / 100
            state.append(JointState(ts=ts, name=joints, position=[0.0] * len(joints)), ts=ts)
    return directory


def _export(directory: Path) -> list[Sample]:
    """Run dataset export on a session folder and return the frames it writes."""
    config = RecordingSchema.read(directory).dataprep_config(
        directory, OutputConfig(format="hdf5", path=directory / "dataset.h5")
    )
    frames: list[Sample] = []

    def writer(samples: Iterator[Sample], _output: OutputConfig) -> Path:
        frames.extend(samples)
        return directory

    run_dataprep(config, writer=writer)
    return frames


def test_collection_export_accepts_the_sim_camera_rate(tmp_path: Path) -> None:
    fps = _camera_fps()

    frames = _export(_session(tmp_path / "session", fps))

    assert len(frames) == round(fps) + 1


def test_collection_export_rejects_half_the_sim_camera_rate(tmp_path: Path) -> None:
    session = _session(tmp_path / "session", _camera_fps() / 2)

    with pytest.raises(RuntimeError, match="failed dataset validation.*source rate"):
        _export(session)
