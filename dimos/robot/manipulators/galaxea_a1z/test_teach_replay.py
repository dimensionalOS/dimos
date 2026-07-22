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

from pathlib import Path

import numpy as np
import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.learning.collection.episode_monitor import EpisodeStatus
from dimos.learning.collection.recorder import CollectionRecorder
from dimos.learning.dataprep.core import Episode
from dimos.learning.lerobot_policy import LeRobotPolicyConfig, LeRobotPolicyModule
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.galaxea_a1z.blueprints.basic import (
    A1Z_TEACH_CAMERA_FPS,
    A1Z_TEACH_CAMERA_HEIGHT,
    A1Z_TEACH_CAMERA_WIDTH,
    make_a1z_learned_policy_blueprint,
    make_a1z_policy_blueprint,
    make_a1z_teach_blueprint,
)


class CustomPolicyModule(LeRobotPolicyModule):
    pass


from dimos.robot.manipulators.galaxea_a1z.teach_replay import (
    _REPLAY_VELOCITY_MAX,
    A1Z_JOINT_NAMES,
    RecordedEpisode,
    build_execution_trajectory,
    load_recorded_episode,
    prepare_episode,
)


def _positions(count: int = 5) -> np.ndarray:
    base = np.array([0.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.05])
    return np.repeat(base[np.newaxis, :], count, axis=0)


def _recorded(positions: np.ndarray, period: float = 0.1) -> RecordedEpisode:
    timestamps = np.arange(len(positions), dtype=float) * period
    return RecordedEpisode(
        episode=Episode(id="ep_000000", start_ts=10.0, end_ts=10.0 + timestamps[-1]),
        episode_index=0,
        timestamps=timestamps,
        positions=positions,
    )


def test_teach_blueprint_records_from_configured_webcam(tmp_path: Path) -> None:
    blueprint = make_a1z_teach_blueprint(tmp_path / "teach.db", camera_index=3)
    camera_atom = next(atom for atom in blueprint.blueprints if atom.module is CameraModule)
    recorder_atom = next(atom for atom in blueprint.blueprints if atom.module is CollectionRecorder)

    camera = camera_atom.kwargs["hardware"]()

    assert isinstance(camera, Webcam)
    assert camera.config.camera_index == 3
    assert (camera.config.width, camera.config.height, camera.config.fps) == (
        A1Z_TEACH_CAMERA_WIDTH,
        A1Z_TEACH_CAMERA_HEIGHT,
        A1Z_TEACH_CAMERA_FPS,
    )
    assert camera.config.frame_id_prefix is None
    transform = camera_atom.kwargs["transform"]
    assert (transform.frame_id, transform.child_frame_id) == ("coordinator", "camera_link")
    assert recorder_atom.kwargs["tf_tolerance"] == 1.5
    assert blueprint.blueprints.index(camera_atom) > blueprint.blueprints.index(recorder_atom)


def test_policy_blueprint_wires_camera_policy_and_seven_joint_servo() -> None:
    blueprint = make_a1z_policy_blueprint(
        "checkpoints/a1z",
        task="pick up cube",
        camera_index=2,
        device="cuda",
    )
    camera_atom = next(atom for atom in blueprint.blueprints if atom.module is CameraModule)
    policy_atom = next(atom for atom in blueprint.blueprints if atom.module is LeRobotPolicyModule)
    control_atom = next(atom for atom in blueprint.blueprints if atom.module is ControlCoordinator)

    camera = camera_atom.kwargs["hardware"]()
    hardware = control_atom.kwargs["hardware"][0]
    servo = control_atom.kwargs["tasks"][0]

    assert camera.config.camera_index == 2
    policy = policy_atom.kwargs["policies"]["default"]
    assert policy.policy_path == "checkpoints/a1z"
    assert policy_atom.kwargs["joint_names"] == hardware.all_joints == list(A1Z_JOINT_NAMES)
    assert policy.task == "pick up cube"
    assert policy.device == "cuda"
    assert servo.type == "servo"
    assert servo.joint_names == list(A1Z_JOINT_NAMES)


def test_multi_policy_blueprint_uses_custom_skill_module() -> None:
    policies = {
        "pick_up_cup": LeRobotPolicyConfig(policy_path="checkpoints/cup"),
        "place_cup": LeRobotPolicyConfig(policy_path="checkpoints/place"),
    }

    blueprint = make_a1z_learned_policy_blueprint(
        policies,
        policy_module=CustomPolicyModule,
    )

    policy_atom = next(atom for atom in blueprint.blueprints if atom.module is CustomPolicyModule)
    assert policy_atom.kwargs["policies"] == policies
    assert policy_atom.kwargs["robot_type"] == "galaxea_a1z"


def test_loads_saved_memory2_episode_and_orders_joints(tmp_path: Path) -> None:
    path = tmp_path / "teach.db"
    store = SqliteStore(path=path)
    status = store.stream("status", EpisodeStatus)
    joints = store.stream("coordinator_joint_state", JointState)
    status.append(
        EpisodeStatus(
            ts=10.0,
            state="recording",
            episodes_saved=0,
            episodes_discarded=0,
            last_event="start",
        ),
        ts=10.0,
    )
    reversed_names = list(reversed(A1Z_JOINT_NAMES))
    base = _positions(1)[0]
    for index, ts in enumerate((10.05, 10.15, 10.25)):
        sample = base.copy()
        sample[0] += index / 10
        values = dict(zip(A1Z_JOINT_NAMES, sample, strict=True))
        joints.append(
            JointState(
                ts=ts,
                name=reversed_names,
                position=[values[name] for name in reversed_names],
            ),
            ts=ts,
        )
    status.append(
        EpisodeStatus(
            ts=10.3,
            state="idle",
            episodes_saved=1,
            episodes_discarded=0,
            last_event="save",
        ),
        ts=10.3,
    )
    store.stop()

    loaded = load_recorded_episode(path)

    assert loaded.episode_index == 0
    np.testing.assert_allclose(loaded.timestamps, [0.0, 0.1, 0.2])
    np.testing.assert_allclose(loaded.positions[0], base)


def test_prepare_rejects_recorded_positions_instead_of_clipping() -> None:
    positions = _positions()
    positions[2, 0] = 2.2

    with pytest.raises(ValueError, match=r"arm/joint1=2\.2000.*No values were clipped"):
        prepare_episode(_recorded(positions))


def test_prepare_smooths_resamples_and_time_scales_fast_motion() -> None:
    positions = _positions()
    positions[:, 0] = np.linspace(0.0, 1.0, len(positions))

    prepared = prepare_episode(
        _recorded(positions, period=0.025),
        speed=1.0,
        sample_rate_hz=100.0,
        smoothing_window_s=0.05,
    )

    assert prepared.effective_speed < 1.0
    assert prepared.duration > prepared.recorded.timestamps[-1]
    assert len(prepared.timestamps) > len(positions)
    assert np.all(np.diff(prepared.timestamps) > 0)
    assert np.max(np.abs(prepared.velocities[:, 0])) <= _REPLAY_VELOCITY_MAX[0]


def test_execution_trajectory_approaches_then_replays_all_seven_joints() -> None:
    positions = _positions()
    positions[:, 0] = np.linspace(0.2, 0.4, len(positions))
    prepared = prepare_episode(_recorded(positions), smoothing_window_s=0.0)
    current = dict(zip(A1Z_JOINT_NAMES, [0.0, 0.4, -0.4, 0.1, 0.0, 0.0, 0.03], strict=True))

    trajectory = build_execution_trajectory(current, prepared)

    assert trajectory.joint_names == list(A1Z_JOINT_NAMES)
    assert trajectory.points[0].positions == pytest.approx(list(current.values()))
    assert trajectory.points[-1].positions == pytest.approx(prepared.positions[-1])
    assert trajectory.points[-1].velocities == pytest.approx([0.0] * 7)
    assert all(
        previous.time_from_start < current_point.time_from_start
        for previous, current_point in zip(trajectory.points, trajectory.points[1:], strict=False)
    )
