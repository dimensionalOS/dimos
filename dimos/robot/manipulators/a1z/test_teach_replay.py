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
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.imitation.collection.episode_monitor import EpisodeStatus
from dimos.imitation.collection.recorder import CollectionRecorder
from dimos.imitation.dataprep.core import Episode
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.a1z.blueprints.learning import (
    A1Z_TEACH_CAMERA_FPS,
    A1Z_TEACH_CAMERA_HEIGHT,
    A1Z_TEACH_CAMERA_WIDTH,
    make_a1z_policy_blueprint,
    make_a1z_replay_blueprint,
    make_a1z_teach_blueprint,
)
from dimos.robot.manipulators.a1z.teach_replay import (
    _REPLAY_VELOCITY_MAX,
    A1Z_JOINT_NAMES,
    RecordedEpisode,
    build_execution_trajectory,
    load_recorded_episode,
    prepare_episode,
)


class _FakePolicyModule(Module):
    color_image: In[Image]
    coordinator_joint_state: In[JointState]
    joint_command: Out[JointState]


def _module_kwargs(blueprint, module_type: type) -> dict[str, object]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


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


def test_teach_blueprint_records_webcam_and_all_joint_state(tmp_path: Path) -> None:
    blueprint = make_a1z_teach_blueprint(tmp_path / "teach.db", camera_index=3)
    camera_kwargs = _module_kwargs(blueprint, CameraModule)
    recorder_kwargs = _module_kwargs(blueprint, CollectionRecorder)
    control_kwargs = _module_kwargs(blueprint, ControlCoordinator)

    camera = camera_kwargs["hardware"]()

    assert isinstance(camera, Webcam)
    assert camera.config.camera_index == 3
    assert (camera.config.width, camera.config.height, camera.config.fps) == (
        A1Z_TEACH_CAMERA_WIDTH,
        A1Z_TEACH_CAMERA_HEIGHT,
        A1Z_TEACH_CAMERA_FPS,
    )
    assert recorder_kwargs["tf_tolerance"] == 1.5
    hardware = control_kwargs["hardware"][0]
    assert hardware.joints == list(A1Z_JOINT_NAMES)


def test_replay_blueprint_controls_arm_and_gripper() -> None:
    blueprint = make_a1z_replay_blueprint()
    control_kwargs = _module_kwargs(blueprint, ControlCoordinator)

    task = control_kwargs["tasks"][0]

    assert task.type == "trajectory"
    assert task.joint_names == list(A1Z_JOINT_NAMES)


def test_policy_blueprint_uses_checkpoint_and_all_joints() -> None:
    blueprint = make_a1z_policy_blueprint(
        "checkpoints/pick",
        policy_module=_FakePolicyModule,
    )
    policy_kwargs = _module_kwargs(blueprint, _FakePolicyModule)
    control_kwargs = _module_kwargs(blueprint, ControlCoordinator)

    assert policy_kwargs["policy_path"] == "checkpoints/pick"
    assert policy_kwargs["joint_names"] == list(A1Z_JOINT_NAMES)
    assert policy_kwargs["robot_type"] == "galaxea_a1z"
    assert control_kwargs["tasks"][0].joint_names == list(A1Z_JOINT_NAMES)


def test_loads_saved_episode_and_orders_joints(tmp_path: Path) -> None:
    path = tmp_path / "teach.db"
    store = SqliteStore(path=path)
    try:
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
    finally:
        store.stop()

    loaded = load_recorded_episode(path)

    assert loaded.episode_index == 0
    np.testing.assert_allclose(loaded.timestamps, [0.0, 0.1, 0.2])
    np.testing.assert_allclose(loaded.positions[0], base)


def test_prepare_rejects_recorded_positions_instead_of_clipping() -> None:
    positions = _positions()
    positions[2, 0] = 2.2

    with pytest.raises(ValueError, match=r"arm_joint1=2\.2000.*No values were clipped"):
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


def test_execution_trajectory_approaches_then_replays_all_joints() -> None:
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
