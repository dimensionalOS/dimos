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
from __future__ import annotations

from pathlib import Path

import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.navigation.bench.playback import frames, play


def test_frames_read_coordinator_joint_state_when_joint_state_is_empty(tmp_path: Path) -> None:
    recording = tmp_path / "memory.db"
    store = SqliteStore(path=str(recording))
    store.start()
    store.stream("joint_state", JointState)
    poses = store.stream("ground_truth", PoseStamped)
    joints = store.stream("coordinator_joint_state", JointState)
    for ts, knee in ((1.0, 0.3), (2.0, 0.9)):
        poses.append(PoseStamped(ts=ts, position=[ts, 0.0, 0.5]), ts=ts)
        joints.append(JointState(ts=ts, name=["fl_knee_joint"], position=[knee]), ts=ts)
    store.stop()

    first, second = frames(recording)

    assert first.joints == {"fl_knee_joint": 0.3}
    assert second.joints == {"fl_knee_joint": 0.9}


def test_play_rejects_infinite_speed_before_loading_an_episode(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="finite"):
        play(tmp_path, speed=float("inf"))
