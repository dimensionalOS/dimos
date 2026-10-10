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

from pathlib import Path
import time

import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.navigation.bench.playback import Puppet, frames, paced
from dimos.simulation.go2_legged.policy import OnnxGo2Policy
from dimos.simulation.scenes.procedural import office

JOINTS = list(OnnxGo2Policy.joint_names)
T0 = 1000.0


def _record(path: Path) -> None:
    store = SqliteStore(path=str(path))
    store.start()
    try:
        poses = store.stream("ground_truth", PoseStamped)
        joints = store.stream("joint_state", JointState)
        for k in range(3):
            t = T0 + 0.02 * k
            poses.append(
                PoseStamped(1.0 + k, 2.0, 0.3, 0.0, 0.0, 0.0, 1.0, ts=t, frame_id="odom"), ts=t
            )
            angles = [0.1 * k] * len(JOINTS)
            joints.append(JointState(ts=t + 0.001, name=JOINTS, position=angles), ts=t + 0.001)
    finally:
        store.stop()


def test_frames_pair_each_pose_with_the_nearest_joint_state(tmp_path: Path) -> None:
    _record(tmp_path / "memory.db")
    recorded = frames(tmp_path / "memory.db")
    assert [f.position[0] for f in recorded] == [1.0, 2.0, 3.0]
    assert recorded[2].joints["FL_thigh_joint"] == pytest.approx(0.2)
    assert recorded[1].quaternion.tolist() == [1.0, 0.0, 0.0, 0.0]


def test_puppet_takes_the_recorded_pose(tmp_path: Path) -> None:
    _record(tmp_path / "memory.db")
    puppet = Puppet(office(1))
    puppet.pose(frames(tmp_path / "memory.db")[-1])
    assert puppet.trunk_position() == pytest.approx([3.0, 2.0, 0.3])
    assert puppet.data.qpos[7:] == pytest.approx([0.2] * len(JOINTS))


def test_paced_keeps_the_recorded_rhythm(tmp_path: Path) -> None:
    _record(tmp_path / "memory.db")
    started = time.monotonic()
    stamps = [f.t for f in paced(frames(tmp_path / "memory.db"), speed=0.1)]
    assert time.monotonic() - started >= (stamps[-1] - stamps[0]) / 0.1 - 0.01
