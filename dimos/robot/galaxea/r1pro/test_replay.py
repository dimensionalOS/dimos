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

import numpy as np
import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.galaxea.r1pro.replay import CAMERA_INFO_REPUBLISH_S, R1ProReplay

CAMERA_INFO_YAML = """
image_width: 640
image_height: 480
camera_name: head_left
camera_matrix:
  rows: 3
  cols: 3
  data: [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [400.0, 0.0, 320.0, 0.0, 0.0, 400.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
"""


def cloud(ts: float) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.array([[1.0, 0.0, 0.0]], dtype=np.float32),
        frame_id="lidar_chassis_left_link",
        timestamp=ts,
    )


@pytest.fixture
def recording(tmp_path: Path):
    """Write a recording, then hand back its path. Timestamps span 0.1 s so the
    whole thing replays inside a test."""

    def write(**streams) -> Path:
        path = tmp_path / "r1pro.db"
        store = SqliteStore(path=str(path))
        with store:
            for name, (msg_type, messages) in streams.items():
                stream = store.stream(name, msg_type)
                for index, msg in enumerate(messages):
                    stream.append(msg, ts=1000.0 + index * 0.05)
        return path

    return write


@pytest.fixture
def module():
    built = []

    def build(**config):
        instance = R1ProReplay(**config)
        built.append(instance)
        return instance

    yield build
    for instance in built:
        instance.dispose()


def settle(messages: list, expected: int) -> list:
    deadline = time.monotonic() + 5.0
    while len(messages) < expected and time.monotonic() < deadline:
        time.sleep(0.01)
    return messages


def test_replays_a_recorded_stream_onto_the_live_port(recording, module):
    path = recording(lidar=(PointCloud2, [cloud(0.0), cloud(0.0)]))
    instance = module(dataset=str(path))

    clouds: list = []
    instance.lidar.subscribe(clouds.append)
    instance.start()

    assert len(settle(clouds, 2)) == 2
    assert clouds[0].frame_id == "lidar_chassis_left_link"


def test_the_recorded_topic_slug_spelling_is_accepted(recording, module):
    """The generic recorder names the stream after the zenoh topic, so
    ``motor_states`` arrives spelled ``r1pro_motor_states``."""
    path = recording(
        r1pro_motor_states=(JointState, [JointState(name=["torso_joint1"], position=[0.0], ts=0.0)])
    )
    instance = module(dataset=str(path))

    states: list = []
    instance.motor_states.subscribe(states.append)
    instance.start()

    assert len(settle(states, 1)) == 1


def test_stream_remapping_overrides_the_candidates(recording, module):
    path = recording(chassis_pose=(PoseStamped, [PoseStamped(ts=0.0, frame_id="odom")]))
    instance = module(dataset=str(path), stream_remapping={"chassis_odom": "chassis_pose"})

    poses: list = []
    instance.chassis_odom.subscribe(poses.append)
    instance.start()

    assert len(settle(poses, 1)) == 1


def test_a_missing_stream_leaves_its_port_silent(recording, module):
    """A recording made before the head camera existed still has to drive the
    lidar half of the stack rather than fail to start."""
    path = recording(lidar=(PointCloud2, [cloud(0.0)]))
    instance = module(dataset=str(path))

    clouds: list = []
    depth: list = []
    instance.lidar.subscribe(clouds.append)
    instance.head_depth.subscribe(depth.append)
    instance.start()

    # Wait on the stream that is present, so the recording has demonstrably
    # played through by the time the absent one is checked.
    assert len(settle(clouds, 1)) == 1
    assert depth == []


def test_camera_info_yaml_stands_in_for_a_recording_without_one(recording, module, tmp_path):
    yaml_path = tmp_path / "head_left.yaml"
    yaml_path.write_text(CAMERA_INFO_YAML)
    path = recording(lidar=(PointCloud2, [cloud(0.0)]))
    instance = module(dataset=str(path), head_camera_info_path=str(yaml_path))

    infos: list = []
    instance.head_camera_info.subscribe(infos.append)
    instance.start()

    assert len(settle(infos, 1)) == 1
    assert infos[0].frame_id == "camera_head_left_link"
    assert infos[0].K[0] == pytest.approx(400.0)


def test_a_recorded_camera_info_wins_over_the_yaml(recording, module, tmp_path):
    """Otherwise a stale calibration file would silently override the one the
    recording was actually made with."""
    yaml_path = tmp_path / "head_left.yaml"
    yaml_path.write_text(CAMERA_INFO_YAML)
    recorded = CameraInfo.from_yaml(str(yaml_path), frame_id="camera_head_left_link")
    recorded.K[0] = 999.0
    path = recording(head_camera_info=(CameraInfo, [recorded]))
    instance = module(dataset=str(path), head_camera_info_path=str(yaml_path))

    infos: list = []
    instance.head_camera_info.subscribe(infos.append)
    instance.start()

    settle(infos, 1)
    # Past one republish period, so a yaml fallback would have shown itself.
    time.sleep(CAMERA_INFO_REPUBLISH_S * 1.5)
    assert infos
    assert [info.K[0] for info in infos] == [pytest.approx(999.0)] * len(infos)


def test_a_dataset_is_required(module):
    instance = module()
    with pytest.raises(ValueError, match="needs a recording"):
        instance.start()
