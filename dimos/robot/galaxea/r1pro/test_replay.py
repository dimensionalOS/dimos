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
import threading
import time

import numpy as np
import pytest

from dimos.hardware.sensors.camera.depth_cloud import DepthCloud
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_UPPER_BODY_JOINTS,
    ArticulatedTf,
    R1ProConnectionConfig,
)
from dimos.robot.galaxea.r1pro.replay import CAMERA_INFO_REPUBLISH_S, R1ProReplay
from dimos.utils.threadpool import get_scheduler, max_workers

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


@pytest.fixture(scope="module", autouse=True)
def shared_scheduler_workers():
    """Start the shared reactivex pool's workers before any test is watched.

    `In.observable()` runs on a process-wide pool whose workers spawn lazily, so
    the first test to move a message through one looks like it leaked them and
    the thread monitor in dimos/conftest.py fails it. Occupying every worker at
    once is what forces them all to exist; submitting serially reuses one.
    """
    gate = threading.Barrier(max_workers + 1)
    for _ in range(max_workers):
        get_scheduler().executor.submit(gate.wait)
    gate.wait(timeout=10.0)


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


class _Bus:
    """In-process stand-in for a Transport, so DepthCloud's `In` ports can be fed."""

    def __init__(self) -> None:
        self.subscribers: list = []

    def subscribe(self, callback, stream=None):
        self.subscribers.append(callback)
        return lambda: self.subscribers.remove(callback)

    def publish(self, msg) -> None:
        for callback in list(self.subscribers):
            callback(msg)

    def stop(self) -> None:
        pass


def test_a_head_only_obstacle_lands_where_it_was_authored(recording, module):
    """The head camera's whole reason to exist: seeing what the lidar cannot.

    A slab is authored in base_link above the chassis lidar, back-projected into
    a depth image through the very transform the connection derives from joint
    angles, and then rebuilt by the same path the robot uses — replay, DepthCloud,
    tf. Drop that transform, stack a second optical rotation onto it, or let the
    cloud inherit the vendor's optical frame instead of the calibration frame,
    and the points come back somewhere else.
    """
    config = R1ProConnectionConfig()
    fk = ArticulatedTf(R1PRO_MODEL, config.articulated_frame_ids, root_link=config.frame_id)
    neutral = JointState(
        ts=1000.0,
        frame_id=config.frame_id,
        name=R1PRO_UPPER_BODY_JOINTS,
        position=[0.0] * len(R1PRO_UPPER_BODY_JOINTS),
    )
    head = next(
        t for t in fk.transforms(neutral) if t.child_frame_id == config.head_camera_frame_id
    )

    fx = fy = 400.0
    width, height = 640, 480
    slab_y, slab_z = np.meshgrid(np.arange(-0.4, 0.4, 0.02), np.arange(1.0, 1.4, 0.02))
    slab_x = 2.5
    slab = np.stack([np.full(slab_y.size, slab_x), slab_y.ravel(), slab_z.ravel()], axis=1)

    homogeneous = np.hstack([slab, np.ones((len(slab), 1))])
    in_camera = (homogeneous @ np.linalg.inv(head.to_matrix()).T)[:, :3]
    columns = np.round(fx * in_camera[:, 0] / in_camera[:, 2] + width / 2).astype(int)
    rows = np.round(fy * in_camera[:, 1] / in_camera[:, 2] + height / 2).astype(int)
    depth_frame = np.zeros((height, width), dtype=np.float32)
    depth_frame[rows, columns] = in_camera[:, 2]

    info = CameraInfo.from_intrinsics(
        fx=fx,
        fy=fy,
        cx=width / 2,
        cy=height / 2,
        width=width,
        height=height,
        frame_id=config.head_camera_frame_id,
    )
    depth = Image(data=depth_frame, format=ImageFormat.DEPTH, frame_id="vendor_optical", ts=1000.0)
    # Several frames because DepthCloud drops depth that arrives before the
    # intrinsics, exactly as it drops the first frames off a real camera.
    frames = 5
    path = recording(
        head_depth=(Image, [depth] * frames),
        head_camera_info=(CameraInfo, [info] * frames),
        tf=(TFMessage, [TFMessage(head)] * frames),
    )

    tf = MultiTBuffer()
    clouds: list = []
    depth_cloud = DepthCloud(decimation=1, max_range_m=6.0)
    depth_cloud.depth.transport = _Bus()
    depth_cloud.camera_info.transport = _Bus()
    depth_cloud.cloud.subscribe(clouds.append)

    instance = module(dataset=str(path))
    instance.tf.subscribe(tf.receive_tfmessage)
    instance.head_camera_info.subscribe(depth_cloud.camera_info.transport.publish)
    instance.head_depth.subscribe(depth_cloud.depth.transport.publish)
    try:
        depth_cloud.start()
        instance.start()
        assert settle(clouds, frames)

        assert clouds[0].frame_id == config.head_camera_frame_id
        edge = tf.get(config.frame_id, clouds[0].frame_id, clouds[0].ts)
        assert edge is not None, "the recording carries no transform for the head camera"

        points = clouds[0].points_f32()
        rebuilt = (np.hstack([points, np.ones((len(points), 1))]) @ edge.to_matrix().T)[:, :3]
        np.testing.assert_allclose(rebuilt[:, 0], slab_x, atol=0.01)
        assert rebuilt[:, 2].min() > 0.99
        assert rebuilt[:, 2].max() < 1.39
    finally:
        depth_cloud.dispose()


def test_a_dataset_is_required(module):
    instance = module()
    with pytest.raises(ValueError, match="needs a recording"):
        instance.start()
