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

import time

import pytest

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.protocol.tf.tf_pose_source import TfPoseSource


class FakeTF(MultiTBuffer):
    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispose(self) -> None:
        pass


def _make_module(**kwargs: object) -> TfPoseSource:
    module = TfPoseSource(**kwargs)
    module._tf = FakeTF()  # type: ignore[assignment]
    return module


def _collect_published(module: TfPoseSource) -> list[Odometry]:
    published: list[Odometry] = []
    module.odometry.publish = published.append  # type: ignore[method-assign]
    return published


def test_tf_pose_source_publishes_odometry_from_tf_lookup() -> None:
    module = _make_module(target_frame="world", source_frame="camera", tf_tolerance_s=1.0)
    published = _collect_published(module)
    transform = Transform(
        translation=Vector3(1.0, 2.0, 3.0),
        rotation=Quaternion(0.0, 0.0, 0.707107, 0.707107),
        frame_id="world",
        child_frame_id="camera",
        ts=time.time(),
    )
    module.tfbuffer.receive_transform(transform)

    try:
        assert module.tick()

        assert len(published) == 1
        odometry = published[0]
        assert odometry.x == pytest.approx(1.0)
        assert odometry.y == pytest.approx(2.0)
        assert odometry.z == pytest.approx(3.0)
        assert odometry.orientation == transform.rotation
        assert odometry.vx == 0.0
        assert odometry.vy == 0.0
        assert odometry.vz == 0.0
        assert odometry.wx == 0.0
        assert odometry.wy == 0.0
        assert odometry.wz == 0.0
    finally:
        module.stop()


def test_tf_pose_source_skips_missing_and_stale_tf() -> None:
    module = _make_module(target_frame="world", source_frame="camera", tf_tolerance_s=0.05)
    published = _collect_published(module)

    try:
        assert not module.tick()
        assert published == []

        module.tfbuffer.receive_transform(
            Transform(frame_id="world", child_frame_id="camera", ts=time.time() - 10.0)
        )

        assert not module.tick()
        assert published == []
    finally:
        module.stop()


def test_tf_pose_source_sets_frame_ids() -> None:
    module = _make_module(target_frame="map", source_frame="wrist_camera", tf_tolerance_s=1.0)
    published = _collect_published(module)
    module.tfbuffer.receive_transform(
        Transform(frame_id="map", child_frame_id="wrist_camera", ts=time.time())
    )

    try:
        assert module.tick()

        assert published[0].frame_id == "map"
        assert published[0].child_frame_id == "wrist_camera"
    finally:
        module.stop()


def test_tf_pose_source_fixed_rate_lifecycle() -> None:
    module = _make_module(
        target_frame="world",
        source_frame="camera",
        tf_tolerance_s=1.0,
        publish_rate_hz=20.0,
    )
    published = _collect_published(module)
    module.tfbuffer.receive_transform(
        Transform(frame_id="world", child_frame_id="camera", ts=time.time())
    )

    module.start()
    time.sleep(0.16)
    module.stop()
    count_after_stop = len(published)
    time.sleep(0.08)

    assert 2 <= count_after_stop <= 5
    assert len(published) == count_after_stop
