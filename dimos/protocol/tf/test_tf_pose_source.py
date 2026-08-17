from __future__ import annotations

import time
from typing import Any, cast

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


def _module(**kwargs: object) -> TfPoseSource:
    module = TfPoseSource(**kwargs)
    cast("Any", module)._tf = FakeTF()
    return module


def test_pose_source_publishes_pose_only_odometry() -> None:
    module = _module(target_frame="world", source_frame="camera")
    published: list[Odometry] = []
    module.odometry.publish = published.append  # type: ignore[method-assign]
    transform = Transform(
        translation=Vector3(1.0, 2.0, 3.0),
        rotation=Quaternion(0.0, 0.0, 0.707107, 0.707107),
        frame_id="world",
        child_frame_id="camera",
        ts=time.time(),
    )
    module.tf.receive_transform(transform)

    assert module.tick()
    assert len(published) == 1
    assert published[0].x == pytest.approx(1.0)
    assert published[0].orientation == transform.rotation
    assert published[0].vx == published[0].vy == published[0].vz == 0.0
    assert published[0].wx == published[0].wy == published[0].wz == 0.0
    module.stop()


def test_pose_source_reports_missing_tf_without_publishing(mocker: Any) -> None:
    module = _module()
    published: list[Odometry] = []
    module.odometry.publish = published.append  # type: ignore[method-assign]
    get = mocker.spy(module.tfbuffer, "get")

    try:
        assert not module.tick()
        assert published == []
        get.assert_not_called()
    finally:
        module.stop()


def test_pose_source_forwards_latest_capture_even_when_publication_is_delayed() -> None:
    module = _module()
    published: list[Odometry] = []
    module.odometry.publish = published.append  # type: ignore[method-assign]
    capture_time = time.time() - 0.05
    try:
        module.tf.receive_transform(
            Transform(frame_id="world", child_frame_id="base_link", ts=capture_time)
        )

        assert module.tick()
        assert len(published) == 1
        assert published[0].ts == capture_time
    finally:
        module.stop()


def test_pose_source_fixed_rate_lifecycle() -> None:
    module = _module(publish_rate_hz=20.0)
    published: list[Odometry] = []
    module.odometry.publish = published.append  # type: ignore[method-assign]
    module.tf.receive_transform(
        Transform(frame_id="world", child_frame_id="base_link", ts=time.time())
    )

    module.start()
    time.sleep(0.16)
    module.stop()
    count = len(published)
    time.sleep(0.05)

    assert 2 <= count <= 5
    assert len(published) == count


def test_pose_source_stream_type_is_odometry() -> None:
    assert TfPoseSource.__annotations__["odometry"]
