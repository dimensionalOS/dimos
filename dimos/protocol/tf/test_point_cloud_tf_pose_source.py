# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from __future__ import annotations

from collections.abc import Callable, Iterator
from typing import Any, cast

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.tf.point_cloud_tf_pose_source import PointCloudTfPoseSource
from dimos.protocol.tf.tf import MultiTBuffer


@pytest.fixture
def make_module() -> Iterator[Callable[..., PointCloudTfPoseSource]]:
    modules: list[PointCloudTfPoseSource] = []

    def make(**kwargs: object) -> PointCloudTfPoseSource:
        module = PointCloudTfPoseSource(**kwargs)
        cast("dict[str, Any]", module.__dict__)["_tf"] = MultiTBuffer()
        modules.append(module)
        return module

    yield make
    for module in modules:
        cast("dict[str, Any]", module.__dict__)["_tf"] = None
        module.dispose()


def _cloud(timestamp: float = 12.5, frame_id: str = "camera") -> PointCloud2:
    return PointCloud2.from_numpy(
        np.empty((0, 3), dtype=np.float32),
        frame_id=frame_id,
        timestamp=timestamp,
    )


def test_cloud_triggers_one_capture_stamped_odometry(
    make_module: Callable[..., PointCloudTfPoseSource], mocker: Any
) -> None:
    module = make_module(fixed_frame="world", tf_tolerance_s=0.01, tf_forward_tolerance_s=0.0)
    publish = mocker.patch.object(module.odometry, "publish")
    transform = Transform(
        translation=Vector3(1.0, 2.0, 3.0),
        rotation=Quaternion(0.0, 0.0, 0.707107, 0.707107),
        frame_id="world",
        child_frame_id="camera",
        ts=12.499,
    )
    module.tfbuffer.receive_transform(transform)

    assert module.on_pointcloud(_cloud()) is True

    published = cast("Odometry", publish.call_args.args[0])
    assert published.ts == 12.5
    assert published.frame_id == "world"
    assert published.child_frame_id == "camera"
    assert published.position == transform.translation
    assert published.orientation == transform.rotation


def test_missing_capture_tf_publishes_nothing(
    make_module: Callable[..., PointCloudTfPoseSource], mocker: Any
) -> None:
    module = make_module(tf_tolerance_s=0.001, tf_forward_tolerance_s=0.0)
    publish = mocker.patch.object(module.odometry, "publish")

    assert module.on_pointcloud(_cloud()) is False
    publish.assert_not_called()


def test_stale_tf_is_not_used_as_latest_fallback(
    make_module: Callable[..., PointCloudTfPoseSource], mocker: Any
) -> None:
    module = make_module(tf_tolerance_s=0.01, tf_forward_tolerance_s=0.0)
    publish = mocker.patch.object(module.odometry, "publish")
    module.tfbuffer.receive_transform(Transform(frame_id="world", child_frame_id="camera", ts=11.0))

    assert module.on_pointcloud(_cloud(timestamp=12.5)) is False
    publish.assert_not_called()


def test_empty_cloud_frame_is_rejected(
    make_module: Callable[..., PointCloudTfPoseSource], mocker: Any
) -> None:
    module = make_module()
    publish = mocker.patch.object(module.odometry, "publish")

    assert module.on_pointcloud(_cloud(frame_id="")) is False
    publish.assert_not_called()


def test_config_rejects_empty_fixed_frame() -> None:
    with pytest.raises(ValueError):
        PointCloudTfPoseSource(fixed_frame="")
