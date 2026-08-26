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

from typing import Any

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.nav_3d.mls_planner.start_relay import StartRelay
from dimos.protocol.tf.tf import MultiTBuffer

MOUNT_Z = 0.163


class FakeTF(MultiTBuffer):
    """In-memory tf with the dispose() hook and call counter the module tests need."""

    def __init__(self) -> None:
        super().__init__()
        self.gets = 0

    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
        *,
        forward_tolerance: float = 0.0,
    ) -> Transform | None:
        self.gets += 1
        return super().get(
            parent_frame,
            child_frame,
            time_point,
            time_tolerance,
            forward_tolerance=forward_tolerance,
        )

    def dispose(self) -> None:
        pass


def _mount() -> Transform:
    return Transform(
        translation=Vector3(0.0, 0.0, MOUNT_Z),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="base_link",
        child_frame_id="mid360_link",
        ts=1.0,
    )


def _odom_edge() -> Transform:
    return Transform(
        translation=Vector3(1.0, 2.0, 3.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="odom",
        child_frame_id="mid360_link",
        ts=2.0,
    )


def _relay(tf: FakeTF, **config: Any) -> tuple[StartRelay, list[PoseStamped]]:
    module = StartRelay(**config)
    module._tf = tf
    captured: list[PoseStamped] = []
    module.start_pose.subscribe(captured.append)
    return module, captured


def test_start_pose_is_the_tf_base_pose() -> None:
    tf = FakeTF()
    tf.receive_transform(_mount())
    tf.receive_transform(_odom_edge())
    module, captured = _relay(tf)
    try:
        module._on_tf(TFMessage())
        # The base sits MOUNT_Z below the sensor along the mount leg.
        assert len(captured) == 1
        assert abs(captured[0].position.x - 1.0) < 1e-9
        assert abs(captured[0].position.y - 2.0) < 1e-9
        assert abs(captured[0].position.z - (3.0 - MOUNT_Z)) < 1e-9
    finally:
        module.stop()


def test_nothing_published_while_the_chain_is_incomplete() -> None:
    tf = FakeTF()
    tf.receive_transform(_mount())
    module, captured = _relay(tf)
    try:
        module._on_tf(TFMessage())
        assert captured == []
    finally:
        module.stop()


def test_lookup_retries_are_throttled_during_an_outage() -> None:
    tf = FakeTF()
    module, captured = _relay(tf)
    try:
        module._on_tf(TFMessage())
        module._on_tf(TFMessage())
        assert captured == []
        assert tf.gets == 1
    finally:
        module.stop()
