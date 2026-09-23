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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage

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
    ) -> TransformStamped | None:
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


def _mount() -> TransformStamped:
    return TransformStamped(
        header=Header(frame_id="base_link", stamp=Time(sec=1)),
        child_frame_id="mid360_link",
        transform=Transform(translation=Vector3(z=MOUNT_Z), rotation=Quaternion(w=1.0)),
    )


def _odom_edge() -> TransformStamped:
    return TransformStamped(
        header=Header(frame_id="odom", stamp=Time(sec=2)),
        child_frame_id="mid360_link",
        transform=Transform(translation=Vector3(x=1.0, y=2.0, z=3.0), rotation=Quaternion(w=1.0)),
    )


def _relay(tf: FakeTF, **config: Any) -> tuple[StartRelay, list[PoseStamped]]:
    module = StartRelay(**config)
    module._tf = tf
    captured: list[PoseStamped] = []
    module.start_pose.subscribe(lambda pose: captured.append(PoseStamped.decode(pose.encode())))
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
        assert abs(captured[0].pose.position.x - 1.0) < 1e-9
        assert abs(captured[0].pose.position.y - 2.0) < 1e-9
        assert abs(captured[0].pose.position.z - (3.0 - MOUNT_Z)) < 1e-9
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
