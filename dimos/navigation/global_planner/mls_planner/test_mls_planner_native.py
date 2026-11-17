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

"""The navigation RPCs on the MLS planner wrapper, without starting the binary."""

from __future__ import annotations

from collections.abc import Callable, Generator
import math
from typing import NamedTuple

import pytest

from dimos.core.introspection.module.info import extract_rpc_info
from dimos.core.stream import Stream, Transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative


class _GoalTransport(Transport[PoseStamped]):
    """Keeps every goal message the wrapper sends."""

    def __init__(self) -> None:
        self.sent: list[PoseStamped] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def broadcast(self, selfstream: Stream[PoseStamped] | None, value: PoseStamped) -> None:
        self.sent.append(value)

    def subscribe(
        self,
        callback: Callable[[PoseStamped], object],
        selfstream: Stream[PoseStamped] | None = None,
    ) -> Callable[[], None]:
        return lambda: None


class _Rig(NamedTuple):
    module: MLSPlannerNative
    sent: list[PoseStamped]


@pytest.fixture()
def rig() -> Generator[_Rig, None, None]:
    module = MLSPlannerNative()
    transport = _GoalTransport()
    module.goal.transport = transport
    yield _Rig(module, transport.sent)
    module._close_module()


def test_the_planner_exposes_navigation_as_plain_rpcs(rig: _Rig) -> None:
    assert rig.module.get_skills() == []
    assert {"go_to", "stop_navigation"} <= MLSPlannerNative.rpcs.keys()


def test_go_to_takes_a_pose_stamped() -> None:
    pose, face_heading = extract_rpc_info(MLSPlannerNative.go_to).params

    assert (pose.name, pose.annotation) == ("pose", PoseStamped)
    assert face_heading.name == "face_heading"


def test_go_to_sends_the_pose_position_under_a_new_stamp_and_returns_it(rig: _Rig) -> None:
    pose = PoseStamped(1.0, "map", position=(4.0, 2.0, 1.2))

    stamp = rig.module.go_to(pose)

    (goal,) = rig.sent
    assert (goal.x, goal.y, goal.z) == pytest.approx((4.0, 2.0, 1.2))
    assert goal.frame_id == "map"
    assert goal.ts != pose.ts
    assert [stamp.sec, stamp.nsec] == goal.ros_timestamp()


def test_go_to_sends_the_pose_heading_when_asked(rig: _Rig) -> None:
    facing = Quaternion.from_euler(Vector3(0.0, 0.0, math.radians(90.0)))

    rig.module.go_to(PoseStamped(position=(4.0, 2.0, 0.0), orientation=facing), face_heading=True)

    (goal,) = rig.sent
    assert goal.yaw == pytest.approx(math.radians(90.0))


def test_go_to_sends_no_heading_by_default(rig: _Rig) -> None:
    facing = Quaternion.from_euler(Vector3(0.0, 0.0, math.radians(90.0)))

    rig.module.go_to(PoseStamped(position=(4.0, 2.0, 0.0), orientation=facing))
    rig.module.go_to(PoseStamped(position=(4.0, 2.0, 0.0)))

    assert [goal.orientation.is_zero() for goal in rig.sent] == [True, True]


def test_a_second_go_to_replaces_the_one_under_way(rig: _Rig) -> None:
    rig.module.go_to(PoseStamped(position=(1.0, 0.0, 0.0)))
    rig.module.go_to(PoseStamped(position=(4.0, 2.0, 0.0)))

    # The planner drops a goal when the next one arrives, so no cancel goes between them.
    assert [(point.x, point.y) for point in rig.sent] == [(1.0, 0.0), (4.0, 2.0)]


def test_stop_navigation_sends_the_cancel_with_no_go_to_under_way(rig: _Rig) -> None:
    rig.module.stop_navigation()

    (cancel,) = rig.sent
    assert all(math.isnan(v) for v in (cancel.x, cancel.y, cancel.z))
