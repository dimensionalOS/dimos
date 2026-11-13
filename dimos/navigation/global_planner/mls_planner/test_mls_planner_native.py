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
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative


class _GoalTransport(Transport[PointStamped]):
    """Keeps every goal message the wrapper sends."""

    def __init__(self) -> None:
        self.sent: list[PointStamped] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def broadcast(self, selfstream: Stream[PointStamped] | None, value: PointStamped) -> None:
        self.sent.append(value)

    def subscribe(
        self,
        callback: Callable[[PointStamped], object],
        selfstream: Stream[PointStamped] | None = None,
    ) -> Callable[[], None]:
        return lambda: None


class _Rig(NamedTuple):
    module: MLSPlannerNative
    sent: list[PointStamped]


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
    (param,) = extract_rpc_info(MLSPlannerNative.go_to).params

    assert (param.name, param.annotation) == ("pose", PoseStamped)


def test_go_to_sends_the_pose_position_and_returns_its_stamp(rig: _Rig) -> None:
    stamp = rig.module.go_to(PoseStamped(position=(4.0, 2.0, 1.2), frame_id="map"))

    (point,) = rig.sent
    assert (point.x, point.y, point.z) == pytest.approx((4.0, 2.0, 1.2))
    assert point.frame_id == "map"
    assert [stamp.sec, stamp.nsec] == point.ros_timestamp()


@pytest.mark.parametrize("frame_id", ["odom", "map", ""])
def test_go_to_keeps_the_pose_frame_and_stamps_a_new_goal(rig: _Rig, frame_id: str) -> None:
    pose = PoseStamped(1.0, frame_id, position=(4.0, 2.0, 0.0))

    rig.module.go_to(pose)

    (point,) = rig.sent
    assert point.frame_id == frame_id
    assert point.ts != pose.ts


def test_a_second_go_to_replaces_the_one_under_way(rig: _Rig) -> None:
    rig.module.go_to(PoseStamped(position=(1.0, 0.0, 0.0)))
    rig.module.go_to(PoseStamped(position=(4.0, 2.0, 0.0)))

    # The planner drops a goal when the next one arrives, so no cancel goes between them.
    assert [(point.x, point.y) for point in rig.sent] == [(1.0, 0.0), (4.0, 2.0)]


def test_stop_navigation_sends_the_cancel_with_no_go_to_under_way(rig: _Rig) -> None:
    rig.module.stop_navigation()

    (cancel,) = rig.sent
    assert all(math.isnan(v) for v in (cancel.x, cancel.y, cancel.z))
