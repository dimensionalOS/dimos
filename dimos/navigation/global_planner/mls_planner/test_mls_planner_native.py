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
from unittest.mock import Mock

import pytest
from pytest_mock import MockerFixture

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
    pose: Mock


@pytest.fixture()
def rig(mocker: MockerFixture) -> Generator[_Rig, None, None]:
    module = MLSPlannerNative(start_z_offset_m=0.3)
    transport = _GoalTransport()
    module.goal.transport = transport
    pose = mocker.patch.object(module, "_pose", return_value=None)
    yield _Rig(module, transport.sent, pose)
    module._close_module()


def test_the_planner_exposes_navigation_as_plain_rpcs(rig: _Rig) -> None:
    assert rig.module.get_skills() == []
    assert {"go_to", "stop_navigation"} <= MLSPlannerNative.rpcs.keys()


def test_go_to_sends_the_point_at_the_floor_height_and_returns_its_stamp(rig: _Rig) -> None:
    rig.pose.return_value = PoseStamped(position=(0.0, 0.0, 1.5))

    stamp = rig.module.go_to(4.0, 2.0)

    (point,) = rig.sent
    assert (point.x, point.y, point.z) == pytest.approx((4.0, 2.0, 1.2))
    assert point.frame_id == "odom"
    assert [stamp.sec, stamp.nsec] == point.ros_timestamp()


def test_go_to_raises_without_a_pose(rig: _Rig) -> None:
    with pytest.raises(LookupError):
        rig.module.go_to(4.0, 2.0)
    assert rig.sent == []


def test_a_second_go_to_replaces_the_one_under_way(rig: _Rig) -> None:
    rig.pose.return_value = PoseStamped(position=(1.0, 0.0, 0.3))

    rig.module.go_to(1.0, 0.0)
    rig.module.go_to(4.0, 2.0)

    # The planner drops a goal when the next one arrives, so no cancel goes between them.
    assert [(point.x, point.y) for point in rig.sent] == [(1.0, 0.0), (4.0, 2.0)]


def test_stop_navigation_sends_the_cancel_with_no_go_to_under_way(rig: _Rig) -> None:
    rig.module.stop_navigation()

    (cancel,) = rig.sent
    assert all(math.isnan(v) for v in (cancel.x, cancel.y, cancel.z))
