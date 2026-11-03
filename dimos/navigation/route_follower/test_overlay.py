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

from __future__ import annotations

import json

from dimos_lcm.std_msgs import String

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.route_follower.messages import build_state_document
from dimos.navigation.route_follower.overlay import route_path_overlay, route_state_overlay
from dimos.navigation.route_follower.route import Route


def _state_msg(route: Route) -> String:
    return String(data=json.dumps(build_state_document(route, robot_position_known=True)))


def _rgba(rgb: tuple[int, int, int]) -> int:
    r, g, b = rgb
    return (r << 24) | (g << 16) | (b << 8) | 255


GREY = (128, 128, 128)
YELLOW = (255, 255, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)


def test_empty_route_yields_empty_point_set() -> None:
    points = route_state_overlay(_state_msg(Route()))

    assert points.positions.as_arrow_array().to_pylist() == []
    assert points.labels.as_arrow_array().to_pylist() == []
    assert points.colors.as_arrow_array().to_pylist() == []


def test_waypoints_yield_points_labelled_with_their_index() -> None:
    route = Route()
    route.add_waypoint(1.5, -0.5, 0.0)
    route.add_waypoint(3.0, 0.0, 0.0)
    route.add_waypoint(4.5, 1.0, 0.0)

    points = route_state_overlay(_state_msg(route))

    assert points.positions.as_arrow_array().to_pylist() == [
        [1.5, -0.5, 0.0],
        [3.0, 0.0, 0.0],
        [4.5, 1.0, 0.0],
    ]
    assert points.labels.as_arrow_array().to_pylist() == ["1", "2", "3"]


def test_point_colours_follow_waypoint_state() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)
    route.add_waypoint(3.0, 0.0, 0.0)
    route.start()
    route.mark_current_reached()

    points = route_state_overlay(_state_msg(route))
    assert points.colors.as_arrow_array().to_pylist() == [
        _rgba(GREEN),
        _rgba(YELLOW),
        _rgba(GREY),
    ]

    failed = Route()
    failed.add_waypoint(1.0, 0.0, 0.0)
    failed.start()
    failed.fail_current()

    points = route_state_overlay(_state_msg(failed))
    assert points.colors.as_arrow_array().to_pylist() == [_rgba(RED)]


def test_route_path_draws_in_a_colour_distinct_from_the_planner_path() -> None:
    path = Path(
        frame_id="world",
        poses=[
            PoseStamped(ts=0.0, frame_id="world", position=[1.0, 0.0, 0.0]),
            PoseStamped(ts=0.0, frame_id="world", position=[2.0, 0.0, 0.0]),
        ],
    )

    line = route_path_overlay(path)

    assert line.strips.as_arrow_array().to_pylist() == [[[1.0, 0.0, 0.5], [2.0, 0.0, 0.5]]]
    assert (
        line.colors.as_arrow_array().to_pylist()
        != path.to_rerun().colors.as_arrow_array().to_pylist()
    )
    assert line.colors.as_arrow_array().to_pylist() == [_rgba((255, 128, 0))]
