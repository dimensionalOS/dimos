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

from dimos.navigation.route_follower.route import Route, RouteStatus, WaypointState


def test_add_twentieth_is_accepted_and_twenty_first_is_refused() -> None:
    route = Route()
    for i in range(20):
        assert route.add_waypoint(float(i), 0.0, 0.0) is True
    assert [wp.index for wp in route.waypoints] == list(range(1, 21))

    assert route.add_waypoint(20.0, 0.0, 0.0) is False
    assert len(route.waypoints) == 20
    assert route.message == "route is full"


def test_add_rejects_non_finite_and_out_of_range_positions() -> None:
    route = Route()
    assert route.add_waypoint(float("nan"), 0.0, 0.0) is False
    assert route.message == "invalid waypoint position"
    assert route.add_waypoint(0.0, float("inf"), 0.0) is False
    assert route.add_waypoint(501.0, 0.0, 0.0) is False
    assert route.add_waypoint(0.0, -501.0, 0.0) is False
    assert route.add_waypoint(0.0, 0.0, 51.0) is False
    assert route.waypoints == ()

    assert route.add_waypoint(500.0, -500.0, 50.0) is True
    assert [wp.index for wp in route.waypoints] == [1]


def test_remove_renumbers_waypoints() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)
    route.add_waypoint(3.0, 0.0, 0.0)

    assert route.remove_waypoint(2) is True
    assert [(wp.index, wp.x) for wp in route.waypoints] == [(1, 1.0), (2, 3.0)]


def test_remove_out_of_range_leaves_the_route_unchanged() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)

    assert route.remove_waypoint(2) is False
    assert route.message == "no waypoint 2"
    assert route.remove_waypoint(0) is False
    assert route.message == "no waypoint 0"
    assert [(wp.index, wp.x) for wp in route.waypoints] == [(1, 1.0)]


def test_clear_empties_the_route() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)

    assert route.clear() is True
    assert route.waypoints == ()
    assert route.current_index is None


def test_set_add_mode_toggles_the_setting() -> None:
    route = Route()
    assert route.add_mode is False

    route.set_add_mode(True)
    assert route.add_mode is True

    route.set_add_mode(False)
    assert route.add_mode is False


def test_start_without_waypoints_is_refused() -> None:
    route = Route()
    assert route.start() is False
    assert route.status is RouteStatus.PLANNING
    assert route.message == "no waypoints to follow"


def test_start_enters_running_with_first_waypoint_current() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)

    assert route.start() is True
    assert route.status is RouteStatus.RUNNING
    assert route.current_index == 1
    assert route.waypoints[0].state is WaypointState.CURRENT
    assert route.waypoints[1].state is WaypointState.PENDING


def test_start_while_running_is_ignored() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    assert route.start() is True
    revision = route.revision

    assert route.start() is False
    assert route.revision == revision
    assert route.status is RouteStatus.RUNNING
    assert route.current_index == 1


def test_advance_marks_current_reached_and_next_current() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)
    route.add_waypoint(3.0, 0.0, 0.0)
    route.start()

    assert route.mark_current_reached() is True
    assert route.waypoints[0].state is WaypointState.REACHED
    assert route.waypoints[1].state is WaypointState.CURRENT
    assert route.waypoints[2].state is WaypointState.PENDING
    assert route.current_index == 2


def test_reaching_the_last_waypoint_completes_the_route() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)
    route.start()
    route.mark_current_reached()

    assert route.mark_current_reached() is True
    assert route.status is RouteStatus.COMPLETED
    assert route.current_index is None
    assert [wp.state for wp in route.waypoints] == [WaypointState.REACHED, WaypointState.REACHED]


def test_stop_ends_the_route_keeping_waypoint_states() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)
    route.start()
    route.mark_current_reached()

    assert route.stop() is True
    assert route.status is RouteStatus.STOPPED
    assert route.current_index is None
    assert route.waypoints[0].state is WaypointState.REACHED
    assert route.waypoints[1].state is WaypointState.CURRENT


def test_interrupt_ends_the_running_route() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.start()

    assert route.interrupt() is True
    assert route.status is RouteStatus.INTERRUPTED
    assert route.current_index is None
    assert route.waypoints[0].state is WaypointState.CURRENT


def test_hold_keeps_the_current_waypoint() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)
    route.start()

    assert route.hold() is True
    assert route.status is RouteStatus.HELD
    assert route.current_index == 1
    assert route.waypoints[0].state is WaypointState.CURRENT


def test_fail_current_marks_the_waypoint_failed_and_names_it() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)
    route.start()
    route.mark_current_reached()

    assert route.fail_current() is True
    assert route.status is RouteStatus.FAILED
    assert route.current_index is None
    assert route.waypoints[1].state is WaypointState.FAILED
    assert route.message == "waypoint 2 could not be reached"


def test_start_from_held_resumes_from_the_first_unreached_waypoint() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)
    route.add_waypoint(3.0, 0.0, 0.0)
    route.start()
    route.mark_current_reached()
    route.hold()

    assert route.start() is True
    assert route.status is RouteStatus.RUNNING
    assert route.current_index == 2
    assert route.waypoints[1].state is WaypointState.CURRENT
    assert route.waypoints[2].state is WaypointState.PENDING


def test_start_from_an_ended_route_is_refused_until_edit() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.start()
    route.stop()

    assert route.start() is False
    assert route.message == "edit the route first"
    assert route.status is RouteStatus.STOPPED

    route.clear()
    route.add_waypoint(1.0, 0.0, 0.0)
    assert route.start() is True
    assert route.status is RouteStatus.RUNNING


def test_edits_are_refused_while_running() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.start()
    before = [(wp.index, wp.x) for wp in route.waypoints]

    assert route.add_waypoint(2.0, 0.0, 0.0) is False
    assert route.message == "stop the route first"
    assert route.remove_waypoint(1) is False
    assert route.message == "stop the route first"
    assert route.clear() is False
    assert route.message == "stop the route first"

    assert [(wp.index, wp.x) for wp in route.waypoints] == before
    assert route.status is RouteStatus.RUNNING
    assert route.current_index == 1


def test_first_edit_after_an_ended_route_resets_waypoint_states() -> None:
    for end in (Route.stop, Route.interrupt, Route.hold, Route.fail_current):
        route = Route()
        route.add_waypoint(1.0, 0.0, 0.0)
        route.add_waypoint(2.0, 0.0, 0.0)
        route.start()
        route.mark_current_reached()
        assert end(route) is True

        assert route.add_waypoint(3.0, 0.0, 0.0) is True
        assert route.status is RouteStatus.PLANNING
        assert route.current_index is None
        assert [wp.state for wp in route.waypoints] == [WaypointState.PENDING] * 3


def test_first_edit_after_completion_resets_waypoint_states() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.start()
    route.mark_current_reached()
    assert route.status is RouteStatus.COMPLETED

    assert route.clear() is True
    assert route.status is RouteStatus.PLANNING
    assert route.waypoints == ()


def test_set_message_records_a_refusal_text() -> None:
    route = Route()
    revision = route.revision

    route.set_message("stop the route first")
    assert route.message == "stop the route first"
    assert route.revision == revision + 1

    route.set_message(None)
    assert route.message is None
    assert route.revision == revision + 2


def test_revision_increments_on_every_change() -> None:
    route = Route()
    assert route.revision == 0

    route.add_waypoint(1.0, 0.0, 0.0)
    assert route.revision == 1
    route.add_waypoint(2.0, 0.0, 0.0)
    assert route.revision == 2
    route.set_add_mode(True)
    assert route.revision == 3
    route.remove_waypoint(2)
    assert route.revision == 4
    route.add_waypoint(2.0, 0.0, 0.0)
    assert route.revision == 5
    route.start()
    assert route.revision == 6
    route.mark_current_reached()
    assert route.revision == 7
    route.hold()
    assert route.revision == 8
    route.start()
    assert route.revision == 9
    route.interrupt()
    assert route.revision == 10
    route.remove_waypoint(99)
    assert route.revision == 11
    route.clear()
    assert route.revision == 12
    route.add_waypoint(1.0, 0.0, 0.0)
    assert route.revision == 13
    route.start()
    assert route.revision == 14
    route.fail_current()
    assert route.revision == 15
    route.stop()
    assert route.revision == 15


def test_mark_current_reached_while_held_advances_without_completing() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)
    route.start()
    route.hold()

    assert route.mark_current_reached() is True
    assert route.status is RouteStatus.HELD
    assert [wp.state for wp in route.waypoints] == [WaypointState.REACHED, WaypointState.CURRENT]
    assert route.current_index == 2


def test_complete_ends_the_route_running_or_held() -> None:
    for end in (RouteStatus.RUNNING, RouteStatus.HELD):
        route = Route()
        route.add_waypoint(1.0, 0.0, 0.0)
        route.add_waypoint(2.0, 0.0, 0.0)
        route.start()
        if end is RouteStatus.HELD:
            route.hold()

        assert route.complete() is True
        assert route.status is RouteStatus.COMPLETED
        assert route.current_index is None
        assert route.waypoints[0].state is WaypointState.REACHED
        assert route.waypoints[1].state is WaypointState.PENDING


def test_interrupt_ends_a_held_route() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.start()
    route.hold()

    assert route.interrupt() is True
    assert route.status is RouteStatus.INTERRUPTED
