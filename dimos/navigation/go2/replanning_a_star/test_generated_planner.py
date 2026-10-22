# Copyright 2025-2026 Dimensional Inc.
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

from contextlib import ExitStack
from copy import deepcopy
import math
from threading import Event

from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest

from dimos.core.transport import LCMTransport
from dimos.message_utils.geometry import quaternion_from_euler, yaw
from dimos.message_utils.occupancy import occupancy_cell_value, occupancy_from_array
from dimos.message_utils.time import time_from_nanoseconds
from dimos.messages.geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
    TwistWithCovariance,
)
from dimos.messages.nav_msgs.msg import Odometry
from dimos.messages.std_msgs.msg import Bool, Header
from dimos.navigation.go2.replanning_a_star.controllers import PController, PdController, _velocity
from dimos.navigation.go2.replanning_a_star.module import ReplanningAStarPlanner
from dimos.navigation.go2.replanning_a_star.position_tracker import PositionTracker
from dimos.navigation.go2.replanning_a_star.replan_limiter import ReplanLimiter
from dimos.protocol.rpc.pubsubrpc import LCMRPC


class MemoryRPC(LCMRPC):
    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


def pose_at(x=0.0, y=0.0, angle=0.0, stamp=1700000000123456789):
    return PoseStamped(
        header=Header(frame_id="world", stamp=time_from_nanoseconds(stamp)),
        pose=Pose(position=Point(x=x, y=y, z=0), orientation=quaternion_from_euler(0, 0, angle)),
    )


def odometry_at(pose):
    return Odometry(
        header=deepcopy(pose.header),
        child_frame_id="base_link",
        pose=PoseWithCovariance(pose=deepcopy(pose.pose), covariance=np.zeros(36)),
        twist=TwistWithCovariance(twist=_velocity(), covariance=np.zeros(36)),
    )


@pytest.mark.parametrize(
    "target,expected",
    [
        ((1.0, 0.0), (0.55, 0.0)),
        ((0.0, 0.0), (0.0, 0.0)),
        ((math.cos(2), math.sin(2)), (0.0, 0.55)),
    ],
)
def test_generated_p_controller_retains_advance_and_rotate_then_drive_commands(target, expected):
    controller = PController(False, 0.55, 10)
    command = controller.advance(np.array(target), pose_at())
    assert (command.linear.x, command.angular.z) == expected
    assert (command.linear.y, command.linear.z, command.angular.x, command.angular.y) == (
        0,
        0,
        0,
        0,
    )
    assert decode(encode(command), type(command)) == command


def test_generated_pd_controller_retains_derivative_and_acceleration_state():
    controller = PdController(False, 0.55, 10)
    assert controller.rotate(1).angular.z == -0.2
    assert controller.rotate(1).angular.z == 0
    controller.reset_errors()
    controller.reset_yaw_error(1)
    assert controller.rotate(1).angular.z == 0.2


def test_generated_position_tracker_uses_existing_wall_clock_window(monkeypatch):
    now = [0.0]
    monkeypatch.setattr(
        "dimos.navigation.go2.replanning_a_star.position_tracker.time.time", lambda: now[0]
    )
    tracker = PositionTracker(2, 0.4)
    assert not tracker.is_stuck()
    tracker.add_position(pose_at(stamp=-1))
    assert tracker.is_stuck()
    tracker.add_position(pose_at(x=1, stamp=0))
    assert not tracker.is_stuck()
    now[0] = 3.0
    assert not tracker.is_stuck()
    tracker.reset_data()
    tracker.add_position(pose_at())
    assert tracker.is_stuck()


def test_generated_replanning_attempts_reset_only_after_existing_distance_limit():
    limiter = ReplanLimiter()
    for _ in range(6):
        assert limiter.can_retry(Point(x=0, y=0, z=0))
        limiter.will_retry()
    assert not limiter.can_retry(Point(x=1, y=0, z=0))
    assert limiter.can_retry(Point(x=2, y=0, z=0))
    assert limiter.get_attempt() == 0


@pytest.mark.parametrize(
    "position,expected", [((0, 0), 0), ((1, 0), 100), ((2, 0), -1), ((3, 0), -1), ((-1, 0), -1)]
)
def test_generated_world_cell_lookup_preserves_unknown_outside_map(position, expected):
    grid = occupancy_from_array(
        np.array([[0, 100, -1]], dtype=np.int8),
        header=Header(frame_id="world", stamp=time_from_nanoseconds(0)),
        resolution=1,
    )
    assert occupancy_cell_value(grid, Point(x=position[0], y=position[1], z=0)) == expected


@pytest.fixture
def planner_module():
    module = ReplanningAStarPlanner(rpc_transport=MemoryRPC)
    yield module
    module.stop()


@pytest.mark.parametrize("stamp", [0, -1, 1700000000123456789])
def test_odometry_conversion_retains_owned_header_and_nested_pose(planner_module, stamp):
    source = odometry_at(pose_at(x=1, y=2, stamp=stamp))
    planner_module._handle_odometry(source)
    result = planner_module._planner._current_odom
    assert result.header == source.header and result.pose == source.pose.pose
    source.header.frame_id = "changed"
    source.pose.pose.position.x = 100
    assert result.header.frame_id == "world" and result.pose.position.x == 1


@pytest.mark.parametrize("stamp", [0, -1, 1700000000123456789])
def test_clicked_point_conversion_retains_owned_header_and_identity_orientation(
    planner_module, stamp
):
    source = PointStamped(
        header=Header(frame_id="world", stamp=time_from_nanoseconds(stamp)),
        point=Point(x=1, y=2, z=0),
    )
    planner_module._handle_clicked_point(source)
    result = planner_module._planner._current_goal
    assert result.header == source.header and result.pose.position == source.point
    assert result.pose.orientation == Quaternion(x=0, y=0, z=0, w=1)
    source.header.frame_id = "changed"
    source.point.x = 100
    assert result.header.frame_id == "world" and result.pose.position.x == 1


def test_actual_planner_module_emits_path_velocity_and_stop_over_cdr_memory_ports():
    with ExitStack() as cleanup:
        module = ReplanningAStarPlanner(rpc_transport=MemoryRPC)
        wires = {}
        for name in [
            "odom",
            "odometry",
            "global_costmap",
            "goal_request",
            "clicked_point",
            "target",
            "stop_movement",
            "goal_reached",
            "navigation_state",
            "nav_cmd_vel",
            "path",
            "navigation_costmap",
        ]:
            port = getattr(module, name)
            wire = LCMTransport("planner_" + name, port.type, url="memq://")
            port.transport = wire
            wires[name] = wire
            cleanup.callback(wire.stop)
        threads = []

        def shutdown():
            current = module._planner._local_planner._thread
            if current is not None:
                threads.append(current)
            module.stop()
            for thread in threads:
                thread.join(5)
                assert not thread.is_alive()

        cleanup.callback(shutdown)
        paths = []
        commands = []
        path_ready = Event()
        velocity_ready = Event()
        motion_ready = Event()
        stopped = Event()

        def receive_path(value):
            paths.append(value)
            if value.poses:
                path_ready.set()

        def receive_command(value):
            commands.append(value)
            if value.linear.x != 0 or value.angular.z != 0:
                motion_ready.set()
            if value.linear.x > 0:
                velocity_ready.set()
            elif velocity_ready.is_set() and value == _velocity():
                stopped.set()

        cleanup.callback(wires["path"].subscribe(receive_path))
        cleanup.callback(wires["nav_cmd_vel"].subscribe(receive_command))
        module.start()
        pose = pose_at(x=0.125, y=0.125)
        costmap = occupancy_from_array(
            np.zeros((20, 20), dtype=np.int8), header=deepcopy(pose.header), resolution=0.25
        )
        map_ready = Event()
        odom_ready = Event()
        cleanup.callback(wires["global_costmap"].subscribe(lambda _: map_ready.set()))
        cleanup.callback(wires["odometry"].subscribe(lambda _: odom_ready.set()))
        wires["global_costmap"].publish(costmap)
        wires["odometry"].publish(odometry_at(pose))
        assert map_ready.wait(5) and odom_ready.wait(5)
        wires["goal_request"].publish(pose_at(x=1.5, y=0.125))
        assert path_ready.wait(5)
        assert motion_ready.wait(5)
        route = next(p for p in paths if p.poses)
        # Supply the same orientation feedback a moving robot would publish.
        wires["odometry"].publish(
            odometry_at(
                pose_at(
                    x=0.125,
                    y=0.125,
                    angle=yaw(route.poses[0].pose.orientation),
                    stamp=1700000000123456790,
                )
            )
        )
        assert velocity_ready.wait(5)
        assert route.header == costmap.header
        assert all(p.header == costmap.header for p in route.poses)
        assert commands[-1].linear.x > 0
        current = module._planner._local_planner._thread
        if current is not None:
            threads.append(current)
        wires["stop_movement"].publish(Bool(data=True))
        assert stopped.wait(5)
        assert module._planner._current_goal is None
