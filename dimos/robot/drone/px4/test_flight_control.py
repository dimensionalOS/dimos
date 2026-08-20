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

import asyncio
from collections.abc import AsyncIterator, Iterator
import math
from threading import Event
from types import SimpleNamespace
from typing import Any
from unittest.mock import AsyncMock, MagicMock

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.drone.px4 import flight_control
from dimos.robot.drone.px4.flight_control import FlightController


def _mock_system() -> MagicMock:
    return MagicMock(
        action=MagicMock(
            arm=AsyncMock(),
            disarm=AsyncMock(),
            set_takeoff_altitude=AsyncMock(),
            takeoff=AsyncMock(),
            land=AsyncMock(),
            hold=AsyncMock(),
        ),
        offboard=MagicMock(
            set_velocity_body=AsyncMock(),
            set_position_ned=AsyncMock(),
            start=AsyncMock(),
            stop=AsyncMock(),
        ),
        mocap=MagicMock(set_vision_position_estimate=AsyncMock()),
        telemetry=MagicMock(),
    )


@pytest.fixture(scope="module")
def flight_controller() -> Iterator[FlightController]:
    module = FlightController()
    yield module
    module.__dict__.pop("_system", None)
    module.stop()


@pytest.fixture
def controller(flight_controller: FlightController) -> Iterator[FlightController]:
    _run(flight_controller, flight_controller._cancel_move_watchdog())
    flight_controller.config.move_watchdog_timeout_s = 0.5
    flight_controller._system = _mock_system()
    flight_controller._connected = False
    flight_controller._armed = None
    flight_controller._in_air = None
    flight_controller._flight_mode = None
    flight_controller._offboard_active = False
    flight_controller._position_ned = None
    flight_controller._yaw_deg = None
    flight_controller._telemetry_tasks.clear()
    yield flight_controller
    _run(flight_controller, flight_controller._cancel_move_watchdog())
    for task in flight_controller._telemetry_tasks:
        if not task.done():
            task.cancel()
    flight_controller._telemetry_tasks.clear()
    flight_controller.__dict__.pop("_system", None)


def _mavsdk_error(error_type: type[Exception]) -> Exception:
    result = SimpleNamespace(result="DENIED", result_str="denied")
    return error_type(result, "test")


def _odometry() -> Odometry:
    message = Odometry(ts=12.3456789, frame_id="odom", child_frame_id="mid360_link")
    message.pose.position.x, message.pose.position.y, message.pose.position.z = (3.0, -2.0, 1.5)
    message.pose.orientation.w = 1.0
    message.pose.covariance = np.eye(6).reshape(-1)
    return message


def _run(controller: FlightController, coroutine: Any) -> Any:
    assert controller._loop is not None
    return asyncio.run_coroutine_threadsafe(coroutine, controller._loop).result()


def test_pointlio_odometry_builds_mavsdk_vision_estimate() -> None:
    estimate = flight_control._build_vision_position_estimate(_odometry())

    assert estimate.time_usec == 12_345_679
    assert (
        estimate.position_body.x_m,
        estimate.position_body.y_m,
        estimate.position_body.z_m,
    ) == pytest.approx((2.973909, 2.02329, -1.443681), abs=1e-6)
    assert len(estimate.pose_covariance.covariance_matrix) == 21


def test_invalid_vision_covariance_uses_unknown_sentinel() -> None:
    covariance = flight_control._transform_pose_covariance_to_frd((1.0,), (1.0, 0.0, 0.0, 0.0))

    assert len(covariance) == 1
    assert math.isnan(covariance[0])


def test_external_vision_rejects_invalid_samples() -> None:
    message = _odometry()
    message.frame_id = "map"
    with pytest.raises(
        flight_control._InvalidExternalVisionSampleError, match="odom -> mid360_link"
    ):
        flight_control._build_vision_position_estimate(message)

    message = _odometry()
    message.ts = 0.0
    with pytest.raises(flight_control._InvalidExternalVisionSampleError, match="timestamp"):
        flight_control._build_vision_position_estimate(message)

    message = _odometry()
    message.pose.orientation.w = 2.0
    with pytest.raises(
        flight_control._InvalidExternalVisionSampleError, match="orientation quaternion"
    ):
        flight_control._build_vision_position_estimate(message)

    message = _odometry()
    message.pose.position.x = math.inf
    with pytest.raises(
        flight_control._InvalidExternalVisionSampleError, match="position must be finite"
    ):
        flight_control._build_vision_position_estimate(message)


@pytest.mark.parametrize(
    ("method", "system_method", "success"),
    (
        ("arm", "arm", "arm command sent"),
        ("disarm", "disarm", "disarm command sent"),
        ("land", "land", "land command sent"),
        ("hold", "hold", "hold mode entered"),
    ),
)
def test_action_skills_report_success(
    controller: FlightController, method: str, system_method: str, success: str
) -> None:
    action = getattr(controller._system.action, system_method)
    action.return_value = None

    result = getattr(controller, method)()

    assert result == success
    action.assert_awaited_once_with()


def test_action_skill_reports_mavsdk_error(controller: FlightController) -> None:
    controller._system.action.arm.side_effect = _mavsdk_error(flight_control.ActionError)

    result = controller.arm()

    assert result.startswith("arm failed: DENIED")


@pytest.mark.parametrize("method", ("land", "hold"))
def test_action_mode_changes_leave_move_watchdog_running(
    controller: FlightController, method: str
) -> None:
    assert controller.move(forward=1.0) == "move command sent"
    watchdog_task = controller._move_watchdog_task
    assert watchdog_task is not None

    getattr(controller, method)()

    assert controller._move_watchdog_task is watchdog_task
    assert not watchdog_task.done()


def test_takeoff_validates_altitude_and_sends_both_commands(
    controller: FlightController,
) -> None:
    assert controller.takeoff(math.nan) == (
        "takeoff failed: altitude must be a finite positive number"
    )

    result = controller.takeoff(4.5)

    assert result == "takeoff command sent for 4.5 m"
    controller._system.action.set_takeoff_altitude.assert_awaited_once_with(4.5)
    controller._system.action.takeoff.assert_awaited_once_with()


def test_move_converts_flu_velocity_to_mavsdk_frd(controller: FlightController) -> None:
    result = controller.move(forward=1.0, left=2.0, up=3.0, yaw_rate=math.pi / 2)

    assert result == "move command sent"
    velocity = controller._system.offboard.set_velocity_body.await_args.args[0]
    assert (
        velocity.forward_m_s,
        velocity.right_m_s,
        velocity.down_m_s,
        velocity.yawspeed_deg_s,
    ) == pytest.approx((1.0, -2.0, -3.0, -90.0))
    assert controller._move_watchdog_task is not None


def test_move_replaces_existing_watchdog(controller: FlightController) -> None:
    first_result = controller.move(forward=1.0)
    first_task = controller._move_watchdog_task
    assert first_task is not None

    second_result = controller.move(left=1.0)

    assert first_result == second_result == "move command sent"
    assert first_task.cancelled()
    assert controller._move_watchdog_task is not None
    assert controller._move_watchdog_task is not first_task


def test_move_watchdog_stops_velocity(controller: FlightController, mocker: MockerFixture) -> None:
    sleep_started = Event()

    async def make_release_event() -> asyncio.Event:
        return asyncio.Event()

    release_watchdog = _run(controller, make_release_event())

    async def expire_watchdog(timeout: float) -> None:
        assert timeout == 0.5
        sleep_started.set()
        await release_watchdog.wait()

    mocker.patch.object(flight_control.asyncio, "sleep", side_effect=expire_watchdog)

    result = controller.move(forward=1.0)
    assert sleep_started.wait(timeout=1.0)

    async def wait_for_watchdog() -> None:
        task = controller._move_watchdog_task
        assert task is not None
        release_watchdog.set()
        await task

    _run(controller, wait_for_watchdog())

    assert result == "move command sent"
    velocities = controller._system.offboard.set_velocity_body.await_args_list
    assert len(velocities) == 2
    stopped = velocities[-1].args[0]
    assert (
        stopped.forward_m_s,
        stopped.right_m_s,
        stopped.down_m_s,
        stopped.yawspeed_deg_s,
    ) == (0.0, 0.0, 0.0, 0.0)
    assert controller._move_watchdog_task is None


def test_move_rejects_non_finite_input(controller: FlightController) -> None:
    result = controller.move(forward=math.inf)

    assert result == "move failed: frame conversion inputs must be finite"
    controller._system.offboard.set_velocity_body.assert_not_awaited()


def test_move_rejects_invalid_watchdog_timeout(controller: FlightController) -> None:
    controller.config.move_watchdog_timeout_s = 0.0

    result = controller.move(forward=1.0)

    assert result == "move failed: watchdog timeout must be a finite positive number"
    controller._system.offboard.set_velocity_body.assert_not_awaited()


def test_goto_validates_and_sends_ned_target(controller: FlightController) -> None:
    assert controller.goto(math.nan, 0.0, 0.0) == ("goto failed: position and yaw must be finite")

    result = controller.goto(1.0, 2.0, -3.0, 45.0)

    assert result == "goto command sent"
    target = controller._system.offboard.set_position_ned.await_args.args[0]
    assert (target.north_m, target.east_m, target.down_m, target.yaw_deg) == (
        1.0,
        2.0,
        -3.0,
        45.0,
    )


def test_offboard_and_hover_follow_latest_telemetry(controller: FlightController) -> None:
    assert controller.hover() == ("hover failed: local NED position and yaw are unavailable")

    assert controller.move(forward=1.0) == "move command sent"
    enter_watchdog = controller._move_watchdog_task
    assert enter_watchdog is not None
    assert controller.enter_offboard() == "offboard mode entered"
    assert enter_watchdog.cancelled()
    assert controller._offboard_active is True
    controller._position_ned = (4.0, 5.0, -6.0)
    controller._yaw_deg = 30.0
    assert controller.hover() == "hover command sent"
    assert controller.move(left=1.0) == "move command sent"
    exit_watchdog = controller._move_watchdog_task
    assert exit_watchdog is not None
    assert controller.exit_offboard() == "offboard mode exited"
    assert exit_watchdog.cancelled()
    assert controller._offboard_active is False

    targets = controller._system.offboard.set_position_ned.await_args_list
    target = targets[-1].args[0]
    assert (target.north_m, target.east_m, target.down_m, target.yaw_deg) == (
        4.0,
        5.0,
        -6.0,
        30.0,
    )


def test_stream_handlers_forward_commands_and_reject_bad_odometry(
    controller: FlightController, mocker: MockerFixture
) -> None:
    assert controller.move(forward=0.5) == "move command sent"
    watchdog_task = controller._move_watchdog_task
    assert watchdog_task is not None

    twist = Twist()
    twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z = (1.0, 2.0, 3.0, 0.5)
    _run(controller, controller.handle_cmd_vel(twist))
    assert watchdog_task.cancelled()
    assert controller._move_watchdog_task is None
    assert controller._system.offboard.set_velocity_body.await_count == 2

    invalid_odometry = mocker.Mock(frame_id="map", child_frame_id="mid360_link")
    _run(controller, controller.handle_odometry(invalid_odometry))
    controller._system.mocap.set_vision_position_estimate.assert_not_awaited()


async def _values(*items: Any) -> AsyncIterator[Any]:
    for item in items:
        yield item


def test_telemetry_watchers_update_status(controller: FlightController) -> None:
    controller._system.telemetry.armed.return_value = _values(True)
    controller._system.telemetry.in_air.return_value = _values(False)
    controller._system.telemetry.flight_mode.return_value = _values(
        None, SimpleNamespace(name="OFFBOARD")
    )
    controller._system.telemetry.position_velocity_ned.return_value = _values(
        SimpleNamespace(position=SimpleNamespace(north_m=1.0, east_m=2.0, down_m=-3.0))
    )
    controller._system.telemetry.attitude_euler.return_value = _values(
        SimpleNamespace(yaw_deg=25.0)
    )

    async def watch_all() -> None:
        await asyncio.gather(
            controller._watch_armed(),
            controller._watch_in_air(),
            controller._watch_flight_mode(),
            controller._watch_position_velocity_ned(),
            controller._watch_attitude_euler(),
        )

    _run(
        controller,
        watch_all(),
    )

    assert controller.get_status() == {
        "connected": False,
        "armed": True,
        "in_air": False,
        "flight_mode": "OFFBOARD",
    }
    assert controller._offboard_active is True
    assert controller._position_ned == (1.0, 2.0, -3.0)
    assert controller._yaw_deg == 25.0


def test_start_mavsdk_reports_connection_timeout(
    controller: FlightController, mocker: MockerFixture
) -> None:
    system = mocker.Mock()
    system.connect = mocker.AsyncMock()
    mocker.patch.object(flight_control, "System", return_value=system)
    mocker.patch.object(
        controller,
        "_wait_until_connected",
        new_callable=mocker.AsyncMock,
        side_effect=TimeoutError,
    )

    with pytest.raises(flight_control.MavsdkConnectionTimeoutError) as captured:
        _run(controller, controller._start_mavsdk())

    assert str(captured.value) == (
        "MAVSDK did not connect to serial:///dev/ttyTHS3:921600 within 10 seconds"
    )
    system.connect.assert_awaited_once_with(system_address="serial:///dev/ttyTHS3:921600")


def test_stop_mavsdk_cancels_telemetry_and_resets_state(
    controller: FlightController,
) -> None:
    system = controller._system

    async def create_task() -> asyncio.Task[None]:
        return asyncio.create_task(asyncio.sleep(60))

    task = _run(controller, create_task())
    controller._telemetry_tasks = [task]
    assert controller.move(forward=1.0) == "move command sent"
    watchdog_task = controller._move_watchdog_task
    assert watchdog_task is not None
    controller._connected = True
    controller._offboard_active = True
    controller._armed = True
    controller._in_air = True
    controller._flight_mode = "OFFBOARD"

    _run(controller, controller._stop_mavsdk())

    assert task.cancelled()
    assert watchdog_task.cancelled()
    system.offboard.stop.assert_awaited_once_with()
    assert controller.get_status() == {
        "connected": False,
        "armed": None,
        "in_air": None,
        "flight_mode": None,
    }
    assert not hasattr(controller, "_system")
