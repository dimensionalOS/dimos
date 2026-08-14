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
from types import SimpleNamespace
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.drone.px4 import flight_control
from dimos.robot.drone.px4.flight_control import FlightController


@pytest.fixture
def controller(mocker: MockerFixture) -> Iterator[FlightController]:
    module = FlightController()
    module._system = mocker.Mock(
        action=mocker.Mock(
            arm=mocker.AsyncMock(),
            disarm=mocker.AsyncMock(),
            set_takeoff_altitude=mocker.AsyncMock(),
            takeoff=mocker.AsyncMock(),
            land=mocker.AsyncMock(),
            hold=mocker.AsyncMock(),
        ),
        offboard=mocker.Mock(
            set_velocity_body=mocker.AsyncMock(),
            set_position_ned=mocker.AsyncMock(),
            start=mocker.AsyncMock(),
            stop=mocker.AsyncMock(),
        ),
        mocap=mocker.Mock(set_vision_position_estimate=mocker.AsyncMock()),
        telemetry=mocker.Mock(),
    )
    yield module
    module.__dict__.pop("_system", None)
    module.stop()


def _mavsdk_error(error_type: type[Exception]) -> Exception:
    result = SimpleNamespace(result="DENIED", result_str="denied")
    return error_type(result, "test")


def _run(controller: FlightController, coroutine: Any) -> Any:
    assert controller._loop is not None
    return asyncio.run_coroutine_threadsafe(coroutine, controller._loop).result()


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


def test_move_rejects_non_finite_input(controller: FlightController) -> None:
    result = controller.move(forward=math.inf)

    assert result == "move failed: frame conversion inputs must be finite"
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

    assert controller.enter_offboard() == "offboard mode entered"
    assert controller._offboard_active is True
    controller._position_ned = (4.0, 5.0, -6.0)
    controller._yaw_deg = 30.0
    assert controller.hover() == "hover command sent"
    assert controller.exit_offboard() == "offboard mode exited"
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
    twist = Twist()
    twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z = (1.0, 2.0, 3.0, 0.5)
    _run(controller, controller.handle_cmd_vel(twist))
    controller._system.offboard.set_velocity_body.assert_awaited_once()

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
    controller._connected = True
    controller._offboard_active = True
    controller._armed = True
    controller._in_air = True
    controller._flight_mode = "OFFBOARD"

    _run(controller, controller._stop_mavsdk())

    assert task.cancelled()
    system.offboard.stop.assert_awaited_once_with()
    assert controller.get_status() == {
        "connected": False,
        "armed": None,
        "in_air": None,
        "flight_mode": None,
    }
    assert not hasattr(controller, "_system")
