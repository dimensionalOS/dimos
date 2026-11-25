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

"""What each console request asks of the coordinator and the homing module."""

from collections.abc import Iterator
from unittest.mock import Mock

from fastapi.testclient import TestClient
import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.manipulation.arm_home import ArmHomeModule
from dimos.teleop.console.module import CONSOLE_PATH, OperatorConsoleModule
from dimos.web.robot_web_interface import RobotWebInterface


@pytest.fixture
def coordinator() -> Mock:
    return Mock(spec=ControlCoordinator)


@pytest.fixture
def arm_home() -> Mock:
    arm_home = Mock(spec=ArmHomeModule)
    arm_home.get_homes.return_value = {"left": {"home": [0.1], "saved": False}}
    return arm_home


@pytest.fixture
def console(coordinator: Mock, arm_home: Mock) -> Iterator[OperatorConsoleModule]:
    instance = OperatorConsoleModule()
    instance.coordinator = coordinator
    instance.arm_home = arm_home
    yield instance
    instance.stop()


@pytest.fixture
def client(console: OperatorConsoleModule) -> TestClient:
    server = RobotWebInterface(host="127.0.0.1", port=0)
    console._routes(server)
    return TestClient(server.app)


def test_the_stop_reaches_the_coordinator_before_moves_are_cancelled(
    coordinator: Mock, arm_home: Mock, client: TestClient
) -> None:
    order = Mock()
    order.attach_mock(coordinator.set_estop, "set_estop")
    order.attach_mock(arm_home.cancel, "cancel")

    assert client.post("/api/estop", json={"estopped": True}).json() == {"estopped": True}

    assert [call[0] for call in order.mock_calls] == ["set_estop", "cancel"]
    coordinator.set_estop.assert_called_once_with(True)
    assert client.get("/api/state").json() == {
        "estopped": True,
        "arms": {"left": {"home": [0.1], "saved": False}},
    }


def test_clearing_the_stop_cancels_nothing(
    coordinator: Mock, arm_home: Mock, client: TestClient
) -> None:
    client.post("/api/estop", json={"estopped": False})

    coordinator.set_estop.assert_called_once_with(False)
    arm_home.cancel.assert_not_called()


def test_a_manual_move_names_its_arm_and_target(arm_home: Mock, client: TestClient) -> None:
    arm_home.manual_move.return_value = "started: 4.0 s"

    reply = client.post("/api/move", json={"arm": "left", "target": "rest"})

    assert reply.json() == {"result": "started: 4.0 s"}
    arm_home.manual_move.assert_called_once_with("left", "rest")


def test_a_refused_request_comes_back_as_a_readable_error(
    arm_home: Mock, client: TestClient
) -> None:
    arm_home.save_home.side_effect = RuntimeError("no measured position for ['j1']")

    reply = client.post("/api/home/save", json={"arm": "left"})

    assert reply.status_code == 400
    assert reply.json() == {"detail": "no measured position for ['j1']"}


def test_a_move_to_an_unknown_target_is_rejected_before_it_reaches_the_arm(
    arm_home: Mock, client: TestClient
) -> None:
    assert client.post("/api/move", json={"arm": "left", "target": "zero"}).status_code == 422
    arm_home.manual_move.assert_not_called()


def test_the_console_page_is_served_at_its_own_path(client: TestClient) -> None:
    """The web server base class owns "/", so the console must not rely on it."""
    reply = client.get(CONSOLE_PATH)

    assert reply.status_code == 200
    assert "DimOS Operator Console" in reply.text
    assert "DimOS Operator Console" not in client.get("/").text
