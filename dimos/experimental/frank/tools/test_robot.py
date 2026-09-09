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

"""Frank CLI regressions; never connect to a robot."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def robot(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location(
        "frank_robot_test", Path(__file__).with_name("robot.py")
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "MOTION_OFF", tmp_path / "MOTION_OFF")
    return module


@pytest.fixture
def connection(robot, mocker):
    conn = mocker.Mock()
    conn.move.return_value = True
    mocker.patch.object(robot, "_dimos", return_value=SimpleNamespace(GO2Connection=conn))
    mocker.patch.object(robot, "stop")
    mocker.patch.object(robot.time, "sleep")
    return conn


def test_invented_sport_turn_fails_without_dispatch(robot, mocker, capsys):
    post = mocker.patch.object(robot.requests, "post")
    adapter = mocker.patch.object(robot, "McpAdapter")
    assert robot._main(["execute_sport_command", '{"command_name":"turn"}']) == 1
    assert "robot.py turn -20" in capsys.readouterr().out
    post.assert_not_called()
    adapter.assert_not_called()


def test_right_turn_uses_negative_yaw_and_stops(robot, connection, mocker):
    mocker.patch.object(robot, "pose", side_effect=[(0, 0, 10), (0, 0, -10), (0, 0, -10)])
    assert "turned -20 deg" in robot.turn(-20)
    robot.stop.assert_called_once_with()
    assert connection.move.call_args_list[0].args[0].angular.z < 0
    assert connection.move.call_args_list[-1].args[0].angular.z == 0


def test_turn_stops_when_odometry_fails(robot, connection, mocker):
    mocker.patch.object(robot, "pose", side_effect=[(0, 0, 0), RuntimeError("odom lost")])
    with pytest.raises(RuntimeError, match="odom lost"):
        robot.turn(-20)
    assert connection.move.call_args_list[-1].args[0].angular.z == 0


def test_turn_timeout_is_failure(robot, connection, mocker):
    mocker.patch.object(robot, "pose", return_value=(0, 0, 0))
    with pytest.raises(RuntimeError, match="Turn incomplete"):
        robot.turn(-20, timeout=0)
    assert connection.move.call_args_list[-1].args[0].angular.z == 0


def test_turn_rejection_is_failure(robot, connection, mocker):
    mocker.patch.object(robot, "pose", return_value=(0, 0, 0))
    connection.move.return_value = False
    with pytest.raises(RuntimeError, match="rejected"):
        robot.turn(-20)
    assert connection.move.call_args_list[-1].args[0].angular.z == 0


def test_motion_off_prevents_turn(robot, connection):
    robot.MOTION_OFF.touch()
    with pytest.raises(SystemExit, match="movement is off"):
        robot.turn(-20)
    connection.move.assert_not_called()


def test_observe_creates_output_directory(robot, mocker, tmp_path):
    adapter = mocker.patch.object(robot, "McpAdapter")
    adapter.return_value.call_tool.return_value = {"content": [{"type": "image", "data": "YWJj"}]}
    out = tmp_path / "new" / "look.jpg"
    assert robot.observe(str(out)) == str(out)
    assert out.read_bytes() == b"abc"
