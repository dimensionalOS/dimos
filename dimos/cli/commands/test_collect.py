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

from typing import Any

from pytest_mock import MockerFixture

from dimos.cli.commands.collect import TeachCollectionApp, TeachCollectionSession
from dimos.core.introspection.module.info import ModuleInfo, RpcInfo
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus


def _status(state: str = "idle") -> EpisodeStatus:
    return EpisodeStatus(
        ts=1.0,
        state=state,  # type: ignore[arg-type]
        episodes_saved=0,
        episodes_discarded=0,
        task_label="pick up the block",
    )


def _session(mocker: MockerFixture) -> tuple[TeachCollectionSession, Any, Any, Any]:
    client = mocker.Mock()
    monitor = mocker.Mock()
    coordinator = mocker.Mock()
    monitor.get_status.return_value = _status()
    monitor.command.side_effect = [_status("recording"), _status("idle")]
    coordinator.task_invoke.return_value = True
    return TeachCollectionSession(client, monitor, coordinator), client, monitor, coordinator


def test_session_routes_episode_and_gripper_commands(mocker: MockerFixture) -> None:
    session, _, monitor, coordinator = _session(mocker)

    assert session.command("toggle").state == "recording"
    session.set_gripper(1.0)
    session.set_gripper(0.0)

    monitor.command.assert_called_once_with("toggle")
    assert coordinator.task_invoke.call_args_list == [
        mocker.call(
            "arm_gripper",
            "set_normalized",
            {"values": [1.0], "t_now": None},
        ),
        mocker.call(
            "arm_gripper",
            "set_normalized",
            {"values": [0.0], "t_now": None},
        ),
    ]
    assert session.gripper_target == 0.0


def test_panel_actions_route_keys_and_guard_quit(mocker: MockerFixture) -> None:
    session, _, monitor, coordinator = _session(mocker)
    app = TeachCollectionApp(session)
    mocker.patch.object(app, "_refresh")
    exit_mock = mocker.patch.object(app, "exit")

    app.action_toggle_recording()
    app.action_quit()
    app.action_open_gripper()
    app.action_close_gripper()
    app.action_discard()
    app.action_quit()

    assert monitor.command.call_args_list == [mocker.call("toggle"), mocker.call("discard")]
    assert coordinator.task_invoke.call_count == 2
    exit_mock.assert_called_once_with()


def test_panel_binds_the_documented_keys() -> None:
    assert {binding.key: binding.action for binding in TeachCollectionApp.BINDINGS} == {
        "space": "toggle_recording",
        "d": "discard",
        "o": "open_gripper",
        "c": "close_gripper",
        "q": "quit",
        "ctrl+c": "quit",
    }


def test_rpc_failure_detaches_without_stopping_the_daemon(mocker: MockerFixture) -> None:
    session, client, monitor, _ = _session(mocker)
    app = TeachCollectionApp(session)
    mocker.patch.object(app, "_refresh")
    monitor.command.side_effect = RuntimeError("stack disappeared")

    app.action_toggle_recording()

    assert app._detached is True
    client.stop.assert_called_once_with()


def test_connect_rejects_the_wrong_stack_and_closes_client(mocker: MockerFixture) -> None:
    client = mocker.Mock()
    client.list_modules.return_value = [
        ModuleInfo(
            name="EpisodeMonitorModule",
            instance_name="EpisodeMonitorModule",
            rpcs=[RpcInfo(name="command"), RpcInfo(name="get_status")],
        )
    ]
    mocker.patch("dimos.cli.commands.collect.Dimos.connect", return_value=client)

    try:
        TeachCollectionSession.connect()
    except RuntimeError as exc:
        assert "ControlCoordinator" in str(exc)
    else:
        raise AssertionError("wrong stack should fail validation")
    client.stop.assert_called_once_with()
