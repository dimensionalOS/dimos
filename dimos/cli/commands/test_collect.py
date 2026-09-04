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
from textual.widgets import Button, Static

from dimos.cli.commands.collect import TeachCollectionApp, TeachCollectionSession
from dimos.core.introspection.module.info import ModuleInfo, RpcInfo
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus


def _status(
    state: str = "idle",
    *,
    event: str = "init",
    saved: int = 0,
    discarded: int = 0,
) -> EpisodeStatus:
    return EpisodeStatus(
        ts=1.0,
        state=state,  # type: ignore[arg-type]
        episodes_saved=saved,
        episodes_discarded=discarded,
        last_event=event,  # type: ignore[arg-type]
        task_label="pick up the block",
    )


def _session(mocker: MockerFixture) -> tuple[TeachCollectionSession, Any, Any]:
    client = mocker.Mock()
    monitor = mocker.Mock()
    monitor.get_status.return_value = _status()
    monitor.command.side_effect = [
        _status("recording", event="start"),
        _status("idle", event="discard", discarded=1),
    ]
    return TeachCollectionSession(client, monitor), client, monitor


def test_session_routes_episode_commands(mocker: MockerFixture) -> None:
    session, _, monitor = _session(mocker)

    assert session.command("toggle").state == "recording"

    monitor.command.assert_called_once_with("toggle")


def test_panel_actions_route_keys_and_guard_quit(mocker: MockerFixture) -> None:
    session, _, monitor = _session(mocker)
    app = TeachCollectionApp(session)
    mocker.patch.object(app, "_refresh")
    exit_mock = mocker.patch.object(app, "exit")

    app.action_toggle_recording()
    app.action_quit()
    app.action_discard()
    app.action_quit()

    assert monitor.command.call_args_list == [mocker.call("toggle"), mocker.call("discard")]
    exit_mock.assert_called_once_with()


def test_panel_binds_the_documented_keys() -> None:
    assert {binding.key: binding.action for binding in TeachCollectionApp.BINDINGS} == {
        "space": "toggle_recording",
        "d": "discard",
        "q": "quit",
        "ctrl+c": "quit",
    }


async def test_dashboard_buttons_follow_episode_state(mocker: MockerFixture) -> None:
    session, _, monitor = _session(mocker)
    app = TeachCollectionApp(session)
    mocker.patch.object(app, "set_interval")

    async with app.run_test(size=(80, 24)) as pilot:
        assert str(app.query_one("#state", Static).render()) == "READY"
        assert str(app.query_one("#toggle", Button).label) == "Start recording"
        assert app.query_one("#discard", Button).disabled

        await pilot.click("#toggle")

        assert "RECORDING" in str(app.query_one("#state", Static).render())
        assert str(app.query_one("#toggle", Button).label) == "Save episode"
        assert not app.query_one("#discard", Button).disabled
        assert app.query_one("#detach", Button).disabled

        await pilot.click("#discard")

        assert str(app.query_one("#state", Static).render()) == "READY"
        assert str(app.query_one("#discarded", Static).render()) == "DISCARDED\n1"
        assert not app.query_one("#detach", Button).disabled
        assert monitor.command.call_args_list == [mocker.call("toggle"), mocker.call("discard")]


def test_dashboard_formats_recording_time() -> None:
    assert TeachCollectionApp._format_elapsed(0.0) == "00:00.0"
    assert TeachCollectionApp._format_elapsed(62.34) == "01:02.3"


async def test_rpc_failure_disables_controls_without_stopping_daemon(
    mocker: MockerFixture,
) -> None:
    session, client, monitor = _session(mocker)
    app = TeachCollectionApp(session)
    mocker.patch.object(app, "set_interval")
    monitor.command.side_effect = RuntimeError("stack disappeared")

    async with app.run_test(size=(80, 24)) as pilot:
        await pilot.click("#toggle")

        assert str(app.query_one("#state", Static).render()) == "DISCONNECTED"
        assert app.query_one("#toggle", Button).disabled
        assert app.query_one("#discard", Button).disabled
        assert not app.query_one("#detach", Button).disabled
        assert str(app.query_one("#detach", Button).label) == "Exit"

    # Closing the failed dashboard must only detach this RPC client.
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
