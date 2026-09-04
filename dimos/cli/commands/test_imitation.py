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

from pathlib import Path
from typing import Any

from pytest_mock import MockerFixture
from textual.widgets import Button, Static
from typer.testing import CliRunner

from dimos.cli.commands.imitation import (
    CollectionApp,
    CollectionSession,
    _default_dataset,
    _default_recording,
    imitation_app,
)
from dimos.constants import STATE_DIR
from dimos.imitation.workflows import get_collection_workflow
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.robot.manipulators.openyam.learning import OPENYAM_TEACH_IO


def _status(
    state: str = "idle", *, event: str = "init", saved: int = 0, discarded: int = 0
) -> EpisodeStatus:
    return EpisodeStatus(
        ts=1.0,
        state=state,  # type: ignore[arg-type]
        episodes_saved=saved,
        episodes_discarded=discarded,
        last_event=event,  # type: ignore[arg-type]
        task_label="pick up the block",
    )


def _session(mocker: MockerFixture) -> tuple[CollectionSession, Any, Any]:
    driver = mocker.Mock()
    monitor = mocker.Mock()
    driver.get_module.return_value = monitor
    monitor.get_status.return_value = _status()
    monitor.command.side_effect = [
        _status("recording", event="start"),
        _status("idle", event="discard", discarded=1),
    ]
    return CollectionSession(driver), driver, monitor


def test_cli_exposes_one_workflow_group_and_removes_old_commands() -> None:
    from dimos.cli.dimos import main

    result = CliRunner().invoke(main, ["--help"])

    assert result.exit_code == 0
    assert "imitation" in result.output
    assert "collect" not in result.output
    assert "dataprep" not in result.output


def test_imitation_help_exposes_the_complete_workflow() -> None:
    result = CliRunner().invoke(imitation_app, ["--help"])

    assert result.exit_code == 0
    for command in ("list", "collect", "prepare", "inspect", "train", "run"):
        assert command in result.output


def test_list_does_not_load_hardware_modules(mocker: MockerFixture) -> None:
    imported = mocker.patch("importlib.import_module")

    result = CliRunner().invoke(imitation_app, ["list"])

    assert result.exit_code == 0
    assert "openyam-teach" in result.output
    assert "openyam-quest" in result.output
    assert "dual-openyam-quest" in result.output
    assert "dual-openyam-abc" in result.output
    imported.assert_not_called()


def test_train_forwards_arguments_and_exit_code(mocker: MockerFixture) -> None:
    completed = mocker.Mock(returncode=17)
    run = mocker.patch("dimos.cli.commands.imitation.subprocess.run", return_value=completed)

    result = CliRunner().invoke(
        imitation_app,
        ["train", "--policy.type=act", "--dataset.repo_id=local/test"],
    )

    assert result.exit_code == 17
    command = run.call_args.args[0]
    assert command[-3:] == [
        "lerobot-train",
        "--policy.type=act",
        "--dataset.repo_id=local/test",
    ]
    assert run.call_args.kwargs == {"check": False}


def test_prepare_rejects_an_existing_output(tmp_path: Path) -> None:
    recording = tmp_path / "session.mcap"
    output = tmp_path / "dataset"
    output.mkdir()

    result = CliRunner().invoke(
        imitation_app,
        ["prepare", "openyam-teach", str(recording), "--output", str(output)],
    )

    assert result.exit_code == 2
    assert "already exists" in result.output


def test_default_artifacts_live_in_state_and_are_unique() -> None:
    workflow = get_collection_workflow("openyam-teach")

    first = _default_recording(workflow)
    second = _default_recording(workflow)

    assert first.parent == STATE_DIR / "recordings"
    assert first.suffix == ".mcap"
    assert first != second
    assert _default_dataset(first) == STATE_DIR / "datasets" / first.stem


def test_session_routes_commands_and_stops_owned_stack(mocker: MockerFixture) -> None:
    session, driver, monitor = _session(mocker)

    assert session.command("toggle").state == "recording"
    session.close()

    monitor.command.assert_called_once_with("toggle")
    driver.stop.assert_called_once_with()


def test_collect_stops_driver_when_stack_start_fails(
    tmp_path: Path,
    mocker: MockerFixture,
) -> None:
    workflow = mocker.Mock(name="workflow")
    workflow.name = "openyam-teach"
    workflow.dual_can = False
    workflow.load_builder.return_value = mocker.Mock(return_value="blueprint")
    workflow.load_profile.return_value = OPENYAM_TEACH_IO
    driver = mocker.Mock()
    driver.run.side_effect = RuntimeError("hardware failed")
    mocker.patch(
        "dimos.cli.commands.imitation.get_collection_workflow",
        return_value=workflow,
    )
    mocker.patch("dimos.cli.commands.imitation._require_idle_coordinator")
    mocker.patch("dimos.cli.commands.imitation.Dimos", return_value=driver)

    result = CliRunner().invoke(
        imitation_app,
        [
            "collect",
            "openyam-teach",
            "--task",
            "pick up block",
            "--recording",
            str(tmp_path / "session.mcap"),
            "--camera",
            "wrist_image=0",
        ],
    )

    assert result.exit_code == 1
    assert "hardware failed" in result.output
    driver.stop.assert_called_once_with()


def test_dual_collection_requires_both_can_interfaces(tmp_path: Path) -> None:
    result = CliRunner().invoke(
        imitation_app,
        [
            "collect",
            "dual-openyam-quest",
            "--task",
            "fold towel",
            "--recording",
            str(tmp_path / "dual.mcap"),
            "--camera",
            "left_wrist_image=0",
            "--camera",
            "right_wrist_image=1",
            "--left-can-port",
            "follower_l",
        ],
    )

    assert result.exit_code == 2
    assert "Dual OpenYAM requires both --left-can-port" in result.output
    assert "--right-can-port" in result.output


def test_collection_app_guards_normal_quit_while_recording(mocker: MockerFixture) -> None:
    session, _, monitor = _session(mocker)
    app = CollectionApp(session, "openyam-teach")
    mocker.patch.object(app, "_refresh")
    exit_mock = mocker.patch.object(app, "exit")

    app.action_toggle_recording()
    app.action_quit()
    app.action_discard()
    app.action_quit()
    app.action_quit()

    assert monitor.command.call_args_list == [mocker.call("toggle"), mocker.call("discard")]
    exit_mock.assert_called_once_with()


async def test_collection_dashboard_tracks_episode_state(mocker: MockerFixture) -> None:
    session, _, monitor = _session(mocker)
    app = CollectionApp(session, "openyam-teach")
    mocker.patch.object(app, "set_interval")

    async with app.run_test(size=(80, 24)) as pilot:
        assert str(app.query_one("#state", Static).render()) == "READY"
        await pilot.click("#toggle")
        assert "RECORDING" in str(app.query_one("#state", Static).render())
        assert app.query_one("#stop", Button).disabled
        await pilot.click("#discard")
        assert str(app.query_one("#state", Static).render()) == "READY"

    assert monitor.command.call_args_list == [mocker.call("toggle"), mocker.call("discard")]
