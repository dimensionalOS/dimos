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


import pytest
from textual.widgets import Button, Static

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.imitation.collection.episode_monitor import EpisodeCommand, EpisodeControlSpec
from dimos.imitation.policy.lerobot.module import RolloutControlSpec
from dimos.imitation.tui import CollectionApp, CollectionSession, RolloutApp, RolloutSession
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.porcelain.dimos import Dimos


@pytest.fixture
def collection_session(mocker):
    driver = mocker.Mock()
    monitor = driver.find_module_by_spec.return_value
    monitor.get_status.return_value = EpisodeStatus(
        ts=1.0, episodes_saved=0, episodes_discarded=0, state="idle", task_label="pick"
    )
    monitor.command.return_value = EpisodeStatus(
        ts=1.0,
        episodes_saved=0,
        episodes_discarded=0,
        state="recording",
        last_event="start",
        task_label="pick",
    )
    session = CollectionSession(driver)
    yield session, driver, monitor
    session.close()


async def test_collection_dashboard_records_and_confirms_detach(collection_session, mocker):
    session, driver, monitor = collection_session
    app = CollectionApp(session)
    mocker.patch.object(app, "set_interval")
    async with app.run_test(size=(80, 24)) as pilot:
        assert str(app.query_one("#state", Static).render()) == "READY"
        await pilot.click("#toggle")
        assert "RECORDING" in str(app.query_one("#state", Static).render())
        assert not app.query_one("#stop", Button).disabled
        await pilot.press("q")
        assert "Recording will continue" in str(app.query_one("#message", Static).render())
        await pilot.press("q")
    driver.find_module_by_spec.assert_called_once_with(EpisodeControlSpec)
    monitor.command.assert_called_once_with("toggle")
    monitor.stop.assert_not_called()
    driver.stop.assert_called_once_with()


def test_collection_disconnect_does_not_save_or_discard(collection_session, mocker):
    session, driver, monitor = collection_session
    app = CollectionApp(session)
    mocker.patch.object(app, "_refresh")
    monitor.get_status.side_effect = ConnectionError("lost")
    app._poll()
    assert app._disconnected
    monitor.command.assert_not_called()
    driver.stop.assert_called_once_with()


def test_rollout_attach_and_exit_do_not_change_policy(mocker):
    driver = mocker.Mock()
    policy = driver.find_module_by_spec.return_value
    policy.rollout_status.return_value = {"active": True, "task": "pick", "last_error": None}
    session = RolloutSession(driver)
    app = RolloutApp(session)
    exit_app = mocker.patch.object(app, "exit")
    app.action_quit()
    session.close()
    session.close()
    driver.find_module_by_spec.assert_called_once_with(RolloutControlSpec)
    exit_app.assert_called_once_with()
    policy.start_rollout.assert_not_called()
    policy.stop_rollout.assert_not_called()
    policy.preflight_rollout.assert_not_called()
    driver.stop.assert_called_once_with()


def test_rollout_preflights_only_explicit_start(mocker):
    driver = mocker.Mock()
    policy = driver.find_module_by_spec.return_value
    policy.rollout_status.return_value = {"active": False}
    policy.preflight_rollout.return_value = {"policy_ready": False, "observations_ready": False}
    session = RolloutSession(driver)
    try:
        assert session.toggle() == policy.preflight_rollout.return_value
        policy.start_rollout.assert_not_called()
        policy.preflight_rollout.return_value = {"policy_ready": True, "observations_ready": True}
        assert session.toggle() == policy.start_rollout.return_value
        policy.start_rollout.assert_called_once_with()
    finally:
        session.close()


class ExternalEpisodeController(Module):
    """An independently implemented controller, not an EpisodeMonitor subclass."""

    @rpc
    def get_status(self) -> EpisodeStatus:
        return EpisodeStatus(ts=1.0, state="idle", episodes_saved=2, episodes_discarded=0)

    @rpc
    def command(self, event: EpisodeCommand) -> EpisodeStatus:
        return EpisodeStatus(
            ts=1.0, state="recording", last_event="start", episodes_saved=2, episodes_discarded=0
        )


@pytest.fixture
def external_controller():
    coordinator = ModuleCoordinator(g=GlobalConfig(n_workers=0, viewer="none"))
    coordinator.start()
    try:
        coordinator.deploy(ExternalEpisodeController, instance_name="vendor/operator")
        coordinator.start_rpc_service()
        yield coordinator
    finally:
        coordinator.stop()


def test_attached_tui_discovers_external_controller_by_spec(external_controller):
    driver = Dimos.connect()
    try:
        session = CollectionSession(driver)
        assert session.get_status().episodes_saved == 2
        assert session.command("start").state == "recording"
        session.close()
        reattached = Dimos.connect()
        try:
            assert (
                reattached.find_module_by_spec(EpisodeControlSpec).get_status().episodes_saved == 2
            )
        finally:
            reattached.stop()
    finally:
        driver.stop()


def test_rollout_disconnect_disables_commands_without_stopping_policy(mocker):
    driver = mocker.Mock()
    policy = driver.find_module_by_spec.return_value
    policy.rollout_status.return_value = {"active": True, "task": "pick", "last_error": None}
    session = RolloutSession(driver)
    app = RolloutApp(session)
    mocker.patch.object(app, "_refresh")
    policy.rollout_status.side_effect = ConnectionError("lost")
    try:
        app._poll()
        app.action_toggle_rollout()
        assert app._disconnected
        assert "policy may still be running" in app._message
        policy.stop_rollout.assert_not_called()
        policy.start_rollout.assert_not_called()
        driver.stop.assert_called_once_with()
    finally:
        session.close()
