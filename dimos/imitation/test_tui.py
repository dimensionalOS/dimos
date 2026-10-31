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


from threading import Thread

import pytest
from textual.widgets import Button, Static

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import GlobalConfig
from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.collection.episode_monitor import EpisodeControlSpec
from dimos.imitation.policy.lerobot.module import RolloutControlSpec
from dimos.imitation.tui import ControlApp, ControlSession
from dimos.porcelain.dimos import NoMatchingModuleError


@pytest.fixture
def controls(mocker):
    driver = mocker.Mock()
    episode = mocker.Mock()
    policy = mocker.Mock()
    episode.get_status.return_value = EpisodeStatus(
        ts=1.0, state="idle", episodes_saved=2, episodes_discarded=1, task_label="pick"
    )
    policy.rollout_status.return_value = {
        "active": False,
        "task": "pick",
        "last_error": None,
        "policy_ready": True,
        "observations_ready": True,
    }
    policy.preflight_rollout.return_value = policy.rollout_status.return_value
    policy.start_rollout.return_value = {**policy.rollout_status.return_value, "active": True}
    policy.stop_rollout.return_value = policy.rollout_status.return_value
    coordinator = mocker.patch("dimos.imitation.tui.CoordinatorRPC.connect").return_value
    coordinator.call.return_value = "original-instance"
    return driver, episode, policy, coordinator


def session_for(controls, collection=True, rollout=True):
    driver, episode, policy, _coordinator = controls

    def discover(spec):
        if spec is EpisodeControlSpec and collection:
            return episode
        if spec is RolloutControlSpec and rollout:
            return policy
        raise NoMatchingModuleError("absent")

    driver.find_module_by_spec.side_effect = discover
    return ControlSession(driver)


@pytest.mark.parametrize("collection,rollout", [(True, False), (False, True), (True, True)])
async def test_panel_shows_available_controls_and_detach_does_not_operate(
    controls, mocker, collection, rollout
):
    driver, episode, policy, coordinator = controls
    session = session_for(controls, collection, rollout)
    app = ControlApp(session)
    mocker.patch.object(app, "set_interval")
    async with app.run_test(size=(100, 40)) as pilot:
        assert bool(app.query("#start-episode")) is collection
        assert bool(app.query("#start-policy")) is rollout
        assert not app.query_one("#confirmation").display
        await pilot.press("q")
    session.close()
    episode.command.assert_not_called()
    policy.preflight_rollout.assert_not_called()
    policy.stop_rollout.assert_not_called()
    coordinator.call.assert_called_once_with("instance_id")
    coordinator.stop.assert_called_once_with()
    driver.stop.assert_called_once_with()


async def test_episode_buttons_save_and_discard_before_shutdown(controls, mocker):
    _driver, episode, _policy, coordinator = controls
    session = session_for(controls)
    idle = episode.get_status.return_value
    recording = idle.model_copy(update={"state": "recording", "last_event": "start"})
    episode.command.side_effect = [
        recording,
        idle.model_copy(update={"last_event": "save"}),
        recording,
        idle.model_copy(update={"last_event": "discard"}),
    ]
    app = ControlApp(session)
    mocker.patch.object(app, "set_interval")
    async with app.run_test(size=(100, 40)) as pilot:
        await pilot.click("#start-episode")
        episode.get_status.return_value = recording
        await pilot.click("#stop-stack")
        assert "Save or discard" in str(app.query_one("#message", Static).render())
        assert not app.query_one("#confirmation").display
        await pilot.click("#save-episode")
        await pilot.click("#start-episode")
        await pilot.click("#discard-episode")
        episode.get_status.return_value = idle
        await pilot.click("#stop-stack")
        assert app.query_one("#confirmation").display
        await pilot.click("#confirm-stop")
        assert app.query_one("#start-policy", Button).disabled
    assert [call.args[0] for call in episode.command.call_args_list] == [
        "start",
        "save",
        "start",
        "discard",
    ]
    coordinator.call.assert_any_call("shutdown", expected_instance_id="original-instance")


async def test_policy_start_preflights_and_stop_is_explicit(controls, mocker):
    _driver, _episode, policy, coordinator = controls
    session = session_for(controls, collection=False)
    app = ControlApp(session)
    mocker.patch.object(app, "set_interval")
    async with app.run_test(size=(100, 40)) as pilot:
        policy.preflight_rollout.assert_not_called()
        await pilot.press("space")
        policy.start_rollout.assert_not_called()
        await pilot.click("#start-policy")
        assert "ACTIVE" in str(app.query_one("#policy-status", Static).render())
        await pilot.click("#stop-policy")
    policy.preflight_rollout.assert_called_once_with()
    policy.start_rollout.assert_called_once_with()
    policy.stop_rollout.assert_called_once_with()
    coordinator.call.assert_called_once_with("instance_id")


def test_failed_preflight_does_not_start_policy(controls):
    _driver, _episode, policy, _coordinator = controls
    session = session_for(controls, collection=False)
    policy.preflight_rollout.return_value = {"policy_ready": False, "observations_ready": False}
    try:
        assert session.start_policy() == {"policy_ready": False, "observations_ready": False}
        policy.start_rollout.assert_not_called()
    finally:
        session.close()


@pytest.mark.parametrize(
    "error",
    [ValueError("ambiguous"), LookupError("cannot inspect module classes"), TimeoutError("lost")],
)
def test_discovery_errors_are_not_treated_as_absence(controls, error):
    driver, _episode, _policy, coordinator = controls
    driver.find_module_by_spec.side_effect = error
    with pytest.raises(type(error), match=str(error)):
        ControlSession(driver)
    coordinator.stop.assert_called_once_with()


def test_no_capabilities_explains_attach_failure(controls):
    with pytest.raises(LookupError, match="no episode or rollout controls"):
        session_for(controls, collection=False, rollout=False)


def test_shutdown_rechecks_episode_at_confirmation(controls):
    _driver, episode, _policy, coordinator = controls
    session = session_for(controls)
    episode.get_status.return_value = episode.get_status.return_value.model_copy(
        update={"state": "recording"}
    )
    try:
        with pytest.raises(RuntimeError, match="Save or discard"):
            session.stop_stack()
        coordinator.call.assert_called_once_with("instance_id")
    finally:
        session.close()


async def test_cancel_shutdown_and_connection_loss_never_stop_stack(controls, mocker):
    _driver, episode, policy, coordinator = controls
    session = session_for(controls)
    app = ControlApp(session)
    mocker.patch.object(app, "set_interval")
    async with app.run_test(size=(100, 40)) as pilot:
        await pilot.click("#stop-stack")
        await pilot.click("#cancel-stop")
        assert not app.query_one("#confirmation").display
        episode.get_status.side_effect = ConnectionError("lost")
        app._poll()
        assert app.query_one("#start-policy", Button).disabled
        assert app.query_one("#stop-stack", Button).disabled
        assert "may still be running" in str(app.query_one("#message", Static).render())
    coordinator.call.assert_called_once_with("instance_id")
    policy.stop_rollout.assert_not_called()


def test_shutdown_rejects_replacement_coordinator():
    original = ModuleCoordinator(g=GlobalConfig(n_workers=0, viewer="none"))
    replacement = ModuleCoordinator(g=GlobalConfig(n_workers=0, viewer="none"))
    try:
        with pytest.raises(ValueError, match="Coordinator changed"):
            replacement.rpcs["shutdown"](expected_instance_id=original.rpcs["instance_id"]())
        assert not replacement._shutdown_event.is_set()
        replacement.rpcs["shutdown"](expected_instance_id=replacement.rpcs["instance_id"]())
        assert replacement._shutdown_event.is_set()
    finally:
        original.stop()
        replacement.stop()


def test_confirmed_shutdown_unblocks_loop_and_stops_modules(mocker):
    coordinator = ModuleCoordinator(g=GlobalConfig(n_workers=0, viewer="none"))
    mocker.patch.object(coordinator, "start_rpc_service")
    stop = mocker.patch.object(coordinator, "stop")
    loop = Thread(target=coordinator.loop)
    loop.start()
    try:
        coordinator.rpcs["shutdown"](expected_instance_id=coordinator.rpcs["instance_id"]())
        loop.join(timeout=1)
        assert not loop.is_alive()
        stop.assert_called_once_with()
    finally:
        coordinator.shutdown()
        loop.join(timeout=1)
