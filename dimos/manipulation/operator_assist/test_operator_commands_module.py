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

from collections.abc import Iterator
import json
import threading
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.agents.skill_result import SkillResult
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OperatorEstopStatus,
    OperatorHoldStatus,
)
from dimos.core.rpc_client import RpcCall
from dimos.manipulation.operator_assist.operator_commands_module import (
    ObjectBox,
    OperatorCommand,
    OperatorCommandsModule,
    OperatorState,
    PickResult,
    decode_operator_command,
)
from dimos.web.codecs import resolve_decoder, resolve_encoder

# Colour-image pixels: the red ball sits inside a bowl; the yellow ball is on its own.
BOXES = {
    "red-id": [100, 100, 160, 160],
    "bowl-id": [80, 80, 220, 220],
    "yellow-id": [300, 100, 360, 160],
}
SCANNED = [
    {"object_id": "red-id", "name": "red ball"},
    {"object_id": "bowl-id", "name": "bowl"},
    {"object_id": "yellow-id", "name": "yellow ball"},
]


class Rig:
    def __init__(self) -> None:
        self.module = OperatorCommandsModule()
        self.pick, self.scene, self.coordinator = MagicMock(), MagicMock(), MagicMock()
        self.module._pick = self.pick
        self.module._scene = self.scene
        self.module._coordinator = self.coordinator
        self.pick.scan_objects.return_value = SkillResult(
            "Detected 3 object(s)", metadata={"prompts": ["ball"], "objects": SCANNED}
        )
        self.pick_result = SkillResult("Pick complete")
        self.scene.get_object_bboxes.return_value = BOXES
        self.hold = OperatorHoldStatus(on=False)
        self.calls: list[tuple[Any, ...]] = []

        def task_invoke(task: str, method: str, kwargs: dict[str, Any]) -> OperatorHoldStatus:
            self.calls.append((task, method))
            return self.hold

        def pick_object(object_id: str) -> SkillResult:
            self.calls.append(("pick_object", object_id))
            return self.pick_result

        self.coordinator.task_invoke.side_effect = task_invoke
        self.pick.pick_object.side_effect = pick_object
        self.states: list[OperatorState] = []
        self.module.operator_state.subscribe(self.states.append)

    def send(self, msg: OperatorCommand) -> OperatorState:
        """Send one command, wait for a scan or pick to finish, return the last state."""
        self.module._on_command(msg)
        if self.module._worker is not None:
            self.module._worker.join(timeout=5.0)
            assert not self.module._worker.is_alive()
        return self.states[-1]

    def scan(self) -> OperatorState:
        return self.send(OperatorCommand("scan", prompts=["ball"]))


@pytest.fixture
def rig() -> Iterator[Rig]:
    rig = Rig()
    yield rig
    rig.module.stop()


def test_scan_fills_objects_with_their_boxes_and_forgets_the_target(rig: Rig) -> None:
    rig.module._state = OperatorState(target_id="old-id", target_name="old")

    state = rig.scan()

    rig.pick.scan_objects.assert_called_once_with(["ball"])
    assert state.objects == [
        ObjectBox(obj["object_id"], obj["name"], BOXES[obj["object_id"]]) for obj in SCANNED
    ]
    assert (state.target_id, state.target_name) == ("", "")
    assert state.last_result is not None
    assert (state.last_result.ok, state.last_result.message) == (True, "Detected 3 object(s)")
    assert state.last_result.duration_ms >= 0
    assert (state.busy, state.last_action, state.error) == (False, "scan", "")
    assert rig.states[0].busy is True


def test_scan_without_prompts_is_an_error(rig: Rig) -> None:
    state = rig.send(OperatorCommand("scan", prompts=[" "]))

    assert "prompts" in state.error
    rig.pick.scan_objects.assert_not_called()


def test_click_inside_boxes_targets_the_smallest(rig: Rig) -> None:
    rig.scan()

    state = rig.send(OperatorCommand("target", x=130, y=130))

    assert (state.target_id, state.target_name, state.error) == ("red-id", "red ball", "")


def test_click_near_a_box_targets_the_nearest_centre(rig: Rig) -> None:
    rig.scan()

    # 50 px below the yellow ball's centre, outside every box.
    state = rig.send(OperatorCommand("target", x=330, y=180))

    assert state.target_id == "yellow-id"


def test_click_far_from_every_box_is_an_error(rig: Rig) -> None:
    rig.scan()

    state = rig.send(OperatorCommand("target", x=600, y=400))

    assert state.error == "no object there"
    assert state.target_id == ""


def test_target_by_object_id(rig: Rig) -> None:
    rig.scan()

    assert rig.send(OperatorCommand("target", object_id="bowl-id")).target_name == "bowl"
    assert "no object" in rig.send(OperatorCommand("target", object_id="gone")).error


def test_target_without_a_click_or_id_is_an_error(rig: Rig) -> None:
    rig.scan()

    assert "x" in rig.send(OperatorCommand("target")).error


def test_pick_releases_a_hold_then_picks_the_target(rig: Rig) -> None:
    rig.scan()
    rig.send(OperatorCommand("target", x=130, y=130))
    rig.hold = OperatorHoldStatus(on=True, route="manual", reason="operator request")

    state = rig.send(OperatorCommand("pick"))

    assert rig.calls == [
        ("operator_hold", "get_status"),
        ("operator_hold", "acknowledge"),
        ("pick_object", "red-id"),
    ]
    assert state.released_hold is True
    assert state.last_result == PickResult(True, "Pick complete", state.last_result.duration_ms)
    assert (state.busy, state.last_action, state.error) == (False, "pick", "")


def test_pick_with_no_hold_does_not_acknowledge(rig: Rig) -> None:
    rig.scan()
    rig.send(OperatorCommand("target", object_id="yellow-id"))

    state = rig.send(OperatorCommand("pick"))

    assert rig.calls == [("operator_hold", "get_status"), ("pick_object", "yellow-id")]
    assert state.released_hold is False


def test_pick_that_does_not_hold_the_object_is_not_ok(rig: Rig) -> None:
    rig.scan()
    rig.send(OperatorCommand("target", object_id="red-id"))
    rig.pick_result = SkillResult("No path to any grasp")

    state = rig.send(OperatorCommand("pick"))

    assert state.last_result is not None
    assert (state.last_result.ok, state.last_result.message) == (False, "No path to any grasp")


def test_pick_that_raises_is_reported_not_raised(rig: Rig) -> None:
    rig.scan()
    rig.send(OperatorCommand("target", object_id="red-id"))
    rig.pick.pick_object.side_effect = TimeoutError("timed out after 600 s")

    state = rig.send(OperatorCommand("pick"))

    assert state.error == "pick failed: timed out after 600 s"
    assert state.last_result is not None and state.last_result.ok is False
    assert state.busy is False


def test_pick_is_refused_without_a_target(rig: Rig) -> None:
    rig.scan()

    state = rig.send(OperatorCommand("pick"))

    assert "no target" in state.error
    assert rig.calls == []


def test_pick_is_refused_while_estopped(rig: Rig) -> None:
    rig.scan()
    rig.send(OperatorCommand("target", object_id="red-id"))
    rig.hold = OperatorHoldStatus(on=True, route="failsafe", reason="e-stop")
    rig.module._on_estop_status(OperatorEstopStatus(estopped=True, stamp=1.0))

    state = rig.send(OperatorCommand("pick"))

    assert "E-STOP" in state.error
    assert rig.calls == []

    rig.module._on_estop_status(OperatorEstopStatus(estopped=False, stamp=2.0))
    assert rig.send(OperatorCommand("pick")).released_hold is True


def test_command_while_busy_is_refused_not_queued(rig: Rig) -> None:
    started, release = threading.Event(), threading.Event()

    def slow_scan(prompts: list[str]) -> SkillResult:
        started.set()
        assert release.wait(timeout=5.0)
        return SkillResult("Detected 0 object(s)", metadata={"objects": []})

    rig.pick.scan_objects.side_effect = slow_scan
    rig.module._on_command(OperatorCommand("scan", prompts=["ball"]))
    assert started.wait(timeout=5.0)

    rig.module._on_command(OperatorCommand("pick"))
    refused = rig.states[-1]
    release.set()
    assert rig.module._worker is not None
    rig.module._worker.join(timeout=5.0)

    assert refused.busy is True
    assert refused.error == "busy with scan; wait for it"
    assert rig.calls == []
    assert (rig.states[-1].busy, rig.states[-1].last_action) == (False, "scan")


def test_clear_forgets_objects_and_target(rig: Rig) -> None:
    rig.scan()
    rig.send(OperatorCommand("target", object_id="red-id"))

    state = rig.send(OperatorCommand("clear"))

    assert (state.objects, state.target_id, state.target_name) == ([], "", "")


def test_unknown_action_is_an_error_and_never_raises(rig: Rig) -> None:
    rig.module._on_command(OperatorCommand("dance"))

    assert "unknown action" in rig.states[-1].error


def test_state_goes_to_the_page_as_nested_json(rig: Rig) -> None:
    rig.scan()
    rig.send(OperatorCommand("target", object_id="red-id"))

    # The channel's json.v1 encoder; the page reads the same nested objects.
    encoder = resolve_encoder("json.v1", OperatorState).encode
    sent = json.loads(encoder(rig.states[-1]))

    assert sent["objects"][0] == {
        "object_id": "red-id",
        "name": "red ball",
        "bbox": BOXES["red-id"],
    }
    assert set(sent["last_result"]) == {"ok", "message", "duration_ms"}
    assert (sent["target_id"], sent["target_name"], sent["error"]) == ("red-id", "red ball", "")
    assert sent["stamp"] > 0


def test_decoder_reads_page_commands() -> None:
    assert decode_operator_command({"action": "scan", "prompts": ["red ball"]}) == (
        OperatorCommand("scan", prompts=["red ball"])
    )
    assert decode_operator_command({"action": "target", "x": 130, "y": 140}) == (
        OperatorCommand("target", x=130.0, y=140.0)
    )
    assert decode_operator_command({"action": "target", "object_id": "red-id"}) == (
        OperatorCommand("target", object_id="red-id")
    )
    assert decode_operator_command({"action": "pick"}) == OperatorCommand("pick")
    assert (
        resolve_decoder("operator_command.json.v1", OperatorCommand).decode
        is decode_operator_command
    )


@pytest.mark.parametrize(
    ("value", "complaint"),
    [
        ({"prompts": ["ball"]}, "action"),
        ({"action": "dance"}, "action"),
        ({"action": "scan", "prompts": "ball"}, "prompts"),
        ({"action": "scan", "prompts": [1]}, "prompts"),
        ({"action": "target", "x": "130", "y": 140}, "x"),
        ({"action": "target", "x": 130, "y": True}, "y"),
        ({"action": "target", "object_id": 7}, "object_id"),
        ("scan", "object"),
    ],
)
def test_decoder_rejects_bad_commands(value: Any, complaint: str) -> None:
    with pytest.raises(ValueError, match=complaint):
        decode_operator_command(value)


def test_start_listens_and_sends_the_state_stop_unsubscribes() -> None:
    module = OperatorCommandsModule()
    commands, estop = MagicMock(), MagicMock()
    module.operator_command.transport = commands
    module.operator_estop_status.transport = estop
    states: list[OperatorState] = []
    module.operator_state.subscribe(states.append)
    try:
        module.start()
        assert len(states) == 1 and states[0].busy is False

        estop.subscribe.call_args.args[0](OperatorEstopStatus(estopped=True, stamp=1.0))
        assert module._estopped is True
        commands.subscribe.call_args.args[0](OperatorCommand("clear"))
        assert states[-1].last_action == "clear"
    finally:
        module.stop()
    commands.subscribe.return_value.assert_called_once_with()
    estop.subscribe.return_value.assert_called_once_with()


def test_pick_goes_through_the_module_rpc_client() -> None:
    module = OperatorCommandsModule()
    own_rpc, handle_rpc, module_rpc = module.rpc, MagicMock(), MagicMock()
    module.rpc = module_rpc
    module_rpc.call_sync.return_value = (SkillResult("Pick complete"), lambda: None)
    module._pick = SimpleNamespace(  # type: ignore[assignment]
        pick_object=RpcCall(None, handle_rpc, "pick_object", "PickAndPlaceModule", [])
    )
    try:
        assert module._pick_object("red-id").message == "Pick complete"
    finally:
        module.rpc = own_rpc
        module.stop()
    module_rpc.call_sync.assert_called_once_with(
        "PickAndPlaceModule/pick_object", (("red-id",), {})
    )
    handle_rpc.call_sync.assert_not_called()
