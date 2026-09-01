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

"""Unit tests for ArmCommandModule's operator-command handling.

The real module is constructed with only the framework ``Module.__init__``
patched out (see the ``module`` fixture); its ports and coordinator ref are
mocked. Camera mux / telemetry / stats live in separate modules now (see
test_camera_mux.py / test_hosted_stats.py); this file covers only the command
plane.
"""

from __future__ import annotations

from collections.abc import Iterator
import json
import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import pytest

from dimos.core.module import Module
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.teleop.hosted.arm_command import ArmCommandModule
from dimos.teleop.quest.quest_types import Hand, QuestControllerState
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
def module(monkeypatch: pytest.MonkeyPatch) -> Iterator[ArmCommandModule]:
    """A real ArmCommandModule with only the framework ``Module.__init__``
    skipped — the quest-layer and command-plane inits (engage state, decoder
    table, estop/twist gates) run for real. Ports / coordinator ref / config
    are mocked; config is seeded by the patched init."""

    def _fake_init(self: Any, **kwargs: Any) -> None:
        self.config = SimpleNamespace(
            control_loop_hz=50.0,
            cmd_stale_after_sec=0.5,
            enable_ui_scaling=False,
        )

    monkeypatch.setattr(Module, "__init__", _fake_init)
    module = ArmCommandModule()
    for port in (
        "left_controller_output",
        "right_controller_output",
        "buttons",
        "cmd_ack",
        "robot_state",
        "ee_twist_command",
        "gripper_command",
        "coordinator",
    ):
        setattr(module, port, MagicMock())
    module._cmd.start()
    yield module
    module._cmd.stop()


def _pose_bytes(frame_id: str, ts: float | None = None) -> bytes:
    return PoseStamped(ts=time.time() if ts is None else ts, frame_id=frame_id).lcm_encode()


def _twist_bytes(x: float = 0.1, angular_x: float = 0.0, ts: float | None = None) -> bytes:
    # ts=None keeps TwistStamped's default stamp (now) — a fresh command.
    kwargs = {} if ts is None else {"ts": ts}
    return TwistStamped(
        frame_id="eef_twist_arm", linear=[x, 0.0, 0.0], angular=[angular_x, 0.0, 0.0], **kwargs
    ).lcm_encode()


def _tick(module: ArmCommandModule) -> None:
    """One control-loop iteration (the loop body, without the thread)."""
    with module._lock:
        module._handle_engage()
        for hand in Hand:
            if not module._should_publish(hand):
                continue
            output_pose = module._get_output_pose(hand)
            if output_pose is not None:
                module._publish_msg(hand, output_pose)


def _sent_acks(module: ArmCommandModule) -> list[dict[str, Any]]:
    return [json.loads(call.args[0]) for call in module.cmd_ack.publish.call_args_list]


def _publish_right(module: ArmCommandModule) -> None:
    module._on_cmd_raw(_pose_bytes("right"))
    module._controllers[Hand.RIGHT] = QuestControllerState(is_left=False)
    _tick(module)


# ─── Command plane: pose dispatch ──────────────────────────────────────


def test_cmd_raw_pose_routes_to_hand(module: ArmCommandModule) -> None:
    module._on_cmd_raw(_pose_bytes("right"))
    assert module._current_poses[Hand.RIGHT] is not None
    assert module._current_poses[Hand.LEFT] is None


def test_cmd_raw_bad_frame_id_dropped(module: ArmCommandModule) -> None:
    module._on_cmd_raw(_pose_bytes("torso"))
    assert module._current_poses[Hand.LEFT] is None
    assert module._current_poses[Hand.RIGHT] is None


def test_cmd_raw_foreign_bytes_ignored(module: ArmCommandModule) -> None:
    module._on_cmd_raw(b"\x00\x01\x02\x03garbage-frame")
    assert module._current_poses[Hand.RIGHT] is None


def test_stale_pose_dropped(module: ArmCommandModule) -> None:
    module._on_cmd_raw(_pose_bytes("right", ts=time.time() - 1.0))
    assert module._current_poses[Hand.RIGHT] is None


def test_out_of_order_pose_dropped(module: ArmCommandModule) -> None:
    t = time.time() - 0.2
    module._on_cmd_raw(_pose_bytes("right", ts=t))
    module._on_cmd_raw(_pose_bytes("right", ts=t - 0.1))
    accepted = module._current_poses[Hand.RIGHT]
    module._on_cmd_raw(_pose_bytes("right", ts=t + 0.1))
    assert module._current_poses[Hand.RIGHT] is not accepted


def test_future_stamped_pose_dropped(module: ArmCommandModule) -> None:
    module._on_cmd_raw(_pose_bytes("right", ts=time.time() + 5.0))
    assert module._current_poses[Hand.RIGHT] is None
    module._on_cmd_raw(_pose_bytes("right"))
    assert module._current_poses[Hand.RIGHT] is not None


def test_pose_watermark_is_per_hand(module: ArmCommandModule) -> None:
    t = time.time()
    module._on_cmd_raw(_pose_bytes("right", ts=t))
    module._on_cmd_raw(_pose_bytes("left", ts=t - 0.05))
    assert module._current_poses[Hand.LEFT] is not None


# ─── Browser keyboard EE-twist → coordinator eef_twist ─────────────────


def test_twist_republished_without_task_address(module: ArmCommandModule) -> None:
    module._on_cmd_raw(_twist_bytes(0.2))
    module.ee_twist_command.publish.assert_called_once()
    out = module.ee_twist_command.publish.call_args.args[0]
    assert out.frame_id == ""  # addressing is the port wiring, not the payload
    assert out.linear.x == pytest.approx(0.2)


def test_ui_scale_disabled_is_rejected(module: ArmCommandModule) -> None:
    module._on_state_json(b'{"type": "teleop_scale", "scale": 0.5, "nonce": 3}')

    assert _sent_acks(module) == [{"type": "cmd_ack", "nonce": 3, "ok": False}]
    assert module._translation_scale == 1.0


def test_ui_scale_updates_pose_and_keyboard_twist(module: ArmCommandModule) -> None:
    module.config.enable_ui_scaling = True
    module._on_state_json(b'{"type": "teleop_scale", "scale": 0.5, "nonce": 4}')
    module._on_cmd_raw(_twist_bytes(0.2, angular_x=0.3))

    assert _sent_acks(module) == [{"type": "cmd_ack", "nonce": 4, "ok": True}]
    assert module._translation_scale == 0.5
    out = module.ee_twist_command.publish.call_args.args[0]
    assert out.linear.x == pytest.approx(0.1)
    assert out.angular.x == pytest.approx(0.3)


def test_twist_dropped_while_estopped(module: ArmCommandModule) -> None:
    module._estopped = True
    module._on_cmd_raw(_twist_bytes(0.2))
    module.ee_twist_command.publish.assert_not_called()


def test_stale_twist_dropped(module: ArmCommandModule) -> None:
    module._on_cmd_raw(_twist_bytes(0.2, ts=time.time() - 1.0))  # > cmd_stale_after_sec
    module.ee_twist_command.publish.assert_not_called()


def test_future_stamped_twist_dropped(module: ArmCommandModule) -> None:
    module._on_cmd_raw(_twist_bytes(0.2, ts=time.time() + 5.0))
    module.ee_twist_command.publish.assert_not_called()
    # ...and it must not advance the ordering watermark (would stall real cmds).
    module._on_cmd_raw(_twist_bytes(0.3))
    module.ee_twist_command.publish.assert_called_once()


def test_out_of_order_twist_dropped(module: ArmCommandModule) -> None:
    t = time.time()
    module._on_cmd_raw(_twist_bytes(0.2, ts=t))
    module._on_cmd_raw(_twist_bytes(0.3, ts=t - 0.1))  # older than the last accepted
    assert module.ee_twist_command.publish.call_count == 1


def test_stale_twist_warning_rate_limited(module: ArmCommandModule) -> None:
    with patch("dimos.teleop.hosted.arm_command.logger") as log:
        for _ in range(5):
            module._on_cmd_raw(_twist_bytes(0.2, ts=time.time() - 1.0))
    assert log.warning.call_count == 1  # burst of stale frames → one warning per second


# ─── Gripper toggle (state_reliable JSON) ──────────────────────────────


def test_gripper_toggle_publishes_normalized_opening(module: ArmCommandModule) -> None:
    module._on_state_json(b'{"type": "gripper", "closed": true}')
    module.gripper_command.publish.assert_called_once()
    assert module.gripper_command.publish.call_args.args[0].data == pytest.approx(0.0)

    module._on_state_json(b'{"type": "gripper", "closed": false}')
    assert module.gripper_command.publish.call_args.args[0].data == pytest.approx(1.0)


def test_gripper_dropped_while_estopped(module: ArmCommandModule) -> None:
    module._estopped = True
    module._on_state_json(b'{"type": "gripper", "closed": true}')
    module.gripper_command.publish.assert_not_called()


# ─── Engage → publish on the hand's own port ───────────────────────────


def test_controller_pose_publishes_on_hand_port(module: ArmCommandModule) -> None:
    _publish_right(module)
    assert not module._is_engaged[Hand.RIGHT]
    module.right_controller_output.publish.assert_called()
    out = module.right_controller_output.publish.call_args.args[0]
    assert out.frame_id == "right"  # handedness preserved; no task-name overwrite
    module.left_controller_output.publish.assert_not_called()


def test_face_button_release_does_not_gate_raw_pose(module: ArmCommandModule) -> None:
    _publish_right(module)
    module.right_controller_output.publish.reset_mock()
    module._controllers[Hand.RIGHT] = QuestControllerState(is_left=False, primary=False)
    _tick(module)
    assert not module._is_engaged[Hand.RIGHT]
    module.right_controller_output.publish.assert_called_once()


# ─── E-STOP latch ──────────────────────────────────────────────────────


def test_estop_disengages_blocks_publish_and_acks(module: ArmCommandModule) -> None:
    _publish_right(module)
    module.right_controller_output.publish.reset_mock()

    module._on_state_json(b'{"type": "estop", "nonce": 7}')

    assert module._estopped
    assert not module._is_engaged[Hand.RIGHT]
    wait_until(lambda: bool(_sent_acks(module)), timeout=2.0)  # latch runs off-thread
    module.coordinator.set_estop.assert_called_once_with(True)
    _tick(module)
    assert not module._is_engaged[Hand.RIGHT]
    module.right_controller_output.publish.assert_not_called()
    assert _sent_acks(module) == [{"type": "cmd_ack", "nonce": 7, "ok": True}]


def test_estop_nacked_when_coordinator_latch_fails(module: ArmCommandModule) -> None:
    module.coordinator.set_estop.side_effect = RuntimeError("coordinator wedged")
    module._on_state_json(b'{"type": "estop", "nonce": 5}')
    wait_until(lambda: bool(_sent_acks(module)), timeout=2.0)
    assert module._estopped  # local latch still gates operator input
    assert _sent_acks(module) == [{"type": "cmd_ack", "nonce": 5, "ok": False}]


def test_estop_clear_keeps_raw_pose_forwarding(module: ArmCommandModule) -> None:
    _publish_right(module)
    module._on_state_json(b'{"type": "estop", "nonce": 1}')
    wait_until(lambda: len(_sent_acks(module)) == 1, timeout=2.0)
    module.right_controller_output.publish.reset_mock()

    module._on_state_json(b'{"type": "estop_clear", "nonce": 2}')
    assert not module._estopped
    wait_until(lambda: len(_sent_acks(module)) == 2, timeout=2.0)
    module.coordinator.set_estop.assert_called_with(False)

    _tick(module)
    assert not module._is_engaged[Hand.RIGHT]
    module.right_controller_output.publish.assert_called_once()


def test_operator_lost_disengages(module: ArmCommandModule) -> None:
    _publish_right(module)
    module._on_state_json(b'{"type": "operator_lost"}')
    assert not module._is_engaged[Hand.RIGHT]
    assert not module._estopped  # loss is not an estop; re-engage allowed


# ─── State plane: robot_state telemetry ────────────────────────────────


def test_robot_state_reports_estop_and_engage(module: ArmCommandModule) -> None:
    module._on_state_json(b'{"type": "estop", "nonce": 1}')
    payload = json.loads(module.robot_state.publish.call_args.args[0])
    assert payload["estopped"] is True
    assert payload["engaged"] == {"left": False, "right": False}
