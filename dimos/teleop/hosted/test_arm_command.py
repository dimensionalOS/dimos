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

import json
import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import pytest

from dimos.core.module import Module
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.teleop.hosted.arm_command import ArmCommandModule
from dimos.teleop.quest.quest_types import Hand, QuestControllerState


@pytest.fixture
def module(monkeypatch: pytest.MonkeyPatch) -> ArmCommandModule:
    """A real ArmCommandModule with only the framework ``Module.__init__``
    skipped — the quest-layer and command-plane inits (engage state, decoder
    table, estop/twist gates) run for real. Ports / coordinator ref / config
    are mocked; config is seeded by the patched init since the quest layer
    reads ``config.task_names`` while constructing."""

    def _fake_init(self: Any, **kwargs: Any) -> None:
        self.config = SimpleNamespace(
            task_names={"right": "teleop_xarm"},
            control_loop_hz=50.0,
            cmd_stale_after_sec=0.5,
            linear_scale=1.0,
            angular_scale=1.0,
            video_jpeg_quality=80,
        )

    monkeypatch.setattr(Module, "__init__", _fake_init)
    module = ArmCommandModule()
    for port in (
        "left_controller_output",
        "right_controller_output",
        "buttons",
        "cmd_ack",
        "robot_state",
        "coordinator_ee_twist_command",
        "gripper_command",
        "coordinator",
    ):
        setattr(module, port, MagicMock())
    return module


def _pose_bytes(frame_id: str, ts: float | None = None) -> bytes:
    return PoseStamped(ts=time.time() if ts is None else ts, frame_id=frame_id).lcm_encode()


def _twist_bytes(x: float = 0.1, ts: float | None = None) -> bytes:
    # ts=None keeps TwistStamped's default stamp (now) — a fresh command.
    kwargs = {} if ts is None else {"ts": ts}
    return TwistStamped(frame_id="eef_twist_arm", linear=[x, 0.0, 0.0], **kwargs).lcm_encode()


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


def _engage_right(module: ArmCommandModule) -> None:
    module._on_cmd_raw(_pose_bytes("right"))
    module._controllers[Hand.RIGHT] = QuestControllerState(is_left=False, primary=True)
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
    t = time.time()
    module._on_cmd_raw(_pose_bytes("right", ts=t))
    module._on_cmd_raw(_pose_bytes("right", ts=t - 0.1))
    accepted = module._current_poses[Hand.RIGHT]
    module._on_cmd_raw(_pose_bytes("right", ts=t + 0.1))
    assert module._current_poses[Hand.RIGHT] is not accepted


def test_pose_watermark_is_per_hand(module: ArmCommandModule) -> None:
    t = time.time()
    module._on_cmd_raw(_pose_bytes("right", ts=t))
    module._on_cmd_raw(_pose_bytes("left", ts=t - 0.05))
    assert module._current_poses[Hand.LEFT] is not None


# ─── Browser keyboard EE-twist → coordinator eef_twist ─────────────────


def test_twist_routes_to_eef_twist_task(module: ArmCommandModule) -> None:
    module._on_cmd_raw(_twist_bytes(0.2))
    module.coordinator_ee_twist_command.publish.assert_called_once()
    out = module.coordinator_ee_twist_command.publish.call_args.args[0]
    assert out.frame_id == EEF_TWIST_TASK_NAME
    assert out.linear.x == pytest.approx(0.2)


def test_twist_dropped_while_estopped(module: ArmCommandModule) -> None:
    module._estopped = True
    module._on_cmd_raw(_twist_bytes(0.2))
    module.coordinator_ee_twist_command.publish.assert_not_called()


def test_stale_twist_dropped(module: ArmCommandModule) -> None:
    module._on_cmd_raw(_twist_bytes(0.2, ts=time.time() - 1.0))  # > cmd_stale_after_sec
    module.coordinator_ee_twist_command.publish.assert_not_called()


def test_future_stamped_twist_dropped(module: ArmCommandModule) -> None:
    module._on_cmd_raw(_twist_bytes(0.2, ts=time.time() + 5.0))
    module.coordinator_ee_twist_command.publish.assert_not_called()
    # ...and it must not advance the ordering watermark (would stall real cmds).
    module._on_cmd_raw(_twist_bytes(0.3))
    module.coordinator_ee_twist_command.publish.assert_called_once()


def test_out_of_order_twist_dropped(module: ArmCommandModule) -> None:
    t = time.time()
    module._on_cmd_raw(_twist_bytes(0.2, ts=t))
    module._on_cmd_raw(_twist_bytes(0.3, ts=t - 0.1))  # older than the last accepted
    assert module.coordinator_ee_twist_command.publish.call_count == 1


def test_stale_twist_warning_rate_limited(module: ArmCommandModule) -> None:
    with patch("dimos.teleop.hosted.arm_command.logger") as log:
        for _ in range(5):
            module._on_cmd_raw(_twist_bytes(0.2, ts=time.time() - 1.0))
    assert log.warning.call_count == 1  # burst of stale frames → one warning per second


# ─── Gripper toggle (state_reliable JSON) ──────────────────────────────


def test_gripper_toggle_publishes_bool(module: ArmCommandModule) -> None:
    module._on_state_json(b'{"type": "gripper", "closed": true}')
    module.gripper_command.publish.assert_called_once()
    assert module.gripper_command.publish.call_args.args[0].data is True

    module._on_state_json(b'{"type": "gripper", "closed": false}')
    assert module.gripper_command.publish.call_args.args[0].data is False


def test_gripper_dropped_while_estopped(module: ArmCommandModule) -> None:
    module._estopped = True
    module._on_state_json(b'{"type": "gripper", "closed": true}')
    module.gripper_command.publish.assert_not_called()


# ─── Engage → publish with task-name routing ───────────────────────────


def test_engage_publishes_task_routed_pose(module: ArmCommandModule) -> None:
    _engage_right(module)
    assert module._is_engaged[Hand.RIGHT]
    module.right_controller_output.publish.assert_called()
    out = module.right_controller_output.publish.call_args.args[0]
    assert out.frame_id == "teleop_xarm"
    module.left_controller_output.publish.assert_not_called()


def test_release_disengages(module: ArmCommandModule) -> None:
    _engage_right(module)
    module._controllers[Hand.RIGHT] = QuestControllerState(is_left=False, primary=False)
    _tick(module)
    assert not module._is_engaged[Hand.RIGHT]


# ─── E-STOP latch ──────────────────────────────────────────────────────


def test_estop_disengages_blocks_publish_and_acks(module: ArmCommandModule) -> None:
    _engage_right(module)
    module.right_controller_output.publish.reset_mock()

    module._on_state_json(b'{"type": "estop", "nonce": 7}')

    assert module._estopped
    assert not module._is_engaged[Hand.RIGHT]
    module.coordinator.set_estop.assert_called_once_with(True)
    _tick(module)  # primary still held — must NOT re-engage or publish
    assert not module._is_engaged[Hand.RIGHT]
    module.right_controller_output.publish.assert_not_called()
    assert _sent_acks(module) == [{"type": "cmd_ack", "nonce": 7, "ok": True}]


def test_estop_clear_reengages_held_button_from_current_pose(module: ArmCommandModule) -> None:
    _engage_right(module)
    module._on_state_json(b'{"type": "estop", "nonce": 1}')
    module.right_controller_output.publish.reset_mock()

    module._on_state_json(b'{"type": "estop_clear", "nonce": 2}')
    assert not module._estopped
    module.coordinator.set_estop.assert_called_with(False)

    # Button still held from before the estop: the next tick re-engages and
    # rebaselines to the CURRENT pose (delta zero), so the arm resumes tracking
    # from where it is — no jump.
    _tick(module)
    assert module._is_engaged[Hand.RIGHT]


def test_operator_lost_disengages(module: ArmCommandModule) -> None:
    _engage_right(module)
    module._on_state_json(b'{"type": "operator_lost"}')
    assert not module._is_engaged[Hand.RIGHT]
    assert not module._estopped  # loss is not an estop; re-engage allowed


# ─── State plane: robot_state telemetry ────────────────────────────────


def test_robot_state_reports_estop_and_engage(module: ArmCommandModule) -> None:
    module._on_state_json(b'{"type": "estop", "nonce": 1}')
    payload = json.loads(module.robot_state.publish.call_args.args[0])
    assert payload["estopped"] is True
    assert payload["engaged"] == {"left": False, "right": False}
