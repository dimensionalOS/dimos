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

No coordinator / no WebRTC: ``ArmCommandModule.__init__`` builds a whole Module,
so instances are assembled bare (``object.__new__``) with only the fields the
tested paths touch, and output ports are mocked. Camera mux / telemetry / stats
live in separate modules now (see test_camera_mux.py / test_hosted_stats.py);
this file covers only the command plane.
"""

from __future__ import annotations

import json
import threading
import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

from dimos_lcm.geometry_msgs import (
    PoseStamped as LCMPoseStamped,
    TwistStamped as LCMTwistStamped,
)
from dimos_lcm.sensor_msgs import Joy as LCMJoy
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.teleop.hosted.arm_command import ArmCommandModule
from dimos.teleop.quest.quest_types import Hand, QuestControllerState


def _bare_module() -> ArmCommandModule:
    """An ArmCommandModule with only the fields the command paths need."""
    module = object.__new__(ArmCommandModule)
    module._lock = threading.RLock()
    module._is_engaged = {Hand.LEFT: False, Hand.RIGHT: False}
    module._initial_poses = {Hand.LEFT: None, Hand.RIGHT: None}
    module._current_poses = {Hand.LEFT: None, Hand.RIGHT: None}
    module._controllers = {Hand.LEFT: None, Hand.RIGHT: None}
    module._decoders = {
        LCMPoseStamped._get_packed_fingerprint(): module._on_pose_bytes,
        LCMJoy._get_packed_fingerprint(): module._on_joy_bytes,
        LCMTwistStamped._get_packed_fingerprint(): module._on_twist_bytes,
    }
    module._estopped = False
    module._last_twist_ts = 0.0
    module._last_stale_warn = 0.0
    module._last_future_warn = 0.0
    module._task_names = {Hand.RIGHT: "teleop_xarm"}
    module.config = SimpleNamespace(
        task_names={"right": "teleop_xarm"},
        control_loop_hz=50.0,
        cmd_stale_after_sec=0.5,
    )
    module.left_controller_output = MagicMock()
    module.right_controller_output = MagicMock()
    module.buttons = MagicMock()
    module.cmd_ack = MagicMock()
    module.robot_state = MagicMock()
    module.coordinator_ee_twist_command = MagicMock()
    module.gripper_command = MagicMock()
    return module


@pytest.fixture
def module() -> ArmCommandModule:
    return _bare_module()


def _pose_bytes(frame_id: str, ts: float = 1.0) -> bytes:
    return PoseStamped(ts=ts, frame_id=frame_id).lcm_encode()


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


def test_cmd_raw_accepts_str(module: ArmCommandModule) -> None:
    # A str payload can't match an LCM fingerprint, but must not crash.
    module._on_cmd_raw("hello")


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
    _tick(module)  # primary still held — must NOT re-engage or publish
    assert not module._is_engaged[Hand.RIGHT]
    module.right_controller_output.publish.assert_not_called()
    assert _sent_acks(module) == [{"type": "cmd_ack", "nonce": 7, "ok": True}]


def test_estop_clear_rearms_but_does_not_resume(module: ArmCommandModule) -> None:
    _engage_right(module)
    module._on_state_json(b'{"type": "estop", "nonce": 1}')
    module.right_controller_output.publish.reset_mock()

    module._on_state_json(b'{"type": "estop_clear", "nonce": 2}')
    assert not module._estopped

    # Button still held from before the estop: engaging again is allowed only
    # because a held primary re-engages from the CURRENT pose (delta zero) —
    # the arm doesn't jump.
    _tick(module)
    assert module._is_engaged[Hand.RIGHT]


def test_operator_lost_disengages(module: ArmCommandModule) -> None:
    _engage_right(module)
    module._on_state_json(b'{"type": "operator_lost"}')
    assert not module._is_engaged[Hand.RIGHT]
    assert not module._estopped  # loss is not an estop; re-engage allowed


# ─── State plane: malformed input + robot_state ────────────────────────


def test_malformed_json_ignored(module: ArmCommandModule) -> None:
    module._on_state_json(b"not json")  # must not raise
    module._on_state_json(b'{"type": "unknown_kind"}')  # unhandled kind, no-op


def test_robot_state_reports_estop_and_engage(module: ArmCommandModule) -> None:
    module._on_state_json(b'{"type": "estop", "nonce": 1}')
    # estop publishes robot_state; last payload should show estopped:true.
    payload = json.loads(module.robot_state.publish.call_args.args[0])
    assert payload["estopped"] is True
    assert payload["engaged"] == {"left": False, "right": False}
