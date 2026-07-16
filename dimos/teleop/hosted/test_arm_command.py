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
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

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
    s = object.__new__(ArmCommandModule)
    s._lock = threading.RLock()
    s._is_engaged = {Hand.LEFT: False, Hand.RIGHT: False}
    s._initial_poses = {Hand.LEFT: None, Hand.RIGHT: None}
    s._current_poses = {Hand.LEFT: None, Hand.RIGHT: None}
    s._controllers = {Hand.LEFT: None, Hand.RIGHT: None}
    s._decoders = {
        LCMPoseStamped._get_packed_fingerprint(): s._on_pose_bytes,
        LCMJoy._get_packed_fingerprint(): s._on_joy_bytes,
        LCMTwistStamped._get_packed_fingerprint(): s._on_twist_bytes,
    }
    s._estopped = False
    s._task_names = {Hand.RIGHT: "teleop_xarm"}
    s.config = SimpleNamespace(
        task_names={"right": "teleop_xarm"},
        control_loop_hz=50.0,
    )
    s.left_controller_output = MagicMock()
    s.right_controller_output = MagicMock()
    s.buttons = MagicMock()
    s.cmd_ack = MagicMock()
    s.robot_state = MagicMock()
    s.coordinator_ee_twist_command = MagicMock()
    s.gripper_command = MagicMock()
    return s


@pytest.fixture
def cmd() -> ArmCommandModule:
    return _bare_module()


def _pose_bytes(frame_id: str, ts: float = 1.0) -> bytes:
    return PoseStamped(ts=ts, frame_id=frame_id).lcm_encode()


def _twist_bytes(x: float = 0.1) -> bytes:
    return TwistStamped(frame_id="eef_twist_arm", linear=[x, 0.0, 0.0]).lcm_encode()


def _tick(s: ArmCommandModule) -> None:
    """One control-loop iteration (the loop body, without the thread)."""
    with s._lock:
        s._handle_engage()
        for hand in Hand:
            if not s._should_publish(hand):
                continue
            output_pose = s._get_output_pose(hand)
            if output_pose is not None:
                s._publish_msg(hand, output_pose)


def _sent_acks(s: ArmCommandModule) -> list[dict[str, Any]]:
    return [json.loads(call.args[0]) for call in s.cmd_ack.publish.call_args_list]


def _engage_right(s: ArmCommandModule) -> None:
    s._on_cmd_raw(_pose_bytes("right"))
    s._controllers[Hand.RIGHT] = QuestControllerState(is_left=False, primary=True)
    _tick(s)


# ─── Command plane: pose dispatch ──────────────────────────────────────


def test_cmd_raw_pose_routes_to_hand(cmd: ArmCommandModule) -> None:
    cmd._on_cmd_raw(_pose_bytes("right"))
    assert cmd._current_poses[Hand.RIGHT] is not None
    assert cmd._current_poses[Hand.LEFT] is None


def test_cmd_raw_bad_frame_id_dropped(cmd: ArmCommandModule) -> None:
    cmd._on_cmd_raw(_pose_bytes("torso"))
    assert cmd._current_poses[Hand.LEFT] is None
    assert cmd._current_poses[Hand.RIGHT] is None


def test_cmd_raw_foreign_bytes_ignored(cmd: ArmCommandModule) -> None:
    cmd._on_cmd_raw(b"\x00\x01\x02\x03garbage-frame")
    assert cmd._current_poses[Hand.RIGHT] is None


def test_cmd_raw_accepts_str(cmd: ArmCommandModule) -> None:
    # A str payload can't match an LCM fingerprint, but must not crash.
    cmd._on_cmd_raw("hello")


# ─── Browser keyboard EE-twist → coordinator eef_twist ─────────────────


def test_twist_routes_to_eef_twist_task(cmd: ArmCommandModule) -> None:
    cmd._on_cmd_raw(_twist_bytes(0.2))
    cmd.coordinator_ee_twist_command.publish.assert_called_once()
    out = cmd.coordinator_ee_twist_command.publish.call_args.args[0]
    assert out.frame_id == EEF_TWIST_TASK_NAME
    assert out.linear.x == pytest.approx(0.2)


def test_twist_dropped_while_estopped(cmd: ArmCommandModule) -> None:
    cmd._estopped = True
    cmd._on_cmd_raw(_twist_bytes(0.2))
    cmd.coordinator_ee_twist_command.publish.assert_not_called()


# ─── Gripper toggle (state_reliable JSON) ──────────────────────────────


def test_gripper_toggle_publishes_bool(cmd: ArmCommandModule) -> None:
    cmd._on_state_json(b'{"type": "gripper", "closed": true}')
    cmd.gripper_command.publish.assert_called_once()
    assert cmd.gripper_command.publish.call_args.args[0].data is True

    cmd._on_state_json(b'{"type": "gripper", "closed": false}')
    assert cmd.gripper_command.publish.call_args.args[0].data is False


# ─── Engage → publish with task-name routing ───────────────────────────


def test_engage_publishes_task_routed_pose(cmd: ArmCommandModule) -> None:
    _engage_right(cmd)
    assert cmd._is_engaged[Hand.RIGHT]
    cmd.right_controller_output.publish.assert_called()
    out = cmd.right_controller_output.publish.call_args.args[0]
    assert out.frame_id == "teleop_xarm"
    cmd.left_controller_output.publish.assert_not_called()


def test_release_disengages(cmd: ArmCommandModule) -> None:
    _engage_right(cmd)
    cmd._controllers[Hand.RIGHT] = QuestControllerState(is_left=False, primary=False)
    _tick(cmd)
    assert not cmd._is_engaged[Hand.RIGHT]


# ─── E-STOP latch ──────────────────────────────────────────────────────


def test_estop_disengages_blocks_publish_and_acks(cmd: ArmCommandModule) -> None:
    _engage_right(cmd)
    cmd.right_controller_output.publish.reset_mock()

    cmd._on_state_json(b'{"type": "estop", "nonce": 7}')

    assert cmd._estopped
    assert not cmd._is_engaged[Hand.RIGHT]
    _tick(cmd)  # primary still held — must NOT re-engage or publish
    assert not cmd._is_engaged[Hand.RIGHT]
    cmd.right_controller_output.publish.assert_not_called()
    assert _sent_acks(cmd) == [{"type": "cmd_ack", "nonce": 7, "ok": True}]


def test_estop_clear_rearms_but_does_not_resume(cmd: ArmCommandModule) -> None:
    _engage_right(cmd)
    cmd._on_state_json(b'{"type": "estop", "nonce": 1}')
    cmd.right_controller_output.publish.reset_mock()

    cmd._on_state_json(b'{"type": "estop_clear", "nonce": 2}')
    assert not cmd._estopped

    # Button still held from before the estop: engaging again is allowed only
    # because a held primary re-engages from the CURRENT pose (delta zero) —
    # the arm doesn't jump.
    _tick(cmd)
    assert cmd._is_engaged[Hand.RIGHT]


def test_operator_lost_disengages(cmd: ArmCommandModule) -> None:
    _engage_right(cmd)
    cmd._on_state_json(b'{"type": "operator_lost"}')
    assert not cmd._is_engaged[Hand.RIGHT]
    assert not cmd._estopped  # loss is not an estop; re-engage allowed


# ─── State plane: malformed input + robot_state ────────────────────────


def test_malformed_json_ignored(cmd: ArmCommandModule) -> None:
    cmd._on_state_json(b"not json")  # must not raise
    cmd._on_state_json(b'{"type": "unknown_kind"}')  # unhandled kind, no-op


def test_robot_state_reports_estop_and_engage(cmd: ArmCommandModule) -> None:
    cmd._on_state_json(b'{"type": "estop", "nonce": 1}')
    # estop publishes robot_state; last payload should show estopped:true.
    payload = json.loads(cmd.robot_state.publish.call_args.args[0])
    assert payload["estopped"] is True
    assert payload["engaged"] == {"left": False, "right": False}
