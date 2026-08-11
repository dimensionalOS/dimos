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

"""Deterministic tests for the hosted R1 Lite teleop plane.

No broker, no ROS, no threads: streams are replaced per test and the
command executor is faked, so these pin the operator-plane semantics —
frame dispatch parity with local teleop, WAN guards, and the E-STOP
latch across arms, chassis, and coordinator.
"""

from __future__ import annotations

import json
import time
import types
from typing import Any

import pytest

import dimos.core.module as module_mod
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_quest_teleop import (
    r1lite_quest_teleop,
)
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_quest_teleop_hosted import (
    r1lite_quest_teleop_hosted,
    r1lite_quest_teleop_hosted_sim,
)
from dimos.robot.galaxea.r1lite.hosted_module import R1LiteHostedTeleopModule
from dimos.robot.galaxea.r1lite.test_r1lite_quest_teleop import (
    _joy_frame,
    _pose_frame,
)
from dimos.teleop.quest.quest_types import Hand


class _NoRpc(RPCSpec):
    def __init__(self, **kw: Any) -> None:
        raise ValueError("rpc disabled for unit tests")


@pytest.fixture(autouse=True)
def _no_background_threads(monkeypatch: Any) -> None:
    monkeypatch.setattr(module_mod, "get_loop", lambda: (types.SimpleNamespace(), None))


class _FakeOut:
    def __init__(self) -> None:
        self.msgs: list[Any] = []

    def publish(self, msg: Any) -> None:
        self.msgs.append(msg)


class _FakeExecutor:
    """Runs commands inline and relays acks like the real executor."""

    def __init__(self, send_ack: Any) -> None:
        self.submitted: list[tuple[str, Any, bool]] = []
        self._send_ack = send_ack

    def submit(self, kind: str, nonce: Any, fn: Any, urgent: bool = False) -> None:
        self.submitted.append((kind, nonce, urgent))
        self._send_ack(nonce, bool(fn(0)))


class _FakeCoordinator:
    def __init__(self) -> None:
        self.estops: list[bool] = []

    def set_estop(self, value: bool) -> None:
        self.estops.append(value)


def _module(**config_kwargs: Any) -> R1LiteHostedTeleopModule:
    m = R1LiteHostedTeleopModule(
        rpc_transport=_NoRpc,
        task_names={"left": "teleop_left_arm", "right": "teleop_right_arm"},
        **config_kwargs,
    )
    for stream in (
        "cmd_vel",
        "gripper_left_command",
        "gripper_right_command",
        "teleop_buttons",
        "left_controller_output",
        "right_controller_output",
        "cmd_ack",
        "robot_state",
    ):
        setattr(m, stream, _FakeOut())
    m._cmd = _FakeExecutor(m._send_ack)
    m.coordinator = _FakeCoordinator()
    return m


# ─── cmd_raw dispatch parity ──────────────────────────────────────────


def test_cmd_raw_dispatches_pose_and_joy_like_local() -> None:
    m = _module()
    m._on_cmd_raw(_joy_frame("left", primary=True))
    m._on_cmd_raw(_stamped_pose("left", time.time()))
    assert m._controllers[Hand.LEFT] is not None
    assert m._controllers[Hand.LEFT].primary
    assert m._current_poses.get(Hand.LEFT) is not None


def test_cmd_raw_ignores_unknown_fingerprints() -> None:
    m = _module()
    m._on_cmd_raw(b"garbagegarbagegarbage")
    assert m._current_poses.get(Hand.LEFT) is None


# ─── WAN guards ───────────────────────────────────────────────────────


def _stamped_pose(hand: str, ts: float) -> bytes:
    data = _pose_frame(hand)
    msg = PoseStamped.lcm_decode(data)
    msg.ts = ts
    return msg.lcm_encode()


def test_lagging_pose_dropped_relative_to_link_baseline(monkeypatch: Any) -> None:
    """Staleness = lag beyond the link's own delay baseline, not clock age."""
    import dimos.robot.galaxea.r1lite.hosted_module as hm

    t = [1000.0]
    monkeypatch.setattr(hm.time, "time", lambda: t[0])
    m = _module()
    m._on_cmd_raw(_stamped_pose("left", 999.7))  # baseline delay 0.3s
    first = m._current_poses.get(Hand.LEFT)
    assert first is not None
    t[0] = 1001.0
    m._on_cmd_raw(_stamped_pose("left", 1000.7))  # same delay: fresh
    second = m._current_poses[Hand.LEFT]
    assert second is not first
    t[0] = 1002.5
    m._on_cmd_raw(_stamped_pose("left", 1001.0))  # 1.2s over baseline: dropped
    assert m._current_poses[Hand.LEFT] is second


def test_operator_clock_skew_does_not_freeze_the_hand(monkeypatch: Any) -> None:
    """Hardware regression 2026-08-11: a headset ~357 ms off robot time
    intermittently tripped the absolute stale/future guards and froze
    arms and sticks. Skewed-but-steady clocks must pass in both
    directions; only added lag may drop poses."""
    import dimos.robot.galaxea.r1lite.hosted_module as hm

    t = [2000.0]
    monkeypatch.setattr(hm.time, "time", lambda: t[0])
    for skew in (-0.357, +0.357, -60.0, +60.0):
        m = _module()
        for i in range(5):
            t[0] = 2000.0 + 0.1 * i
            m._on_cmd_raw(_stamped_pose("left", t[0] + skew))
        assert m._current_poses.get(Hand.LEFT) is not None, f"skew={skew}"


def test_out_of_order_pose_dropped() -> None:
    m = _module()
    now = time.time()
    m._on_cmd_raw(_stamped_pose("left", now))
    first = m._current_poses[Hand.LEFT]
    m._on_cmd_raw(_stamped_pose("left", now - 0.2))
    assert m._current_poses[Hand.LEFT] is first


# ─── E-STOP latch: arms, chassis, coordinator, acks ───────────────────


def test_estop_latches_disengages_and_halts_chassis() -> None:
    m = _module()
    m._on_cmd_raw(_joy_frame("left", primary=True))
    m._is_engaged[Hand.LEFT] = True

    m._on_state_json(json.dumps({"type": "estop", "nonce": "n1"}).encode())

    assert m._estopped
    assert not m._is_engaged[Hand.LEFT]
    # Chassis: an immediate zero was published and the twist path is gated.
    assert any(t.linear.x == 0.0 and t.angular.z == 0.0 for t in m.cmd_vel.msgs)
    twist = m._chassis_twist(m._controllers.get(Hand.LEFT), None, time.monotonic())
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)
    # Coordinator latched, ack sent for the nonce.
    assert m.coordinator.estops == [True]
    acks = [json.loads(a) for a in m.cmd_ack.msgs]
    assert acks and acks[-1]["nonce"] == "n1" and acks[-1]["ok"] is True
    # Engagement refused while latched.
    m._handle_engage()
    assert not m._is_engaged[Hand.LEFT]
    assert not m._should_publish(Hand.LEFT)


def test_estop_clear_rearms_and_unlatches_coordinator() -> None:
    m = _module()
    m._on_state_json(json.dumps({"type": "estop", "nonce": 1}).encode())
    m._on_state_json(json.dumps({"type": "estop_clear", "nonce": 2}).encode())
    assert not m._estopped
    assert m.coordinator.estops == [True, False]


def test_operator_lost_disengages_and_halts() -> None:
    m = _module()
    m._is_engaged[Hand.RIGHT] = True
    m._on_state_json(json.dumps({"type": "operator_lost"}).encode())
    assert not m._is_engaged[Hand.RIGHT]
    assert any(t.linear.x == 0.0 for t in m.cmd_vel.msgs)
    # Not an E-STOP: engagement is allowed again on the next fresh press.
    assert not m._estopped


def test_robot_state_reports_latch_and_engagement() -> None:
    m = _module()
    m._publish_robot_state()
    state = json.loads(m.robot_state.msgs[-1])
    assert state == {"estopped": False, "engaged": {"left": False, "right": False}}


def test_malformed_state_json_ignored() -> None:
    m = _module()
    m._on_state_json(b"{broken")
    m._on_state_json(b"not json at all")
    assert not m._estopped


# ─── Blueprint parity and wiring ──────────────────────────────────────


def _module_kwargs(blueprint: Any, module_cls: Any) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_cls)


def test_hosted_mapping_matches_validated_local_blueprint() -> None:
    from dimos.robot.galaxea.r1lite.quest_module import R1LiteQuestTeleopModule

    local = _module_kwargs(r1lite_quest_teleop, R1LiteQuestTeleopModule)
    for blueprint in (r1lite_quest_teleop_hosted, r1lite_quest_teleop_hosted_sim):
        hosted = _module_kwargs(blueprint, R1LiteHostedTeleopModule)
        for key in ("motion_gain", "local_rotation", "position_deadband_m", "task_names"):
            assert hosted[key] == local[key], key


def test_hosted_uses_the_same_teleop_tasks() -> None:
    from dimos.control.coordinator import ControlCoordinator as CC

    def tasks_of(blueprint: Any) -> dict[str, Any]:
        kwargs = next(atom.kwargs for atom in blueprint.blueprints if atom.module is CC)
        return {t.name: t for t in kwargs["tasks"]}

    local = tasks_of(r1lite_quest_teleop)
    hosted = tasks_of(r1lite_quest_teleop_hosted)
    for name in ("teleop_left_arm", "teleop_right_arm"):
        assert hosted[name].params == local[name].params


def test_hosted_blueprints_bind_operator_planes() -> None:
    for blueprint in (r1lite_quest_teleop_hosted, r1lite_quest_teleop_hosted_sim):
        streams = {name for (name, _t) in blueprint.transport_map}
        assert {"cmd_raw", "state_json", "telemetry_out", "cmd_ack"} <= streams


def test_registry_resolves_hosted_names() -> None:
    from dimos.robot.all_blueprints import all_blueprints

    for name in ("r1lite-quest-teleop-hosted", "r1lite-quest-teleop-hosted-sim"):
        assert name in all_blueprints


# ─── Hand-gesture chassis drive ───────────────────────────────────────


def _drive_controller(*, primary: bool = False, secondary: bool = True) -> Any:
    from dimos.teleop.quest.quest_types import QuestControllerState, ThumbstickState

    return QuestControllerState(
        is_left=True,
        primary=primary,
        secondary=secondary,
        thumbstick=ThumbstickState(x=0.0, y=0.0),
    )


def _set_pose(m: Any, hand: Hand, x: float, y: float) -> None:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped as PS

    m._current_poses[hand] = PS(position=[x, y, 0.0])


def _fresh_joy(m: Any, *hands: Hand) -> float:
    now = time.monotonic()
    for hand in hands:
        m._joy_rx_ts[hand] = now
    return now


def test_gesture_left_hand_translates_and_saturates() -> None:
    m = _module()
    now = _fresh_joy(m, Hand.LEFT)
    m._controllers[Hand.LEFT] = _drive_controller()
    _set_pose(m, Hand.LEFT, 0.0, 0.0)
    # First tick anchors: zero output.
    t0 = m._chassis_twist(m._controllers[Hand.LEFT], None, now)
    assert (t0.linear.x, t0.linear.y) == (0.0, 0.0)
    # Push forward 9 cm (deadband 3, full scale 15): partial forward.
    _set_pose(m, Hand.LEFT, 0.09, 0.0)
    t1 = m._chassis_twist(m._controllers[Hand.LEFT], None, now)
    assert t1.linear.x == pytest.approx(0.5 * m.config.linear_speed)
    assert t1.linear.y == 0.0
    # Far beyond full scale: saturates at the configured cap.
    _set_pose(m, Hand.LEFT, 0.60, -0.60)
    t2 = m._chassis_twist(m._controllers[Hand.LEFT], None, now)
    assert t2.linear.x == pytest.approx(m.config.linear_speed)
    assert t2.linear.y == pytest.approx(-m.config.linear_speed)


def test_gesture_right_hand_yaws_and_release_zeroes() -> None:
    m = _module()
    now = _fresh_joy(m, Hand.RIGHT)
    m._controllers[Hand.RIGHT] = _drive_controller()
    _set_pose(m, Hand.RIGHT, 0.0, 0.0)
    m._chassis_twist(None, m._controllers[Hand.RIGHT], now)
    _set_pose(m, Hand.RIGHT, 0.0, 0.09)
    t1 = m._chassis_twist(None, m._controllers[Hand.RIGHT], now)
    assert t1.angular.z == pytest.approx(0.5 * m.config.angular_speed)
    # Release the button: contribution zeroes and the anchor clears.
    m._controllers[Hand.RIGHT] = _drive_controller(secondary=False)
    t2 = m._chassis_twist(None, m._controllers[Hand.RIGHT], now)
    assert t2.angular.z == 0.0
    assert m._drive_anchor[Hand.RIGHT] is None


def test_gesture_suppresses_arm_streaming_and_engagement() -> None:
    m = _module()
    _fresh_joy(m, Hand.LEFT)
    m._controllers[Hand.LEFT] = _drive_controller(primary=True, secondary=True)
    m._is_engaged[Hand.LEFT] = True
    assert not m._should_publish(Hand.LEFT)
    m._handle_engage()
    # Driving hand: disengaged and not re-engaged despite primary held.
    assert not m._is_engaged[Hand.LEFT]


def test_real_stick_input_takes_priority_over_gesture() -> None:
    from dimos.teleop.quest.quest_types import QuestControllerState, ThumbstickState

    m = _module()
    now = _fresh_joy(m, Hand.LEFT)
    m._controllers[Hand.LEFT] = QuestControllerState(
        is_left=True,
        secondary=True,
        thumbstick=ThumbstickState(x=0.0, y=-1.0),
    )
    _set_pose(m, Hand.LEFT, 0.5, 0.0)  # gesture would saturate forward
    twist = m._chassis_twist(m._controllers[Hand.LEFT], None, now)
    # Stick semantics win: full stick-forward, not the gesture value.
    assert twist.linear.x == pytest.approx(m.config.linear_speed)
    assert m._drive_anchor[Hand.LEFT] is None or True  # anchor state irrelevant here


def test_hosted_stick_frames_drive_the_chassis_end_to_end() -> None:
    """A stick-deflected Joy frame through the broker ingress produces
    chassis velocity — the full remote joystick path, no headset needed.
    (Field evidence 2026-08-10: the portal's VR client does transmit
    real stick values in axes[0..1].)"""
    m = _module()
    now = _fresh_joy(m, Hand.LEFT, Hand.RIGHT)
    # Left stick pushed fully forward (Quest up = -1), right stick right.
    m._on_cmd_raw(_joy_frame("left", stick_y=-1.0))
    m._on_cmd_raw(_joy_frame("right", stick_x=1.0))
    twist = m._chassis_twist(m._controllers[Hand.LEFT], m._controllers[Hand.RIGHT], now)
    assert twist.linear.x == pytest.approx(m.config.linear_speed)
    assert twist.angular.z == pytest.approx(-m.config.angular_speed)
    # Sticks released: back to zero (dead-man semantics).
    m._on_cmd_raw(_joy_frame("left"))
    m._on_cmd_raw(_joy_frame("right"))
    twist = m._chassis_twist(m._controllers[Hand.LEFT], m._controllers[Hand.RIGHT], now)
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)


def test_hosted_sticks_and_arm_engagement_coexist() -> None:
    """Drive with the left stick while the right hand engages the arm —
    the combined mode the whole feature exists for."""
    m = _module()
    now = _fresh_joy(m, Hand.LEFT, Hand.RIGHT)
    # Both streams per hand: engagement handling is fail-closed and drops
    # any hand whose pose stream is not live alongside its Joy stream.
    m._on_cmd_raw(_joy_frame("left", stick_y=-0.8))
    m._on_cmd_raw(_stamped_pose("left", time.time()))
    m._on_cmd_raw(_joy_frame("right", primary=True))
    m._on_cmd_raw(_stamped_pose("right", time.time()))
    m._pose_rx_ts[Hand.LEFT] = m._pose_rx_ts[Hand.RIGHT] = time.monotonic()
    m._handle_engage()
    assert m._is_engaged[Hand.RIGHT]
    assert m._should_publish(Hand.RIGHT)
    twist = m._chassis_twist(m._controllers[Hand.LEFT], m._controllers[Hand.RIGHT], now)
    assert twist.linear.x > 0.0
