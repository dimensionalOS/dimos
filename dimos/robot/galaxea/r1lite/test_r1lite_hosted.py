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


def test_stale_pose_dropped() -> None:
    m = _module()
    m._on_cmd_raw(_stamped_pose("left", time.time() - 5.0))
    assert m._current_poses.get(Hand.LEFT) is None


def test_future_pose_dropped_without_advancing_watermark() -> None:
    m = _module()
    m._on_cmd_raw(_stamped_pose("left", time.time() + 60.0))
    assert m._current_poses.get(Hand.LEFT) is None
    # A fresh, sane pose afterwards must still be accepted.
    m._on_cmd_raw(_stamped_pose("left", time.time()))
    assert m._current_poses.get(Hand.LEFT) is not None


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
