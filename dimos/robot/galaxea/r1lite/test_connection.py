# Copyright 2025-2026 Dimensional Inc.
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

"""Deterministic unit tests for R1LiteConnection.

Run without ROS or hardware: RawROS is faked and the ROS message modules the
handlers import lazily are injected into sys.modules.
"""

from __future__ import annotations

import ast
from pathlib import Path
import sys
import time
import types
from typing import Any

import pytest

import dimos.core.module as module_mod
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.galaxea.r1lite import connection as conn_mod
from dimos.robot.galaxea.r1lite.connection import R1LiteConnection

_CONN_SRC = Path(conn_mod.__file__)


class _Msg:
    def __init__(self, **kw: Any) -> None:
        self.__dict__.update(kw)


class _FakeRos:
    def __init__(self, stamp_available: bool = True) -> None:
        self.published: list[tuple[Any, Any]] = []
        self._stamp_available = stamp_available

    def now_stamp(self) -> Any:
        if not self._stamp_available:
            return None
        return _Msg(sec=0, nanosec=0)

    def publish(self, topic: Any, message: Any) -> None:
        self.published.append((topic, message))

    def stop(self) -> None:
        pass


def _ns(**kw: Any) -> types.SimpleNamespace:
    return types.SimpleNamespace(**kw)


@pytest.fixture(autouse=True)
def _fake_ros_msgs(monkeypatch: Any) -> None:
    class _RosJointState:
        def __init__(self) -> None:
            self.header = _ns(stamp=None)
            self.name: list[str] = []
            self.position: list[float] = []
            self.velocity: list[float] = []
            self.effort: list[float] = []

    class _TwistStamped:
        def __init__(self) -> None:
            self.header = _ns(stamp=None)
            self.twist = _ns(
                linear=_ns(x=0.0, y=0.0, z=0.0),
                angular=_ns(x=0.0, y=0.0, z=0.0),
            )

    class _Bool:
        def __init__(self, data: bool = False) -> None:
            self.data = data

    for mod_name, attrs in (
        ("sensor_msgs", {"JointState": _RosJointState}),
        ("geometry_msgs", {"TwistStamped": _TwistStamped}),
        ("std_msgs", {"Bool": _Bool}),
    ):
        top = types.ModuleType(mod_name)
        sub = types.ModuleType(f"{mod_name}.msg")
        for k, v in attrs.items():
            setattr(sub, k, v)
        top.msg = sub  # type: ignore[attr-defined]
        monkeypatch.setitem(sys.modules, mod_name, top)
        monkeypatch.setitem(sys.modules, f"{mod_name}.msg", sub)


class _NoRpc(RPCSpec):
    """Module.__init__ treats a ValueError from the rpc factory as rpc disabled."""

    def __init__(self, **kw: Any) -> None:
        raise ValueError("rpc disabled for unit tests")


@pytest.fixture(autouse=True)
def _no_background_threads(monkeypatch: Any) -> None:
    monkeypatch.setattr(module_mod, "get_loop", lambda: (types.SimpleNamespace(), None))


def _bare(stamp_available: bool = True) -> R1LiteConnection:
    c = R1LiteConnection(rpc_transport=_NoRpc)
    c._ros = _FakeRos(stamp_available=stamp_available)
    c._cmd_left_topic = "left"
    c._cmd_right_topic = "right"
    c._cmd_gripper_left_topic = "gl"
    c._cmd_gripper_right_topic = "gr"
    c._speed_topic = "speed"
    c._acc_topic = "acc"
    c._brake_topic = "brake"
    now = time.monotonic()
    c._torso_seen = c._left_seen = c._right_seen = True
    c._torso_ts = c._left_ts = c._right_ts = now
    return c


class _LogCapture:
    def __init__(self) -> None:
        self.messages: list[str] = []

    def _record(self, msg: str, *args: Any) -> None:
        self.messages.append(msg % args if args else msg)

    def debug(self, msg: str, *args: Any, **kw: Any) -> None:
        self._record(msg, *args)

    info = warning = error = exception = debug


def _motor_cmd(num_joints: int = 16) -> Any:
    q = [float(i) for i in range(num_joints)]
    return _Msg(num_joints=num_joints, q=q, dq=[0.0] * num_joints)


def _arm_commands(c: R1LiteConnection) -> list[Any]:
    return [m for t, m in c._ros.published if t in ("left", "right")]


def test_no_module_level_ros_import() -> None:
    tree = ast.parse(_CONN_SRC.read_text())
    for node in tree.body:
        names = []
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = [node.module or ""]
        for n in names:
            assert not n.startswith(("rclpy", "sensor_msgs", "geometry_msgs", "std_msgs")), (
                f"module-level ROS import {n} would break `import dimos` without ROS"
            )


def test_copy_segment_rejects_wrong_position_length() -> None:
    dst = [0.0] * 6
    ok = R1LiteConnection._copy_segment(
        _Msg(position=[1.0] * 5, velocity=[], effort=[]), dst, [0.0] * 6, [0.0] * 6
    )
    assert ok is False
    assert dst == [0.0] * 6


def test_copy_segment_rejects_wrong_velocity_length() -> None:
    dst = [0.0] * 6
    dq = [9.0] * 6
    ok = R1LiteConnection._copy_segment(
        _Msg(position=[1.0] * 6, velocity=[2.0] * 4, effort=[]), dst, dq, [0.0] * 6
    )
    assert ok is False
    assert dst == [0.0] * 6
    assert dq == [9.0] * 6


def test_copy_segment_accepts_exact_lengths() -> None:
    dst = [0.0] * 6
    ok = R1LiteConnection._copy_segment(
        _Msg(position=[1.0] * 6, velocity=[2.0] * 6, effort=[3.0] * 6),
        dst,
        [0.0] * 6,
        [0.0] * 6,
    )
    assert ok is True
    assert dst == [1.0] * 6


def test_tracking_velocities_zero_maps_to_speed() -> None:
    c = _bare()
    c.config.tracking_speed = 0.7
    assert c._tracking_velocities([0.0, 0.0]) == [0.7, 0.7]
    assert c._tracking_velocities([1.5]) == [1.5]


def test_stale_segments_never_seen_is_stale() -> None:
    c = _bare()
    c._torso_seen = c._left_seen = c._right_seen = False
    assert c._stale_segments(time.monotonic()) == (True, True, True)


def test_stale_segments_old_timestamp_is_stale() -> None:
    c = _bare()
    c._left_ts = time.monotonic() - 10.0
    torso_stale, left_stale, right_stale = c._stale_segments(time.monotonic())
    assert left_stale and not torso_stale and not right_stale


def test_motor_command_wrong_count_ignored() -> None:
    c = _bare()
    c._on_motor_command(_motor_cmd(num_joints=12))
    assert _arm_commands(c) == []


def test_motor_command_maps_arms_and_drops_torso() -> None:
    c = _bare()
    c._on_motor_command(_motor_cmd())
    by_topic = {t: m for t, m in c._ros.published}
    assert set(by_topic) == {"left", "right"}
    assert by_topic["left"].position == [4.0, 5.0, 6.0, 7.0, 8.0, 9.0]
    assert by_topic["right"].position == [10.0, 11.0, 12.0, 13.0, 14.0, 15.0]


def test_motor_command_dropped_when_feedback_never_seen() -> None:
    c = _bare()
    c._left_seen = False
    c._on_motor_command(_motor_cmd())
    assert _arm_commands(c) == []


def test_motor_command_dropped_when_feedback_stale() -> None:
    c = _bare()
    c._right_ts = time.monotonic() - 10.0
    c._on_motor_command(_motor_cmd())
    assert _arm_commands(c) == []


def test_motor_command_resumes_after_fresh_feedback() -> None:
    c = _bare()
    c._left_ts = time.monotonic() - 10.0
    c._on_motor_command(_motor_cmd())
    assert _arm_commands(c) == []
    c._left_ts = time.monotonic()
    c._on_motor_command(_motor_cmd())
    assert len(_arm_commands(c)) == 2


def test_motor_command_dropped_without_stamp() -> None:
    c = _bare(stamp_available=False)
    c._on_motor_command(_motor_cmd())
    assert _arm_commands(c) == []


def test_publish_chassis_command_publishes_all_three() -> None:
    c = _bare()
    assert c._publish_chassis_command(conn_mod.Twist()) is True
    topics = [t for t, _ in c._ros.published]
    assert topics == ["acc", "brake", "speed"]


def test_publish_chassis_command_reports_failure_without_topics() -> None:
    c = _bare()
    c._speed_topic = None
    assert c._publish_chassis_command(conn_mod.Twist()) is False
    assert c._ros.published == []


def test_stream_chassis_zero_publishes_through_ros() -> None:
    c = _bare()
    c.config.publish_rate_hz = 100.0
    c._last_chassis_fb_ts = time.monotonic() + 60.0
    c._stream_chassis_zero(0.05)
    speed_msgs = [m for t, m in c._ros.published if t == "speed"]
    assert len(speed_msgs) >= 2
    assert all(m.twist.linear.x == 0.0 for m in speed_msgs)


def test_stream_chassis_zero_unconfirmed_without_fresh_feedback(monkeypatch: Any) -> None:
    c = _bare()
    c.config.publish_rate_hz = 100.0
    c._last_chassis_fb_ts = 0.0
    cap = _LogCapture()
    monkeypatch.setattr(conn_mod, "logger", cap)
    c._stream_chassis_zero(0.03)
    assert any("unconfirmed" in m for m in cap.messages)


def test_stream_chassis_zero_reports_not_settled(monkeypatch: Any) -> None:
    c = _bare()
    c.config.publish_rate_hz = 100.0
    c._last_chassis_fb_ts = time.monotonic() + 60.0
    c._last_chassis_ang = 0.5
    cap = _LogCapture()
    monkeypatch.setattr(conn_mod, "logger", cap)
    c._stream_chassis_zero(0.03)
    assert any("not settled" in m for m in cap.messages)


def test_stream_chassis_zero_no_publication_logs_latched(monkeypatch: Any) -> None:
    c = _bare()
    c.config.publish_rate_hz = 100.0
    c._speed_topic = None
    cap = _LogCapture()
    monkeypatch.setattr(conn_mod, "logger", cap)
    c._stream_chassis_zero(0.03)
    assert any("may be latched" in m for m in cap.messages)


def test_on_chassis_speed_records_feedback_even_without_odom() -> None:
    c = _bare()
    c.config.publish_odom = False
    msg = _Msg(
        header=_ns(stamp=_ns(sec=1, nanosec=0)),
        twist=_ns(linear=_ns(x=0.3, y=0.0, z=0.0), angular=_ns(x=0.0, y=0.0, z=0.2)),
    )
    c._on_chassis_speed(msg, None)
    assert c._last_chassis_lin == pytest.approx(0.3)
    assert c._last_chassis_ang == pytest.approx(0.2)
    assert c._last_chassis_fb_ts > 0.0


def test_single_use_guard() -> None:
    c = _bare()
    c._stop_event.set()
    with pytest.raises(RuntimeError):
        c._check_single_use()


def test_gripper_out_of_range_ignored() -> None:
    c = _bare()
    c._on_gripper_command("left", _Msg(position=[250.0]))
    assert c._ros.published == []
