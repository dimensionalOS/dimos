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

Run without ROS or hardware: RawROS is faked and the ROS message modules
the handlers import lazily are injected into sys.modules. The publish
loop thread never starts; tests drive single ticks directly.
"""

from __future__ import annotations

import ast
import math
from pathlib import Path
import queue
import sys
import threading
import time
import types
from typing import Any

import cv2
import numpy as np
import pytest

import dimos.core.module as module_mod
from dimos.hardware.whole_body.spec import VEL_STOP
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.galaxea.r1lite import config as cfg, connection as conn_mod
from dimos.robot.galaxea.r1lite.connection import ConnectionState, R1LiteConnection

_CONN_SRC = Path(conn_mod.__file__)


class _Msg:
    def __init__(self, **kw: Any) -> None:
        self.__dict__.update(kw)


class _FakeRos:
    def __init__(self, stamp_available: bool = True) -> None:
        self.published: list[tuple[Any, Any]] = []
        self.claimed: list[Any] = []
        self._stamp_available = stamp_available

    def now_stamp(self) -> Any:
        if not self._stamp_available:
            return None
        return _Msg(sec=0, nanosec=0)

    def ensure_publisher(self, topic: Any) -> None:
        self.claimed.append(topic)

    def publish(self, topic: Any, message: Any) -> None:
        self.published.append((topic, message))

    def stop(self) -> None:
        pass


class _FakeOut:
    def __init__(self) -> None:
        self.msgs: list[Any] = []

    def publish(self, msg: Any) -> None:
        self.msgs.append(msg)


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


def _fresh_segment(segment: Any, values: list[float] | None = None) -> None:
    segment.seen = True
    segment.rx_monotonic = time.monotonic()
    segment.stamp = time.time()
    if values is not None:
        segment.q[:] = values


def _construct() -> R1LiteConnection:
    c = R1LiteConnection(rpc_transport=_NoRpc)
    # Module.__init__ leaves rpc unset when the factory raises ValueError;
    # Module.stop() reads it.
    c.rpc = None  # type: ignore[assignment]
    return c


def _bare(state: ConnectionState = ConnectionState.READY_DISARMED) -> R1LiteConnection:
    c = _construct()
    c._ros = _FakeRos()  # type: ignore[assignment]
    c._cmd_left_topic = "left"  # type: ignore[assignment]
    c._cmd_right_topic = "right"  # type: ignore[assignment]
    c._cmd_gripper_left_topic = "gl"  # type: ignore[assignment]
    c._cmd_gripper_right_topic = "gr"  # type: ignore[assignment]
    c._speed_topic = "speed"  # type: ignore[assignment]
    c._acc_topic = "acc"  # type: ignore[assignment]
    c._brake_topic = "brake"  # type: ignore[assignment]
    _fresh_segment(c._left, [0.1] * 6)
    _fresh_segment(c._right, [0.2] * 6)
    _fresh_segment(c._torso, [0.3] * 4)
    now = time.monotonic()
    c._gripper_fb_seen = {"left": True, "right": True}
    c._gripper_fb_rx = {"left": now, "right": now}
    c._chassis_speed_seen = True
    c._last_chassis_fb_ts = now
    c._state = state
    c._arming_nonce = "abc123"
    for stream in (
        "motor_states",
        "torso_states",
        "imu_chassis",
        "imu_torso",
        "gripper_left_state",
        "gripper_right_state",
        "odom",
        "connection_status",
    ):
        setattr(c, stream, _FakeOut())
    return c


def _armed(c: R1LiteConnection) -> R1LiteConnection:
    c._on_arming(String(data="ARM RC5 abc123"))
    assert c._armed
    return c


def _motor_cmd(num_joints: int = 12) -> Any:
    q = [float(i) for i in range(num_joints)]
    return _Msg(num_joints=num_joints, q=q, dq=[0.0] * num_joints)


def _vendor_arm_msgs(c: R1LiteConnection) -> list[tuple[Any, Any]]:
    return [(t, m) for t, m in c._ros.published if t in ("left", "right")]  # type: ignore[union-attr]


def _fb(position: list[float], velocity: list[float] | None = None, stamp_sec: int = 5) -> Any:
    return _Msg(
        header=_ns(stamp=_ns(sec=stamp_sec, nanosec=0)),
        position=position,
        velocity=velocity if velocity is not None else [],
        effort=[],
    )


# Wire contract


def test_motor_command_12_joints_sliced_left_then_right() -> None:
    c = _armed(_bare())
    c._on_motor_command(_motor_cmd())
    assert c._run_one_tick()
    msgs = _vendor_arm_msgs(c)
    assert [t for t, _ in msgs] == ["left", "right"]
    assert msgs[0][1].position == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert msgs[1][1].position == [6.0, 7.0, 8.0, 9.0, 10.0, 11.0]


@pytest.mark.parametrize("count", [11, 13, 16])
def test_motor_command_wrong_length_rejected_whole(count: int) -> None:
    c = _armed(_bare())
    c._on_motor_command(_motor_cmd(count))
    assert c._arm_cmd is None
    assert c._run_one_tick()
    assert _vendor_arm_msgs(c) == []


def test_motor_states_is_12_joints_left_then_right() -> None:
    c = _bare()
    c._publish_feedback_streams()
    out = c.motor_states  # type: ignore[assignment]
    assert len(out.msgs) == 1
    msg = out.msgs[0]
    assert msg.name == cfg.R1LITE_ARM_JOINTS
    assert msg.position == [0.1] * 6 + [0.2] * 6


def test_torso_stream_separate_and_isolated() -> None:
    c = _bare()
    c._publish_feedback_streams()
    assert c.torso_states.msgs[0].name == cfg.R1LITE_TORSO_JOINTS
    # Torso staleness never pauses motor_states.
    c._torso.rx_monotonic -= 10.0
    c._publish_feedback_streams()
    assert len(c.motor_states.msgs) == 2
    assert len(c.torso_states.msgs) == 1


def test_arm_staleness_pauses_motor_states() -> None:
    c = _bare()
    c._left.rx_monotonic -= 10.0
    c._publish_feedback_streams()
    assert c.motor_states.msgs == []


def test_malformed_feedback_rejected_without_partial_copy() -> None:
    c = _bare()
    before = list(c._left.q)
    c._on_arm_feedback("left", _fb([1.0] * 5))
    assert c._left.q == before
    c._on_arm_feedback("left", _fb([9.0] * 6, velocity=[1.0] * 4))
    assert c._left.q == before


# Timestamps


def test_motor_states_ts_is_older_arm_stamp() -> None:
    c = _bare()
    c._left.stamp = 100.0
    c._right.stamp = 90.0
    c._publish_feedback_streams()
    assert c.motor_states.msgs[0].ts == 90.0


def test_zero_vendor_stamp_falls_back_and_counts() -> None:
    c = _bare()
    c._on_arm_feedback("left", _fb([1.0] * 6, stamp_sec=0))
    assert c._stamp_fallbacks["arm_left"] == 1
    assert c._left.stamp > 1e9


def test_vendor_stamp_preserved() -> None:
    c = _bare()
    c._on_arm_feedback("left", _fb([1.0] * 6, stamp_sec=42))
    assert c._left.stamp == 42.0


def test_gripper_feedback_passthrough_stamp_and_frame() -> None:
    c = _bare()
    msg = _fb([55.0], stamp_sec=7)
    msg.header.frame_id = "vendor_gripper_frame"
    c._on_gripper_feedback("left", msg)
    out = c.gripper_left_state.msgs[0]
    assert out.ts == 7.0
    assert out.frame_id == "vendor_gripper_frame"
    assert out.position == [55.0]


def test_gripper_zero_stamp_counts_fallback() -> None:
    c = _bare()
    c._on_gripper_feedback("right", _fb([10.0], stamp_sec=0))
    assert c._stamp_fallbacks["gripper_right"] == 1


def test_torso_stream_preserves_vendor_frame() -> None:
    c = _bare()
    msg = _fb([0.0] * 4, stamp_sec=3)
    msg.header.frame_id = "vendor_torso_frame"
    c._on_torso_feedback(msg, None)
    c._publish_feedback_streams()
    assert c.torso_states.msgs[0].frame_id == "vendor_torso_frame"


def test_odom_zero_stamp_counts_fallback() -> None:
    c = _bare()

    def speed(sec: int) -> Any:
        return _Msg(
            header=_ns(stamp=_ns(sec=sec, nanosec=0)),
            twist=_ns(linear=_ns(x=1.0, y=0.0, z=0.0), angular=_ns(x=0.0, y=0.0, z=0.0)),
        )

    c._on_chassis_speed(speed(0), None)
    assert c._stamp_fallbacks["odom"] == 1


def test_odom_derived_from_chassis_speed_with_vendor_stamp() -> None:
    c = _bare()

    def speed(sec: int, vx: float) -> Any:
        return _Msg(
            header=_ns(stamp=_ns(sec=sec, nanosec=0)),
            twist=_ns(linear=_ns(x=vx, y=0.0, z=0.0), angular=_ns(x=0.0, y=0.0, z=0.0)),
        )

    c._on_chassis_speed(speed(10, 1.0), None)
    c._on_chassis_speed(speed(11, 1.0), None)
    assert len(c.odom.msgs) == 1
    pose = c.odom.msgs[0]
    assert pose.frame_id == "odom"
    assert pose.ts == 11.0
    assert pose.position.x == pytest.approx(1.0)


# Arming and disarmed behavior


def test_disarmed_drops_all_actuator_inputs() -> None:
    c = _bare()
    c._on_motor_command(_motor_cmd())
    c._on_cmd_vel(Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
    c._on_gripper_command("left", JointState(position=[50.0]))
    assert c._arm_cmd is None
    assert c._latest_cmd_vel is None
    assert c._gripper_targets["left"] is None
    assert c._run_one_tick()
    assert c._ros.published == []  # type: ignore[union-attr]


def test_arm_requires_exact_nonce_and_state() -> None:
    c = _bare()
    c._on_arming(String(data="ARM RC5 wrong"))
    assert not c._armed
    c._on_arming(String(data="ARM abc123"))
    assert not c._armed
    c._on_arming(String(data="ARM RC5 abc123"))
    assert c._armed
    assert c._state is ConnectionState.ARMED
    # The used nonce never arms again.
    assert c._arming_nonce != "abc123"


def test_arm_rejected_when_feedback_stale() -> None:
    c = _bare()
    c._left.rx_monotonic -= 10.0
    c._on_arming(String(data="ARM RC5 abc123"))
    assert not c._armed


def test_disarm_returns_to_ready_and_can_rearm() -> None:
    c = _armed(_bare())
    c._on_motor_command(_motor_cmd())
    c._on_cmd_vel(Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
    c._on_arming(String(data="DISARM"))
    assert not c._armed
    assert c._state is ConnectionState.READY_DISARMED
    assert c._arm_cmd is None
    assert c._latest_cmd_vel is None
    # A fresh ARM with the rotated nonce works without a restart.
    c._on_arming(String(data=f"ARM RC5 {c._arming_nonce}"))
    assert c._armed
    assert c._state is ConnectionState.ARMED


@pytest.mark.parametrize(
    "make_stale",
    [
        lambda c: setattr(c._left, "rx_monotonic", c._left.rx_monotonic - 10.0),
        lambda c: setattr(c._right, "rx_monotonic", c._right.rx_monotonic - 10.0),
        lambda c: setattr(c._torso, "rx_monotonic", c._torso.rx_monotonic - 10.0),
        lambda c: c._gripper_fb_rx.__setitem__("left", 0.0),
        lambda c: c._gripper_fb_rx.__setitem__("right", 0.0),
        lambda c: setattr(c, "_last_chassis_fb_ts", 0.0),
    ],
    ids=["arm_left", "arm_right", "torso", "gripper_left", "gripper_right", "chassis_speed"],
)
def test_arm_rejected_per_stale_required_source(make_stale: Any) -> None:
    c = _bare()
    make_stale(c)
    c._on_arming(String(data="ARM RC5 abc123"))
    assert not c._armed


@pytest.mark.parametrize(
    "make_stale",
    [
        lambda c: setattr(c._left, "rx_monotonic", 0.0),
        lambda c: setattr(c._torso, "rx_monotonic", 0.0),
        lambda c: c._gripper_fb_rx.__setitem__("right", 0.0),
        lambda c: setattr(c, "_last_chassis_fb_ts", 0.0),
    ],
    ids=["arm_left", "torso", "gripper_right", "chassis_speed"],
)
def test_stale_required_source_while_armed_disarms(make_stale: Any) -> None:
    c = _armed(_bare())
    make_stale(c)
    assert c._run_one_tick()
    assert not c._armed
    assert c._state is ConnectionState.READY_DISARMED


def test_stale_feedback_disarms_and_recovery_never_rearms() -> None:
    c = _armed(_bare())
    c._on_motor_command(_motor_cmd())
    c._left.rx_monotonic -= 10.0
    assert c._run_one_tick()
    assert not c._armed
    assert c._state is ConnectionState.READY_DISARMED
    assert c._arm_cmd is None
    published_before = len(c._ros.published)  # type: ignore[union-attr]
    # Feedback returns fresh: still disarmed, still nothing published.
    _fresh_segment(c._left)
    c._on_motor_command(_motor_cmd())
    assert c._run_one_tick()
    assert not c._armed
    assert len(c._ros.published) == published_before  # type: ignore[union-attr]


# Publish-gate linearization


class _GateProbe:
    """Lock wrapper that reports when a named thread waits to acquire it."""

    def __init__(self, inner: threading.Lock, watch_thread_name: str) -> None:
        self._inner = inner
        self._watch = watch_thread_name
        self.waiting = threading.Event()

    def __enter__(self) -> None:
        if threading.current_thread().name == self._watch:
            self.waiting.set()
        self._inner.acquire()

    def __exit__(self, *exc: Any) -> None:
        self._inner.release()


def test_stop_wins_gate_then_nothing_publishes() -> None:
    c = _armed(_bare())
    c.config.stop_zero_duration_s = 0.01
    c._on_motor_command(_motor_cmd())
    c.stop()
    assert c._state is ConnectionState.STOPPED
    assert _vendor_arm_msgs(c) == []
    assert c._run_one_tick() is False
    assert _vendor_arm_msgs(c) == []


def test_tick_wins_gate_completes_before_stop_transition() -> None:
    c = _armed(_bare())
    c.config.stop_zero_duration_s = 0.01
    c._on_motor_command(_motor_cmd())
    entered = threading.Event()
    release = threading.Event()
    original = c._publish_vendor_commands

    def blocking_publish(snap: Any) -> None:
        entered.set()
        assert release.wait(timeout=5.0)
        original(snap)

    c._publish_vendor_commands = blocking_publish  # type: ignore[method-assign]
    probe = _GateProbe(c._publish_gate, watch_thread_name="stopper")
    c._publish_gate = probe  # type: ignore[assignment]

    tick = threading.Thread(target=c._run_one_tick, name="tick")
    tick.start()
    assert entered.wait(timeout=5.0)

    stopper = threading.Thread(target=c.stop, name="stopper")
    stopper.start()
    # Proof, not sleep: the stop thread has reached the gate and is blocked
    # there while the tick holds it; the transition has not happened.
    assert probe.waiting.wait(timeout=5.0)
    assert c._state is ConnectionState.ARMED
    assert _vendor_arm_msgs(c) == []

    release.set()
    tick.join(timeout=5.0)
    stopper.join(timeout=10.0)
    assert not tick.is_alive() and not stopper.is_alive()
    # The in-flight publication landed, then stop transitioned; no arm
    # command follows the transition (stop's own zero stream may follow).
    assert len(_vendor_arm_msgs(c)) == 2
    assert c._state is ConnectionState.STOPPED
    assert c._run_one_tick() is False
    assert len(_vendor_arm_msgs(c)) == 2


@pytest.mark.parametrize(
    "failing_method",
    ["_publish_vendor_commands", "_publish_feedback_streams", "_publish_telemetry"],
)
def test_tick_failure_disarms_fails_and_zeros_chassis(failing_method: str) -> None:
    c = _armed(_bare())
    c.config.stop_zero_duration_s = 0.02
    c._on_motor_command(_motor_cmd())
    c._on_cmd_vel(Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
    c._telem_tick = int(c.config.publish_rate_hz) - 1

    def boom(*args: Any, **kw: Any) -> None:
        raise RuntimeError("injected publish failure")

    setattr(c, failing_method, boom)
    assert c._safe_tick() is False
    assert not c._armed
    assert c._state is ConnectionState.FAILED
    assert c._arm_cmd is None
    assert c._latest_cmd_vel is None
    speeds = [m for t, m in c._ros.published if t == "speed"]  # type: ignore[union-attr]
    assert speeds, "failure path must attempt a chassis zero stream"
    # A publication authorized before the failure may carry the live
    # command; everything from the failure onward must be zeros.
    assert speeds[-1].twist.linear.x == 0.0
    zero_tail = [m for m in speeds if m.twist.linear.x == 0.0]
    assert len(zero_tail) >= 1
    with pytest.raises(RuntimeError):
        c._stream_chassis_zero(0.01)


# Stop sequence and lifecycle


class _RecordingLogger:
    """The module logger does not propagate to the root logger, so log
    assertions capture through a swapped-in recorder instead of caplog."""

    def __init__(self) -> None:
        self.lines: list[str] = []

    def _record(self, msg: str, *args: Any) -> None:
        self.lines.append(msg % args if args else msg)

    def debug(self, msg: str, *args: Any, **_: Any) -> None:
        self._record(msg, *args)

    info = warning = error = exception = debug


def test_stop_streams_chassis_zero_and_reports_settled(monkeypatch: Any) -> None:
    log = _RecordingLogger()
    monkeypatch.setattr(conn_mod, "logger", log)
    c = _armed(_bare())
    c.config.stop_zero_duration_s = 0.02
    c._on_cmd_vel(Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
    c._last_chassis_fb_ts = time.monotonic() + 10.0
    c._last_chassis_lin = 0.0
    c._last_chassis_ang = 0.0
    c.stop()
    speeds = [m for t, m in c._ros.published if t == "speed"]  # type: ignore[union-attr]
    assert speeds
    assert all(m.twist.linear.x == 0.0 for m in speeds)
    assert c._state is ConnectionState.STOPPED
    assert any("chassis stop settled" in line for line in log.lines)


def test_stop_with_moving_feedback_logs_not_settled(monkeypatch: Any) -> None:
    log = _RecordingLogger()
    monkeypatch.setattr(conn_mod, "logger", log)
    c = _armed(_bare())
    c.config.stop_zero_duration_s = 0.02
    c._last_chassis_fb_ts = time.monotonic() + 10.0
    c._last_chassis_lin = 0.5
    c._last_chassis_ang = 0.0
    c.stop()
    assert any("chassis not settled after stop" in line for line in log.lines)
    assert not any("chassis stop settled" in line for line in log.lines)


def test_stop_zero_path_reserved_for_stop() -> None:
    c = _armed(_bare())
    with pytest.raises(RuntimeError):
        c._stream_chassis_zero(0.01)


def test_release_sensor_context_skips_dead_context(monkeypatch: Any) -> None:
    # An external shutdown may kill the context first; the cleanup stack
    # must not raise on the corpse (2026-07-28 teardown FAILED symptom).
    calls: list[str] = []
    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.shutdown = lambda context=None: calls.append("shutdown")  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    c = _construct()
    c._sensor_context = types.SimpleNamespace(ok=lambda: False)
    c._release_sensor_context()
    assert calls == []
    assert c._sensor_context is None


def test_release_sensor_context_swallows_shutdown_raise(monkeypatch: Any) -> None:
    def _boom(context: Any = None) -> None:
        raise RuntimeError("already shut down")

    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.shutdown = _boom  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    c = _construct()
    c._sensor_context = types.SimpleNamespace(ok=lambda: True)
    c._release_sensor_context()
    assert c._sensor_context is None


def test_stop_idempotent_and_from_created() -> None:
    c = _construct()
    c.stop()
    assert c._state is ConnectionState.STOPPED
    c.stop()
    assert c._state is ConnectionState.STOPPED


def test_single_use_start_after_stop_raises() -> None:
    c = _bare()
    c.config.stop_zero_duration_s = 0.01
    c.stop()
    with pytest.raises(RuntimeError):
        c.start()


class _FakeIn:
    def __init__(self, fail: bool = False) -> None:
        self._fail = fail

    def subscribe(self, _cb: Any) -> Any:
        if self._fail:
            raise RuntimeError("injected: input subscribe")
        return lambda: None


class _RosHarness:
    """Records every resource the production start path creates/releases."""

    def __init__(self, fail_at: str | None) -> None:
        self.fail_at = fail_at
        self.events: list[str] = []


def _install_fake_ros_stack(monkeypatch: Any, harness: _RosHarness) -> None:
    h = harness

    class FakeRawROSTopic:
        def __init__(self, name: str, _msg_type: Any, qos: Any = None) -> None:
            self.name = name

    class FakeRawROS:
        def __init__(self, node_name: str) -> None:
            h.events.append(f"ros_created:{node_name}")

        def start(self) -> None:
            if h.fail_at == "ros_start":
                raise RuntimeError("injected: ros start")
            h.events.append("ros_started")

        def subscribe(self, topic: Any, _cb: Any) -> Any:
            if h.fail_at == f"sub:{topic.name}":
                raise RuntimeError(f"injected: subscribe {topic.name}")
            h.events.append(f"subscribed:{topic.name}")
            return lambda: h.events.append(f"unsubscribed:{topic.name}")

        def ensure_publisher(self, topic: Any) -> None:
            h.events.append(f"claimed:{topic.name}")

        def stop(self) -> None:
            h.events.append("ros_stopped")

        def now_stamp(self) -> Any:
            return _Msg(sec=1, nanosec=0)

        def publish(self, topic: Any, message: Any) -> None:
            h.events.append(f"published:{getattr(topic, 'name', topic)}")

    fake_rospubsub = types.ModuleType("dimos.protocol.pubsub.impl.rospubsub")
    fake_rospubsub.RawROS = FakeRawROS  # type: ignore[attr-defined]
    fake_rospubsub.RawROSTopic = FakeRawROSTopic  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "dimos.protocol.pubsub.impl.rospubsub", fake_rospubsub)

    class FakeContext:
        def ok(self) -> bool:
            return True

    class FakeNode:
        def __init__(self, _name: str, context: Any = None) -> None:
            if h.fail_at == "sensor_node":
                raise RuntimeError("injected: sensor node")
            h.events.append("sensor_node_created")

        def create_subscription(self, *_a: Any, **_kw: Any) -> Any:
            if h.fail_at == "sensor_subscription":
                raise RuntimeError("injected: sensor subscription")
            h.events.append("sensor_subscription")
            return object()

        def destroy_node(self) -> None:
            h.events.append("sensor_node_destroyed")

    class FakeExecutor:
        def __init__(self, num_threads: int = 1, context: Any = None) -> None:
            if h.fail_at == "sensor_executor":
                raise RuntimeError("injected: sensor executor")
            h.events.append("sensor_executor_created")

        def add_node(self, _node: Any) -> None:
            if h.fail_at == "sensor_add_node":
                raise RuntimeError("injected: executor add_node")

        def shutdown(self, timeout_sec: float = 0.0) -> None:
            h.events.append("sensor_executor_shutdown")

        def spin_once(self, timeout_sec: float = 0.0) -> None:
            time.sleep(0.01)

    fake_rclpy = types.ModuleType("rclpy")

    def _init(context: Any = None) -> None:
        if h.fail_at == "sensor_context":
            raise RuntimeError("injected: sensor context")
        h.events.append("rclpy_init")

    def _shutdown(context: Any = None) -> None:
        h.events.append("rclpy_shutdown")

    fake_rclpy.init = _init  # type: ignore[attr-defined]
    fake_rclpy.shutdown = _shutdown  # type: ignore[attr-defined]
    ctx_mod = types.ModuleType("rclpy.context")
    ctx_mod.Context = FakeContext  # type: ignore[attr-defined]
    exec_mod = types.ModuleType("rclpy.executors")
    exec_mod.MultiThreadedExecutor = FakeExecutor  # type: ignore[attr-defined]
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = FakeNode  # type: ignore[attr-defined]
    qos_mod = types.ModuleType("rclpy.qos")

    class _QoSProfile:
        def __init__(self, **kw: Any) -> None:
            pass

    class _Policy:
        BEST_EFFORT = RELIABLE = VOLATILE = 0

    qos_mod.QoSProfile = _QoSProfile  # type: ignore[attr-defined]
    qos_mod.ReliabilityPolicy = _Policy  # type: ignore[attr-defined]
    qos_mod.DurabilityPolicy = _Policy  # type: ignore[attr-defined]
    for name, mod in (
        ("rclpy", fake_rclpy),
        ("rclpy.context", ctx_mod),
        ("rclpy.executors", exec_mod),
        ("rclpy.node", node_mod),
        ("rclpy.qos", qos_mod),
    ):
        monkeypatch.setitem(sys.modules, name, mod)


def _lifecycle_conn(monkeypatch: Any, harness: _RosHarness) -> R1LiteConnection:
    _install_fake_ros_stack(monkeypatch, harness)
    sensor_msg_mod = sys.modules["sensor_msgs.msg"]
    for name in ("CompressedImage", "Image", "Imu"):
        monkeypatch.setattr(sensor_msg_mod, name, type(name, (), {}), raising=False)
    monkeypatch.setattr(conn_mod, "_FEEDBACK_DISCOVERY_TIMEOUT_S", 0.05)

    real_thread = threading.Thread

    class SelectiveThread(real_thread):
        def start(self) -> None:
            if harness.fail_at == f"thread:{self.name}":
                raise RuntimeError(f"injected: thread start {self.name}")
            super().start()

    monkeypatch.setattr(conn_mod, "Thread", SelectiveThread)

    c = _construct()
    c.config.enable_cameras = False
    for stream in (
        "motor_command",
        "cmd_vel",
        "gripper_left_command",
        "gripper_right_command",
    ):
        setattr(c, stream, _FakeIn())
    c.arming = _FakeIn(fail=harness.fail_at == "input_subscribe")  # type: ignore[assignment]
    for stream in ("connection_status",):
        setattr(c, stream, _FakeOut())
    return c


@pytest.mark.parametrize(
    "fail_at",
    [
        "ros_start",
        "sub:/hdas/feedback_arm_left",
        "sub:/motion_control/chassis_speed",
        "sensor_context",
        "sensor_node",
        "sensor_executor",
        "sensor_add_node",
        "sensor_subscription",
        "thread:r1lite-imu_chassis",
        "thread:r1lite-imu_torso",
        "thread:r1lite-sensor-spin",
        "input_subscribe",
        "thread:r1lite-publish",
    ],
)
def test_partial_start_releases_every_created_resource(monkeypatch: Any, fail_at: str) -> None:
    harness = _RosHarness(fail_at)
    c = _lifecycle_conn(monkeypatch, harness)
    threads_before = {t.name for t in threading.enumerate()}
    with pytest.raises(RuntimeError, match="injected"):
        c.start()
    assert c._state is ConnectionState.FAILED
    assert c._cleanup_stack == []
    # Every successful subscribe was unsubscribed, in reverse order.
    subs = [e.split(":", 1)[1] for e in harness.events if e.startswith("subscribed:")]
    unsubs = [e.split(":", 1)[1] for e in harness.events if e.startswith("unsubscribed:")]
    assert unsubs == list(reversed(subs))
    if "ros_started" in harness.events:
        assert "ros_stopped" in harness.events
    if "rclpy_init" in harness.events:
        assert "rclpy_shutdown" in harness.events
    if "sensor_node_created" in harness.events:
        assert "sensor_node_destroyed" in harness.events
    # No connection-owned thread survives.
    leaked = {t.name for t in threading.enumerate()} - threads_before
    assert not {n for n in leaked if n.startswith("r1lite")}
    c.stop()
    assert c._state is ConnectionState.STOPPED


def test_start_claims_every_actuator_topic_while_disarmed(monkeypatch: Any) -> None:
    # Ownership must be observable before anyone arms: the preflight
    # sole-writer check runs against a DISARMED connection, and ROS
    # publishers are otherwise only created on the first publish.
    harness = _RosHarness(fail_at=None)
    c = _lifecycle_conn(monkeypatch, harness)
    c.config.stop_zero_duration_s = 0.01
    c.start()
    assert c._state is ConnectionState.READY_DISARMED
    claimed = {e.split(":", 1)[1] for e in harness.events if e.startswith("claimed:")}
    assert claimed == {
        cfg.CMD_ARM_LEFT,
        cfg.CMD_ARM_RIGHT,
        cfg.CMD_GRIPPER_LEFT,
        cfg.CMD_GRIPPER_RIGHT,
        cfg.CMD_CHASSIS_SPEED,
        cfg.CMD_CHASSIS_ACC_LIMIT,
        cfg.CMD_BRAKE_MODE,
    }
    # Claiming is not publishing: nothing reached the vendor while disarmed.
    assert [e for e in harness.events if e.startswith("published:")] == []
    c.stop()


def test_full_start_then_stop_releases_everything(monkeypatch: Any) -> None:
    harness = _RosHarness(fail_at=None)
    c = _lifecycle_conn(monkeypatch, harness)
    threads_before = {t.name for t in threading.enumerate()}
    c.config.stop_zero_duration_s = 0.01
    c.start()
    assert c._state is ConnectionState.READY_DISARMED
    c.stop()
    assert c._state is ConnectionState.STOPPED
    assert c._cleanup_stack == []
    subs = [e.split(":", 1)[1] for e in harness.events if e.startswith("subscribed:")]
    unsubs = [e.split(":", 1)[1] for e in harness.events if e.startswith("unsubscribed:")]
    assert unsubs == list(reversed(subs))
    assert "ros_stopped" in harness.events and "rclpy_shutdown" in harness.events
    leaked = {t.name for t in threading.enumerate()} - threads_before
    assert not {n for n in leaked if n.startswith("r1lite")}
    with pytest.raises(RuntimeError):
        c.start()


# Chassis dead-man and command mapping


def test_chassis_streams_fresh_then_zeros_when_stale() -> None:
    c = _armed(_bare())
    c._on_cmd_vel(Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
    assert c._run_one_tick()
    speeds = [m for t, m in c._ros.published if t == "speed"]  # type: ignore[union-attr]
    assert speeds[-1].twist.linear.x == pytest.approx(0.5)
    c._latest_cmd_vel_ts -= 1.0
    assert c._run_one_tick()
    speeds = [m for t, m in c._ros.published if t == "speed"]  # type: ignore[union-attr]
    assert speeds[-1].twist.linear.x == 0.0
    accs = [m for t, m in c._ros.published if t == "acc"]  # type: ignore[union-attr]
    brakes = [m for t, m in c._ros.published if t == "brake"]  # type: ignore[union-attr]
    assert accs and brakes and brakes[-1].data is False


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_cmd_vel_non_finite_rejected_whole(bad: float) -> None:
    c = _armed(_bare())
    c._on_cmd_vel(Twist(linear=Vector3(bad, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
    assert c._latest_cmd_vel is None
    c._on_cmd_vel(Twist(linear=Vector3(0.1, 0.0, 0.0), angular=Vector3(0.0, 0.0, bad)))
    assert c._latest_cmd_vel is None
    assert c._run_one_tick()
    speeds = [m for t, m in c._ros.published if t == "speed"]  # type: ignore[union-attr]
    assert all(m.twist.linear.x == 0.0 and m.twist.angular.z == 0.0 for m in speeds)


def test_cmd_vel_clamped_to_configured_ceilings() -> None:
    c = _armed(_bare())
    c._on_cmd_vel(Twist(linear=Vector3(50.0, -50.0, 3.0), angular=Vector3(1.0, 1.0, -99.0)))
    cached = c._latest_cmd_vel
    assert cached is not None
    assert cached.linear.x == pytest.approx(0.5)
    assert cached.linear.y == pytest.approx(-0.5)
    assert cached.linear.z == 0.0
    assert cached.angular.x == 0.0
    assert cached.angular.y == 0.0
    assert cached.angular.z == pytest.approx(-1.0)
    assert c._run_one_tick()
    speeds = [m for t, m in c._ros.published if t == "speed"]  # type: ignore[union-attr]
    assert speeds[-1].twist.linear.x == pytest.approx(0.5)
    assert speeds[-1].twist.angular.z == pytest.approx(-1.0)


@pytest.mark.parametrize("field,index", [("q", 3), ("q", 11), ("dq", 0), ("dq", 7)])
def test_motor_command_non_finite_rejected_whole(field: str, index: int) -> None:
    c = _armed(_bare())
    for bad in (math.nan, math.inf):
        msg = _motor_cmd()
        getattr(msg, field)[index] = bad
        c._on_motor_command(msg)
        assert c._arm_cmd is None
    assert c._run_one_tick()
    assert _vendor_arm_msgs(c) == []


def test_tracking_velocity_mapping() -> None:
    c = _bare()
    c.config.tracking_speed = 1.25
    assert c._tracking_velocities([0.0, VEL_STOP, 2.0]) == [1.25, 1.25, 2.0]


def test_stale_arm_command_cache_not_republished() -> None:
    c = _armed(_bare())
    c._on_motor_command(_motor_cmd())
    c._arm_cmd_ts -= 1.0
    assert c._run_one_tick()
    assert _vendor_arm_msgs(c) == []


def test_gripper_range_validated_and_streamed_while_fresh() -> None:
    c = _armed(_bare())
    c._on_gripper_command("left", JointState(position=[150.0]))
    assert c._gripper_targets["left"] is None
    c._on_gripper_command("left", JointState(position=[50.0]))
    assert c._run_one_tick()
    gl = [m for t, m in c._ros.published if t == "gl"]  # type: ignore[union-attr]
    assert gl and gl[0].position == [50.0]


def test_stop_with_stuck_publish_thread_ends_failed_not_stopped(monkeypatch: Any) -> None:
    c = _armed(_bare())
    c.config.stop_zero_duration_s = 0.01
    monkeypatch.setattr(conn_mod, "DEFAULT_THREAD_JOIN_TIMEOUT", 0.05)
    release = threading.Event()
    stuck = threading.Thread(target=release.wait, name="r1lite-publish-stuck")
    stuck.start()
    c._publish_thread = stuck
    try:
        c.stop()
        assert c._state is ConnectionState.FAILED
        assert c._publish_thread is stuck
        # Resources the thread can reach were not torn down.
        assert c._ros is not None
        # The safety zero still ran: STOPPING made ordinary publication inert.
        speeds = [m for t, m in c._ros.published if t == "speed"]  # type: ignore[union-attr]
        assert speeds and speeds[-1].twist.linear.x == 0.0
    finally:
        release.set()
        stuck.join(timeout=2.0)


def _stuck_thread(release: threading.Event, name: str) -> threading.Thread:
    t = threading.Thread(target=release.wait, name=name)
    t.start()
    return t


@pytest.mark.parametrize("which", ["spin", "worker"])
def test_stuck_sensor_thread_halts_teardown_and_fails(monkeypatch: Any, which: str) -> None:
    c = _armed(_bare())
    c.config.stop_zero_duration_s = 0.01
    monkeypatch.setattr(conn_mod, "DEFAULT_THREAD_JOIN_TIMEOUT", 0.05)
    release = threading.Event()
    sentinel_ran: list[str] = []
    # The sentinel sits BELOW the sensor entry: a surviving sensor thread
    # must halt the unwind before it.
    c._cleanup_stack.append(("sentinel", lambda: sentinel_ran.append("ran")))
    if which == "spin":
        stuck = _stuck_thread(release, "r1lite-sensor-spin-stuck")
        c._sensor_spin_thread = stuck
        c._cleanup_stack.append(("sensor_spin", c._release_sensor_spin))
    else:
        stuck = _stuck_thread(release, "r1lite-worker-stuck")
        c._sensor_workers.append(stuck)
        c._cleanup_stack.append(("sensor_workers", c._release_sensor_workers))
    try:
        c.stop()
        assert c._state is ConnectionState.FAILED
        assert sentinel_ran == []
        assert len(c._cleanup_stack) >= 1
        if which == "spin":
            assert c._sensor_spin_thread is stuck
        else:
            assert stuck in c._sensor_workers
    finally:
        release.set()
        stuck.join(timeout=2.0)


def test_raising_cleanup_halts_teardown_and_fails() -> None:
    # A cleanup exception is incomplete cleanup, never success: the failing
    # entry and everything beneath it stay retained and stop() ends FAILED.
    c = _armed(_bare())
    c.config.stop_zero_duration_s = 0.01
    sentinel_ran: list[str] = []
    c._cleanup_stack.append(("sentinel", lambda: sentinel_ran.append("ran")))

    def boom() -> None:
        raise RuntimeError("injected cleanup failure")

    c._cleanup_stack.append(("boom", boom))
    c.stop()
    assert c._state is ConnectionState.FAILED
    assert sentinel_ran == []
    assert [name for name, _ in c._cleanup_stack] == ["sentinel", "boom"]


def test_second_worker_start_failure_joins_started_first_worker(
    monkeypatch: Any,
) -> None:
    harness = _RosHarness("thread:r1lite-imu_torso")
    c = _lifecycle_conn(monkeypatch, harness)
    with pytest.raises(RuntimeError, match="injected"):
        c.start()
    assert c._state is ConnectionState.FAILED
    assert c._cleanup_stack == []
    # The first worker started, so it must have been joined and released.
    assert c._sensor_workers == []
    leaked = {t.name for t in threading.enumerate() if t.name.startswith("r1lite")}
    assert leaked == set()


@pytest.mark.parametrize("stream", ["wrist_left_depth", "wrist_right_depth"])
@pytest.mark.parametrize("stamp_sec", [6, 0])
def test_depth_conversion_applies_vendor_stamp_and_frame(
    monkeypatch: Any, stream: str, stamp_sec: int
) -> None:
    c = _bare()
    fake_conv = types.ModuleType("dimos.protocol.pubsub.impl.rospubsub_conversion")

    class _Converted:
        # Mimics the generic converter: wall time and its own frame.
        ts = 999999.0
        frame_id = "converter_invented"

    fake_conv.ros_to_dimos = lambda _msg, _t: _Converted()  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "dimos.protocol.pubsub.impl.rospubsub_conversion", fake_conv)
    out = _FakeOut()
    setattr(c, stream, out)
    q: queue.Queue[Any] = queue.Queue()
    msg = _fb([0.0], stamp_sec=stamp_sec)
    msg.header.frame_id = "vendor_depth_frame"
    q.put(msg)
    q.put(None)
    c._depth_decode_loop(stream, q)
    assert len(out.msgs) == 1
    published = out.msgs[0]
    assert published.frame_id == "vendor_depth_frame"
    if stamp_sec:
        assert published.ts == float(stamp_sec)
        assert c._stamp_fallbacks[stream] == 0
    else:
        assert published.ts > 1e9
        assert c._stamp_fallbacks[stream] == 1


@pytest.mark.parametrize("which", ["imu_chassis", "imu_torso"])
@pytest.mark.parametrize("stamp_sec", [8, 0])
def test_imu_conversion_applies_vendor_stamp_and_frame(
    monkeypatch: Any, which: str, stamp_sec: int
) -> None:
    c = _bare()
    fake_conv = types.ModuleType("dimos.protocol.pubsub.impl.rospubsub_conversion")

    class _ConvertedImu:
        ts = 999999.0
        frame_id = "imu_link"

    fake_conv.ros_to_dimos = lambda _msg, _t: _ConvertedImu()  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "dimos.protocol.pubsub.impl.rospubsub_conversion", fake_conv)
    q: queue.Queue[Any] = queue.Queue()
    msg = _fb([0.0], stamp_sec=stamp_sec)
    msg.header.frame_id = "vendor_imu_frame"
    q.put(msg)
    q.put(None)
    c._imu_decode_loop(q, which)
    stored = c._latest_imu_chassis if which == "imu_chassis" else c._latest_imu_torso
    assert stored is not None
    assert stored.frame_id == "vendor_imu_frame"
    if stamp_sec:
        assert stored.ts == float(stamp_sec)
        assert c._stamp_fallbacks[which] == 0
    else:
        assert stored.ts > 1e9
        assert c._stamp_fallbacks[which] == 1


@pytest.mark.parametrize("stream", ["head_left_color", "head_right_color"])
@pytest.mark.parametrize("stamp_sec", [4, 0])
def test_camera_conversion_applies_vendor_stamp_and_frame(stream: str, stamp_sec: int) -> None:
    c = _bare()
    out = _FakeOut()
    setattr(c, stream, out)
    ok, jpeg = cv2.imencode(".jpg", np.zeros((4, 4, 3), dtype=np.uint8))
    assert ok
    q: queue.Queue[Any] = queue.Queue()
    msg = _fb([0.0], stamp_sec=stamp_sec)
    msg.header.frame_id = "vendor_cam_frame"
    msg.data = jpeg.tobytes()
    q.put(msg)
    q.put(None)
    c._compressed_decode_loop(stream, q)
    assert len(out.msgs) == 1
    published = out.msgs[0]
    # Vendor frame only; never the stream name.
    assert published.frame_id == "vendor_cam_frame"
    if stamp_sec:
        assert published.ts == float(stamp_sec)
        assert c._stamp_fallbacks[stream] == 0
    else:
        assert published.ts > 1e9
        assert c._stamp_fallbacks[stream] == 1


def test_arming_required_set_reconciles_with_preflight_config() -> None:
    # The connection's arming invariant must cover exactly the preflight
    # feedback set minus the documented preflight-only topics.
    expected = {
        cfg.FB_ARM_LEFT,
        cfg.FB_ARM_RIGHT,
        cfg.FB_TORSO,
        cfg.FB_GRIPPER_LEFT,
        cfg.FB_GRIPPER_RIGHT,
        cfg.FB_CHASSIS_SPEED,
    }
    assert cfg.ARMING_REQUIRED_FEEDBACK == expected
    assert cfg.PREFLIGHT_ONLY_FEEDBACK == {cfg.FB_CHASSIS}
    # And the implementation tracks one freshness source per required topic.
    c = _bare()
    with c._lifecycle_lock:
        assert c._stale_required_sources_locked(time.monotonic()) == []
    assert len(expected) == 6


def test_motor_states_has_no_frame() -> None:
    c = _bare()
    c._publish_feedback_streams()
    assert c.motor_states.msgs[0].frame_id == ""


def test_per_output_fallback_counters_are_independent() -> None:
    c = _bare()
    zero = _fb([0.0], stamp_sec=0)
    for key in ("head_left_color", "wrist_left_depth", "imu_chassis", "imu_torso"):
        c._stamp_or_fallback(zero, key)
    assert c._stamp_fallbacks["head_left_color"] == 1
    assert c._stamp_fallbacks["wrist_left_depth"] == 1
    assert c._stamp_fallbacks["imu_chassis"] == 1
    assert c._stamp_fallbacks["imu_torso"] == 1
    assert c._stamp_fallbacks["head_right_color"] == 0


def test_stamp_or_fallback_preserves_vendor_stamp() -> None:
    c = _bare()
    assert c._stamp_or_fallback(_fb([0.0], stamp_sec=9), "head_left_color") == 9.0
    assert c._stamp_fallbacks["head_left_color"] == 0


def test_torso_frame_is_vendor_only() -> None:
    c = _bare()
    c._on_torso_feedback(_fb([0.0] * 4, stamp_sec=2), None)
    c._publish_feedback_streams()
    # The fake feedback carries no frame_id: the output must not invent one.
    assert c.torso_states.msgs[-1].frame_id == ""


# Import hygiene


def test_no_ros_import_at_module_level() -> None:
    tree = ast.parse(_CONN_SRC.read_text())
    banned = {"rclpy", "sensor_msgs", "geometry_msgs", "std_msgs", "builtin_interfaces"}
    for node in tree.body:
        if isinstance(node, ast.Import):
            names = {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom):
            names = {(node.module or "").split(".")[0]}
        else:
            continue
        assert not (names & banned), f"module-level ROS import: {names & banned}"


def test_camera_streams_selects_decode_pipelines() -> None:
    """Hosted teleop decodes only the operator-facing stream; the other
    five pipelines cost CPU the pose path needs (2026-08-11 hardware)."""
    from dimos.robot.galaxea.r1lite.connection import (
        _COMPRESSED_CAMERAS,
        _DEPTH_CAMERAS,
        R1LiteConnectionConfig,
        _selected_cameras,
    )

    all_on = R1LiteConnectionConfig()
    assert len(_selected_cameras(_COMPRESSED_CAMERAS, all_on)) == len(_COMPRESSED_CAMERAS)
    assert len(_selected_cameras(_DEPTH_CAMERAS, all_on)) == len(_DEPTH_CAMERAS)

    head_only = R1LiteConnectionConfig(camera_streams=["head_left_color"])
    assert _selected_cameras(_COMPRESSED_CAMERAS, head_only) == [
        ("head_left_color", _COMPRESSED_CAMERAS["head_left_color"])
    ]
    assert _selected_cameras(_DEPTH_CAMERAS, head_only) == []

    off = R1LiteConnectionConfig(enable_cameras=False, camera_streams=["head_left_color"])
    assert _selected_cameras(_COMPRESSED_CAMERAS, off) == []
