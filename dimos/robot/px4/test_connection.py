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

"""Px4DroneConnection shell: the port contract, the RPC surface, start refusal, stop order.

No aircraft: the module is constructed but never started against a socket (nothing
opens in ``__init__``), and the gimbal path is exercised through a fake MavlinkIO.
"""

from __future__ import annotations

from collections.abc import Iterator
import math
import socket
import threading
import time
from typing import Any
from unittest.mock import MagicMock, patch

from dimos_lcm.sensor_msgs.NavSatFix import NavSatFix as LCMNavSatFix
from dimos_lcm.sensor_msgs.NavSatStatus import NavSatStatus
from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.px4.connection import Px4DroneConnection
from dimos.robot.px4.mavlink import (
    MAIN_OFFBOARD,
    MAIN_POSCTL,
    Heartbeat,
    LocalPosition,
    VehicleSnapshot,
    VehicleState,
)
from dimos.robot.px4.supervisor_core import GotoGoal, SupervisorCore, TakeoffPoint
from dimos.robot.px4.test_mavlink import Msg

# The port contract. New modules bind by exact name; renaming one breaks them.
_CONTRACT_OUT = {
    "odometry",
    "odom",
    "imu",
    "gps",
    "battery",
    "gimbal_attitude",
    "tf",
    "vehicle_status",
    "statustext",
    "supervisor_state",
    "global_pose",
}
_CONTRACT_IN = {
    "cmd_vel",
    "gimbal_target",
    "target_state",
    "target_valid",
    "target_los",
}

# Anything that could move the aircraft must not be reachable over RPC.
_FORBIDDEN_RPCS = {
    "arm",
    "disarm",
    "set_mode",
    "send_position_setpoint",
    "send_velocity_setpoint",
    "send_gimbal_pitchyaw",
    "claim_gimbal_control",
}
_REQUIRED_RPCS = {
    "takeoff",
    "go_to",
    "land",
    "hold",
    "set_guidance_mode",
    "estop",
    "estop_land",
    "estop_clear",
    "status",
    "sensor_stats",
    "sitl_enable",
}


@pytest.fixture
def module() -> Iterator[Px4DroneConnection]:
    m = Px4DroneConnection(writer_lock_port=0)
    yield m
    m.stop()


def test_port_contract(module: Px4DroneConnection) -> None:
    assert set(module.outputs) == _CONTRACT_OUT
    assert set(module.inputs) == _CONTRACT_IN


def test_rpc_surface_has_no_actuation(module: Px4DroneConnection) -> None:
    names = set(module.rpcs)
    assert not (names & _FORBIDDEN_RPCS), names & _FORBIDDEN_RPCS
    assert _REQUIRED_RPCS <= names, _REQUIRED_RPCS - names


def test_start_refuses_when_another_writer_holds_the_lock_port() -> None:
    holder = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    holder.bind(("127.0.0.1", 0))
    port = holder.getsockname()[1]
    m = Px4DroneConnection(writer_lock_port=port)
    try:
        with pytest.raises(RuntimeError, match="another Offboard writer"):
            m.start()
        assert m._io is None  # the MAVLink socket was never opened
    finally:
        holder.close()
        m.stop()


def test_stop_joins_every_thread_before_closing_io(module: Px4DroneConnection) -> None:
    module._stop_event.clear()
    threads = []
    for name in ("px4-tick", "px4-heartbeat", "px4-publish", "px4-stats"):
        t = threading.Thread(target=module._stop_event.wait, name=name, daemon=True)
        t.start()
        threads.append((name, t))
    module._threads = threads
    io = module._io = MagicMock()
    module._writer_lock_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    alive: list[bool] = []
    io.stop.side_effect = lambda: alive.extend(t.is_alive() for _, t in threads)

    module.stop()

    # No thread that could still send a setpoint is running when the socket closes.
    assert alive == [False] * 4
    assert module._io is None and module._writer_lock_sock is None


def test_a_failed_start_releases_the_reader_and_the_writer_lock() -> None:
    free = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    free.bind(("127.0.0.1", 0))
    port = free.getsockname()[1]
    free.close()
    io = MagicMock()
    io.wait_for_px4.side_effect = RuntimeError("no PX4 heartbeat")
    m = Px4DroneConnection(writer_lock_port=port)
    try:
        with patch("dimos.robot.px4.connection.MavlinkIO", return_value=io):
            for _ in range(2):  # the second attempt must not find its own lock still held
                with pytest.raises(RuntimeError, match="no PX4 heartbeat"):
                    m.start()
                assert m._io is None and m._writer_lock_sock is None
        assert io.stop.call_count == 2
    finally:
        m.stop()


def _snapshot(d: float = 0.0, flying: bool = False) -> VehicleSnapshot:
    now = time.time()
    main = MAIN_OFFBOARD if flying else MAIN_POSCTL
    return VehicleSnapshot(
        heartbeat=Heartbeat(flying, main, 0, now),
        heartbeat_age=0.0,
        local=LocalPosition(0.0, 0.0, d, 0.0, 0.0, 0.0, now),
        local_age=0.0,
        gps=None,
        sys_status=None,
        rc=None,
        rc_age=math.inf,
        landed_state=None,
        yaw_deg=0.0,
        px4_msg_age=0.0,
    )


@pytest.fixture
def commanded(module: Px4DroneConnection) -> Px4DroneConnection:
    """The module with a supervisor core and a vehicle snapshot but no socket."""
    module._core = SupervisorCore(module.config.limits, module.config.guidance)
    module._io = MagicMock()
    module._state = MagicMock()
    module._state.snapshot.return_value = _snapshot()
    return module


def test_takeoff_rpc_passes_the_altitude_to_the_core(
    commanded: Px4DroneConnection,
) -> None:
    assert commanded.takeoff(2.5) == {"accepted": True, "rejection": None, "state": "PREFLIGHT"}
    assert commanded._core is not None and commanded._core.takeoff_alt_m == 2.5
    assert commanded.takeoff(2.0) == {
        "accepted": False,
        "rejection": "wrong_state",
        "state": "PREFLIGHT",
    }


def test_go_to_rpc_sets_the_goal_and_a_refusal_leaves_it_running(
    commanded: Px4DroneConnection,
) -> None:
    core = commanded._core
    assert core is not None
    core.state, core.takeoff = "HOVER", TakeoffPoint(n=0.0, e=0.0, d0=0.0, yaw=0.0)
    commanded._state.snapshot.return_value = _snapshot(d=-2.0, flying=True)

    assert commanded.go_to(north_m=-2.0, altitude_m=3.0)["accepted"]
    assert core.state == "GOTO" and core.goal == GotoGoal(n=-2.0, e=0.0, d=-3.0)
    assert commanded.go_to(north_m=100.0) == {
        "accepted": False,
        "rejection": "fence",
        "state": "GOTO",
    }
    assert core.goal == GotoGoal(n=-2.0, e=0.0, d=-3.0)


def test_an_unknown_guidance_mode_is_a_rejection_not_an_exception(
    commanded: Px4DroneConnection,
) -> None:
    assert commanded.set_guidance_mode("bogus") == {
        "accepted": False,
        "rejection": "invalid_argument",
        "state": "IDLE",
    }
    assert commanded.set_guidance_mode("teleop")["accepted"]


def test_a_publish_that_raises_does_not_end_the_flight_loop(commanded: Px4DroneConnection) -> None:
    core = commanded._core
    assert core is not None
    commanded._stop_event.clear()
    commanded.supervisor_state.publish = MagicMock(side_effect=RuntimeError("transport down"))
    tick = threading.Thread(target=commanded._tick_loop, daemon=True)
    tick.start()
    try:
        assert commanded.takeoff(2.0)["accepted"]  # a transition, so the tick publishes
        deadline = time.time() + 2.0
        while not commanded.supervisor_state.publish.called and time.time() < deadline:
            time.sleep(0.01)
        assert commanded.supervisor_state.publish.called
        time.sleep(0.15)  # a few more ticks
        assert tick.is_alive()
    finally:
        commanded._stop_event.set()
        tick.join(timeout=2.0)


def _vehicle(*msgs: Msg) -> VehicleState:
    state = VehicleState()
    for m in msgs:
        state.handle(m, now=100.0)
    return state


def _tap(module: Px4DroneConnection, *ports: str) -> dict[str, list[Any]]:
    out: dict[str, list[Any]] = {name: [] for name in ports}
    for name, sink in out.items():
        getattr(module, name).publish = sink.append
    return out


def test_publishers_convert_frames_units_and_unknowns(module: Px4DroneConnection) -> None:
    out = _tap(module, "odometry", "odom", "tf", "imu", "gps", "battery")
    state = _vehicle(
        # Heading east, moving north-east and climbing; gyro and accel in body FRD.
        Msg("ATTITUDE", roll=0.0, pitch=0.0, yaw=math.radians(90.0)),
        Msg("LOCAL_POSITION_NED", x=3.0, y=4.0, z=-5.0, vx=1.0, vy=2.0, vz=-0.5),
        Msg("HIGHRES_IMU", xacc=0.1, yacc=0.2, zacc=-9.8, xgyro=0.01, ygyro=0.02, zgyro=0.03),
        Msg(
            "GPS_RAW_INT", fix_type=3, satellites_visible=14, eph=90, epv=130, h_acc=450, v_acc=700
        ),
        Msg("SYS_STATUS", battery_remaining=-1, voltage_battery=65535, current_battery=-1),
    )
    module._publish_odometry(state)
    module._publish_imu(state)
    module._publish_gps(state)
    module._publish_battery(state)

    (odom,) = out["odometry"]
    assert (odom.x, odom.y, odom.z) == (3.0, -4.0, 5.0)
    assert math.degrees(odom.yaw) == pytest.approx(-90.0)
    # The twist is not one frame: linear stays in odom (world FLU), angular is body FLU.
    assert (odom.vx, odom.vy, odom.vz) == (1.0, -2.0, 0.5)
    assert (odom.twist.angular.x, odom.twist.angular.y, odom.twist.angular.z) == (
        0.01,
        -0.02,
        -0.03,
    )
    assert (odom.frame_id, odom.child_frame_id) == ("odom", "base_link")
    (tf,) = out["tf"]
    assert [(t.frame_id, t.child_frame_id) for t in tf.transforms] == [("odom", "base_link")]
    (imu,) = out["imu"]
    acc = imu.linear_acceleration
    assert (acc.x, acc.y, acc.z) == (0.1, -0.2, 9.8)

    # GPS_RAW_INT alone has no position: nothing, rather than a valid-looking fix at 0, 0.
    assert out["gps"] == []
    state.handle(
        Msg("GLOBAL_POSITION_INT", lat=377749000, lon=-1224194000, alt=52000, relative_alt=12000)
    )
    module._publish_gps(state)
    (fix,) = out["gps"]
    assert (fix.latitude, fix.longitude, fix.altitude) == pytest.approx((37.7749, -122.4194, 52.0))
    assert fix.status == NavSatStatus.STATUS_FIX
    assert fix.position_covariance_type == LCMNavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    assert fix.position_covariance[0] == pytest.approx(0.45**2)  # metres squared, not HDOP
    assert fix.position_covariance[8] == pytest.approx(0.7**2)
    state.handle(Msg("GPS_RAW_INT", fix_type=3, satellites_visible=14, h_acc=0, v_acc=0))
    module._publish_gps(state)
    assert out["gps"][-1].position_covariance_type == LCMNavSatFix.COVARIANCE_TYPE_UNKNOWN

    (battery,) = out["battery"]
    assert math.isnan(battery.voltage) and math.isnan(battery.current)
    assert math.isnan(battery.percentage)


def _target(pitch_deg: float, yaw_deg: float) -> JointState:
    return JointState(
        name=["gimbal_pitch", "gimbal_yaw"],
        position=[math.radians(pitch_deg), math.radians(yaw_deg)],
    )


def test_gimbal_target_is_dropped_unless_commands_are_enabled() -> None:
    m = Px4DroneConnection(writer_lock_port=0)
    io = MagicMock()
    io.stats.return_value = {}
    m._io = io
    try:
        m._on_gimbal_target(_target(-10.0, 30.0))
        io.send_gimbal_pitchyaw.assert_not_called()
        assert m.sensor_stats()["gimbal_target"] == {"sent": 0, "dropped": 1}
    finally:
        m._io = None
        m.stop()


def test_gimbal_target_is_clamped_and_sent_when_enabled() -> None:
    m = Px4DroneConnection(writer_lock_port=0, gimbal_commands_enabled=True)
    io = MagicMock()
    m._io = io
    try:
        m._on_gimbal_target(_target(math.nan, 0.0))
        io.send_gimbal_pitchyaw.assert_not_called()
        m._on_gimbal_target(_target(-120.0, 170.0))  # beyond the A8 limits
        io.send_gimbal_pitchyaw.assert_called_once()
        pitch, yaw, device = io.send_gimbal_pitchyaw.call_args.args
        assert (pitch, yaw, device) == (-90.0, 120.0, 154)
        # Rate limit: a second command inside the 10 Hz window is not sent.
        m._on_gimbal_target(_target(0.0, 0.0))
        assert io.send_gimbal_pitchyaw.call_count == 1
    finally:
        m._io = None
        m.stop()


def _los(gimbal_yaw_deg: float) -> PoseStamped:
    """target_los as the bridge sends it: the gimbal's clockwise body yaw as an FLU yaw."""
    q = Quaternion.from_euler(Vector3(0.0, 0.0, -math.radians(gimbal_yaw_deg)))
    return PoseStamped(ts=time.time(), frame_id="base_link", orientation=q)


def test_target_los_alone_reaches_the_supervisor(commanded: Px4DroneConnection) -> None:
    core = commanded._core
    assert core is not None
    # A ray too shallow for a ground position: target_los arrives, target_state never does.
    commanded._on_target_valid(Bool(data=False))
    commanded._on_target_los(_los(40.0))
    target = core.follow.target
    assert target is not None and target.los_valid and not target.valid and target.n is None
    assert target.gimbal_yaw_body_deg == pytest.approx(40.0)
    assert not core.follow.fresh(time.time())

    commanded._on_target_valid(Bool(data=True))
    commanded._on_target_state(
        Odometry(
            ts=time.time(),
            frame_id="odom",
            child_frame_id="target",
            pose=Pose(Vector3(30.0, -5.0, 0.0), Quaternion()),
            twist=Twist(Vector3(1.0, 0.0, 0.0), Vector3()),
        )
    )
    target = core.follow.target
    assert target is not None and target.valid and target.los_valid
    assert (target.n, target.e, target.vn, target.ve) == (30.0, 5.0, 1.0, 0.0)
    assert core.follow.fresh(time.time())

    # A line of sight re-sends the position with the position's own age: FOLLOW loses it
    # target_stale_s after the last target_state, however late the line of sight came.
    stale_s = commanded.config.limits.target_stale_s
    commanded._target_state_t -= 0.9 * stale_s
    commanded._on_target_los(_los(40.0))
    assert core.follow.target is not None and core.follow.target.n == 30.0
    assert core.follow.fresh(time.time())
    assert not core.follow.fresh(time.time() + 0.2 * stale_s)

    # A later line of sight does not revive a position that has gone stale.
    commanded._target_state_t -= stale_s
    commanded._on_target_los(_los(40.0))
    target = core.follow.target
    assert target is not None and target.los_valid and not target.valid and target.n is None


def test_a_non_finite_target_never_reaches_the_supervisor(commanded: Px4DroneConnection) -> None:
    core = commanded._core
    assert core is not None
    commanded._on_target_valid(Bool(data=True))
    commanded._on_target_state(
        Odometry(
            ts=time.time(),
            frame_id="odom",
            child_frame_id="target",
            pose=Pose(Vector3(math.nan, 0.0, 0.0), Quaternion()),
            twist=Twist(),
        )
    )
    for x in (math.nan, math.inf):  # to_euler raises on the first, gives a NaN yaw on the second
        q = Quaternion(x, 0.0, 0.0, 1.0)
        commanded._on_target_los(PoseStamped(ts=time.time(), frame_id="base_link", orientation=q))
    assert core.follow.target is None


# Vehicle clock, gimbal stamp, global_pose.


def _primed(module: Px4DroneConnection, offset_s: float) -> None:
    for i in range(module.config.timebase_samples):
        module._timebase.add_system_time(offset_s + i, float(i), receive_wall_s=offset_s + i)
    assert module._timebase.quality == "system_time"


def test_vehicle_stamps_go_through_the_timebase_only_when_it_is_the_source() -> None:
    m = Px4DroneConnection(writer_lock_port=0)
    rx = Px4DroneConnection(writer_lock_port=0, timebase_source="receive_time")
    try:
        assert m._stamp(40.0, 99.0) == 99.0  # no SYSTEM_TIME yet
        for module in (m, rx):
            _primed(module, 1.8e9)
        assert m._stamp(40.0, 99.0) == 1.8e9 + 40.0
        assert m._stamp(None, 99.0) == 99.0
        assert rx._stamp(40.0, 99.0) == 99.0
    finally:
        m.stop()
        rx.stop()


def test_timebase_samples_sizes_the_timebase() -> None:
    m = Px4DroneConnection(writer_lock_port=0, timebase_samples=10)
    try:
        _primed(m, 1.8e9)  # ten samples are enough: no "below target quality" at every start
    finally:
        m.stop()
    with pytest.raises(ValueError):  # an empty window has no median to guard a jump against
        Px4DroneConnection(writer_lock_port=0, timebase_samples=0)


def test_the_gimbal_is_stamped_at_receipt_not_through_px4s_clock(
    module: Px4DroneConnection,
) -> None:
    out = _tap(module, "gimbal_attitude", "odometry", "odom", "tf")
    _primed(module, 1.8e9)  # PX4 booted at 1.8e9; the A8 booted 120 s later, on its own clock
    now = time.time()
    state = VehicleState()
    q = [1.0, 0.0, 0.0, 0.0]
    state.handle(
        Msg("GIMBAL_DEVICE_ATTITUDE_STATUS", src=(1, 154), q=q, flags=16, failure_flags=0,
            time_boot_ms=380_000),
        now=now,
    )  # fmt: skip
    state.handle(Msg("ATTITUDE", time_boot_ms=500_000, roll=0.0, pitch=0.0, yaw=0.0), now=now)
    state.handle(
        Msg("LOCAL_POSITION_NED", time_boot_ms=500_000, x=0, y=0, z=0, vx=0, vy=0, vz=0), now=now
    )
    module._publish_gimbal(state)
    module._publish_odometry(state)
    (gimbal,) = out["gimbal_attitude"]
    assert gimbal.ts == now  # 1.8e9 + 380 would age it out of every consumer, or alias it
    assert out["odometry"][0].ts == 1.8e9 + 500.0  # PX4's own messages do use its clock


def test_global_pose_is_the_altitude_above_home_and_needs_no_gps(
    module: Px4DroneConnection,
) -> None:
    out = _tap(module, "global_pose", "gps")
    state = _vehicle(
        Msg("GLOBAL_POSITION_INT", lat=377749000, lon=-1224194000, alt=52000, relative_alt=12000)
    )
    module._publish_gps(state)  # no GPS_RAW_INT
    (pose,) = out["global_pose"]
    assert (pose.x, pose.y, pose.z) == (0.0, 0.0, 12.0) and pose.frame_id == "home"
    assert out["gps"] == []
