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

"""The MAVLink layer of the PX4 package: constants, frames, vehicle state, socket.

In the order a message travels: PX4 mode numbers and setpoint masks; frame conversions
between PX4 (NED world, FRD body) and dimOS (FLU), same signs as
``dimos/robot/drone/mavlink_connection.py``; :class:`VehicleState`, the latest value of every
message we use (duck-typed pymavlink messages in, frozen dataclasses out); and
:class:`MavlinkIO`, the one socket of the stack and the only place a byte leaves for the
aircraft.

Unlike ``dimos/robot/drone`` this never calls ``wait_heartbeat`` (it mis-latches on a
multi-component bus) and filters on source system 1 component 1 explicitly.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Mapping
from concurrent.futures import Future, TimeoutError as FutureTimeoutError
from dataclasses import dataclass, replace
import math
import threading
import time
from typing import TYPE_CHECKING, Any

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.hardware.gimbal.siyi.frame import normalize_attitude
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.px4.config import GIMBAL_COMPID, PX4_COMPID, PX4_SYSID
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pymavlink.mavutil import MavlinkConnection

logger = setup_logger()


# PX4 modes (px4_custom_mode.h): HEARTBEAT.custom_mode packs the main mode in bits 16-23 and
# the auto sub-mode in bits 24-31.
MAIN_POSCTL = 3
MAIN_AUTO = 4
MAIN_OFFBOARD = 6
SUB_AUTO_LOITER = 3
SUB_AUTO_LAND = 6
_MAIN_NAMES = {
    1: "MANUAL",
    2: "ALTCTL",
    3: "POSCTL",
    4: "AUTO",
    5: "ACRO",
    6: "OFFBOARD",
    7: "STABILIZED",
}
_SUB_NAMES = {
    1: "READY",
    2: "TAKEOFF",
    3: "LOITER/HOLD",
    4: "MISSION",
    5: "RTL",
    6: "LAND",
    8: "FOLLOW_TARGET",
    9: "PRECLAND",
}

# MAV_LANDED_STATE
LANDED_ON_GROUND = 1
LANDED_IN_AIR = 2

# HEARTBEAT.base_mode bit
MAV_MODE_FLAG_SAFETY_ARMED = 128

# ESTIMATOR_STATUS.flags bits (ESTIMATOR_STATUS_FLAGS): the absolute position estimates
# PX4 itself wants before it arms for Offboard.
ESTIMATOR_POS_HORIZ_ABS = 16
ESTIMATOR_POS_VERT_ABS = 32
# STATUSTEXT.severity at or below this is something the operator should read.
MAV_SEVERITY_WARNING = 4

# SET_POSITION_TARGET_LOCAL_NED type_mask bits (1 = ignore that field).
_IGN_POS = 0x7
_IGN_VEL = 0x38
_IGN_ACC = 0x1C0
_FORCE = 0x200
_IGN_YAW = 0x400
_IGN_YAWRATE = 0x800
MASK_POS_YAW = _IGN_VEL | _IGN_ACC | _FORCE | _IGN_YAWRATE  # 0xBF8
MASK_VEL_YAW = _IGN_POS | _IGN_ACC | _FORCE | _IGN_YAWRATE  # 0xBC7
MASK_VEL_YAWRATE = _IGN_POS | _IGN_ACC | _FORCE | _IGN_YAW  # 0x7C7


def mode_name(main: int | None, sub: int = 0) -> str:
    """Human name such as ``OFFBOARD`` or ``AUTO:LOITER/HOLD``; ``?`` when unknown."""
    if main is None:
        return "?"
    s = _MAIN_NAMES.get(main, str(main))
    if main == MAIN_AUTO:
        s += ":" + _SUB_NAMES.get(sub, str(sub))
    return s


# Frames. dimOS follows ROS: x forward (north), y left (west), z up. MAVLink LOCAL_NED is
# x north, y east, z down; body IMU data is FRD. Roll unchanged, pitch and yaw negated
# (the same signs as dimos/robot/drone; test_mavlink.py pins them).


def ned_to_flu(n: float, e: float, d: float) -> tuple[float, float, float]:
    """NED world vector -> dimOS world vector (x north, y west, z up)."""
    return n, -e, -d


# Body FRD (forward, right, down) -> body FLU (forward, left, up): the same flip.
frd_to_flu = ned_to_flu


def quaternion_from_ned_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """MAVLink ATTITUDE euler (rad, NED/FRD) -> dimOS orientation quaternion."""
    return Quaternion.from_euler(Vector3(roll, -pitch, -yaw))


def body_flu_velocity_to_ned(
    forward: float, left: float, up: float, yaw_ned_rad: float
) -> tuple[float, float, float]:
    """Body-frame FLU velocity command -> NED world velocity using the vehicle heading.

    Used to turn a teleop ``Twist`` (body frame) into the LOCAL_NED velocity setpoint
    PX4 expects. ``yaw_ned_rad`` is the ATTITUDE yaw (clockwise from north).
    """
    right = -left
    c, s = math.cos(yaw_ned_rad), math.sin(yaw_ned_rad)
    vn = forward * c - right * s
    ve = forward * s + right * c
    return vn, ve, -up


# Vehicle state. Every entry carries its wall-clock receive time ``t`` (what the camera
# stamps frames with).

_UINT16_INVALID = 65535
_STATUSTEXT_KEEP = 64
# MAV_SEVERITY names, index = severity value.
_SEVERITY = ("EMERGENCY", "ALERT", "CRITICAL", "ERROR", "WARNING", "NOTICE", "INFO", "DEBUG")


def is_from_px4(msg: Any) -> bool:
    return bool(msg.get_srcSystem() == PX4_SYSID and msg.get_srcComponent() == PX4_COMPID)


@dataclass(frozen=True)
class Attitude:
    """Degrees. The vehicle's is NED/FRD, with its body rates (rad/s, FRD); the gimbal's is
    the PITCHYAW command convention."""

    roll: float
    pitch: float
    yaw: float
    t: float
    rates: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class LocalPosition:
    n: float
    e: float
    d: float
    vn: float
    ve: float
    vd: float
    t: float


@dataclass(frozen=True)
class GlobalPosition:
    lat: float
    lon: float
    alt_msl: float
    t: float


@dataclass(frozen=True)
class GpsFix:
    """``eph``/``epv`` are the receiver's 1-sigma position uncertainty in metres, NaN when unknown."""

    fix: int
    sats: int
    eph: float
    epv: float
    t: float


@dataclass(frozen=True)
class Heartbeat:
    armed: bool
    main: int
    sub: int
    t: float


@dataclass(frozen=True)
class SysStatus:
    batt_pct: int
    volt: float
    current_a: float
    t: float


@dataclass(frozen=True)
class RcChannels:
    chan: tuple[int, ...]
    t: float


@dataclass(frozen=True)
class ImuSample:
    """HIGHRES_IMU in the body FRD frame (m/s^2, rad/s)."""

    xacc: float
    yacc: float
    zacc: float
    xgyro: float
    ygyro: float
    zgyro: float
    t: float


@dataclass(frozen=True)
class HomePosition:
    lat: float
    lon: float
    alt: float
    t: float


@dataclass(frozen=True)
class EstimatorStatus:
    flags: int
    t: float

    @property
    def position_valid(self) -> bool:
        wanted = ESTIMATOR_POS_HORIZ_ABS | ESTIMATOR_POS_VERT_ABS
        return self.flags & wanted == wanted


@dataclass(frozen=True)
class StatusText:
    seq: int
    severity: int
    text: str
    t: float

    @property
    def severity_name(self) -> str:
        if 0 <= self.severity < len(_SEVERITY):
            return _SEVERITY[self.severity]
        return str(self.severity)


@dataclass(frozen=True)
class VehicleSnapshot:
    """What the supervisor core sees each tick. Ages are wall-clock seconds at snapshot time."""

    heartbeat: Heartbeat | None
    heartbeat_age: float
    local: LocalPosition | None
    local_age: float
    gps: GpsFix | None
    sys_status: SysStatus | None
    rc: RcChannels | None
    rc_age: float
    landed_state: int | None
    yaw_deg: float | None
    px4_msg_age: float
    estimator: EstimatorStatus | None = None
    # PX4's latest warning or worse, so a refusal can say why in PX4's own words.
    statustext: StatusText | None = None

    @property
    def armed(self) -> bool:
        return bool(self.heartbeat and self.heartbeat.armed)

    @property
    def in_offboard(self) -> bool:
        return bool(self.heartbeat and self.heartbeat.main == MAIN_OFFBOARD)

    @property
    def batt_pct(self) -> int:
        return self.sys_status.batt_pct if self.sys_status else -1

    @property
    def mode(self) -> str:
        hb = self.heartbeat
        return mode_name(hb.main if hb else None, hb.sub if hb else 0)


def _age(item: Any, now: float) -> float:
    return math.inf if item is None else now - item.t


class VehicleState:
    """Thread-safe latest-value store fed by the MAVLink reader thread."""

    def __init__(self, gimbal_inverted: bool = False) -> None:
        self.lock = threading.Lock()
        self._gimbal_inverted = gimbal_inverted
        self.attitude: Attitude | None = None
        self.gimbal: Attitude | None = None
        self.gimbal_flags = 0
        self.gimbal_failure = 0
        self.local: LocalPosition | None = None
        self.global_pos: GlobalPosition | None = None
        self.gps: GpsFix | None = None
        self.home: HomePosition | None = None
        self.heartbeat: Heartbeat | None = None
        self.landed_state: int | None = None
        self.sys_status: SysStatus | None = None
        self.rc: RcChannels | None = None
        self.imu: ImuSample | None = None
        self.estimator: EstimatorStatus | None = None
        # PX4's own warnings, kept until the publish loop forwards them.
        self.statustext: deque[StatusText] = deque(maxlen=_STATUSTEXT_KEEP)
        self._statustext_seq = 0
        self.last_px4_msg = 0.0

    def handle(self, msg: Any, now: float | None = None) -> None:
        """Ingest one MAVLink message. Messages not from PX4 (1/1) or the A8 are ignored."""
        t = time.time() if now is None else now
        with self.lock:
            self._handle_locked(msg, t)

    def _handle_locked(self, msg: Any, t: float) -> None:
        name = msg.get_type()
        src = (msg.get_srcSystem(), msg.get_srcComponent())
        if src == (PX4_SYSID, GIMBAL_COMPID) and name == "GIMBAL_DEVICE_ATTITUDE_STATUS":
            try:
                roll, pitch, yaw = normalize_attitude(msg.q, self._gimbal_inverted)
            except ValueError:  # zero or NaN quaternion; this is the reader thread
                return
            self.gimbal = Attitude(roll, pitch, yaw, t)
            self.gimbal_flags = int(msg.flags)
            self.gimbal_failure = int(msg.failure_flags)
            return
        if src != (PX4_SYSID, PX4_COMPID):
            return
        self.last_px4_msg = t
        if name == "ATTITUDE":
            roll, pitch, yaw = (math.degrees(a) for a in (msg.roll, msg.pitch, msg.yaw))
            # Body rates from ATTITUDE: HIGHRES_IMU is not streamed on a telemetry radio.
            rates = (msg.rollspeed, msg.pitchspeed, msg.yawspeed)
            self.attitude = Attitude(roll, pitch, yaw, t, rates)
        elif name == "LOCAL_POSITION_NED":
            self.local = LocalPosition(
                n=msg.x, e=msg.y, d=msg.z, vn=msg.vx, ve=msg.vy, vd=msg.vz, t=t
            )
        elif name == "GLOBAL_POSITION_INT":
            self.global_pos = GlobalPosition(
                lat=msg.lat / 1e7,
                lon=msg.lon / 1e7,
                alt_msl=msg.alt / 1000.0,
                t=t,
            )
        elif name == "GPS_RAW_INT":
            self.gps = GpsFix(
                fix=msg.fix_type,
                # 255 is MAVLink's unknown; it does not fit the int8 of VehicleStatus.gps_sats.
                sats=-1 if msg.satellites_visible == 255 else msg.satellites_visible,
                # h_acc/v_acc are millimetres, 0 when unknown. GPS_RAW_INT.eph/epv are
                # HDOP/VDOP x100, not metres.
                eph=(h / 1000.0 if (h := getattr(msg, "h_acc", 0)) else math.nan),
                epv=(v / 1000.0 if (v := getattr(msg, "v_acc", 0)) else math.nan),
                t=t,
            )
        elif name == "HOME_POSITION":
            self.home = HomePosition(
                lat=msg.latitude / 1e7,
                lon=msg.longitude / 1e7,
                alt=msg.altitude / 1000.0,
                t=t,
            )
        elif name == "HEARTBEAT":
            mode = int(msg.custom_mode)
            self.heartbeat = Heartbeat(
                armed=bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED),
                main=(mode >> 16) & 0xFF,
                sub=(mode >> 24) & 0xFF,
                t=t,
            )
        elif name == "EXTENDED_SYS_STATE":
            self.landed_state = int(msg.landed_state)
        elif name == "ESTIMATOR_STATUS":
            self.estimator = EstimatorStatus(flags=int(msg.flags), t=t)
        elif name == "SYS_STATUS":
            self.sys_status = SysStatus(
                batt_pct=int(msg.battery_remaining),
                volt=(
                    math.nan
                    if msg.voltage_battery == _UINT16_INVALID
                    else msg.voltage_battery / 1000.0
                ),
                current_a=math.nan if msg.current_battery == -1 else msg.current_battery / 100.0,
                t=t,
            )
        elif name == "RC_CHANNELS":
            # PX4 fills every channel past chancount with UINT16_MAX. An absent channel
            # must read low, or the enable switch fails open.
            count = int(getattr(msg, "chancount", 18))
            raw = [int(getattr(msg, f"chan{i}_raw")) for i in range(1, 19)]
            chans = tuple(0 if i >= count or v == _UINT16_INVALID else v for i, v in enumerate(raw))
            self.rc = RcChannels(chan=chans, t=t)
        elif name == "HIGHRES_IMU":
            self.imu = ImuSample(
                xacc=msg.xacc,
                yacc=msg.yacc,
                zacc=msg.zacc,
                xgyro=msg.xgyro,
                ygyro=msg.ygyro,
                zgyro=msg.zgyro,
                t=t,
            )
        elif name == "STATUSTEXT":
            self._statustext_seq += 1
            raw = msg.text
            text = raw.decode("utf-8", "replace") if isinstance(raw, bytes) else str(raw)
            self.statustext.append(
                StatusText(
                    seq=self._statustext_seq,
                    severity=int(msg.severity),
                    text=text.rstrip("\0"),
                    t=t,
                )
            )

    def snapshot(self, now: float | None = None) -> VehicleSnapshot:
        """Immutable copy of what the supervisor needs, with ages computed at ``now``."""
        wall = time.time() if now is None else now
        with self.lock:
            return VehicleSnapshot(
                heartbeat=self.heartbeat,
                heartbeat_age=_age(self.heartbeat, wall),
                local=self.local,
                local_age=_age(self.local, wall),
                gps=self.gps,
                sys_status=self.sys_status,
                rc=self.rc,
                rc_age=_age(self.rc, wall),
                landed_state=self.landed_state,
                yaw_deg=self.attitude.yaw if self.attitude else None,
                px4_msg_age=(math.inf if not self.last_px4_msg else wall - self.last_px4_msg),
                estimator=self.estimator,
                statustext=next(
                    (s for s in reversed(self.statustext) if s.severity <= MAV_SEVERITY_WARNING),
                    None,
                ),
            )


# The socket. px4-reader is the only recv_match caller in the process. It feeds every
# message to VehicleState and resolves COMMAND_ACK futures keyed by command id, so no RPC
# ever blocks on the socket. The setpoint senders live here and only here; the supervisor
# calls them through the Px4Actuator protocol from the tick thread. Exactly one thing
# produces Offboard setpoints.

# MAVLink common command ids and enum values used here.
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_DO_SET_MODE = 176
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_FRAME_LOCAL_NED = 1
_MAV_TYPE_ONBOARD_CONTROLLER = 18
_MAV_AUTOPILOT_INVALID = 8
_MAV_STATE_ACTIVE = 4
_MAV_CMD_SET_MESSAGE_INTERVAL = 511
_MAV_RESULT_ACCEPTED = 0

_RECV_TIMEOUT_S = 0.5
# PX4's boot clock running this far backwards is a reboot, not reordering or sample age.
_REBOOT_BACKSTEP_MS = 1000


@dataclass
class _MsgStat:
    received: int = 0
    last_mono: float = 0.0


def _boot_ms() -> int:
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


def _interval(name: str, hz: float) -> tuple[float, float]:
    """SET_MESSAGE_INTERVAL params for a message by name: id, interval in microseconds."""
    from pymavlink import mavutil

    return float(getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}")), 1e6 / hz


class MavlinkIO:
    """Owns the socket. Construct cheap; ``start()`` opens, ``stop()`` closes."""

    def __init__(
        self,
        state: VehicleState,
        *,
        url: str,
        source_system: int,
        source_component: int,
    ) -> None:
        self._state = state
        self._url = url  # a serial device takes its baud as "dev,baud"
        self._source_system = source_system
        self._source_component = source_component
        self._conn: MavlinkConnection | None = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._send_lock = threading.Lock()
        self._stats_lock = threading.Lock()
        self._stats: dict[str, _MsgStat] = {}
        self._bad_data = 0
        self._acks: dict[int, Future[int]] = {}
        self._ack_results: dict[int, int] = {}
        # Message rates asked of PX4 (name -> Hz); PX4 forgets them when it reboots.
        self._rates: dict[str, float] = {}
        self._px4_boot_ms: int | None = None
        self._px4_reboots = 0

    @property
    def writer_id(self) -> str:
        return f"{self._source_system}/{self._source_component}"

    # Lifecycle

    def start(self) -> None:
        # Lazy import: pymavlink is an optional extra and ~60 MB.
        from pymavlink import mavutil

        self._conn = mavutil.mavlink_connection(
            self._url,
            source_system=self._source_system,
            source_component=self._source_component,
            autoreconnect=True,
        )
        self._stop.clear()
        self._reader = threading.Thread(target=self._reader_loop, name="px4-reader", daemon=True)
        self._reader.start()
        logger.info("MAVLink opened", url=self._url, writer=self.writer_id)

    def stop(self) -> None:
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=_RECV_TIMEOUT_S + DEFAULT_THREAD_JOIN_TIMEOUT)
            self._reader = None
        if self._conn is not None:
            self._conn.close()
            self._conn = None
        for fut in self._acks.values():
            if not fut.done():
                fut.cancel()
        self._acks.clear()

    def wait_for_px4(self, timeout_s: float) -> None:
        """Block until a HEARTBEAT from system 1 / component 1 has been seen.

        With a ``udpin`` URL pymavlink cannot transmit until it has received one packet
        (it needs the peer address), so every sender waits on this first.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._state.heartbeat is not None:
                return
            if self._stop.wait(0.05):
                raise RuntimeError("MavlinkIO stopped while waiting for PX4")
        raise RuntimeError(
            f"no PX4 heartbeat on {self._url} within {timeout_s:.0f}s "
            "(is mavlink-routerd running and the endpoint configured?)"
        )

    # Reader

    def _reader_loop(self) -> None:
        conn = self._conn
        assert conn is not None
        while not self._stop.is_set():
            msg = conn.recv_match(blocking=True, timeout=_RECV_TIMEOUT_S)
            if msg is None:
                continue
            if msg.get_type() == "BAD_DATA":
                self._bad_data += 1
                continue
            try:
                self._ingest(msg)
            except Exception:  # one malformed message must not end the reader
                self._bad_data += 1
                logger.exception("MAVLink message dropped", type=msg.get_type())

    def _ingest(self, msg: Any) -> None:
        """One decoded message: counters, vehicle state, acks."""
        name = msg.get_type()
        now = time.time()
        with self._stats_lock:
            st = self._stats.setdefault(name, _MsgStat())
            st.received += 1
            st.last_mono = time.monotonic()
        self._state.handle(msg, now)
        if not is_from_px4(msg):
            return
        boot_ms = getattr(msg, "time_boot_ms", None)
        if boot_ms is not None:
            if self._px4_boot_ms is not None and boot_ms < self._px4_boot_ms - _REBOOT_BACKSTEP_MS:
                self._on_px4_reboot()
            self._px4_boot_ms = boot_ms
        if name == "COMMAND_ACK":
            self._resolve_ack(int(msg.command), int(msg.result))

    def _on_px4_reboot(self) -> None:
        """PX4 restarted (a battery swap on the bench): ask again for the rates it forgot.

        From the reader thread, so without waiting for acks (the reader resolves them);
        twice, as a radio loses the odd command.
        """
        self._px4_reboots += 1
        rates = dict(self._rates)  # request_rates may add to it from the start thread
        logger.warning("PX4 rebooted; asking again for message rates", rates=rates)
        for _ in range(2):
            for name, rate in rates.items():
                self.send_command(_MAV_CMD_SET_MESSAGE_INTERVAL, *_interval(name, rate))

    def _resolve_ack(self, command: int, result: int) -> None:
        with self._send_lock:
            self._ack_results[command] = result
            fut = self._acks.pop(command, None)
        if fut is not None and not fut.done():
            fut.set_result(result)

    # Senders

    def send_heartbeat(self) -> None:
        conn = self._conn
        if conn is None:
            return
        with self._send_lock:
            conn.mav.heartbeat_send(
                _MAV_TYPE_ONBOARD_CONTROLLER, _MAV_AUTOPILOT_INVALID, 0, 0, _MAV_STATE_ACTIVE
            )

    def send_command(self, command: int, *params: float) -> Future[int]:
        """COMMAND_LONG to PX4; the future resolves with the COMMAND_ACK result code."""
        if self._conn is None:
            raise RuntimeError("MavlinkIO not started")
        p = list(params) + [0.0] * (7 - len(params))
        fut: Future[int] = Future()
        with self._send_lock:
            self._conn.mav.command_long_send(PX4_SYSID, PX4_COMPID, command, 0, *p[:7])
            self._acks[command] = fut
        return fut

    def request_rates(self, hz: Mapping[str, float], timeout_s: float) -> list[str]:
        """Ask PX4 to stream each message at its rate (SET_MESSAGE_INTERVAL); returns the
        ones it did not accept.

        One at a time, since acks are keyed by command id, and each tried twice: a radio
        link loses the odd command or ack. Asked again, unacknowledged, after a PX4 reboot.
        """
        self._rates.update(hz)
        refused = []
        for name, rate in hz.items():
            result = None
            for _ in range(2):
                fut = self.send_command(_MAV_CMD_SET_MESSAGE_INTERVAL, *_interval(name, rate))
                try:
                    result = fut.result(timeout=timeout_s)
                    break
                except (TimeoutError, FutureTimeoutError):  # one class only from Python 3.11
                    continue
            if result != _MAV_RESULT_ACCEPTED:
                logger.warning("PX4 did not accept a message rate", message=name, hz=rate)
                refused.append(name)
        return refused

    # Px4Actuator protocol. Fire-and-forget: the core watches HEARTBEAT for the effect, and
    # the ack result is kept for sensor_stats.

    def set_mode(self, main: int, sub: int = 0) -> None:
        self.send_command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub)

    def arm(self, value: bool) -> None:
        self.send_command(MAV_CMD_COMPONENT_ARM_DISARM, 1.0 if value else 0.0)

    def send_position_setpoint(self, n: float, e: float, d: float, yaw_rad: float) -> None:
        self._send_setpoint(MASK_POS_YAW, n, e, d, 0.0, 0.0, 0.0, yaw_rad, 0.0)

    def send_velocity_setpoint(
        self,
        vn: float,
        ve: float,
        vd: float,
        yaw_rad: float | None = None,
        yaw_rate_rad: float | None = None,
    ) -> None:
        if yaw_rad is not None:
            mask, yaw, rate = MASK_VEL_YAW, yaw_rad, 0.0
        else:
            mask, yaw, rate = MASK_VEL_YAWRATE, 0.0, (yaw_rate_rad or 0.0)
        self._send_setpoint(mask, 0.0, 0.0, 0.0, vn, ve, vd, yaw, rate)

    def _send_setpoint(
        self,
        mask: int,
        n: float,
        e: float,
        d: float,
        vn: float,
        ve: float,
        vd: float,
        yaw_rad: float,
        yaw_rate_rad: float,
    ) -> None:
        """One SET_POSITION_TARGET_LOCAL_NED."""
        if self._conn is None:
            raise RuntimeError("MavlinkIO not started")
        with self._send_lock:
            self._conn.mav.set_position_target_local_ned_send(
                _boot_ms(), PX4_SYSID, PX4_COMPID, MAV_FRAME_LOCAL_NED, mask,
                n, e, d, vn, ve, vd, 0, 0, 0, yaw_rad, yaw_rate_rad,
            )  # fmt: skip

    # Diagnostics

    def stats(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._stats_lock:
            per_msg = {
                name: {"received": s.received, "age_s": now - s.last_mono}
                for name, s in sorted(self._stats.items())
            }
        with self._send_lock:
            acks = dict(self._ack_results)
        return {
            "messages": per_msg,
            "bad_data": self._bad_data,
            "last_ack_result": acks,
            "px4_reboots": self._px4_reboots,
        }

    def stats_snapshot(self) -> dict[str, _MsgStat]:
        with self._stats_lock:
            return {k: replace(v) for k, v in self._stats.items()}
