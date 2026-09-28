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

"""PX4 drone connection Module: MAVLink bridge + Offboard flight supervisor, one process.

Owns the one MAVLink socket of the dimOS stack and exposes the vehicle as dimos streams:
odometry, IMU, GPS, battery, gimbal attitude and status, plus the supervisor's own streams.
Operator commands are RPCs; teleop arrives on ``cmd_vel``.

Mirrors ``dimos/robot/galaxea/r1pro/connection.py``: Field config, lazy driver import and
socket opened in ``start()``, one drift-free publish loop, ``sensor_stats``. The flight
logic (``supervisor_core.py``) and the MAVLink layer (``mavlink.py``) are plain classes
this module owns.

Safety invariants (README): the RC pilot always wins; exactly one writer of Offboard
setpoints (this module's tick thread through MavlinkIO); if it stops, the stream stops and
PX4's Offboard-loss failsafe takes over; E-STOP is Hold plus a latch; no arm, mode or
setpoint RPC exists on this surface.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
import math
import socket
import threading
import time
from typing import Any, Literal

from dimos_lcm.sensor_msgs.BatteryState import BatteryState as LCMBatteryState
from dimos_lcm.sensor_msgs.NavSatFix import NavSatFix as LCMNavSatFix
from dimos_lcm.sensor_msgs.NavSatStatus import NavSatStatus
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.gimbal.siyi.frame import MOUNT_PRESETS
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.px4_msgs.VehicleStatus import VehicleStatus
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.px4.config import (
    DIMOS_COMPID,
    PX4_SYSID,
    ROUTER_MAV_URL,
    WRITER_LOCK_PORT,
    GuidanceConfig,
    SupervisorLimits,
)
from dimos.robot.px4.mavlink import (
    MAIN_AUTO,
    SUB_AUTO_LOITER,
    MavlinkIO,
    VehicleSnapshot,
    VehicleState,
    frd_to_flu,
    mode_name,
    ned_to_flu,
    quaternion_from_ned_euler,
)
from dimos.robot.px4.supervisor_core import (
    ARMED_STATES,
    GUIDANCE_MODES,
    GuidanceMode,
    Rejection,
    SupervisorCore,
    TeleopCommand,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# One 100 Hz loop serves every output stream on its own divisor, like R1ProConnection's
# publish loop; a thread per stream would buy nothing for messages this small.
_PUBLISH_BASE_HZ = 100.0
_JITTER_WINDOW = 2000
_GIMBAL_MAX_AGE_S = 1.0
_GIMBAL_JOINTS = ["gimbal_roll", "gimbal_pitch", "gimbal_yaw"]


class Px4DroneConnectionConfig(ModuleConfig):
    # Our own mavlink-router endpoint; never share a port with another MAVLink service.
    mav_url: str = Field(default=ROUTER_MAV_URL)
    # Our component id on the vehicle (system 1); never a ground station (255).
    source_component: int = Field(default=DIMOS_COMPID)
    connect_timeout_s: float = Field(default=30.0)
    heartbeat_hz: float = Field(default=1.0, gt=0.0)
    odom_frame_id: str = Field(default="odom")
    base_frame_id: str = Field(default="base_link")
    gimbal_base_frame_id: str = Field(default="gimbal_base")
    # "flight" = A8 hanging under the frame; "bench" = base down.
    gimbal_mount_preset: Literal["flight", "bench"] = Field(default="flight")
    # Output rates. PX4 streams LOCAL_POSITION_NED at 30 Hz and HIGHRES_IMU at 50 Hz on the
    # Onboard profile; the rest are as fast as anyone downstream needs.
    odom_hz: float = Field(default=30.0)
    imu_hz: float = Field(default=50.0)
    status_hz: float = Field(default=5.0)
    gps_hz: float = Field(default=5.0)
    battery_hz: float = Field(default=1.0)
    gimbal_hz: float = Field(default=10.0)
    statustext_hz: float = Field(default=10.0)
    # SITL: the RC enable switch is faked by the sitl_enable RPC and RC checks are skipped.
    sitl: bool = Field(default=False)
    sensor_stats_interval_s: float = Field(default=10.0)
    # Host-local single-writer lock (UDP port on 127.0.0.1); 0 disables it.
    writer_lock_port: int = Field(default=WRITER_LOCK_PORT)
    tick_hz: float = Field(default=20.0, gt=0.0)
    limits: SupervisorLimits = Field(default_factory=SupervisorLimits)
    guidance: GuidanceConfig = Field(default_factory=GuidanceConfig)


def _divisor(hz: float) -> int:
    return max(1, round(_PUBLISH_BASE_HZ / hz)) if hz > 0 else 0


def _navsat_status(fix_type: int) -> int:
    # MAVLink GPS_FIX_TYPE: 0/1 none, 2 2D, 3 3D, 4 DGPS, 5 RTK float, 6 RTK fixed.
    status: int = NavSatStatus.STATUS_NO_FIX
    if fix_type >= 5:
        status = NavSatStatus.STATUS_GBAS_FIX
    elif fix_type == 4:
        status = NavSatStatus.STATUS_SBAS_FIX
    elif fix_type >= 2:
        status = NavSatStatus.STATUS_FIX
    return status


def _nan_if_inf(v: float) -> float:
    return math.nan if math.isinf(v) else v


class Px4DroneConnection(Module):
    """PX4 drone Module: the MAVLink bridge and the Offboard supervisor, one socket, one writer."""

    # Unlike R1ProConnection this runs a 20 Hz flight loop; its own process keeps the tick
    # jitter away from every other module's Python interpreter.
    dedicated_worker = True

    config: Px4DroneConnectionConfig

    # Control inputs.
    cmd_vel: In[Twist]

    # Vehicle feedback. The odometry twist is linear in odom (world FLU), like PX4's
    # vehicle_odometry, and angular in base_link.
    odometry: Out[Odometry]
    odom: Out[PoseStamped]
    tf: Out[TFMessage]
    imu: Out[Imu]
    gps: Out[NavSatFix]
    battery: Out[BatteryState]
    gimbal_attitude: Out[JointState]
    vehicle_status: Out[VehicleStatus]
    statustext: Out[String]

    # Supervisor streams.
    supervisor_state: Out[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._state: VehicleState | None = None
        self._io: MavlinkIO | None = None
        self._core: SupervisorCore | None = None
        # Guards the core between the tick thread and the RPC threads.
        self._core_lock = threading.Lock()
        self._writer_lock_sock: socket.socket | None = None
        self._stop_event = threading.Event()
        # Joined in this order on stop: tick first so no setpoint can leave after teardown
        # starts, then heartbeat, then publish; the reader goes with MavlinkIO.
        self._threads: list[tuple[str, threading.Thread]] = []
        self._jitter_ms: deque[float] = deque(maxlen=_JITTER_WINDOW)
        self._last_statustext_seq = 0
        self._cmd_vel_rejections: dict[str, int] = {}
        self._cmd_vel_accepted = 0

    # Lifecycle

    @rpc
    def start(self) -> None:
        super().start()
        cfg = self.config
        self._acquire_writer_lock()
        self._state = VehicleState(gimbal_mount=MOUNT_PRESETS[cfg.gimbal_mount_preset])
        self._core = SupervisorCore(cfg.limits, cfg.guidance, sitl=cfg.sitl)
        self._io = MavlinkIO(
            self._state,
            url=cfg.mav_url,
            source_system=PX4_SYSID,
            source_component=cfg.source_component,
        )
        try:
            self._io.start()
            self._io.wait_for_px4(cfg.connect_timeout_s)
        except Exception:
            self._release()  # or the reader and both sockets outlive the failed start
            raise
        self._io.send_heartbeat()

        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))

        self._stop_event.clear()
        self._threads = [
            (name, threading.Thread(target=fn, name=name, daemon=True))
            for name, fn in (
                ("px4-tick", self._tick_loop),
                ("px4-heartbeat", self._heartbeat_loop),
                ("px4-publish", self._publish_loop),
                ("px4-stats", self._stats_loop),
            )
        ]
        for _, t in self._threads:
            t.start()
        logger.info(
            "Px4DroneConnection started",
            writer=self._io.writer_id,
            sitl=cfg.sitl,
        )

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        for _, t in self._threads:
            if t.is_alive():
                t.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._threads = []
        self._release()
        super().stop()

    def _release(self) -> None:
        if self._io is not None:
            self._io.stop()
            self._io = None
        if self._writer_lock_sock is not None:
            self._writer_lock_sock.close()
            self._writer_lock_sock = None

    def _acquire_writer_lock(self) -> None:
        port = self.config.writer_lock_port
        if port <= 0:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("127.0.0.1", port))
        except OSError as e:
            sock.close()
            raise RuntimeError(
                f"another Offboard writer holds UDP 127.0.0.1:{port}; refusing to start"
            ) from e
        self._writer_lock_sock = sock

    # Input handlers

    def _on_cmd_vel(self, msg: Twist) -> None:
        core = self._core
        if core is None:
            return
        now = time.time()
        cmd = TeleopCommand(
            forward=float(msg.linear.x),
            left=float(msg.linear.y),
            up=float(msg.linear.z),
            yaw_rate_ccw=float(msg.angular.z),
            t=now,
        )
        with self._core_lock:
            why = core.on_cmd_vel(cmd, now)
        if why is None:
            self._cmd_vel_accepted += 1
        else:
            self._cmd_vel_rejections[why.value] = self._cmd_vel_rejections.get(why.value, 0) + 1

    # Threads

    def _tick_loop(self) -> None:
        core, io, state = self._core, self._io, self._state
        assert core is not None and io is not None and state is not None
        period = 1.0 / self.config.tick_hz
        next_tick = time.perf_counter()
        while not self._stop_event.is_set():
            actual = time.perf_counter()
            now = time.time()
            snap = state.snapshot(now)
            with self._core_lock:
                try:
                    core.step(snap, io, now)
                    core.stream(io, now)
                except Exception:
                    logger.exception("supervisor step raised; holding")
                    if core.state in ARMED_STATES:
                        core.sp = None
                        if snap.in_offboard:
                            io.set_mode(MAIN_AUTO, SUB_AUTO_LOITER)
                        core.goto("ABORT", "exception in supervisor", now)
                transitions, core.transitions = core.transitions, []
            try:  # a publish that raises must not end the flight loop
                for new_state, reason, _ in transitions:
                    logger.info("supervisor", state=new_state, reason=reason)
                    self.supervisor_state.publish(String(new_state))
            except Exception:
                logger.exception("supervisor publish failed")

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()
            self._jitter_ms.append((time.perf_counter() - actual - period) * 1e3)

    def _heartbeat_loop(self) -> None:
        io = self._io
        assert io is not None
        period = 1.0 / self.config.heartbeat_hz
        while not self._stop_event.is_set():
            io.send_heartbeat()
            self._stop_event.wait(period)

    def _stats_loop(self) -> None:
        interval = self.config.sensor_stats_interval_s
        if interval <= 0:
            return
        io = self._io
        assert io is not None
        prev = io.stats_snapshot()
        prev_t = time.monotonic()
        while not self._stop_event.wait(interval):
            cur = io.stats_snapshot()
            now = time.monotonic()
            dt = now - prev_t
            rates = {
                name: round((s.received - prev[name].received) / dt, 1) if name in prev else None
                for name, s in sorted(cur.items())
            }
            logger.info("MAVLink message rates (Hz)", window_s=round(dt), **rates)
            prev, prev_t = cur, now

    def _publish_loop(self) -> None:
        cfg = self.config
        state = self._state
        assert state is not None
        period = 1.0 / _PUBLISH_BASE_HZ
        divisors = {
            "odom": _divisor(cfg.odom_hz),
            "imu": _divisor(cfg.imu_hz),
            "status": _divisor(cfg.status_hz),
            "gps": _divisor(cfg.gps_hz),
            "battery": _divisor(cfg.battery_hz),
            "gimbal": _divisor(cfg.gimbal_hz),
            "statustext": _divisor(cfg.statustext_hz),
        }
        publishers = {
            "odom": self._publish_odometry,
            "imu": self._publish_imu,
            "status": self._publish_status,
            "gps": self._publish_gps,
            "battery": self._publish_battery,
            "gimbal": self._publish_gimbal,
            "statustext": self._publish_statustext,
        }
        tick = 0
        next_tick = time.perf_counter()
        while not self._stop_event.is_set():
            for name, div in divisors.items():
                if div and tick % div == 0:
                    try:
                        publishers[name](state)
                    except Exception:
                        logger.exception("publish failed", stream=name)
            tick += 1
            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()

    # Publishers (publish thread)

    def _publish_odometry(self, state: VehicleState) -> None:
        with state.lock:
            local, att, imu = state.local, state.attitude, state.imu
        if local is None or att is None:
            return
        cfg = self.config
        ts = local.t
        x, y, z = ned_to_flu(local.n, local.e, local.d)
        vx, vy, vz = ned_to_flu(local.vn, local.ve, local.vd)
        q = quaternion_from_ned_euler(
            math.radians(att.roll), math.radians(att.pitch), math.radians(att.yaw)
        )
        angular = Vector3(*frd_to_flu(imu.xgyro, imu.ygyro, imu.zgyro)) if imu else Vector3()
        pose = PoseStamped(
            ts=ts, frame_id=cfg.odom_frame_id, position=Vector3(x, y, z), orientation=q
        )
        self.odom.publish(pose)
        self.odometry.publish(
            Odometry(
                ts=ts,
                frame_id=cfg.odom_frame_id,
                child_frame_id=cfg.base_frame_id,
                pose=Pose(Vector3(x, y, z), q),
                twist=Twist(Vector3(vx, vy, vz), angular),
            )
        )
        self.tf.publish(TFMessage(Transform.from_pose(cfg.base_frame_id, pose)))

    def _publish_imu(self, state: VehicleState) -> None:
        with state.lock:
            imu, att = state.imu, state.attitude
        if imu is None:
            return
        orientation = (
            quaternion_from_ned_euler(
                math.radians(att.roll), math.radians(att.pitch), math.radians(att.yaw)
            )
            if att
            else Quaternion()
        )
        self.imu.publish(
            Imu(
                angular_velocity=Vector3(*frd_to_flu(imu.xgyro, imu.ygyro, imu.zgyro)),
                linear_acceleration=Vector3(*frd_to_flu(imu.xacc, imu.yacc, imu.zacc)),
                orientation=orientation,
                frame_id=self.config.base_frame_id,
                ts=imu.t,
            )
        )

    def _publish_gps(self, state: VehicleState) -> None:
        with state.lock:
            gps, gp = state.gps, state.global_pos
        # Without GLOBAL_POSITION_INT there is no position: a fix at 0, 0 would look valid.
        if gps is None or gp is None:
            return
        cov_known = not (math.isnan(gps.eph) or math.isnan(gps.epv))
        eph2, epv2 = (gps.eph**2, gps.epv**2) if cov_known else (0.0, 0.0)
        self.gps.publish(
            NavSatFix(
                latitude=gp.lat,
                longitude=gp.lon,
                altitude=gp.alt_msl,
                status=_navsat_status(gps.fix),
                position_covariance=[eph2, 0.0, 0.0, 0.0, eph2, 0.0, 0.0, 0.0, epv2],
                position_covariance_type=(
                    LCMNavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                    if cov_known
                    else LCMNavSatFix.COVARIANCE_TYPE_UNKNOWN
                ),
                frame_id="gps",
                ts=gps.t,
            )
        )

    def _publish_battery(self, state: VehicleState) -> None:
        with state.lock:
            ss = state.sys_status
        if ss is None:
            return
        self.battery.publish(
            BatteryState(
                voltage=ss.volt,
                current=ss.current_a,
                percentage=ss.batt_pct / 100.0 if ss.batt_pct >= 0 else math.nan,
                power_supply_status=LCMBatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
                power_supply_technology=LCMBatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO,
                present=True,
                frame_id="battery",
                ts=ss.t,
            )
        )

    def _publish_gimbal(self, state: VehicleState) -> None:
        with state.lock:
            g, flags, failure = state.gimbal, state.gimbal_flags, state.gimbal_failure
        if g is None or time.time() - g.t > _GIMBAL_MAX_AGE_S:
            return
        self.gimbal_attitude.publish(
            JointState(
                ts=g.t,
                frame_id=self.config.gimbal_base_frame_id,
                name=list(_GIMBAL_JOINTS),
                position=[math.radians(g.roll), math.radians(g.pitch), math.radians(g.yaw)],
                velocity=[],
                effort=[float(flags), float(failure), 0.0],
            )
        )

    def _publish_statustext(self, state: VehicleState) -> None:
        with state.lock:
            pending = [s for s in state.statustext if s.seq > self._last_statustext_seq]
        for st in pending:
            self.statustext.publish(String(f"[{st.severity_name}] {st.text}"))
            self._last_statustext_seq = st.seq

    def _publish_status(self, state: VehicleState) -> None:
        core, io = self._core, self._io
        if core is None or io is None:
            return
        now = time.time()
        snap = state.snapshot(now)
        with self._core_lock:
            streaming = core.sp is not None
            estop = core.estop_latched
            core_state = core.state
        writer = io.writer_id if streaming else ""
        with state.lock:
            home, ss = state.home, state.sys_status
        hb = snap.heartbeat
        jitter = _percentiles(list(self._jitter_ms))
        self.vehicle_status.publish(
            VehicleStatus(
                armed=snap.armed,
                main_mode=hb.main if hb else 0,
                sub_mode=hb.sub if hb else 0,
                mode=mode_name(hb.main if hb else None, hb.sub if hb else 0),
                landed_state=snap.landed_state if snap.landed_state is not None else -1,
                battery_pct=ss.batt_pct if ss else -1,
                voltage=ss.volt if ss else math.nan,
                gps_fix=snap.gps.fix if snap.gps else -1,
                gps_sats=snap.gps.sats if snap.gps else -1,
                gps_eph=snap.gps.eph if snap.gps else math.nan,
                rc_age_s=_nan_if_inf(snap.rc_age),
                heartbeat_age_s=_nan_if_inf(snap.heartbeat_age),
                home_valid=home is not None,
                home_lat=home.lat if home else 0.0,
                home_lon=home.lon if home else 0.0,
                home_alt=home.alt if home else 0.0,
                writer=writer,
                state=core_state,
                estop_latched=estop,
                tick_jitter_p99_ms=jitter["p99"] if jitter["p99"] is not None else math.nan,
                frame_id=self.config.base_frame_id,
                ts=now,
            )
        )

    # RPC surface. Note what is absent: arm, set_mode, send_*_setpoint.

    def _result(self, why: Rejection | None) -> dict[str, Any]:
        core = self._core
        return {
            "accepted": why is None,
            "rejection": None if why is None else why.value,
            "state": core.state if core else None,
        }

    def _snap(self) -> VehicleSnapshot:
        assert self._state is not None
        return self._state.snapshot()

    def _command(
        self, run: Callable[[SupervisorCore, MavlinkIO], Rejection | None]
    ) -> dict[str, Any]:
        """One operator command: run it on the core under the lock, answer."""
        core, io = self._core, self._io
        assert core is not None and io is not None
        with self._core_lock:
            why = run(core, io)
        return self._result(why)

    def _estop(self, land: bool = False) -> dict[str, Any]:
        core, io = self._core, self._io
        assert core is not None and io is not None
        with self._core_lock:
            (core.estop_land if land else core.estop)(self._snap(), io)
        logger.warning("E-STOP latched", land=land)
        return self._result(None)

    @rpc
    def takeoff(self, altitude_m: float | None = None) -> dict[str, Any]:
        """Run preflight and, if it passes, stream, enter OFFBOARD, arm and climb to hover.

        ``altitude_m`` is above the ground; None takes ``limits.takeoff_alt_m``.
        """
        return self._command(lambda core, io: core.takeoff_cmd(self._snap(), alt_m=altitude_m))

    @rpc
    def go_to(
        self,
        north_m: float = 0.0,
        east_m: float = 0.0,
        altitude_m: float | None = None,
        heading_deg: float | None = None,
        relative: bool = True,
    ) -> dict[str, Any]:
        """Fly to a point at walking pace and hover there. Only while flying under this module.

        ``north_m``/``east_m`` count from the vehicle (``relative``) or from the takeoff
        point. ``altitude_m`` is above the takeoff point and ``heading_deg`` is a compass
        heading; None keeps the current one. Refused outside the fence or the ceiling.
        """
        return self._command(
            lambda core, io: core.goto_cmd(
                self._snap(), north_m, east_m, altitude_m, heading_deg, relative
            )
        )

    @rpc
    def land(self) -> dict[str, Any]:
        """AUTO.LAND. Refused unless PX4 is in OFFBOARD (the pilot owns it otherwise)."""
        return self._command(lambda core, io: core.land_cmd(self._snap(), io))

    @rpc
    def hold(self) -> dict[str, Any]:
        """Stop streaming and put PX4 in Hold if we are in OFFBOARD; back to IDLE."""
        return self._command(lambda core, io: core.hold_cmd(self._snap(), io))

    @rpc
    def set_guidance_mode(self, mode: str) -> dict[str, Any]:
        """Select HOVER or TELEOP.

        Immediate while flying, otherwise entered ``hover_settle_s`` after the hover is reached.
        """
        m = mode.upper()
        if m not in GUIDANCE_MODES:
            return self._result(Rejection.INVALID_ARGUMENT)
        guidance: GuidanceMode = m  # type: ignore[assignment]
        return self._command(lambda core, io: core.set_guidance_mode(guidance))

    @rpc
    def estop(self) -> dict[str, Any]:
        """Hold plus latch. Synchronous. Nothing moves the aircraft again until estop_clear."""
        return self._estop()

    @rpc
    def estop_land(self) -> dict[str, Any]:
        """AUTO.LAND plus latch."""
        return self._estop(land=True)

    @rpc
    def estop_clear(self) -> dict[str, Any]:
        """Release the latch. Only works in IDLE."""
        return self._command(lambda core, io: core.estop_clear())

    @rpc
    def sitl_enable(self, value: bool) -> dict[str, Any]:
        """SITL only: fake the RC enable switch."""
        return self._command(lambda core, io: core.sitl_enable(value))

    @rpc
    def status(self) -> dict[str, Any]:
        """The supervisor's status dict plus the setpoint writer and the tick jitter."""
        core, io = self._core, self._io
        assert core is not None and io is not None
        snap = self._snap()
        with self._core_lock:
            out = core.status(snap)
        out["writer"] = io.writer_id if out["setpoint"] is not None else None
        out["tick_jitter_ms"] = _percentiles(list(self._jitter_ms))
        return out

    @rpc
    def sensor_stats(self) -> dict[str, Any]:
        """Per-message counters and ages, ack results, cmd_vel verdicts."""
        io = self._io
        out: dict[str, Any] = {} if io is None else io.stats()
        out["cmd_vel"] = {
            "accepted": self._cmd_vel_accepted,
            "rejected": dict(self._cmd_vel_rejections),
        }
        out["tick_jitter_ms"] = _percentiles(list(self._jitter_ms))
        return out


def _percentiles(samples: list[float]) -> dict[str, float | None]:
    if not samples:
        return {"p50": None, "p99": None, "max": None}
    p50, p99, top = (float(v) for v in np.percentile(samples, [50, 99, 100]))
    return {"p50": p50, "p99": p99, "max": top}
