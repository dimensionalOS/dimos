#!/usr/bin/env python3
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

"""PX4 SITL flavor of MavlinkConnection.

PX4 encodes flight modes differently from ArduPilot, so the existing
``MavlinkConnection.set_mode()`` (ArduPilot custom mode IDs) does not work.
This subclass overrides ``set_mode``, ``takeoff``, ``land``, and ``rtl`` for
PX4 semantics, plus adds an OFFBOARD setpoint streamer required for any
external position/velocity control.

Typical SITL endpoints (from PX4 ``px4-rc.mavlink``):
    instance N → ``udp:127.0.0.1:1454N`` (offboard SDK)
    sysid       = N + 1
"""

from __future__ import annotations

import os
import threading
import time

from pymavlink import mavutil  # type: ignore[import-not-found, import-untyped]

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.drone.mavlink_connection import MavlinkConnection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _env_float(name: str, default: float) -> float:
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError:
        logger.warning(f"Invalid {name}={raw!r}; using default {default}")
        return default


# Vision-tracking safety knobs. Env-overridable so the drone can be tuned in the
# field without code edits (sign flips, speed caps, watchdog timeout).
#   * WATCHDOG_S: if no fresh body-velocity setpoint arrives within this window
#     (tracker died / video froze), the OFFBOARD streamer commands zero velocity
#     (hover-hold) instead of repeating the last command — prevents runaway.
TRACK_BODY_SETPOINT_TIMEOUT_S = _env_float("DIMOS_DRONE_TRACK_WATCHDOG_S", 0.5)
TRACK_MAX_HORIZONTAL_V = _env_float("DIMOS_DRONE_TRACK_MAX_V", 3.0)  # m/s, fwd & strafe
TRACK_MAX_VERTICAL_V = _env_float("DIMOS_DRONE_TRACK_MAX_VZ", 1.0)  # m/s
TRACK_MAX_YAW_RATE = _env_float("DIMOS_DRONE_TRACK_MAX_YAW", 1.0)  # rad/s
TRACK_INVERT_YAW = os.getenv("DIMOS_DRONE_TRACK_INVERT_YAW", "0") == "1"
TRACK_INVERT_LATERAL = os.getenv("DIMOS_DRONE_TRACK_INVERT_LATERAL", "0") == "1"


# PX4 main modes (commander/px4_custom_mode.h)
PX4_CUSTOM_MAIN_MODE_MANUAL = 1
PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
PX4_CUSTOM_MAIN_MODE_POSCTL = 3
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_MAIN_MODE_ACRO = 5
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
PX4_CUSTOM_MAIN_MODE_STABILIZED = 7

# PX4 AUTO sub-modes
PX4_CUSTOM_SUB_MODE_AUTO_READY = 1
PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3
PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5
PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6

# Friendly mode name → (main, sub)
PX4_MODE_TABLE: dict[str, tuple[int, int]] = {
    "MANUAL": (PX4_CUSTOM_MAIN_MODE_MANUAL, 0),
    "POSCTL": (PX4_CUSTOM_MAIN_MODE_POSCTL, 0),
    "ALTCTL": (PX4_CUSTOM_MAIN_MODE_ALTCTL, 0),
    "OFFBOARD": (PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0),
    "STABILIZED": (PX4_CUSTOM_MAIN_MODE_STABILIZED, 0),
    "AUTO.LOITER": (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER),
    "AUTO.TAKEOFF": (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF),
    "AUTO.LAND": (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND),
    "AUTO.RTL": (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL),
    "AUTO.MISSION": (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_MISSION),
}


def default_offboard_endpoint(instance: int = 0, host: str = "127.0.0.1") -> str:
    """Return the PX4 SITL offboard MAVLink endpoint for a given instance."""
    return f"udp:{host}:{14540 + instance}"


class Px4SitlConnection(MavlinkConnection):
    """MAVLink connection tuned for PX4 SITL.

    Adds:
      * PX4 mode encoding (main_mode/sub_mode in MAV_CMD_DO_SET_MODE).
      * OFFBOARD setpoint streamer that keeps PX4 in OFFBOARD by re-sending
        the latest setpoint at ≥ 2 Hz (PX4 falls out of OFFBOARD otherwise).
      * RTL helper.
    """

    OFFBOARD_STREAM_HZ = 20.0  # comfortably above PX4's 2 Hz minimum

    def __init__(
        self,
        connection_string: str | None = None,
        instance: int = 0,
        outdoor: bool = False,
        max_velocity: float = 5.0,
    ) -> None:
        if connection_string is None:
            connection_string = default_offboard_endpoint(instance)
        # PX4 SITL maps instance N to MAV_SYS_ID = N + 1. Pass it down so
        # MavlinkConnection.connect() can skip HEARTBEAT discovery on instances
        # >= 1 (which don't stream HEARTBEAT on their Onboard channel reliably).
        super().__init__(
            connection_string=connection_string,
            outdoor=outdoor,
            max_velocity=max_velocity,
            target_system=instance + 1,
            # GCS presence is fleet-level, not per-drone: every PX4 instance
            # expects its ground station at the ONE shared port 14550, so
            # SwarmCoordinator._gcs_presence_loop owns that socket. Per-drone
            # attempts (pushing at 18570+i, or binding 14550+i) are both dead
            # ends; see the notes on that loop.
        )
        self.instance = instance
        self._offboard_setpoint: tuple[float, float, float, float] | None = None  # (vn, ve, vd, yaw_rate)
        # Body-frame velocity setpoint (vx_fwd, vy_right, vz_down, yaw_rate). Used by
        # vision tracking, which thinks in "forward/strafe relative to the nose" + yaw.
        self._offboard_body_setpoint: tuple[float, float, float, float] | None = None
        # Monotonic timestamp of the last body setpoint update, for the watchdog.
        self._body_setpoint_ts: float = 0.0
        self._offboard_position: tuple[float, float, float, float] | None = None  # (n, e, d, yaw)
        # Path mode: a list of (n, e, d, yaw) waypoints, plus current index + arrival radius.
        # When _offboard_path is non-empty, the streamer walks the list, advancing once
        # the drone is within _path_arrival_radius_m of the current target. After the
        # last waypoint the drone holds in place (path becomes a sticky setpoint).
        self._offboard_path: list[tuple[float, float, float, float]] | None = None
        self._path_index: int = 0
        self._path_arrival_radius_m: float = 2.0
        self._offboard_thread: threading.Thread | None = None
        self._offboard_running = False
        self._offboard_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Mode handling (PX4-specific)
    # ------------------------------------------------------------------

    def set_mode(self, mode: str) -> bool:
        """Set PX4 flight mode by friendly name (e.g. ``OFFBOARD``, ``AUTO.RTL``)."""
        if not self.connected:
            return False
        if mode not in PX4_MODE_TABLE:
            logger.error(f"Unknown PX4 mode: {mode}. Valid: {sorted(PX4_MODE_TABLE)}")
            return False
        main, sub = PX4_MODE_TABLE[mode]
        return self._set_mode_px4(main, sub, label=mode)

    def _set_mode_px4(self, main: int, sub: int, label: str = "") -> bool:
        logger.info(f"PX4 set_mode → {label or f'main={main} sub={sub}'}")
        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            float(main),
            float(sub),
            0,
            0,
            0,
            0,
        )
        ack = self.mavlink.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return True
        logger.warning(f"set_mode {label} not accepted (ack={ack})")
        return False

    # ------------------------------------------------------------------
    # Auto skills (mode-driven; no setpoint stream required)
    # ------------------------------------------------------------------

    def takeoff(self, altitude: float = 3.0) -> bool:
        """Arm and switch to AUTO.TAKEOFF at the requested relative altitude."""
        if not self.connected:
            return False
        # Any leftover OFFBOARD setpoint stream from a previous skill (grid_sweep,
        # goto_drone, etc.) must be torn down BEFORE we re-arm, or PX4 will keep
        # chasing stale waypoints the moment it accepts the new mode.
        self.stop_offboard()
        # Clear whatever nav state the last command left behind. After an RTL
        # completes and the vehicle auto-disarms, PX4 stays latched in AUTO.RTL
        # and denies the next arm with "Resolve system health failures first" --
        # with no Preflight Fail line to explain it. Dropping to AUTO.LOITER
        # first makes the obvious demo loop (take off, RTL, take off again)
        # work. Best effort: a failed mode switch is not itself fatal.
        if not self.set_mode("AUTO.LOITER"):
            logger.debug("takeoff: could not pre-set AUTO.LOITER; arming anyway")
        if not self.arm():
            logger.error("PX4 takeoff failed: arm rejected")
            return False
        # MAV_CMD_NAV_TAKEOFF param7 is altitude ABOVE MEAN SEA LEVEL, not above
        # home. Passing a relative altitude makes PX4 compare, say, 5 m AMSL
        # against a vehicle already sitting at 488 m and reply "Already higher
        # than takeoff altitude" -- it silently does nothing. Convert using the
        # home altitude implied by telemetry: home_amsl = alt - relative_alt.
        target_amsl = self._relative_to_amsl(altitude)
        if target_amsl is None:
            logger.error(
                "PX4 takeoff failed: no GLOBAL_POSITION_INT yet, cannot convert "
                f"{altitude}m relative to an absolute altitude"
            )
            return False
        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            float("nan"),  # yaw → leave unchanged
            float("nan"),  # lat → use current
            float("nan"),  # lon → use current
            target_amsl,
        )
        logger.info(
            f"PX4 takeoff command sent (altitude={altitude}m relative "
            f"-> {target_amsl:.1f}m AMSL)"
        )
        return True

    def _relative_to_amsl(self, relative_alt_m: float) -> float | None:
        """Convert an altitude above home into altitude above mean sea level.

        Returns None when no GLOBAL_POSITION_INT has arrived yet, since guessing
        an absolute altitude would send the vehicle somewhere unintended.
        """
        gpi = self.telemetry.get("GLOBAL_POSITION_INT")
        if not gpi:
            return None
        amsl = gpi.get("alt")
        rel = gpi.get("relative_alt")
        if amsl is None or rel is None:
            return None
        return float(amsl) - float(rel) + relative_alt_m

    def land(self) -> bool:
        """Switch to AUTO.LAND."""
        if not self.connected:
            return False
        # Same reasoning as takeoff: stop the OFFBOARD streamer so PX4 isn't
        # being pulled in two directions while it tries to land.
        self.stop_offboard()
        return self.set_mode("AUTO.LAND")

    def rtl(self) -> bool:
        """Switch to AUTO.RTL."""
        if not self.connected:
            return False
        self.stop_offboard()
        return self.set_mode("AUTO.RTL")

    def hold(self) -> bool:
        """Switch to AUTO.LOITER (hover in place)."""
        if not self.connected:
            return False
        self.stop_offboard()
        return self.set_mode("AUTO.LOITER")

    # ------------------------------------------------------------------
    # OFFBOARD setpoint streaming
    # ------------------------------------------------------------------

    def start_offboard(
        self,
        initial_setpoint: tuple[float, float, float, float] | None = None,
    ) -> bool:
        """Begin streaming OFFBOARD setpoints and switch PX4 into OFFBOARD mode.

        Args:
            initial_setpoint: (vn, ve, vd, yaw_rate). Defaults to hover (all zeros).

        Returns:
            True if PX4 accepted the OFFBOARD mode switch.

        Failure semantics: on ``set_mode("OFFBOARD")`` rejection we **tear the
        streamer back down** so a subsequent call retries from a clean slate.
        Previously a failed mode switch left ``_offboard_running=True`` with a
        live thread but PX4 not in OFFBOARD — later callers checked
        ``if not _offboard_running`` and skipped the retry, leaving the drone
        stranded.
        """
        if self._offboard_running:
            return True

        with self._offboard_lock:
            self._offboard_setpoint = initial_setpoint or (0.0, 0.0, 0.0, 0.0)
            self._offboard_body_setpoint = None
            self._offboard_position = None
            self._offboard_path = None
        self._offboard_running = True
        self._offboard_thread = threading.Thread(
            target=self._offboard_loop, daemon=True, name=f"px4-offboard-{self.instance}"
        )
        self._offboard_thread.start()

        # PX4 needs to see ≥ 1 second of setpoints before it accepts OFFBOARD.
        time.sleep(1.1)
        if self.set_mode("OFFBOARD"):
            return True

        # Mode switch rejected — undo so the next call retries cleanly.
        logger.warning(
            "start_offboard: PX4 rejected mode switch, tearing streamer down for retry"
        )
        self.stop_offboard()
        return False

    def stop_offboard(self) -> None:
        """Halt the OFFBOARD setpoint streamer and clear any pending setpoints.

        Safe to call when the streamer isn't running. Always clears
        ``_offboard_path``, ``_offboard_position``, and ``_offboard_setpoint``
        so a subsequent ``start_offboard()`` starts from a clean slate (no stale
        waypoints leaking between commands).
        """
        self._offboard_running = False
        if self._offboard_thread and self._offboard_thread.is_alive():
            self._offboard_thread.join(timeout=1.0)
        self._offboard_thread = None
        with self._offboard_lock:
            self._offboard_path = None
            self._path_index = 0
            self._offboard_position = None
            self._offboard_setpoint = None

    def set_velocity_ned(
        self, vn: float, ve: float, vd: float, yaw_rate: float = 0.0
    ) -> None:
        """Update the streamed OFFBOARD velocity setpoint (NED frame, m/s).

        Cancels any active position setpoint AND any active path — the streamer
        will switch to pure velocity mode on the next tick.
        """
        with self._offboard_lock:
            self._offboard_setpoint = (vn, ve, vd, yaw_rate)
            self._offboard_body_setpoint = None
            self._offboard_position = None
            self._offboard_path = None

    def set_position_ned(
        self, n: float, e: float, d: float, yaw: float = 0.0
    ) -> None:
        """Update the streamed OFFBOARD position setpoint (local NED, meters).

        ``d`` is positive-down (NED). To fly to 5m altitude, pass ``d = -5``.
        """
        with self._offboard_lock:
            self._offboard_position = (n, e, d, yaw)
            self._offboard_setpoint = None
            self._offboard_body_setpoint = None
            self._offboard_path = None  # explicit position cancels any active path

    def set_position_path(
        self,
        waypoints: list[tuple[float, float, float, float]],
        arrival_radius_m: float = 2.0,
    ) -> None:
        """Stream a sequence of OFFBOARD position waypoints.

        Each waypoint is (n, e, d, yaw) in local NED. ``d`` is positive-down.
        The streamer holds each waypoint until the drone is within
        ``arrival_radius_m`` of it, then advances. After the final waypoint
        the drone hovers at that point (the path becomes a sticky setpoint).

        This is the foundation for multi-leg patterns like a boustrophedon
        ("lawnmower") sweep — the fleet module builds the list, this method
        executes it.

        Args:
            waypoints: ordered list of (north_m, east_m, down_m, yaw_rad).
                Must contain at least one element.
            arrival_radius_m: 3-D distance (meters) at which a waypoint counts
                as "reached". Default 2 m, which matches PX4 SITL position
                tracking accuracy at typical cruise speeds.

        Raises:
            ValueError: if ``waypoints`` is empty.
        """
        if not waypoints:
            raise ValueError("set_position_path requires at least one waypoint")
        with self._offboard_lock:
            self._offboard_path = list(waypoints)
            self._path_index = 0
            self._path_arrival_radius_m = float(arrival_radius_m)
            # Path mode wins over the single-point modes.
            self._offboard_position = None
            self._offboard_setpoint = None
            self._offboard_body_setpoint = None

    def _path_advance_if_arrived(self, target: tuple[float, float, float, float]) -> None:
        """If the drone is within arrival radius of ``target``, advance the path index.

        Holds at the last waypoint (no wraparound).
        """
        if self._offboard_path is None:
            return
        cur = self.get_local_ned()
        if cur is None:
            return
        dn = cur[0] - target[0]
        de = cur[1] - target[1]
        dd = cur[2] - target[2]
        dist = (dn * dn + de * de + dd * dd) ** 0.5
        if dist <= self._path_arrival_radius_m:
            with self._offboard_lock:
                if (
                    self._offboard_path is not None
                    and self._path_index < len(self._offboard_path) - 1
                ):
                    self._path_index += 1

    def _offboard_loop(self) -> None:
        period = 1.0 / self.OFFBOARD_STREAM_HZ
        while self._offboard_running:
            try:
                with self._offboard_lock:
                    path = self._offboard_path
                    idx = self._path_index
                    pos = self._offboard_position
                    vel = self._offboard_setpoint
                    body_vel = self._offboard_body_setpoint
                if path is not None:
                    target = path[min(idx, len(path) - 1)]
                    self._send_position_target(*target)
                    self._path_advance_if_arrived(target)
                elif pos is not None:
                    self._send_position_target(*pos)
                elif body_vel is not None:
                    send = self._body_setpoint_to_send(time.monotonic())
                    if send is not None:
                        self._send_velocity_target_body(*send)
                else:
                    vn, ve, vd, yaw_rate = vel or (0.0, 0.0, 0.0, 0.0)
                    self._send_velocity_target(vn, ve, vd, yaw_rate)
            except Exception as e:  # never let the streamer thread die silently
                logger.debug(f"offboard stream send error: {e}")
            time.sleep(period)

    def _send_velocity_target(self, vn: float, ve: float, vd: float, yaw_rate: float) -> None:
        # type_mask: ignore position, accel, yaw — use velocity + yaw_rate.
        # Bit layout (MAVLink spec): pos(0-2) vel(3-5) acc(6-8) force(9) yaw(10) yaw_rate(11)
        type_mask = 0b0000_1011_1100_0111
        self.mavlink.mav.set_position_target_local_ned_send(
            0,
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,
            float(vn), float(ve), float(vd),
            0, 0, 0,
            0.0,
            float(yaw_rate),
        )

    def _send_velocity_target_body(
        self, vx: float, vy: float, vz: float, yaw_rate: float
    ) -> None:
        """Stream a BODY-frame velocity setpoint (vx fwd, vy right, vz down).

        Unlike the world-NED ``_send_velocity_target``, this uses
        ``MAV_FRAME_BODY_NED`` so the autopilot rotates the command by the
        drone's current heading — "go forward / strafe relative to the nose".
        The type_mask uses velocity + yaw_rate (ignores position, accel, and the
        absolute yaw angle) so vision tracking can rotate the drone to keep the
        target centred.
        """
        # Bit layout (set = ignore): pos(0-2) vel(3-5) acc(6-8) force(9) yaw(10) yaw_rate(11)
        # Use vel + yaw_rate -> ignore pos, acc, force, and absolute yaw.
        type_mask = 0b0000_0111_1100_0111
        self.mavlink.mav.set_position_target_local_ned_send(
            0,
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,
            float(vx), float(vy), float(vz),
            0, 0, 0,
            0.0,
            float(yaw_rate),
        )

    def set_velocity_body(
        self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0
    ) -> None:
        """Update the streamed OFFBOARD body-frame velocity setpoint (m/s, rad/s).

        vx = forward, vy = right, vz = down (positive). Cancels any active
        world-velocity / position / path setpoint.
        """
        with self._offboard_lock:
            self._offboard_body_setpoint = (vx, vy, vz, yaw_rate)
            self._body_setpoint_ts = time.monotonic()
            self._offboard_setpoint = None
            self._offboard_position = None
            self._offboard_path = None

    def _body_setpoint_to_send(
        self, now: float
    ) -> tuple[float, float, float, float] | None:
        """Resolve the body setpoint to stream, applying the staleness watchdog.

        Returns None when no body setpoint is active. Returns a zero setpoint
        (hover-hold) when the latest setpoint is older than the watchdog window
        — this is the failsafe that stops the drone if the tracker or video feed
        dies mid-flight instead of repeating a stale "keep moving" command.
        """
        with self._offboard_lock:
            sp = self._offboard_body_setpoint
            ts = self._body_setpoint_ts
        if sp is None:
            return None
        if (now - ts) > TRACK_BODY_SETPOINT_TIMEOUT_S:
            return (0.0, 0.0, 0.0, 0.0)
        return sp

    @staticmethod
    def _shape_track_twist(
        twist: Twist, lock_altitude: bool
    ) -> tuple[float, float, float, float]:
        """Convert a tracking Twist into a safe, sign-corrected body setpoint.

        Applies the env-tunable sign inversions and velocity/yaw caps. Pure
        function (no I/O) so it is unit-testable without a live MAVLink link.
        """
        vx = float(twist.linear.x)  # forward
        vy = float(twist.linear.y)  # right
        if TRACK_INVERT_LATERAL:
            vy = -vy
        vz = 0.0 if lock_altitude else -float(twist.linear.z)
        yaw_rate = float(twist.angular.z)
        if TRACK_INVERT_YAW:
            yaw_rate = -yaw_rate

        vx = max(-TRACK_MAX_HORIZONTAL_V, min(TRACK_MAX_HORIZONTAL_V, vx))
        vy = max(-TRACK_MAX_HORIZONTAL_V, min(TRACK_MAX_HORIZONTAL_V, vy))
        vz = max(-TRACK_MAX_VERTICAL_V, min(TRACK_MAX_VERTICAL_V, vz))
        yaw_rate = max(-TRACK_MAX_YAW_RATE, min(TRACK_MAX_YAW_RATE, yaw_rate))
        return vx, vy, vz, yaw_rate

    def move_twist(
        self, twist: Twist, duration: float = 0.0, lock_altitude: bool = True
    ) -> None:
        """Drive the drone from a ROS-style Twist via the OFFBOARD streamer.

        Mirrors the Tello/MAVLink ``move_twist`` contract so the same
        ``DroneTrackingModule`` cmd_vel output works on PX4:
            linear.x  -> forward (body)
            linear.y  -> right (body)
            linear.z  -> up (used only when lock_altitude is False)
            angular.z -> yaw rate (rad/s)  [honoured here, unlike mavlink_connection]

        Sign inversions and speed caps are applied (env-tunable) for safe field
        bring-up, and the OFFBOARD streamer's watchdog will zero the command if
        updates stop arriving.
        """
        vx, vy, vz, yaw_rate = self._shape_track_twist(twist, lock_altitude)

        if not self._offboard_running:
            self.start_offboard((0.0, 0.0, 0.0, 0.0))
        self.set_velocity_body(vx, vy, vz, yaw_rate)

        if duration > 0:
            time.sleep(duration)
            self.set_velocity_body(0.0, 0.0, 0.0, 0.0)

    def _send_position_target(self, n: float, e: float, d: float, yaw: float) -> None:
        # type_mask: use position + yaw, ignore velocity, accel, yaw_rate.
        type_mask = 0b0000_1011_1111_1000
        self.mavlink.mav.set_position_target_local_ned_send(
            0,
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            float(n), float(e), float(d),
            0, 0, 0,
            0, 0, 0,
            float(yaw),
            0.0,
        )

    # ------------------------------------------------------------------
    # Body-frame ROS-style move() override (uses OFFBOARD streamer)
    # ------------------------------------------------------------------

    def move(self, velocity: Vector3, duration: float = 0.0) -> None:
        """Send a body-frame velocity command via OFFBOARD.

        ROS convention: x = forward (m/s), y = left, z = up. Translates to
        NED setpoints based on the latest yaw from telemetry.
        """
        # Convert ROS body to NED. Without yaw rotation, treat x=north, y=-east, z=-down.
        # For body-relative motion we'd need yaw; for now keep it simple and use world NED
        # which matches how OFFBOARD setpoints are expected.
        vn = float(velocity.x)
        ve = -float(velocity.y)
        vd = -float(velocity.z)

        if not self._offboard_running:
            self.start_offboard((vn, ve, vd, 0.0))
        else:
            self.set_velocity_ned(vn, ve, vd)

        if duration > 0:
            time.sleep(duration)
            self.set_velocity_ned(0.0, 0.0, 0.0)

    def disconnect(self) -> None:
        self.stop_offboard()
        if hasattr(super(), "disconnect"):
            super().disconnect()  # type: ignore[misc]
        else:
            self.connected = False

    # ------------------------------------------------------------------
    # Convenience accessors used by fleet aggregation
    # ------------------------------------------------------------------

    def get_local_ned(self) -> tuple[float, float, float] | None:
        """Return current LOCAL_POSITION_NED if available."""
        local = self.telemetry.get("LOCAL_POSITION_NED")
        if not local:
            return None
        return (
            float(local.get("x", 0.0)),
            float(local.get("y", 0.0)),
            float(local.get("z", 0.0)),
        )

    def get_global_position(self) -> tuple[float, float, float] | None:
        """Return current world-frame position as (lat_deg, lon_deg, rel_alt_m).

        Returns None if telemetry hasn't produced a GLOBAL_POSITION_INT yet.

        Note: ``MavlinkConnection.update_telemetry`` already normalises lat/lon
        to degrees and relative_alt to meters in-place when stamping the
        GLOBAL_POSITION_INT message into ``self.telemetry`` (see
        ``mavlink_connection.py`` around line 204). So we just read the fields
        as-is — no further unit conversion.

        This is the *correct* basis for inter-drone distance: each drone's
        LOCAL_POSITION_NED is referenced to its own home, so subtracting two
        local NEDs gives nonsense. Subtracting two lat/lons (Haversine) gives
        real meters.
        """
        gp = self.telemetry.get("GLOBAL_POSITION_INT")
        if not gp:
            return None
        lat = gp.get("lat")
        lon = gp.get("lon")
        if lat is None or lon is None:
            return None
        rel_alt = gp.get("relative_alt", 0) or 0
        return (float(lat), float(lon), float(rel_alt))

    def get_battery_pct(self) -> float | None:
        sys_status = self.telemetry.get("SYS_STATUS")
        if not sys_status:
            return None
        pct = sys_status.get("battery_remaining")
        if pct is None or pct < 0:
            return None
        return float(pct)

    def get_armed(self) -> bool:
        hb = self.telemetry.get("HEARTBEAT")
        if not hb:
            return False
        return bool(hb.get("armed", False))

    def get_mode(self) -> str | None:
        """Current PX4 flight mode as a friendly name, e.g. ``AUTO.RTL``.

        Without this an operator cannot tell OFFBOARD from AUTO.RTL in fleet
        telemetry, which is what made a latched AUTO.RTL (a drone that silently
        refuses to arm again) so hard to diagnose: every other field looked
        healthy.

        PX4 packs the mode into HEARTBEAT.custom_mode as
        ``main = (custom_mode >> 16) & 0xFF`` and ``sub = (custom_mode >> 24) & 0xFF``.
        """
        hb = self.telemetry.get("HEARTBEAT")
        if not hb:
            return None
        custom = hb.get("custom_mode")
        if custom is None or custom < 0:
            return None
        main = (int(custom) >> 16) & 0xFF
        sub = (int(custom) >> 24) & 0xFF
        for name, (m, sm) in PX4_MODE_TABLE.items():
            if m == main and sm == sub:
                return name
        # Unmapped combinations are real (e.g. AUTO.MISSION); report the raw
        # numbers rather than None so the operator still sees a change.
        return f"UNKNOWN({main}.{sub})"


__all__ = [
    "PX4_MODE_TABLE",
    "Px4SitlConnection",
    "default_offboard_endpoint",
]
