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

"""Bus constants and flight limits (module wiring lives in ``connection.py``)."""

from __future__ import annotations

from dataclasses import dataclass, field

# MAVLink identities on the vehicle bus. PX4 is 1/1; a MAVLink gimbal (the SIYI A8) answers
# as 1/154 (MAV_COMP_ID_GIMBAL) through PX4.
PX4_SYSID = 1
PX4_COMPID = 1
GIMBAL_COMPID = 154
DIMOS_COMPID = 195

# Our own mavlink-routerd endpoint on the machine running dimOS (loopback; one port per
# MAVLink service, never shared). The same URL on a companion computer (router on PX4's
# serial port) and on a laptop (router on a telemetry radio): only the link profile differs.
ROUTER_MAV_URL = "udpin:127.0.0.1:14556"
# PX4 SITL (`make px4_sitl gz_x500`) sends its onboard-API stream to 14540.
SITL_MAV_URL = "udpin:0.0.0.0:14540"

# Message rates asked of PX4 at start (SET_MESSAGE_INTERVAL, Hz), per link. An Onboard-mode
# port (TELEM2, SITL) already streams what the stack reads. A Normal-mode port (TELEM1, a
# telemetry radio) sends LOCAL_POSITION_NED at 1 Hz, too slow for the freshness checks.
LINK_STREAM_HZ: dict[str, dict[str, float]] = {
    "onboard": {},
    "radio": {"LOCAL_POSITION_NED": 10.0},
}

# Host-local mutex: one Offboard writer per machine (pymavlink's udpin sets SO_REUSEADDR; this does not).
WRITER_LOCK_PORT = 5610


@dataclass(frozen=True)
class GotoConfig:
    """Gains and speed caps of the GOTO law."""

    k_pos: float = 0.6
    k_alt: float = 0.6
    v_max_mps: float = 1.0
    vz_max_mps: float = 0.7
    max_yaw_rate_dps: float = 30.0
    yaw_tolerance_deg: float = 5.0
    # A goal never reached (wind, a PX4 limit) ends in HOVER where the vehicle is. Goals sit
    # goal_margin_m inside the fence, so at most 56 m apart: 56 s at v_max_mps.
    timeout_s: float = 75.0


@dataclass(frozen=True)
class GuidanceConfig:
    goto: GotoConfig = field(default_factory=GotoConfig)


@dataclass(frozen=True)
class SupervisorLimits:
    """Conservative first-flight values."""

    takeoff_alt_m: float = 3.0
    climb_rate_mps: float = 0.7
    max_alt_m: float = 15.0
    geofence_radius_m: float = 30.0
    # Operator-chosen altitudes and go-to goals: no lower than min_alt_m, and this far
    # inside the ceiling and the fence so an overshoot never trips the abort rule.
    min_alt_m: float = 1.0
    goal_margin_m: float = 2.0
    min_batt_pct: int = 40
    # No battery reading (no SYS_STATUS yet, or PX4 reports -1: no power module) refuses
    # takeoff. Off only for a vehicle that cannot measure its pack.
    require_battery: bool = True
    min_fix_type: int = 3
    # The receiver's own horizontal accuracy (GPS_RAW_INT.h_acc), metres; not HDOP.
    max_eph_m: float = 1.5
    # RC enable switch: high above 1500 us.
    enable_channel: int = 7
    enable_threshold_us: int = 1500
    rc_stale_s: float = 1.0
    px4_stale_s: float = 1.0
    setpoint_hz: float = 20.0
    prestream_s: float = 1.5
    hover_tolerance_m: float = 0.5
    hover_settle_s: float = 3.0
    ack_timeout_s: float = 3.0
    teleop_v_xy_mps: float = 1.5
    teleop_v_z_mps: float = 0.7
    teleop_yaw_rate_rps: float = 0.8
    teleop_stale_s: float = 0.5
    # Locked, TELEOP holds the altitude it started at (by teleop_k_alt) and ignores the
    # up axis; the viewer's keyboard has no up or down key.
    teleop_lock_altitude: bool = True
    teleop_k_alt: float = 0.6
