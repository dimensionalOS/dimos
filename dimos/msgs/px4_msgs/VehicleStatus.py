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

from __future__ import annotations

from dataclasses import dataclass, field
from io import BytesIO
import math
import struct
import time
from typing import Any

from dimos.msgs.lcm_wire import (
    check_fingerprint,
    fingerprint,
    read_header,
    read_str,
    write_header,
    write_str,
)
from dimos.types.timestamped import Timestamped

_BASE_HASH = 0x5A2E1B7C90D4F318
_FIXED = ">?BBbbfbbfff?ddfd?f"
_FIXED_SIZE = struct.calcsize(_FIXED)


@dataclass
class VehicleStatus(Timestamped):
    """Status of the dimOS PX4 connection and supervisor (not PX4's uORB vehicle_status).

    Unknown numbers are NaN (floats) or -1 (small ints); ``writer`` is empty when nobody
    is streaming Offboard setpoints.
    """

    msg_name = "px4_msgs.VehicleStatus"

    armed: bool = False
    main_mode: int = 0
    sub_mode: int = 0
    mode: str = "?"
    landed_state: int = -1
    battery_pct: int = -1
    voltage: float = math.nan
    gps_fix: int = -1
    gps_sats: int = -1
    gps_eph: float = math.nan  # metres, the receiver's horizontal accuracy (h_acc), not HDOP
    rc_age_s: float = math.nan
    heartbeat_age_s: float = math.nan
    home_valid: bool = False
    home_lat: float = 0.0
    home_lon: float = 0.0
    home_alt: float = 0.0
    timebase_quality: str = "none"
    timebase_offset_s: float = 0.0
    writer: str = ""
    state: str = "IDLE"
    estop_latched: bool = False
    tick_jitter_p99_ms: float = math.nan
    frame_id: str = ""
    ts: float = field(default_factory=time.time)

    def lcm_encode(self) -> bytes:
        buf = BytesIO()
        buf.write(fingerprint(_BASE_HASH, VehicleStatus))
        write_header(buf, self.ts, self.frame_id)
        buf.write(
            struct.pack(
                _FIXED,
                self.armed,
                self.main_mode,
                self.sub_mode,
                self.landed_state,
                self.battery_pct,
                self.voltage,
                self.gps_fix,
                self.gps_sats,
                self.gps_eph,
                self.rc_age_s,
                self.heartbeat_age_s,
                self.home_valid,
                self.home_lat,
                self.home_lon,
                self.home_alt,
                self.timebase_offset_s,
                self.estop_latched,
                self.tick_jitter_p99_ms,
            )
        )
        for s in (self.mode, self.timebase_quality, self.writer, self.state):
            write_str(buf, s)
        return buf.getvalue()

    @classmethod
    def lcm_decode(cls, data: bytes) -> VehicleStatus:
        buf = BytesIO(data)
        check_fingerprint(buf, fingerprint(_BASE_HASH, VehicleStatus), "VehicleStatus")
        ts, frame_id = read_header(buf)
        v = struct.unpack(_FIXED, buf.read(_FIXED_SIZE))
        mode, quality, writer, state = (read_str(buf) for _ in range(4))
        return cls(
            armed=v[0],
            main_mode=v[1],
            sub_mode=v[2],
            landed_state=v[3],
            battery_pct=v[4],
            voltage=v[5],
            gps_fix=v[6],
            gps_sats=v[7],
            gps_eph=v[8],
            rc_age_s=v[9],
            heartbeat_age_s=v[10],
            home_valid=v[11],
            home_lat=v[12],
            home_lon=v[13],
            home_alt=v[14],
            timebase_offset_s=v[15],
            estop_latched=v[16],
            tick_jitter_p99_ms=v[17],
            mode=mode,
            timebase_quality=quality,
            writer=writer,
            state=state,
            frame_id=frame_id,
            ts=ts,
        )

    def to_rerun(self) -> Any:
        import rerun as rr

        level = "ERROR" if self.estop_latched else ("WARN" if self.armed else "INFO")
        return rr.TextLog(
            f"{self.state} {self.mode} armed={self.armed} batt={self.battery_pct}% "
            f"gps={self.gps_fix}/{self.gps_sats} writer={self.writer or '-'}",
            level=level,
        )
