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

"""Status of the dimOS PX4 connection and its flight supervisor.

Hand-written like ``sensor_msgs.ImuInfo``. LCM schema (fingerprint-compatible bindings
must be generated from this)::

    package px4_msgs;

    struct VehicleStatus {
        std_msgs.Header header;
        boolean armed;
        byte    main_mode;
        byte    sub_mode;
        int8_t  landed_state;
        int8_t  battery_pct;
        float   voltage;
        int8_t  gps_fix;
        int8_t  gps_sats;
        float   gps_eph;
        float   rc_age_s;
        float   heartbeat_age_s;
        boolean home_valid;
        double  home_lat;
        double  home_lon;
        float   home_alt;
        double  timebase_offset_s;
        boolean estop_latched;
        float   tick_jitter_p99_ms;
        string  mode;
        string  timebase_quality;
        string  writer;
        string  state;
    }
"""

from __future__ import annotations

from dataclasses import dataclass, field
from io import BytesIO
import math
import struct
import time
from typing import Any

from dimos_lcm.std_msgs.Header import Header

from dimos.types.timestamped import Timestamped

# lcm-gen's base hash for the struct above; the wire fingerprint mixes in Header's.
_BASE_HASH = 0x4BD2FE8251828D95
# The schema's fixed fields in order, then its strings.
_FIXED = struct.Struct(">?BBbbfbbfff?ddfd?f")
_FIXED_FIELDS = (
    "armed main_mode sub_mode landed_state battery_pct voltage gps_fix gps_sats gps_eph "
    "rc_age_s heartbeat_age_s home_valid home_lat home_lon home_alt timebase_offset_s "
    "estop_latched tick_jitter_p99_ms"
).split()
_STR_FIELDS = ("mode", "timebase_quality", "writer", "state")


def _packed_fingerprint() -> bytes:
    tmphash = (_BASE_HASH + Header._get_hash_recursive([VehicleStatus])) & 0xFFFFFFFFFFFFFFFF
    tmphash = (((tmphash << 1) & 0xFFFFFFFFFFFFFFFF) + (tmphash >> 63)) & 0xFFFFFFFFFFFFFFFF
    return struct.pack(">Q", tmphash)


def _write_str(buf: BytesIO, s: str) -> None:
    raw = s.encode("utf-8")
    buf.write(struct.pack(">I", len(raw) + 1))
    buf.write(raw)
    buf.write(b"\0")


def _read_str(buf: BytesIO) -> str:
    (n,) = struct.unpack(">I", buf.read(4))
    return buf.read(n)[:-1].decode("utf-8")


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
        header = Header()
        header.seq = 0
        header.frame_id = self.frame_id
        header.stamp.sec = int(self.ts)
        header.stamp.nsec = int((self.ts - int(self.ts)) * 1e9)
        buf = BytesIO()
        buf.write(_packed_fingerprint())
        header._encode_one(buf)
        buf.write(_FIXED.pack(*(getattr(self, f) for f in _FIXED_FIELDS)))
        for f in _STR_FIELDS:
            _write_str(buf, getattr(self, f))
        return buf.getvalue()

    @classmethod
    def lcm_decode(cls, data: bytes) -> VehicleStatus:
        buf = BytesIO(data)
        if buf.read(8) != _packed_fingerprint():
            raise ValueError("VehicleStatus: fingerprint mismatch")
        header = Header._decode_one(buf)
        fixed = dict(zip(_FIXED_FIELDS, _FIXED.unpack(buf.read(_FIXED.size)), strict=True))
        strings: dict[str, Any] = {f: _read_str(buf) for f in _STR_FIELDS}
        return cls(
            **fixed,
            **strings,
            frame_id=str(header.frame_id),
            ts=header.stamp.sec + header.stamp.nsec / 1e9,
        )

    def to_rerun(self) -> Any:
        import rerun as rr

        level = "ERROR" if self.estop_latched else ("WARN" if self.armed else "INFO")
        return rr.TextLog(
            f"{self.state} {self.mode} armed={self.armed} batt={self.battery_pct}% "
            f"gps={self.gps_fix}/{self.gps_sats} writer={self.writer or '-'}",
            level=level,
        )
