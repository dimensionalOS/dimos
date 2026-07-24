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

"""M20-specific status message.

The robot's BasicStatus report (Type 1002 / Command 6) carries a bag of
state-machine enums that have no ROS equivalent, so we define a custom type.
Use native ROS types (BatteryState, Imu, Twist, ...) for everything that has a
standard schema.

The documented enum values (manual V0.0.4) are below for reference, but the M20
Pro firmware reports values outside them (e.g. motion_state=17, gait=4097), so
the fields are kept as raw ints rather than validated enums.
"""

from __future__ import annotations

import struct
import time

from dimos.types.timestamped import Timestamped

# MotionState (documented): 0 idle, 1 stand, 2 soft-estop, 3 damping, 4 sit, 6 standard
# Gait (documented): 1 basic, 14 stair
# ControlUsageMode: 0 regular, 1 navigation
# Charge: 0 idle, 1 entering dock, 2 charging, 3 exiting dock, 4 error, 5 docked-not-charging
# Direction: 0 front, 1 back

_FMT = "<d6i2?H"  # ts, 6 int32 fields, 2 bools, version-length


class M20Status(Timestamped):
    """High-level M20 state machine status (BasicStatus report)."""

    msg_name = "deeprobotics.m20.M20Status"

    def __init__(
        self,
        motion_state: int = 0,
        gait: int = 0,
        control_mode: int = 0,
        charge_state: int = 0,
        direction: int = 0,
        status_code: int = 0,
        hard_estop: bool = False,
        sleep: bool = False,
        version: str = "",
        ts: float | None = None,
    ) -> None:
        self.ts = ts if ts is not None else time.time()
        self.motion_state = motion_state
        self.gait = gait
        self.control_mode = control_mode
        self.charge_state = charge_state
        self.direction = direction
        self.status_code = status_code
        self.hard_estop = hard_estop
        self.sleep = sleep
        self.version = version

    def lcm_encode(self) -> bytes:
        v = self.version.encode("utf-8")
        head = struct.pack(
            _FMT,
            self.ts,
            self.motion_state,
            self.gait,
            self.control_mode,
            self.charge_state,
            self.direction,
            self.status_code,
            self.hard_estop,
            self.sleep,
            len(v),
        )
        return head + v

    @classmethod
    def lcm_decode(cls, data: bytes) -> M20Status:
        n = struct.calcsize(_FMT)
        ts, ms, gait, cm, cs, direction, code, hes, sleep, vlen = struct.unpack_from(_FMT, data, 0)
        return cls(
            motion_state=ms,
            gait=gait,
            control_mode=cm,
            charge_state=cs,
            direction=direction,
            status_code=code,
            hard_estop=bool(hes),
            sleep=bool(sleep),
            version=data[n : n + vlen].decode("utf-8"),
            ts=ts,
        )

    def __str__(self) -> str:
        return (
            f"M20Status(motion_state={self.motion_state}, gait={self.gait}, "
            f"mode={self.control_mode}, charge={self.charge_state}, "
            f"hard_estop={self.hard_estop}, version='{self.version}')"
        )
