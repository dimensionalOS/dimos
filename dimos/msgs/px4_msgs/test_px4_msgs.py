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

import math

import pytest

from dimos.msgs.px4_msgs.VehicleStatus import VehicleStatus
from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix


def test_vehicle_status_roundtrip() -> None:
    # Every field off its default, float32 fields exactly representable: == catches a mix-up.
    msg = VehicleStatus(
        armed=True,
        main_mode=6,
        sub_mode=3,
        mode="OFFBOARD",
        landed_state=2,
        battery_pct=73,
        voltage=15.75,
        gps_fix=6,
        gps_sats=27,
        gps_eph=0.5,
        rc_age_s=0.0625,
        heartbeat_age_s=0.375,
        home_valid=True,
        home_lat=37.7749,
        home_lon=-122.4194,
        home_alt=12.5,
        timebase_quality="system_time",
        timebase_offset_s=1_800_000_000.25,
        writer="1/195",
        state="HOVER",
        estop_latched=True,
        tick_jitter_p99_ms=1.25,
        frame_id="base_link",
        ts=1700000000.5,
    )
    assert VehicleStatus.lcm_decode(msg.lcm_encode()) == msg


def test_vehicle_status_unknowns_survive() -> None:
    back = VehicleStatus.lcm_decode(VehicleStatus(ts=1.0).lcm_encode())
    assert math.isnan(back.voltage) and math.isnan(back.rc_age_s)
    assert not (back.armed or back.home_valid or back.estop_latched)
    assert back.battery_pct == -1 and back.gps_fix == -1 and back.writer == ""


def test_fingerprints_differ_between_types() -> None:
    with pytest.raises(ValueError):
        VehicleStatus.lcm_decode(NavSatFix().lcm_encode())
