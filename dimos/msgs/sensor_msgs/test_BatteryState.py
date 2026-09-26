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

from dimos_lcm.sensor_msgs.BatteryState import BatteryState as LCMBatteryState

from dimos.msgs.sensor_msgs.BatteryState import BatteryState


def test_lcm_roundtrip() -> None:
    # Every field set, floats exactly representable in float32: == catches a mix-up.
    batt = BatteryState(
        voltage=15.75,
        current=-12.5,
        percentage=0.75,
        temperature=30.5,
        charge=2.5,
        capacity=5.0,
        design_capacity=5.5,
        power_supply_status=LCMBatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
        power_supply_health=LCMBatteryState.POWER_SUPPLY_HEALTH_GOOD,
        power_supply_technology=LCMBatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO,
        present=True,
        cell_voltage=[3.75, 4.0],
        cell_temperature=[30.5],
        location="bay0",
        serial_number="sn1",
        frame_id="battery",
        ts=1700000000.5,
    )
    assert BatteryState.lcm_decode(batt.lcm_encode()) == batt


def test_unknowns_survive() -> None:
    back = BatteryState.lcm_decode(BatteryState(ts=1.0).lcm_encode())
    assert math.isnan(back.voltage) and math.isnan(back.percentage)
