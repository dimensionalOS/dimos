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
import time

from dimos_lcm.sensor_msgs import BatteryState as LCMBatteryState

from dimos.types.timestamped import Timestamped


class BatteryState(Timestamped):
    """ROS `sensor_msgs/BatteryState`, SI units; NaN for what the pack does not report."""

    msg_name = "sensor_msgs.BatteryState"
    ts: float
    frame_id: str
    voltage: float
    current: float
    percentage: float  # 0..1
    temperature: float
    power_supply_status: int
    cell_voltage: list[float]

    def __init__(
        self,
        ts: float | None = None,
        frame_id: str = "",
        voltage: float = math.nan,
        current: float = math.nan,
        percentage: float = math.nan,
        temperature: float = math.nan,
        power_supply_status: int = 0,
        cell_voltage: list[float] | None = None,
    ) -> None:
        self.ts = time.time() if ts is None else ts
        self.frame_id = frame_id
        self.voltage = voltage
        self.current = current
        self.percentage = percentage
        self.temperature = temperature
        self.power_supply_status = power_supply_status
        self.cell_voltage = cell_voltage if cell_voltage is not None else []

    def lcm_encode(self) -> bytes:
        msg = LCMBatteryState()
        msg.header.stamp.sec = int(self.ts)
        msg.header.stamp.nsec = int((self.ts - int(self.ts)) * 1_000_000_000)
        msg.header.frame_id = self.frame_id
        msg.voltage = self.voltage
        msg.current = self.current
        msg.percentage = self.percentage
        msg.temperature = self.temperature
        msg.charge = msg.capacity = msg.design_capacity = math.nan
        msg.power_supply_status = self.power_supply_status
        msg.present = True
        msg.cell_voltage_length = len(self.cell_voltage)
        msg.cell_voltage = self.cell_voltage
        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> BatteryState:
        msg = LCMBatteryState.lcm_decode(data)
        return cls(
            ts=msg.header.stamp.sec + msg.header.stamp.nsec / 1_000_000_000,
            frame_id=msg.header.frame_id,
            voltage=msg.voltage,
            current=msg.current,
            percentage=msg.percentage,
            temperature=msg.temperature,
            power_supply_status=msg.power_supply_status,
            cell_voltage=list(msg.cell_voltage) if msg.cell_voltage else [],
        )

    def __repr__(self) -> str:
        return (
            f"BatteryState(percentage={self.percentage:.2f}, voltage={self.voltage:.2f}, "
            f"current={self.current:.2f})"
        )
