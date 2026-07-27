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

from __future__ import annotations

import time

from dimos_lcm.sensor_msgs.BatteryState import BatteryState as LCMBatteryState

from dimos.types.timestamped import Timestamped

# power_supply_status constants (ROS sensor_msgs/BatteryState)
POWER_SUPPLY_STATUS_UNKNOWN = 0
POWER_SUPPLY_STATUS_CHARGING = 1
POWER_SUPPLY_STATUS_DISCHARGING = 2
POWER_SUPPLY_STATUS_NOT_CHARGING = 3
POWER_SUPPLY_STATUS_FULL = 4


class BatteryState(Timestamped):
    """Battery telemetry mirroring ROS sensor_msgs/BatteryState.

    ``percentage`` is a 0..1 fraction (ROS convention), not 0..100.
    """

    msg_name = "sensor_msgs.BatteryState"

    def __init__(
        self,
        voltage: float = 0.0,
        temperature: float = 0.0,
        current: float = 0.0,
        charge: float = 0.0,
        capacity: float = 0.0,
        percentage: float = 0.0,
        power_supply_status: int = POWER_SUPPLY_STATUS_UNKNOWN,
        present: bool = False,
        location: str = "",
        serial_number: str = "",
        frame_id: str = "battery",
        ts: float | None = None,
    ) -> None:
        self.ts = ts if ts is not None else time.time()
        self.frame_id = frame_id
        self.voltage = voltage
        self.temperature = temperature
        self.current = current
        self.charge = charge
        self.capacity = capacity
        self.percentage = percentage
        self.power_supply_status = power_supply_status
        self.present = present
        self.location = location
        self.serial_number = serial_number

    def lcm_encode(self) -> bytes:
        msg = LCMBatteryState()
        [msg.header.stamp.sec, msg.header.stamp.nsec] = self.ros_timestamp()
        msg.header.frame_id = self.frame_id
        msg.voltage = self.voltage
        msg.temperature = self.temperature
        msg.current = self.current
        msg.charge = self.charge
        msg.capacity = self.capacity
        msg.percentage = self.percentage
        msg.power_supply_status = self.power_supply_status
        msg.present = self.present
        msg.location = self.location
        msg.serial_number = self.serial_number
        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> BatteryState:
        msg = LCMBatteryState.lcm_decode(data)
        return cls(
            voltage=msg.voltage,
            temperature=msg.temperature,
            current=msg.current,
            charge=msg.charge,
            capacity=msg.capacity,
            percentage=msg.percentage,
            power_supply_status=msg.power_supply_status,
            present=msg.present,
            location=msg.location,
            serial_number=msg.serial_number,
            frame_id=msg.header.frame_id,
            ts=msg.header.stamp.sec + (msg.header.stamp.nsec / 1_000_000_000),
        )

    def __str__(self) -> str:
        return (
            f"BatteryState(loc='{self.location}', {self.percentage * 100:.0f}%, "
            f"{self.voltage:.1f}V, {self.temperature:.1f}C, status={self.power_supply_status})"
        )
