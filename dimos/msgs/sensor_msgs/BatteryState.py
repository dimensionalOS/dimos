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
import time
from typing import Any

from dimos_lcm.sensor_msgs.BatteryState import BatteryState as LCMBatteryState

from dimos.types.timestamped import Timestamped


@dataclass
class BatteryState(Timestamped):
    """Battery telemetry, ROS ``sensor_msgs/BatteryState``. ``percentage`` is 0..1.

    Enum values are the generated ``dimos_lcm`` ones (``BatteryState.POWER_SUPPLY_*``).
    """

    msg_name = "sensor_msgs.BatteryState"

    voltage: float = float("nan")
    current: float = float("nan")
    percentage: float = float("nan")
    temperature: float = float("nan")
    charge: float = float("nan")
    capacity: float = float("nan")
    design_capacity: float = float("nan")
    power_supply_status: int = LCMBatteryState.POWER_SUPPLY_STATUS_UNKNOWN
    power_supply_health: int = LCMBatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
    power_supply_technology: int = LCMBatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
    present: bool = False
    cell_voltage: list[float] = field(default_factory=list)
    cell_temperature: list[float] = field(default_factory=list)
    location: str = ""
    serial_number: str = ""
    frame_id: str = ""
    ts: float = field(default_factory=time.time)

    def lcm_encode(self) -> bytes:
        msg = LCMBatteryState()
        msg.header.seq = 0
        msg.header.frame_id = self.frame_id
        [msg.header.stamp.sec, msg.header.stamp.nsec] = self.ros_timestamp()
        msg.voltage = self.voltage
        msg.temperature = self.temperature
        msg.current = self.current
        msg.charge = self.charge
        msg.capacity = self.capacity
        msg.design_capacity = self.design_capacity
        msg.percentage = self.percentage
        msg.power_supply_status = self.power_supply_status
        msg.power_supply_health = self.power_supply_health
        msg.power_supply_technology = self.power_supply_technology
        msg.present = self.present
        msg.cell_voltage_length = len(self.cell_voltage)
        msg.cell_voltage = list(self.cell_voltage)
        msg.cell_temperature_length = len(self.cell_temperature)
        msg.cell_temperature = list(self.cell_temperature)
        msg.location = self.location
        msg.serial_number = self.serial_number
        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> BatteryState:
        msg = LCMBatteryState.lcm_decode(data)
        return cls(
            voltage=msg.voltage,
            current=msg.current,
            percentage=msg.percentage,
            temperature=msg.temperature,
            charge=msg.charge,
            capacity=msg.capacity,
            design_capacity=msg.design_capacity,
            power_supply_status=msg.power_supply_status,
            power_supply_health=msg.power_supply_health,
            power_supply_technology=msg.power_supply_technology,
            present=msg.present,
            cell_voltage=list(msg.cell_voltage),
            cell_temperature=list(msg.cell_temperature),
            location=msg.location,
            serial_number=msg.serial_number,
            frame_id=msg.header.frame_id,
            ts=msg.header.stamp.sec + msg.header.stamp.nsec / 1e9,
        )

    def to_rerun(self) -> Any:
        import rerun as rr

        return rr.Scalars(self.percentage)
