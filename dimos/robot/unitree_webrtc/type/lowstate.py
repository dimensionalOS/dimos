# Copyright 2025 Dimensional Inc.
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

from typing import Literal, TypedDict

raw_odom_msg_sample = {
    "type": "msg",
    "topic": "rt/lf/lowstate",
    "data": {
        "imu_state": {"rpy": [0.008086, -0.007515, 2.981771]},
        "motor_state": [
            {"q": 0.098092, "temperature": 40, "lost": 0, "reserve": [0, 674]},
            {"q": 0.757921, "temperature": 32, "lost": 0, "reserve": [0, 674]},
            {"q": -1.490911, "temperature": 38, "lost": 6, "reserve": [0, 674]},
            {"q": -0.072477, "temperature": 42, "lost": 0, "reserve": [0, 674]},
            {"q": 1.020276, "temperature": 32, "lost": 5, "reserve": [0, 674]},
            {"q": -2.007172, "temperature": 38, "lost": 5, "reserve": [0, 674]},
            {"q": 0.071382, "temperature": 50, "lost": 5, "reserve": [0, 674]},
            {"q": 0.963379, "temperature": 36, "lost": 6, "reserve": [0, 674]},
            {"q": -1.978311, "temperature": 40, "lost": 5, "reserve": [0, 674]},
            {"q": -0.051066, "temperature": 48, "lost": 0, "reserve": [0, 674]},
            {"q": 0.73103, "temperature": 34, "lost": 10, "reserve": [0, 674]},
            {"q": -1.466473, "temperature": 38, "lost": 6, "reserve": [0, 674]},
            {"q": 0, "temperature": 0, "lost": 0, "reserve": [0, 0]},
            {"q": 0, "temperature": 0, "lost": 0, "reserve": [0, 0]},
            {"q": 0, "temperature": 0, "lost": 0, "reserve": [0, 0]},
            {"q": 0, "temperature": 0, "lost": 0, "reserve": [0, 0]},
            {"q": 0, "temperature": 0, "lost": 0, "reserve": [0, 0]},
            {"q": 0, "temperature": 0, "lost": 0, "reserve": [0, 0]},
            {"q": 0, "temperature": 0, "lost": 0, "reserve": [0, 0]},
            {"q": 0, "temperature": 0, "lost": 0, "reserve": [0, 0]},
        ],
        "bms_state": {
            "version_high": 1,
            "version_low": 18,
            "soc": 55,
            "current": -2481,
            "cycle": 56,
            "bq_ntc": [30, 29],
            "mcu_ntc": [33, 32],
        },
        "foot_force": [97, 84, 81, 81],
        "temperature_ntc1": 48,
        "power_v": 28.331045,
    },
}


class MotorState(TypedDict):
    q: float
    temperature: int
    lost: int
    reserve: list[int]


class ImuState(TypedDict):
    rpy: list[float]


class BmsState(TypedDict):
    version_high: int
    version_low: int
    soc: int
    current: int
    cycle: int
    bq_ntc: list[int]
    mcu_ntc: list[int]


class LowStateData(TypedDict):
    imu_state: ImuState
    motor_state: list[MotorState]
    bms_state: BmsState
    foot_force: list[int]
    temperature_ntc1: int
    power_v: float


class LowStateMsg(TypedDict):
    type: Literal["msg"]
    topic: str
    data: LowStateData
