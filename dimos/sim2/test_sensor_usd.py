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


from typing import Any, get_origin, get_type_hints

import pytest

from dimos.core.hardware_module import HardwareModule
from dimos.core.stream import In, Out
from dimos.robot.assets.usd_utils.frames import usd_frame_names

pytest.importorskip("pxr")

_SENSORS = [
    ("imu", {"left/sim_imu/imu_link"}),
    ("livox", {"left/mid360/lidar_link", "left/mid360/lidar_link/imu_link"}),
]


def _sensor(kind: str) -> Any:
    from dimos.sim2.imu import SimImu
    from dimos.sim2.livox import LivoxSim

    sensor = {"imu": SimImu, "livox": LivoxSim}[kind]
    return sensor(instance_name=f"left/{kind}")


@pytest.mark.parametrize(("kind", "frames"), _SENSORS)
def test_a_sensor_serves_its_usd_under_its_namespace(kind: str, frames: set[str]) -> None:
    sensor = _sensor(kind)
    try:
        usd = sensor.get_usd()
    finally:
        sensor.stop()

    assert frames <= usd_frame_names(usd), f"{kind}'s layer names {sorted(usd_frame_names(usd))}"


def test_a_namespaced_lidar_puts_its_namespace_on_its_imu_frame_too() -> None:
    lidar = _sensor("livox")
    try:
        assert lidar.frame_id == "left/mid360/lidar_link"
        assert lidar.imu_frame_id == "left/mid360/lidar_link/imu_link"
    finally:
        lidar.stop()


def _streams(module: type, *, direction: type) -> dict[str, type]:
    hints = get_type_hints(module)
    return {name: hint.__args__[0] for name, hint in hints.items() if get_origin(hint) is direction}


def test_a_livox_is_hardware_sending_what_a_real_mid360_sends() -> None:
    from dimos.hardware.sensors.lidar.livox.module import Mid360
    from dimos.sim2.livox import LivoxSim
    from dimos.sim2.sensor import SimSensor

    assert issubclass(LivoxSim, HardwareModule)
    assert _streams(LivoxSim, direction=Out) == {
        **_streams(SimSensor, direction=Out),
        **_streams(Mid360, direction=Out),
    }
    # It reads the world where the real one reads its network.
    assert _streams(LivoxSim, direction=In) == _streams(SimSensor, direction=In)
