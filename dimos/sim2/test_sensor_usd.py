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


from typing import Any

import pytest

from dimos.robot.assets.usd_utils.frames import usd_frame_names

pytest.importorskip("pxr")

_SENSORS = [
    ("imu", {"left/sim_imu/imu_link"}),
]


def _sensor(kind: str) -> Any:
    from dimos.sim2.imu import SimImu

    sensor = {"imu": SimImu}[kind]
    return sensor(instance_name=f"left/{kind}")


@pytest.mark.parametrize(("kind", "frames"), _SENSORS)
def test_a_sensor_serves_its_usd_under_its_namespace(kind: str, frames: set[str]) -> None:
    sensor = _sensor(kind)
    try:
        usd = sensor.get_usd()
    finally:
        sensor.stop()

    assert frames <= usd_frame_names(usd), f"{kind}'s layer names {sorted(usd_frame_names(usd))}"
