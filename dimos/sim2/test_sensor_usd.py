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

"""Every sensor describes itself in USD, and describes the frame it was configured with.

A sensor that answers with a hardcoded frame name looks right in a one-robot world and silently
collides in a three-robot one, so the point of each of these is the *configured* frame coming
back, not merely that a layer came back.
"""

from typing import Any

import pytest

from dimos.robot.assets.usd import usd_frame_names

pytest.importorskip("pxr")

# Each sim sensor, the config that places it, and the frames its layer has to name.
_SENSORS = [
    ("camera", {"frame_id": "left/camera_optical"}, {"left/camera_optical"}),
    ("imu", {"frame_id": "left/imu_link"}, {"left/imu_link"}),
    ("l1", {"frame_id": "left/utlidar_lidar"}, {"left/utlidar_lidar"}),
    (
        "livox",
        {"frame_id": "left/mid360_link", "imu_frame_id": "left/lidar_imu"},
        {"left/mid360_link", "left/lidar_imu"},
    ),
]


def _sensor(kind: str, config: dict[str, Any]) -> Any:
    if kind == "camera":
        from dimos.sim2.camera import SimCamera

        return SimCamera(**config)
    if kind == "imu":
        from dimos.sim2.imu import SimImu

        return SimImu(**config)
    if kind == "l1":
        from dimos.sim2.l1 import SimL1

        return SimL1(**config)
    from dimos.sim2.livox import LivoxSim

    return LivoxSim(**config)


@pytest.mark.parametrize(("kind", "config", "frames"), _SENSORS)
def test_a_sensor_describes_itself_at_the_frames_it_was_given(
    kind: str, config: dict[str, Any], frames: set[str]
) -> None:
    sensor = _sensor(kind, config)
    try:
        root, usd = sensor.get_usd()
    finally:
        sensor.stop()

    assert isinstance(root, str), "the asset root is a path or empty, never None"
    assert frames <= usd_frame_names(usd), (
        f"{kind}'s layer names {sorted(usd_frame_names(usd))}, not {sorted(frames)}"
    )


def test_the_simulated_mid360_describes_the_same_box_as_the_real_one() -> None:
    """A sim twin that described itself differently would change shape with which one is running."""
    from dimos.hardware.sensors.lidar.livox.module import mid360_usd
    from dimos.sim2.livox import LivoxSim

    sim = LivoxSim(frame_id="mid360_link", imu_frame_id="imu_link")
    try:
        _, usd = sim.get_usd()
    finally:
        sim.stop()

    assert usd == mid360_usd("mid360_link", "imu_link")
