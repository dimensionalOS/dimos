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


import math
from typing import Any

import pytest

from dimos.robot.assets.usd import usd_frame_names

pytest.importorskip("pxr")

_SENSORS = [
    ("camera", {"frame_id": "left/camera_optical"}, {"left/camera_optical"}),
    ("imu", {"frame_id": "left/imu_link"}, {"left/imu_link"}),
    (
        "livox",
        {"frame_id": "left/mid360_link", "imu_frame_id": "left/lidar_imu"},
        {"left/mid360_link", "left/lidar_imu"},
    ),
]


def _sensor(kind: str, config: dict[str, Any]) -> Any:
    from dimos.sim2.camera import SimCamera
    from dimos.sim2.imu import SimImu
    from dimos.sim2.livox import LivoxSim

    return {"camera": SimCamera, "imu": SimImu, "livox": LivoxSim}[kind](**config)


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


def test_a_camera_says_what_it_can_see_from_its_frame() -> None:
    from pxr import Usd, UsdGeom

    from dimos.sim2.camera import SimCamera

    camera = SimCamera(frame_id="left/camera_optical", fovy=90.0)
    try:
        _, usd = camera.get_usd()
    finally:
        camera.stop()

    stage = Usd.Stage.Open(_anonymous(usd))
    optics = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Camera)]
    assert len(optics) == 1, "one camera, under the frame it looks from"
    lens = UsdGeom.Camera(optics[0])
    aperture = lens.GetVerticalApertureAttr().Get()
    focal = lens.GetFocalLengthAttr().Get()
    assert math.degrees(2 * math.atan(aperture / 2 / focal)) == pytest.approx(90.0)
    forward = lens.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).TransformDir((0, 0, -1))
    assert tuple(round(axis, 6) for axis in forward) == (0.0, 0.0, 1.0)


def _anonymous(usd: str) -> object:
    from pxr import Sdf

    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(usd)
    return layer


def test_the_simulated_mid360_describes_the_same_box_as_the_real_one() -> None:
    from dimos.hardware.sensors.lidar.livox.module import mid360_usd
    from dimos.sim2.livox import LivoxSim

    sim = LivoxSim(frame_id="mid360_link", imu_frame_id="imu_link")
    try:
        root, usd = sim.get_usd()
    finally:
        sim.stop()

    assert usd == mid360_usd("mid360_link", "imu_link")
    assert root, "the root is the driver's own directory, which is where its mesh lives"


def test_a_namespaced_lidar_puts_its_namespace_on_its_imu_frame_too() -> None:
    from dimos.sim2.livox import LivoxSim

    sim = LivoxSim(frame_id="mid360_link", frame_id_prefix="left")
    try:
        assert sim.frame_id == "left/mid360_link"
        assert sim.imu_frame_id == "left/imu_link"
        _, usd = sim.get_usd()
    finally:
        sim.stop()

    assert {"left/mid360_link", "left/imu_link"} <= usd_frame_names(usd)
