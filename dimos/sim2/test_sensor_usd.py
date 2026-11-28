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

from dimos.robot.assets.usd_utils.frames import usd_frame_names

pytest.importorskip("pxr")

_SENSORS = [
    ("camera", {"left/sim_camera/camera_link"}),
    ("imu", {"left/sim_imu/imu_link"}),
    ("livox", {"left/mid360/lidar_link", "left/mid360/lidar_link/imu_link"}),
]


def _sensor(kind: str) -> Any:
    from dimos.sim2.camera import SimCamera
    from dimos.sim2.imu import SimImu
    from dimos.sim2.livox import LivoxSim

    sensor = {"camera": SimCamera, "imu": SimImu, "livox": LivoxSim}[kind]
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


def test_a_camera_usd_says_what_it_can_see_from_its_frame() -> None:
    from pxr import Sdf, Usd, UsdGeom

    from dimos.sim2.sensor_usd import camera_usd

    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(
        camera_usd("camera_link", name="camera", fovy=90.0, aspect=1.0).usd
    )
    stage = Usd.Stage.Open(layer)
    optics = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Camera)]
    assert len(optics) == 1, "one camera, under the frame it looks from"
    lens = UsdGeom.Camera(optics[0])
    aperture = lens.GetVerticalApertureAttr().Get()
    focal = lens.GetFocalLengthAttr().Get()
    assert math.degrees(2 * math.atan(aperture / 2 / focal)) == pytest.approx(90.0)
    forward = lens.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).TransformDir((0, 0, -1))
    assert tuple(round(axis, 6) for axis in forward) == (0.0, 0.0, 1.0)


def test_a_camera_usd_is_as_wide_as_the_images_its_camera_renders() -> None:
    from pxr import Sdf, Usd, UsdGeom

    camera = _sensor("camera")
    try:
        usd, config = camera.get_usd(), camera.config
    finally:
        camera.stop()
    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(usd)
    stage = Usd.Stage.Open(layer)
    lens = UsdGeom.Camera(next(p for p in stage.Traverse() if p.IsA(UsdGeom.Camera)))
    focal = lens.GetFocalLengthAttr().Get()
    rendered = 2 * math.atan(math.tan(math.radians(config.fovy) / 2) * config.width / config.height)
    assert config.width != config.height
    assert 2 * math.atan(lens.GetHorizontalApertureAttr().Get() / 2 / focal) == pytest.approx(
        rendered
    )
