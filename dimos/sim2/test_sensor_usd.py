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
from typing import Any, get_origin, get_type_hints

import pytest

from dimos.core.hardware_module import HardwareModule
from dimos.core.stream import In, Out
from dimos.robot.assets.usd_utils.frames import usd_frame_names

pytest.importorskip("pxr")

_SENSORS = [
    ("camera", {"left/sim_camera/camera_link", "left/sim_camera/camera_link/camera_optical"}),
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


def test_a_camera_is_hardware_sending_what_a_real_camera_sends() -> None:
    from dimos.hardware.sensors.camera.module import WebcamModule
    from dimos.sim2.camera import SimCamera
    from dimos.sim2.sensor import SimSensor

    assert issubclass(SimCamera, HardwareModule)
    real = {
        name: kind for name, kind in _streams(WebcamModule, direction=Out).items() if name != "tf"
    }
    assert _streams(SimCamera, direction=Out) == {**_streams(SimSensor, direction=Out), **real}


def test_a_camera_usd_says_what_it_can_see_from_its_frame() -> None:
    from pxr import Sdf, Usd, UsdGeom

    from dimos.sim2.sensor_usd import camera_usd

    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(
        camera_usd("camera_link", name="camera", fovy=90.0, aspect=1.0).usd
    )
    stage = Usd.Stage.Open(layer)
    optics = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Camera)]
    assert len(optics) == 1, "one camera, under the optical frame it looks from"
    assert optics[0].GetParent().GetName() == "camera_optical"
    lens = UsdGeom.Camera(optics[0])
    aperture = lens.GetVerticalApertureAttr().Get()
    focal = lens.GetFocalLengthAttr().Get()
    assert math.degrees(2 * math.atan(aperture / 2 / focal)) == pytest.approx(90.0)
    # A USD camera looks down its -z; this one looks down its frame's +x, as a ROS camera_link does.
    forward = lens.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).TransformDir((0, 0, -1))
    assert tuple(round(axis, 6) for axis in forward) == (1.0, 0.0, 0.0)


def test_a_camera_usd_is_as_wide_as_the_images_its_camera_renders() -> None:
    from pxr import Sdf, Usd, UsdGeom

    camera = _sensor("camera")
    try:
        usd, config, lens_info = camera.get_usd(), camera.config, camera.lens()
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
    # Its camera_info is the same lens.
    assert 2 * math.atan(config.width / 2 / lens_info.K[0]) == pytest.approx(rendered)
    assert lens_info.frame_id == "left/sim_camera/camera_link/camera_optical"
