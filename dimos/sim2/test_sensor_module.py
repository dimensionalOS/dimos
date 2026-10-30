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

from dataclasses import replace

import numpy as np
import pytest

from dimos.sim2.mujoco import MujocoBackend
from dimos.sim2.runtime import Runtime
from dimos.sim2.sensor_module import SimSensorModule
from dimos.sim2.sensors import MujocoSensors
from dimos.sim2.types import Camera, Imu, Lidar, Scan, SensorReader


@pytest.fixture
def sensor_module_factory():
    modules = []

    def make(**kwargs):
        module = SimSensorModule(reader=MujocoSensors, **kwargs)
        modules.append(module)
        return module

    yield make
    for module in modules:
        module.stop()


@pytest.mark.parametrize(
    "kind,profile,output",
    [
        ("camera", Camera("right/tool", hz=15), "color_image"),
        ("lidar", Lidar("right/tool"), "lidar"),
        ("imu", Imu("right/tool"), "imu"),
    ],
)
def test_one_explicit_sensor_needs_no_other_sensors(
    world, mocker, kind, profile, output, sensor_module_factory
):
    world = replace(world, sensors=(profile,))
    runtime = Runtime(MujocoBackend(world))
    module = sensor_module_factory(world=world, **{kind: profile.frame})
    module.bind(runtime.backend.layout)
    published = {
        name: mocker.patch.object(getattr(module, name), "publish")
        for name in ("color_image", "depth_image", "camera_info", "lidar", "imu")
    }
    reader = mocker.Mock(spec=SensorReader)
    reader.gaps = 0
    reader.camera.return_value = (
        np.zeros((240, 320, 3), dtype=np.uint8),
        np.ones((240, 320), dtype=np.float32),
    )
    reader.lidar.return_value = Scan(np.array([[1.0, 2.0, 3.0]]), np.array([0.0]), runtime.epoch)
    module.publish_sample(runtime.capture(), reader)
    published[output].assert_called_once()
    enabled = {"color_image", "depth_image", "camera_info"} if kind == "camera" else {kind}
    for name in published.keys() - enabled:
        published[name].assert_not_called()
    assert published[output].call_args.args[0].frame_id == (
        "world" if kind == "lidar" else "right/tool"
    )


def test_sensor_binding_reports_missing_frame_before_starting(world, sensor_module_factory):
    module = sensor_module_factory(world=world, camera="unknown")
    with pytest.raises(ValueError, match="configured Camera"):
        module.bind(MujocoBackend(world).layout)
