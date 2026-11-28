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


from pathlib import Path

import mujoco
import pytest

# linux-aarch64 has no usd-core wheel, and every scene here is a USD layer.
pytest.importorskip("pxr")

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.sim2.mujoco.sensor_mount import SensorMount, mount_sensor
from dimos.sim2.mujoco.world import compile_world
from dimos.sim2.scene import toy_robot, usd_layer
from dimos.sim2.sim import SimulatedRobot, compose_usds

SCENE = Path(__file__).parent / "assets" / "scene.usda"
# The toy's base sits here in its own USD, so the dog's frame is offset by this much.
TOY_BASE = (9.0, 9.0, 9.0)


def test_sensors_mount_on_the_frames_the_world_has_and_nowhere_else() -> None:
    robot = toy_robot()
    dog_frame = (-TOY_BASE[0], -TOY_BASE[1], 1.0 - TOY_BASE[2])
    scene = UsdModel.from_file(SCENE).with_frames(
        {
            "overhead": ("scene", (0.0, 0.0, 3.0), (0.0, 0.0, 0.0)),
            "dog": ("scene", dog_frame, (0.0, 0.0, 0.0)),
        }
    )
    world = compose_usds(
        [(scene.usd, None), (usd_under_namespace(robot.usd, "scene/dog"), "scene/dog")]
    )
    spec = compile_world(usd_layer(world), [SimulatedRobot("dog", robot)])
    mount_sensor(spec, SensorMount("camera", "dog/head", "dog/eye", fovy=42.0))
    mount_sensor(spec, SensorMount("camera", "overhead", "world/overhead"))
    mount_sensor(spec, SensorMount("imu", "dog/base", "dog/imu"))
    with pytest.raises(ValueError, match="no site or body 'nowhere'"):
        mount_sensor(spec, SensorMount("imu", "nowhere", "nowhere"))
    model = spec.compile()

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.cam_fovy[model.camera("dog/eye").id] == 42.0
    assert data.cam("world/overhead").xpos.tolist() == pytest.approx([0, 0, 3])
    assert data.site("dog/imu").xpos.tolist() == pytest.approx(data.body("dog/base").xpos)
    assert model.nsensor == 0
