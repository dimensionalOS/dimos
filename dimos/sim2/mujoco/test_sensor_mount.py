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

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.sim2.mujoco.sensor_mount import SensorMount, mount_sensor
from dimos.sim2.mujoco.world import compile_world
from dimos.sim2.scene import toy_robot, usd_layer
from dimos.sim2.sim import SimulatedRobot, compose_usds

SCENE = Path(__file__).parent / "assets" / "scene.usda"


# The toy's base sits here in its own USD, so a spot for it is offset by this much.
TOY_BASE = (9.0, 9.0, 9.0)


def _spot(x: float, y: float, z: float) -> tuple[float, float, float]:
    """The scene frame that puts the toy's base at (x, y, z)."""
    return (x - TOY_BASE[0], y - TOY_BASE[1], z - TOY_BASE[2])


def _scene(**spots: tuple[float, float, float]) -> UsdModel:
    """The test room with a frame at each spot a robot is mounted on, plus an overhead frame."""
    frames = {name: ("scene", xyz, (0.0, 0.0, 0.0)) for name, xyz in spots.items()}
    return UsdModel.from_file(SCENE).with_frames(
        {"overhead": ("scene", (0.0, 0.0, 3.0), (0.0, 0.0, 0.0)), **frames}
    )


def _composed(scene: UsdModel, robots: dict[str, UsdModel]) -> UsdModel:
    """Each robot mounted on the scene's frame of its own name, its prims under that frame, as compose_world composes them."""
    mounted = [
        (usd_under_namespace(robot.usd, f"scene/{robot_id}"), f"scene/{robot_id}")
        for robot_id, robot in robots.items()
    ]
    return compose_usds([(scene.usd, None), *mounted])


@pytest.fixture
def robot_model() -> UsdModel:
    return toy_robot()


@pytest.fixture
def world(robot_model: UsdModel) -> UsdModel:
    return _composed(_scene(dog=_spot(0.0, 0.0, 1.0)), {"dog": robot_model})


def test_a_sensor_mounts_itself_in_a_world_of_its_own(
    world: UsdModel, robot_model: UsdModel
) -> None:
    spec = compile_world(usd_layer(world), [SimulatedRobot("dog", robot_model)])
    mount_sensor(spec, SensorMount("camera", "dog/head", "dog/eye", fovy=42.0))
    mount_sensor(spec, SensorMount("camera", "overhead", "world/overhead"))
    mount_sensor(spec, SensorMount("imu", "dog/base", "dog/imu"))
    model = spec.compile()

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.cam_fovy[model.camera("dog/eye").id] == 42.0
    assert data.cam("world/overhead").xpos.tolist() == pytest.approx([0, 0, 3])
    assert data.site("dog/imu").xpos.tolist() == pytest.approx(data.body("dog/base").xpos)
    assert model.nsensor == 0


def test_mounting_at_a_frame_the_world_does_not_have_is_rejected() -> None:
    spec = compile_world(usd_layer(_scene()))
    with pytest.raises(ValueError, match="no site or body 'nowhere'"):
        mount_sensor(spec, SensorMount("imu", "nowhere", "nowhere"))
