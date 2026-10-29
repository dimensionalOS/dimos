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

"""Small MJCFs exercise the world without LFS, robot SDKs or network services."""

from contextlib import ExitStack
from functools import partial

import numpy as np
import pytest
from robosuite.models.robots.robot_model import RobotModel

from dimos.sim2.environment import MotorScene
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.lidar.models.fibonacci import Fibonacci
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world.runtime import SimulationRuntime
from dimos.sim2.world.scene_types import SceneDescription, SceneEntity, SceneJoint, SceneRegion


class TinyRobotModel(RobotModel):
    """An ordinary upstream robot model with one motor and one position servo."""

    @property
    def init_qpos(self):
        return np.array([0.1, 0.0])


@pytest.fixture
def scene_path(tmp_path):
    scene = tmp_path / "scene.xml"
    scene.write_text("""
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <light pos="0 0 4"/>
    <geom name="floor" type="plane" size="5 5 .1" rgba=".5 .6 .7 1"/>
    <body name="box" pos="1.5 0 .3">
      <freejoint/>
      <geom type="box" size=".2 .2 .3" mass="1" rgba="1 .1 .1 1"/>
    </body>
    <body name="door" pos="-2 0 .5">
      <joint name="door_hinge" type="hinge" axis="0 0 1" range="0 90"/>
      <geom type="box" size=".1 .4 .5" mass="1"/>
    </body>
  </worldbody>
</mujoco>
""")
    scene.with_suffix(".json").write_text(
        SceneDescription(
            id="test-room",
            entities={"box": SceneEntity(body="box", label="Box", kind="box", movable=True)},
            joints={"door": SceneJoint(joint="door_hinge", entity="door", closed=0, opened=1)},
            regions={"table": SceneRegion(kind="support", size=(1, 1, 0.1))},
        ).model_dump_json()
    )
    return scene


@pytest.fixture
def robot_path(tmp_path):
    model = tmp_path / "robot.xml"
    model.write_text("""
<mujoco>
  <worldbody>
    <body name="base" pos="0 0 .3">
      <geom type="box" size=".1 .1 .1" mass="1"/>
      <site name="imu"/>
      <site name="scan" pos="0 0 .05"/>
      <camera name="view" pos="2 -2 2" xyaxes=".707 .707 0 -.408 .408 .816"/>
      <body name="link" pos="0 0 .4">
        <joint name="hinge" type="hinge" axis="0 1 0" damping="1" frictionloss="0" armature="0"/>
        <geom type="capsule" size=".05 .15" mass=".5"/>
        <body name="finger_body" pos="0 0 .3">
          <joint name="finger_joint" type="slide" axis="1 0 0" range="0 .04" damping="1" frictionloss="0" armature="0"/>
          <geom type="box" size=".02 .02 .04" mass=".1"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor" joint="hinge" ctrlrange="-8 8"/>
    <position name="finger" joint="finger_joint" kp="50" ctrlrange="0 .04"/>
  </actuator>
  <sensor>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_accel" site="imu"/>
    <framequat name="imu_quat" objtype="site" objname="imu"/>
  </sensor>
</mujoco>
""")
    return model


@pytest.fixture
def robot_factory(robot_path):
    return partial(TinyRobotModel, str(robot_path), idn="left")


@pytest.fixture
def world_config(scene_path, robot_factory):
    sensors = (
        ("left", Imu("imu", "imu")),
        ("left", Lidar("lidar", "scan", Fibonacci, model_kwargs={"ray_count": 512})),
    )
    robot = SimulatedRobot(
        robot_id="left",
        model=robot_factory,
        joint_names=("shoulder", "gripper"),
        frame_id="left/base",
        control_imu="imu",
        startup_kp=(10.0, 0.0),
        startup_kd=(1.0, 0.0),
        position_scale=(1.0, -0.04 / 850),
        position_offset=(0.0, 0.04),
        position_only_joints=("gripper",),
    )
    return dict(
        environment=partial(
            MotorScene, scene=scene_path, models={"left": robot_factory}, sensors=sensors
        ),
        robots={"left": robot},
        sensors=sensors,
    )


@pytest.fixture
def worlds(world_config):
    with ExitStack() as stack:

        def create(*, config=None):
            config = world_config if config is None else config
            return stack.enter_context(SimulationRuntime(**config))

        yield create


@pytest.fixture
def feedback(mocker):
    """Attach a recording feedback callback to a world; returns the list of samples."""

    def attach(world):
        samples = []
        mocker.patch.object(
            world,
            "publish_feedback",
            side_effect=lambda robot_id, joints, imu, odom: samples.append(
                (robot_id, joints, imu, odom)
            ),
        )
        return samples

    return attach
