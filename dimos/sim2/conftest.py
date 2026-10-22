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

"""Small MJCFs exercise the emulator without LFS, robot SDKs or network services."""

from contextlib import ExitStack
from copy import deepcopy
import time
from uuid import uuid4

import pytest

from dimos.core.transport import pZenohTransport
from dimos.protocol.pubsub.impl.memory import Memory
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.sim2.client import SimRobotClient
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.lidar.models.fibonacci import Fibonacci
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.spec import Joint, RobotConfig, RobotInstance, WorldConfig
from dimos.sim2.world.runtime import SimulationRuntime
from dimos.sim2.world.scene_types import SceneDescription, SceneEntity, SceneJoint, SceneRegion


@pytest.fixture(autouse=True)
def local_transport(request, mocker):
    """Unit tests step synchronously; native tests exercise real asynchronous delivery."""
    if request.node.get_closest_marker("native_e2e") or request.node.get_closest_marker(
        "self_hosted"
    ):
        return
    # A manually stepped world does not age while the test constructs its clients.
    # Deadline tests advance this clock explicitly; native tests keep real clocks.
    clock = mocker.Mock(wraps=time)
    clock.monotonic.return_value = time.monotonic()
    for module in ("client", "streams", "world.runtime"):
        mocker.patch(f"dimos.sim2.{module}.time", clock)
    bus = Memory()
    methods = {}

    def transport(topic):
        value = mocker.create_autospec(pZenohTransport, instance=True)
        value.broadcast.side_effect = lambda stream, message: bus.publish(topic.pattern, message)
        value.subscribe.side_effect = lambda callback: bus.subscribe(
            topic.pattern, lambda msg, _: callback(deepcopy(msg))
        )
        return value

    def rpc(**kwargs):
        value = mocker.create_autospec(ZenohRPC, instance=True)
        value.serve_rpc.side_effect = lambda method, name: methods.update({name: method})

        def call(name, arguments, **kwargs):
            if name not in methods:
                raise TimeoutError("sim2 endpoint unavailable")
            return deepcopy(methods[name](*arguments[0], **arguments[1])), lambda: None

        value.call_sync.side_effect = call
        return value

    mocker.patch("dimos.sim2.streams.pZenohTransport", side_effect=transport)
    mocker.patch("dimos.sim2.streams.ZenohRPC", side_effect=rpc)


@pytest.fixture
def world_config(tmp_path):
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
    model = tmp_path / "robot.xml"
    model.write_text("""
<mujoco>
  <worldbody>
    <body name="base">
      <geom type="box" size=".1 .1 .1" mass="1"/>
      <site name="imu"/>
      <site name="scan" pos="0 0 .05"/>
      <camera name="view" pos="2 -2 2" xyaxes=".707 .707 0 -.408 .408 .816"/>
      <body name="link" pos="0 0 .4">
        <joint name="hinge" type="hinge" axis="0 1 0" damping="1"/>
        <geom type="capsule" size=".05 .15" mass=".5"/>
        <body name="finger_body" pos="0 0 .3">
          <joint name="finger_joint" type="slide" axis="1 0 0" range="0 .04" damping="1"/>
          <geom type="box" size=".02 .02 .04" mass=".1"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor" joint="hinge" ctrlrange="-8 8"/>
    <position name="finger" joint="finger_joint" kp="50" ctrlrange="0 .04"/>
  </actuator>
</mujoco>
""")
    robot = RobotConfig(
        model=model,
        root_body="base",
        joints=(
            Joint("shoulder", "hinge", "motor", kp=10, kd=1),
            Joint(
                "gripper",
                "finger_joint",
                "finger",
                mode="position",
                scale=-0.04 / 850,
                offset=0.04,
                ctrl_scale=-0.04 / 850,
                ctrl_offset=0.04,
                lower=0,
                upper=850,
                max_velocity=1000,
            ),
        ),
        sensors=(
            Imu("imu", "imu"),
            Camera("camera", "view", width=160, height=120),
            Lidar("lidar", "scan", Fibonacci, model_kwargs={"ray_count": 512}),
        ),
    )
    return WorldConfig(
        scene,
        {
            "left": RobotInstance(
                robot,
                xyz=(0, 0, 0.3),
                initial_positions={"shoulder": 0.1, "gripper": 850},
                control_imu="imu",
            )
        },
    )


@pytest.fixture
def worlds(world_config):
    with ExitStack() as stack:

        def create(*, config=None, sim_id=None):
            config = world_config if config is None else config
            return stack.enter_context(SimulationRuntime(config, sim_id or uuid4().hex))

        yield create


@pytest.fixture
def clients():
    with ExitStack() as stack:

        def create(world, robot_id="left", **kwargs):
            return stack.enter_context(
                SimRobotClient(world.robots[robot_id].channel.descriptor, **kwargs)
            )

        yield create
