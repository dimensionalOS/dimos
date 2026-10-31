# Copyright 2025-2026 Dimensional Inc.
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

import numpy as np
import pytest

from dimos.sim2.spec import Mount, SimulatedRobot, SimulatedSensor
from dimos.sim2.view import open_view
from dimos.sim2.world_change import (
    OBJECT,
    ROBOT,
    WorldMirror,
    WorldWriter,
    pose_update,
    tag_update,
    velocity_update,
)


@pytest.fixture
def description(tmp_path: Path) -> dict:
    scene = tmp_path / "scene.xml"
    scene.write_text("""
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="10 10 .1"/>
    <body name="crate" pos="0 2 .5"><freejoint/><geom type="box" size=".1 .1 .1"/></body>
  </worldbody>
</mujoco>
""")
    robot = tmp_path / "robot.xml"
    robot.write_text("""
<mujoco>
  <worldbody>
    <body name="base">
      <freejoint/>
      <geom type="box" size=".2 .1 .1" mass="1"/>
      <body name="head" pos=".2 0 .1"><joint name="neck" type="hinge" axis="0 0 1"/><geom type="sphere" size=".05" mass=".1"/></body>
    </body>
  </worldbody>
</mujoco>
""")
    return {
        "scene": str(scene),
        "simulated": [
            SimulatedRobot("dog", robot, "base"),
            SimulatedSensor("eye", "camera", Mount("head"), "dog"),
            SimulatedSensor("imu", "imu", Mount("base"), "dog"),
            SimulatedSensor("lidar", "lidar", Mount("head"), "dog"),
        ],
    }


class _World:
    """The frames a backend would send: the dog and the crate, then the dog wherever each frame puts it."""

    def __init__(self) -> None:
        self.writer, self.mirror = WorldWriter(), WorldMirror()
        self.writer.add("world")
        self.writer.add("dog", tag_update(ROBOT))
        self.writer.add_joints("dog", ["neck"])
        self.writer.add("crate", tag_update(OBJECT), pose_update([3.0, 0.0, 0.1, 0, 0, 0, 1]))

    def at(self, sim_time: float, x: float, neck: float, vx: float = 0.0) -> WorldMirror:
        self.writer.update(
            "dog", pose_update([x, 0.0, 1.0, 0, 0, 0, 1]), velocity_update([vx, 0, 0, 0, 0, 0])
        )
        self.writer.joints("dog", np.array([[neck, 0.0, 0.0]]))
        self.mirror.apply(self.writer.end_frame(sim_time))
        return self.mirror


def test_the_view_is_posed_by_the_world_alone(description):
    view = open_view(description)
    view.update(_World().at(1.0, 2.0, np.pi / 2))
    assert view.time == 1.0
    assert view.pose("dog/base")[0] == pytest.approx([2.0, 0.0, 1.0])
    assert view.pose("crate")[0] == pytest.approx([3.0, 0.0, 0.1])
    # The neck turned a quarter turn, so the head's mount faces +y.
    w, _, _, z = view.pose("dog/lidar")[1]
    assert abs(2 * np.arctan2(z, w)) == pytest.approx(np.pi / 2, abs=1e-6)
    distance, _ = view.raycaster((0,)).cast(
        np.array([5.0, 0.0, 1.0]), np.array([[0.0, 0.0, -1.0]]), 10.0
    )
    assert distance[0] == pytest.approx(1.0)
    assert view.render("dog/eye", 32, 24).shape == (24, 32, 3)
    view.close()


def test_an_imu_reads_gravity_at_rest_and_acceleration_from_velocity_changes(description):
    view, world = open_view(description), _World()
    view.update(world.at(1.0, 0.0, 0.0))
    gyro, accelerometer, _ = view.imu("dog/imu")
    assert accelerometer == pytest.approx([0.0, 0.0, 9.81])
    assert gyro == pytest.approx([0.0, 0.0, 0.0])
    view.update(world.at(1.5, 0.0, 0.0, vx=1.0))
    _, accelerometer, _ = view.imu("dog/imu")
    assert accelerometer == pytest.approx([2.0, 0.0, 9.81])
