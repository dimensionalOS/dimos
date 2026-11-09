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

from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.spec import SimulatedRobot, SimulatedSensor
from dimos.sim2.view import open_view, sensor_frame
from dimos.sim2.world_observations import WorldMirror, WorldWriter


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
    robot = toy_robot().with_frames(
        {
            "imu": ("base", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            "eye": ("head", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            "lidar": ("head", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        }
    )
    return {
        "scene": str(scene),
        "simulated": [
            SimulatedRobot("dog", robot, "base"),
            SimulatedSensor("camera", "eye", robot_id="dog"),
            SimulatedSensor("imu", "imu", robot_id="dog"),
            SimulatedSensor("lidar", "lidar", robot_id="dog"),
        ],
    }


class _World:
    """The frames a backend would send: the dog and the crate, then the dog wherever each frame puts it."""

    def __init__(self) -> None:
        self.writer, self.mirror = WorldWriter(), WorldMirror()
        # Sim time zero at wall time zero, so ts is sim time.
        self.writer.start_ns = 0
        self.writer.add("world")
        self.writer.add("dog")
        self.writer.add("dog/neck")
        self.writer.add("crate")
        self.writer.position("crate", [3.0, 0.0, 0.1, 0, 0, 0, 1])

    def at(self, sim_time: float, x: float, neck: float) -> WorldMirror:
        self.writer.position("dog", [x, 0.0, 1.0, 0, 0, 0, 1])
        self.writer.joint("dog/neck", neck, 0.0, 0.0)
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
    assert sensor_frame(description, "eye") == "dog/eye"
    assert view.render("dog/eye", 32, 24).shape == (24, 32, 3)
    view.close()


def test_a_sensor_does_not_see_the_body_it_is_mounted_on(description):
    """The lidar site sits at the centre of the head's sphere, as a real one sits inside its shell.

    A collision hull is a few convex shapes around a shape that is neither, so a sensor bolted
    to a robot is inside its mount rather than proud of it -- the G1's Mid-360 is 22 mm inside
    torso_link's. Casting without saying so returns the mount for every ray in every direction,
    and a lidar that drops returns inside its blind range then reports an empty scan.
    """
    view = open_view(description)
    view.update(_World().at(0.0, 0.0, 0.0))
    origin, _ = view.pose("dog/lidar")
    # Forward and down, so it clears the base behind it and reaches the floor.
    out_and_down = np.array([[1.0, 0.0, -1.0]]) / np.sqrt(2.0)

    blinded, _ = view.raycaster((0,)).cast(origin, out_and_down, 10.0)
    seeing, _ = view.raycaster((0,), mounted_at="dog/lidar").cast(origin, out_and_down, 10.0)

    # Starting inside its own mount, the ray gets no further than the mount's own surface --
    # 50 mm here, 22 mm on the real G1, both well inside a Mid-360's 160 mm blind range, so
    # every such return is dropped and the scan comes out empty.
    assert blinded[0] == pytest.approx(0.05, abs=1e-3)
    # With the mount left out it carries on to the floor, a good way below.
    assert seeing[0] > 1.0


def test_blind_to_lets_the_rays_through_further_frames(description):
    """For what the mount rule does not cover: a radome, a mast, a wall a test wants gone."""
    view = open_view(description)
    view.update(_World().at(0.0, 0.0, 0.0))
    # From a metre to the robot's side, level with its base, looking straight at it.
    base, _ = view.pose("dog/base")
    origin = base + np.array([0.0, -1.0, 0.0])
    at_the_robot = np.array([[0.0, 1.0, 0.0]])

    stopped, _ = view.raycaster((0,)).cast(origin, at_the_robot, 10.0)
    through, _ = view.raycaster((0,), blind_to=["dog/base"]).cast(origin, at_the_robot, 10.0)

    # The base's near face, a metre away less its half-width.
    assert stopped[0] == pytest.approx(0.9, abs=1e-3)
    # Nothing else stands in that direction, so with the base ignored the ray reaches nothing.
    assert through[0] < 0


def test_an_imu_reads_gravity_at_rest_and_acceleration_from_pose_changes(description):
    view, world = open_view(description), _World()
    view.update(world.at(1.0, 0.0, 0.0))
    gyro, accelerometer, _ = view.imu("dog/imu")
    assert accelerometer == pytest.approx([0.0, 0.0, 9.81])
    assert gyro == pytest.approx([0.0, 0.0, 0.0])
    # Still, then moving at 1 m/s: 2 m/s^2 over half a second.
    view.update(world.at(1.5, 0.5, 0.0))
    _, accelerometer, _ = view.imu("dog/imu")
    assert accelerometer == pytest.approx([2.0, 0.0, 9.81])
