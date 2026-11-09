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

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd import graft_usd, prefix_usd_frames
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.mujoco.sensor_world import open_view, robot_ids
from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.usd_layer import layer_file
from dimos.sim2.world_observations import WorldMirror, WorldWriter

SCENE = Path(__file__).parent / "assets" / "scene.usda"


@pytest.fixture
def composed() -> Path:
    """The world as a sensor really gets it: a room with the robot already grafted into it, which
    is what `get_scene` answers with. There is no description anywhere in it: the frames the
    sensor needs are in the layer because the robot's model carries them.
    """
    robot = toy_robot().with_frames(
        {
            "imu": ("base", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            "eye": ("head", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            "lidar": ("head", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        }
    )
    scene = UsdModel.from_file(SCENE).with_frames(
        {"dog": ("scene", (0.0, 0.0, 1.0), (0.0, 0.0, 0.0))}
    )
    layer = graft_usd(scene.usd, {"dog": prefix_usd_frames(robot.usd, "dog")})
    return layer_file(layer, called="sensor-world")


def _view(composed: Path, kind: str = "lidar", frame: str = "dog/lidar"):
    """A sensor's own world: the composed scene, with this sensor at its own frame. The robot id
    comes from the world, as it does in a sensor: the first frame of observations announces it.
    """
    return open_view(composed, ["dog"], SensorMount(kind, frame, frame))


class _World:
    """The frames a backend would send: the dog and the crate, then the dog wherever each frame puts it."""

    def __init__(self) -> None:
        self.writer, self.mirror = WorldWriter(), WorldMirror()
        # Sim time zero at wall time zero, so ts is sim time.
        self.writer.start_ns = 0
        self.writer.add("world")
        self.writer.add("dog", model="dog.usda")
        self.writer.add("dog/neck")
        self.writer.add("crate")
        self.writer.position("crate", [3.0, 0.0, 0.1, 0, 0, 0, 1])

    def at(self, sim_time: float, x: float, neck: float) -> WorldMirror:
        self.writer.position("dog", [x, 0.0, 1.0, 0, 0, 0, 1])
        self.writer.joint("dog/neck", neck, 0.0, 0.0)
        self.mirror.apply(self.writer.end_frame(sim_time))
        return self.mirror


def test_the_robots_in_the_world_are_the_ones_the_world_announced():
    """Nobody tells a sensor what is in the world; the world does, on its first frame. A robot is
    announced with the layer it was drawn from and a crate is not. The ids matter because the view
    has to shorten its own copy for exactly the robots the physics shortened for.
    """
    assert robot_ids(_World().at(0.0, 0.0, 0.0)) == ["dog"]


def test_the_view_is_posed_by_the_world_alone(composed):
    view = _view(composed, "camera", "dog/eye")
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


def test_a_sensor_does_not_see_the_body_it_is_mounted_on(composed):
    """The lidar site sits at the centre of the head's sphere, as a real one sits inside its shell:
    a sensor bolted to a robot is inside its mount's collision hull rather than proud of it (the
    G1's Mid-360 is 22 mm inside torso_link's). Casting without saying so returns the mount for
    every ray, and a lidar that drops returns inside its blind range reports an empty scan.
    """
    view = _view(composed)
    view.update(_World().at(0.0, 0.0, 0.0))
    origin, _ = view.pose("dog/lidar")
    # Forward and down, so it clears the base behind it and reaches the floor.
    out_and_down = np.array([[1.0, 0.0, -1.0]]) / np.sqrt(2.0)

    blinded, _ = view.raycaster((0,)).cast(origin, out_and_down, 10.0)
    seeing, _ = view.raycaster((0,), mounted_at="dog/lidar").cast(origin, out_and_down, 10.0)

    # Starting inside its own mount, the ray gets no further than the mount's own surface -- 50 mm
    # here, 22 mm on the real G1, both well inside a Mid-360's 160 mm blind range.
    assert blinded[0] == pytest.approx(0.05, abs=1e-3)
    # With the mount left out it carries on to the floor, a good way below.
    assert seeing[0] > 1.0


def test_blind_to_lets_the_rays_through_further_frames(composed):
    """For what the mount rule does not cover: a radome, a mast, a wall a test wants gone."""
    view = _view(composed)
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


def test_an_imu_reads_gravity_at_rest_and_acceleration_from_pose_changes(composed):
    view, world = _view(composed, "imu", "dog/imu"), _World()
    view.update(world.at(1.0, 0.0, 0.0))
    gyro, accelerometer, _ = view.imu("dog/imu")
    assert accelerometer == pytest.approx([0.0, 0.0, 9.81])
    assert gyro == pytest.approx([0.0, 0.0, 0.0])
    # Still, then moving at 1 m/s: 2 m/s^2 over half a second.
    view.update(world.at(1.5, 0.5, 0.0))
    _, accelerometer, _ = view.imu("dog/imu")
    assert accelerometer == pytest.approx([2.0, 0.0, 9.81])
