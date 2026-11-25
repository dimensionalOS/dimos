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

import mujoco
import numpy as np
import pytest

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.frames import prefix_usd_frames
from dimos.robot.assets.usd_utils.layers import graft_usd
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.mujoco.sensor_world import open_view, robot_ids
from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.usd_layer import layer_file
from dimos.sim2.world_observations import WorldMirror, WorldWriter

# Every scene here is read out of a USD layer, and a runner without usd-core cannot read one:
# linux-aarch64 has no wheel for it (pyproject's marker says so), so without this the tests
# fail there on dimos's own "Reading or writing USD needs usd-core (pxr)". Their neighbours
# in this directory skip the same way.
pytest.importorskip("pxr")

SCENE = Path(__file__).parent / "assets" / "scene.usda"


@pytest.fixture
def composed() -> Path:
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
    return open_view(composed, ["dog"], SensorMount(kind, frame, frame))


class _World:
    def __init__(self) -> None:
        self.writer, self.mirror = WorldWriter(), WorldMirror()
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
    assert robot_ids(_World().at(0.0, 0.0, 0.0)) == ["dog"]


def test_the_view_is_posed_by_the_world_alone(composed):
    view = _view(composed)
    view.update(_World().at(1.0, 2.0, np.pi / 2))
    assert view.time == 1.0
    assert view.pose("dog/base")[0] == pytest.approx([2.0, 0.0, 1.0])
    assert view.pose("crate")[0] == pytest.approx([3.0, 0.0, 0.1])
    w, _, _, z = view.pose("dog/lidar")[1]
    assert abs(2 * np.arctan2(z, w)) == pytest.approx(np.pi / 2, abs=1e-6)
    distance, _ = view.raycaster((0,)).cast(
        np.array([5.0, 0.0, 1.0]), np.array([[0.0, 0.0, -1.0]]), 10.0
    )
    assert distance[0] == pytest.approx(1.0)
    view.close()


@pytest.mark.self_hosted
def test_the_view_renders_from_a_camera_frame(composed, renders):
    view = _view(composed, "camera", "dog/eye")
    view.update(_World().at(1.0, 2.0, np.pi / 2))
    assert view.render("dog/eye", 32, 24).shape == (24, 32, 3)
    view.close()


def test_a_sensor_does_not_see_the_body_it_is_mounted_on(composed):
    view = _view(composed)
    view.update(_World().at(0.0, 0.0, 0.0))
    origin, _ = view.pose("dog/lidar")
    out_and_down = np.array([[1.0, 0.0, -1.0]]) / np.sqrt(2.0)

    blinded, _ = view.raycaster((0,)).cast(origin, out_and_down, 10.0)
    seeing, _ = view.raycaster((0,), mounted_at="dog/lidar").cast(origin, out_and_down, 10.0)

    assert blinded[0] == pytest.approx(0.05, abs=1e-3)
    assert seeing[0] > 1.0


_HOUSING = """#usda 1.0
(
    defaultPrim = "housing"
)
def Xform "housing"
{
    def Xform "housing_link" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    )
    {
        def Cube "shell" (
            prepend apiSchemas = ["PhysicsCollisionAPI"]
        )
        {
            double size = 0.1
        }
    }
}
"""


def test_a_sensor_does_not_see_the_housing_it_is_mounted_in():
    robot = toy_robot().with_frames({"lidar": ("head", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))})
    housed = graft_usd(robot.usd, {"lidar": _HOUSING})
    scene = UsdModel.from_file(SCENE).with_frames(
        {"dog": ("scene", (0.0, 0.0, 1.0), (0.0, 0.0, 0.0))}
    )
    layer = graft_usd(scene.usd, {"dog": prefix_usd_frames(housed, "dog")})
    view = _view(layer_file(layer, called="housed-sensor-world"))
    view.update(_World().at(0.0, 0.0, 0.0))
    origin, _ = view.pose("dog/lidar")
    out_and_down = np.array([[1.0, 0.0, -1.0]]) / np.sqrt(2.0)

    inside, _ = view.raycaster((0,)).cast(origin, out_and_down, 10.0)
    seeing, _ = view.raycaster((0,), mounted_at="dog/lidar").cast(origin, out_and_down, 10.0)
    again, _ = view.raycaster((0,)).cast(origin, out_and_down, 10.0)

    assert seeing[0] > 1.0, "the housing around the lidar should not stop its own rays"
    assert again[0] == pytest.approx(inside[0]), "other sensors still see the housing"
    view.close()


def test_blind_to_lets_the_rays_through_further_frames(composed):
    view = _view(composed)
    view.update(_World().at(0.0, 0.0, 0.0))
    base, _ = view.pose("dog/base")
    origin = base + np.array([0.0, -1.0, 0.0])
    at_the_robot = np.array([[0.0, 1.0, 0.0]])

    stopped, _ = view.raycaster((0,)).cast(origin, at_the_robot, 10.0)
    through, _ = view.raycaster((0,), blind_to=["dog/base"]).cast(origin, at_the_robot, 10.0)

    assert stopped[0] == pytest.approx(0.9, abs=1e-3)
    assert through[0] < 0


def test_blind_to_a_robot_lets_the_rays_through_all_of_its_links(composed):
    view = _view(composed)
    view.update(_World().at(0.0, 0.0, np.pi / 2))
    head, _ = view.pose("dog/eye")
    base, _ = view.pose("dog/base")
    sideways = np.array([[0.0, 1.0, 0.0]])

    seeing = view.raycaster((0,))
    blind = view.raycaster((0,), blind_to_robots=["dog"])

    for link in (head, base):
        assert seeing.cast(link - sideways[0], sideways, 10.0)[0][0] > 0
        assert blind.cast(link - sideways[0], sideways, 10.0)[0][0] < 0
    view.close()


def test_a_blind_spot_belongs_to_its_own_raycaster_and_not_to_the_view(composed):
    view = _view(composed)
    view.update(_World().at(0.0, 0.0, 0.0))
    base, _ = view.pose("dog/base")
    origin = base + np.array([0.0, -1.0, 0.0])
    at_the_robot = np.array([[0.0, 1.0, 0.0]])

    through, _ = view.raycaster((0,), blind_to=["dog/base"]).cast(origin, at_the_robot, 10.0)
    again, _ = view.raycaster((0,)).cast(origin, at_the_robot, 10.0)

    assert through[0] < 0, "the blind cast should have passed through"
    assert again[0] == pytest.approx(0.9, abs=1e-3), "the next caster is still blind to the base"


def test_an_imu_reads_gravity_at_rest_and_acceleration_from_pose_changes(composed):
    view, world = _view(composed, "imu", "dog/imu"), _World()
    view.update(world.at(1.0, 0.0, 0.0))
    gyro, accelerometer, _ = view.imu("dog/imu")
    assert accelerometer == pytest.approx([0.0, 0.0, 9.81])
    assert gyro == pytest.approx([0.0, 0.0, 0.0])
    view.update(world.at(1.5, 0.5, 0.0))
    _, accelerometer, _ = view.imu("dog/imu")
    assert accelerometer == pytest.approx([2.0, 0.0, 9.81])


def test_a_view_reads_its_scene_from_one_parse(composed, monkeypatch):
    from dimos.robot.assets.usd_utils.pxr import load_pxr

    pxr = load_pxr()
    open_stage, already_parsed = pxr.Usd.Stage.Open, []

    def spied(path, *args):
        already_parsed.append(bool(pxr.Sdf.Layer.Find(str(path))))
        return open_stage(path, *args)

    monkeypatch.setattr(pxr.Usd.Stage, "Open", spied)
    _view(composed, "camera", "dog/eye")

    assert len(already_parsed) > 1
    assert already_parsed[1:] == [True] * (len(already_parsed) - 1)


def test_a_sensor_named_for_a_site_the_world_has_is_mounted_without_compiling(
    composed, monkeypatch
):
    from dimos.sim2.mujoco.world import compile_world, mount_sensor

    spec = compile_world(composed, ids=["dog"])
    mount_sensor(spec, SensorMount("imu", "dog/imu", "dog/imu"))
    sites = [site.name for site in spec.sites]
    monkeypatch.setattr(mujoco.MjSpec, "copy", lambda self: pytest.fail("compiled a copy"))
    mount_sensor(spec, SensorMount("imu", "dog/imu", "dog/imu"))

    assert sites == ["dog/imu"]
    assert [site.name for site in spec.sites] == sites
