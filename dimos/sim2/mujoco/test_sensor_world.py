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
from typing import Any, Literal, cast

import mujoco
import numpy as np
import pytest

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.mujoco.sensor_mount import SensorMount
from dimos.sim2.mujoco.sensor_world import WorldView, open_view, robot_ids
from dimos.sim2.mujoco.view import MujocoView
from dimos.sim2.sim import compose_usds
from dimos.sim2.spec import ROBOT
from dimos.sim2.usd_layer import layer_file
from dimos.sim2.world_observations import WorldMirror, WorldWriter

# Every scene here is read out of a USD layer, and a runner without usd-core cannot read one:
# linux-aarch64 has no wheel for it (pyproject's marker says so), so without this the tests
# fail there on dimos's own "Reading or writing USD needs usd-core (pxr)". Their neighbours
# in this directory skip the same way.
pytest.importorskip("pxr")

SCENE = Path(__file__).parent / "assets" / "scene.usda"
# The scene's frame the dog mounts on, and so the dog's namespace.
_DOG = "scene/dog"


@pytest.fixture
def composed() -> Path:
    robot = toy_robot().with_frames(
        {
            "imu": ("toy/Geometry/base", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            "eye": ("toy/Geometry/base/head", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            "lidar": ("toy/Geometry/base/head", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        }
    )
    scene = UsdModel.from_file(SCENE).with_frames(
        {"dog": ("scene", (0.0, 0.0, 1.0), (0.0, 0.0, 0.0))}
    )
    world = compose_usds([(scene.usd, None), (usd_under_namespace(robot.usd, _DOG), _DOG)])
    return layer_file(world.usd, called="sensor-world")


def _view(
    composed: Path, kind: Literal["camera", "lidar", "imu"] = "lidar", frame: str = "dog/lidar"
) -> WorldView:
    return open_view(composed, ["dog"], SensorMount(kind, frame, frame))


_STILL = [0.0] * 6


class _World:
    def __init__(self) -> None:
        self.writer, self.mirror = WorldWriter(), WorldMirror()
        self.writer.start_ns = 0
        self.writer.add("world")
        self.writer.add("dog", **{ROBOT: True})
        self.writer.add("dog/neck")
        self.writer.add("crate")
        self.writer.body_state("crate", [3.0, 0.0, 0.1, 0, 0, 0, 1], _STILL, _STILL)

    def at(self, sim_time: float, x: float, neck: float, speed: float = 0.0) -> WorldMirror:
        """The dog at ``x``, moving along x at ``speed``, its neck at ``neck``."""
        self.writer.body_state("dog", [x, 0.0, 1.0, 0, 0, 0, 1], [speed, 0, 0, 0, 0, 0], _STILL)
        self.writer.joint("dog/neck", neck, 0.0, 0.0)
        self.mirror.apply(self.writer.end_frame(sim_time))
        return self.mirror


def test_the_robots_in_the_world_are_the_ones_the_world_announced() -> None:
    assert robot_ids(_World().at(0.0, 0.0, 0.0)) == ["dog"]


def test_the_view_is_posed_by_the_world_alone(composed: Path) -> None:
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
def test_the_view_renders_from_a_camera_frame(composed: Path, renders: None) -> None:
    view = _view(composed, "camera", "dog/eye")
    view.update(_World().at(1.0, 2.0, np.pi / 2))
    assert view.render("dog/eye", 32, 24).shape == (24, 32, 3)
    view.close()


def test_a_sensor_does_not_see_the_body_it_is_mounted_on(composed: Path) -> None:
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


def test_a_sensor_does_not_see_the_housing_it_is_mounted_in() -> None:
    robot = toy_robot().with_frames(
        {"lidar": ("toy/Geometry/base/head", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))}
    )
    scene = UsdModel.from_file(SCENE).with_frames(
        {"dog": ("scene", (0.0, 0.0, 1.0), (0.0, 0.0, 0.0))}
    )
    lidar = f"{_DOG}/toy/Geometry/base/head/lidar"
    world = compose_usds(
        [
            (scene.usd, None),
            (usd_under_namespace(robot.usd, _DOG), _DOG),
            (usd_under_namespace(_HOUSING, lidar), lidar),
        ]
    )
    view = _view(layer_file(world.usd, called="housed-sensor-world"))
    view.update(_World().at(0.0, 0.0, 0.0))
    origin, _ = view.pose("dog/lidar")
    out_and_down = np.array([[1.0, 0.0, -1.0]]) / np.sqrt(2.0)

    inside, _ = view.raycaster((0,)).cast(origin, out_and_down, 10.0)
    seeing, _ = view.raycaster((0,), mounted_at="dog/lidar").cast(origin, out_and_down, 10.0)
    again, _ = view.raycaster((0,)).cast(origin, out_and_down, 10.0)

    assert seeing[0] > 1.0, "the housing around the lidar should not stop its own rays"
    assert again[0] == pytest.approx(inside[0]), "other sensors still see the housing"
    view.close()


def test_blind_to_a_robot_lets_the_rays_through_all_of_its_links(composed: Path) -> None:
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


def test_a_blind_spot_belongs_to_its_own_raycaster_and_not_to_the_view(composed: Path) -> None:
    view = _view(composed)
    view.update(_World().at(0.0, 0.0, 0.0))
    base, _ = view.pose("dog/base")
    origin = base + np.array([0.0, -1.0, 0.0])
    at_the_robot = np.array([[0.0, 1.0, 0.0]])

    through, _ = view.raycaster((0,), blind_to_robots=["dog"]).cast(origin, at_the_robot, 10.0)
    again, _ = view.raycaster((0,)).cast(origin, at_the_robot, 10.0)

    assert through[0] < 0, "the blind cast should have passed through"
    assert again[0] == pytest.approx(0.9, abs=1e-3), "the next caster is still blind to the base"


def test_a_view_of_one_robot_is_not_moved_by_a_robot_it_does_not_hold(composed: Path) -> None:
    view = open_view(composed, ["cat", "dog"], SensorMount("imu", "dog/imu", "dog/imu"))
    world = _World()
    world.writer.add("cat", **{ROBOT: True})
    world.writer.add("cat/neck")
    world.writer.body_state("cat", [9.0, 0.0, 1.0, 0, 0, 0, 1], _STILL, _STILL)
    world.writer.joint("cat/neck", np.pi / 2, 0.0, 0.0)
    view.update(world.at(1.0, 2.0, 0.0))
    assert view.pose("dog/base")[0] == pytest.approx([2.0, 0.0, 1.0])
    w, _, _, z = view.pose("dog/lidar")[1]
    assert 2 * np.arctan2(z, w) == pytest.approx(0.0, abs=1e-6)


def test_an_imu_reads_gravity_at_rest_and_acceleration_from_velocity_changes(
    composed: Path,
) -> None:
    view, world = _view(composed, "imu", "dog/imu"), _World()
    view.update(world.at(1.0, 0.0, 0.0))
    gyro, accelerometer, _ = view.imu("dog/imu")
    assert accelerometer == pytest.approx([0.0, 0.0, 9.81])
    assert gyro == pytest.approx([0.0, 0.0, 0.0])
    view.update(world.at(1.5, 0.5, 0.0, speed=1.0))
    _, accelerometer, _ = view.imu("dog/imu")
    assert accelerometer == pytest.approx([2.0, 0.0, 9.81])


def test_a_view_reads_its_scene_from_one_parse(
    composed: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from dimos.robot.assets.usd_utils.pxr import load_pxr

    pxr = load_pxr()
    open_stage, already_parsed = pxr.Usd.Stage.Open, []

    def spied(path: Any, *args: Any) -> Any:
        already_parsed.append(bool(pxr.Sdf.Layer.Find(str(path))))
        return open_stage(path, *args)

    monkeypatch.setattr(pxr.Usd.Stage, "Open", spied)
    _view(composed, "camera", "dog/eye")

    assert len(already_parsed) > 1
    assert already_parsed[1:] == [True] * (len(already_parsed) - 1)


def test_a_sensor_named_for_a_site_the_world_has_is_mounted_without_compiling(
    composed: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from dimos.sim2.mujoco.sensor_mount import mount_sensor
    from dimos.sim2.mujoco.world import compile_world

    spec = compile_world(composed, ids=["dog"])
    mount_sensor(spec, SensorMount("imu", "dog/imu", "dog/imu"))
    sites = [site.name for site in spec.sites]
    monkeypatch.setattr(mujoco.MjSpec, "copy", lambda self: pytest.fail("compiled a copy"))
    mount_sensor(spec, SensorMount("imu", "dog/imu", "dog/imu"))

    assert sites == ["dog/imu"]
    assert [site.name for site in spec.sites] == sites


_TEXTURED = """<mujoco>
  <asset>
    <texture name="checker" type="2d" builtin="checker" width="64" height="64"/>
    <material name="checkered" texture="checker"/>
  </asset>
  <worldbody>
    <geom type="box" size="1 1 1" pos="3 0 0" material="checkered"/>
  </worldbody>
</mujoco>
"""


def test_only_a_camera_view_loads_the_textures_and_a_lidar_still_sees_the_textured_box(
    tmp_path: Path,
) -> None:
    scene = tmp_path / "textured.xml"
    scene.write_text(_TEXTURED)
    lidar = cast("MujocoView", open_view(scene, mount=SensorMount("lidar", "world", "lidar")))
    camera = cast("MujocoView", open_view(scene, mount=SensorMount("camera", "world", "eye")))

    distance, _ = lidar.raycaster((0,)).cast(np.zeros(3), np.array([[1.0, 0.0, 0.0]]), 10.0)

    assert lidar.model.tex_data.size < camera.model.tex_data.size
    assert distance[0] == pytest.approx(2.0)
    with pytest.raises(ValueError, match="open a camera view"):
        lidar.render("eye", 32, 24)
