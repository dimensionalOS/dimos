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

# linux-aarch64 has no usd-core wheel, and every scene here is a USD layer.
pytest.importorskip("pxr")

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.sim2.mujoco.sensor_mount import SensorMount, mount_sensor
from dimos.sim2.mujoco.sensor_world import WorldView, open_view, robot_ids
from dimos.sim2.mujoco.view import MujocoView
from dimos.sim2.mujoco.world import compile_world
from dimos.sim2.scene import layer_file, toy_robot
from dimos.sim2.sim import ROBOT, WorldMirror, WorldWriter, compose_usds

SCENE = Path(__file__).parent / "assets" / "scene.usda"
# The scene's frame the dog mounts on, and so the dog's namespace.
_DOG = "scene/dog"
_STILL = [0.0] * 6
_OUT_AND_DOWN = np.array([[1.0, 0.0, -1.0]]) / np.sqrt(2.0)

_HOUSING = """#usda 1.0
(defaultPrim = "housing")
def Xform "housing" {
    def Xform "housing_link" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {
        def Cube "shell" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {
            double size = 0.1
        }
    }
}
"""
_TEXTURED = (
    '<mujoco><asset><texture name="checker" type="2d" builtin="checker" width="64" height="64"/>'
    '<material name="checkered" texture="checker"/></asset><worldbody>'
    '<geom type="box" size="1 1 1" pos="3 0 0" material="checkered"/></worldbody></mujoco>'
)


def _compose(housed: bool = False) -> Path:
    head = "toy/Geometry/base/head"
    robot = toy_robot().with_frames(
        {
            "imu": ("toy/Geometry/base", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            "eye": (head, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            "lidar": (head, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        }
    )
    scene = UsdModel.from_file(SCENE).with_frames(
        {"dog": ("scene", (0.0, 0.0, 1.0), (0.0, 0.0, 0.0))}
    )
    lidar = f"{_DOG}/{head}/lidar"
    housing = [(usd_under_namespace(_HOUSING, lidar), lidar)] if housed else []
    layers = [(scene.usd, None), (usd_under_namespace(robot.usd, _DOG), _DOG), *housing]
    return layer_file(compose_usds(layers).usd, called="sensor-world")


@pytest.fixture
def composed() -> Path:
    return _compose()


def _view(
    composed: Path, kind: Literal["camera", "lidar", "imu"] = "lidar", frame: str = "dog/lidar"
) -> WorldView:
    return open_view(composed, ["dog"], SensorMount(kind, frame, frame))


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


def test_a_view_follows_the_world_its_imu_and_raycasts_included(composed: Path) -> None:
    view = open_view(composed, ["cat", "dog"], SensorMount("imu", "dog/imu", "dog/imu"))
    world = _World()
    world.writer.add("cat", **{ROBOT: True})
    world.writer.add("cat/neck")
    world.writer.body_state("cat", [9.0, 0.0, 1.0, 0, 0, 0, 1], _STILL, _STILL)
    world.writer.joint("cat/neck", np.pi / 2, 0.0, 0.0)
    mirror = world.at(1.0, 0.0, 0.0)
    assert robot_ids(mirror) == ["cat", "dog"]
    view.update(mirror)
    gyro, accelerometer, _ = view.imu("dog/imu")
    assert accelerometer == pytest.approx([0.0, 0.0, 9.81])
    assert gyro == pytest.approx([0.0, 0.0, 0.0])
    # The cat's turned neck, a robot the view holds but not the dog's, leaves the dog's lidar alone.
    w, _, _, z = view.pose("dog/lidar")[1]
    assert 2 * np.arctan2(z, w) == pytest.approx(0.0, abs=1e-6)

    origin, _ = view.pose("dog/lidar")
    blinded, _ = view.raycaster((0,)).cast(origin, _OUT_AND_DOWN, 10.0)
    seeing, _ = view.raycaster((0,), mounted_at="dog/lidar").cast(origin, _OUT_AND_DOWN, 10.0)
    assert blinded[0] == pytest.approx(0.05, abs=1e-3)
    assert seeing[0] > 1.0, "a sensor does not see the body it is mounted on"
    at_the_robot = np.array([[0.0, 1.0, 0.0]])
    beside_the_base = view.pose("dog/base")[0] - at_the_robot[0]
    through, _ = view.raycaster((0,), blind_to_robots=["dog"]).cast(
        beside_the_base, at_the_robot, 10.0
    )
    again, _ = view.raycaster((0,)).cast(beside_the_base, at_the_robot, 10.0)
    assert through[0] < 0, "the blind cast should have passed through"
    assert again[0] == pytest.approx(0.9, abs=1e-3), "the next caster still sees the base"

    view.update(world.at(1.5, 0.5, np.pi / 2, speed=1.0))
    assert view.time == 1.5
    assert view.imu("dog/imu")[1] == pytest.approx([2.0, 0.0, 9.81])
    assert view.pose("dog/base")[0] == pytest.approx([0.5, 0.0, 1.0])
    assert view.pose("crate")[0] == pytest.approx([3.0, 0.0, 0.1])
    w, _, _, z = view.pose("dog/lidar")[1]
    assert abs(2 * np.arctan2(z, w)) == pytest.approx(np.pi / 2, abs=1e-6)
    floor, _ = view.raycaster((0,)).cast(
        np.array([5.0, 0.0, 1.0]), np.array([[0.0, 0.0, -1.0]]), 10.0
    )
    assert floor[0] == pytest.approx(1.0)
    # Blind to a robot is blind to every one of its links, the turned head as well as the base.
    seeing_all, blind = view.raycaster((0,)), view.raycaster((0,), blind_to_robots=["dog"])
    for link in ("dog/eye", "dog/base"):
        start = view.pose(link)[0] - at_the_robot[0]
        assert seeing_all.cast(start, at_the_robot, 10.0)[0][0] > 0
        assert blind.cast(start, at_the_robot, 10.0)[0][0] < 0
    view.close()


@pytest.mark.self_hosted
def test_the_view_renders_from_a_camera_frame(composed: Path, renders: None) -> None:
    view = _view(composed, "camera", "dog/eye")
    view.update(_World().at(1.0, 2.0, np.pi / 2))
    assert view.render("dog/eye", 32, 24).shape == (24, 32, 3)
    view.close()


def test_a_sensor_does_not_see_the_housing_it_is_mounted_in() -> None:
    view = _view(_compose(housed=True))
    view.update(_World().at(0.0, 0.0, 0.0))
    origin, _ = view.pose("dog/lidar")
    inside, _ = view.raycaster((0,)).cast(origin, _OUT_AND_DOWN, 10.0)
    seeing, _ = view.raycaster((0,), mounted_at="dog/lidar").cast(origin, _OUT_AND_DOWN, 10.0)
    again, _ = view.raycaster((0,)).cast(origin, _OUT_AND_DOWN, 10.0)
    assert seeing[0] > 1.0, "the housing around the lidar should not stop its own rays"
    assert again[0] == pytest.approx(inside[0]), "other sensors still see the housing"
    view.close()


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
    spec = compile_world(composed, ids=["dog"])
    mount_sensor(spec, SensorMount("imu", "dog/imu", "dog/imu"))
    monkeypatch.setattr(mujoco.MjSpec, "copy", lambda self: pytest.fail("compiled a copy"))
    mount_sensor(spec, SensorMount("imu", "dog/imu", "dog/imu"))
    assert [site.name for site in spec.sites] == ["dog/imu"]


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
