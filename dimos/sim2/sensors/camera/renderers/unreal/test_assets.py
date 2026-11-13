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

"""Coordinate and revision contracts at the native rendering boundary."""

from pathlib import Path

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.geometry_msgs import Point, Pose, Quaternion
from dimos_lcm.sim_msgs import BodyState, WorldState
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.robot.assets.model import RobotModel
from dimos.sim2.scenes.urdf import Component, prepare_scene
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.renderers.unreal.assets import export_visuals, unreal_pose
from dimos.sim2.sensors.camera.renderers.unreal.renderer import UnrealRenderer


def test_optical_forward_right_down_become_unreal_forward_right_down():
    transform = np.eye(4)
    transform[:3, 3] = [1, 2, 3]
    wire = unreal_pose(transform, camera=True)
    rotation = Rotation.from_quat(wire["quaternion"]).as_matrix()
    assert wire["position"] == [100, -200, 300]
    np.testing.assert_allclose(rotation @ [1, 0, 0], [0, 0, 1], atol=1e-12)
    np.testing.assert_allclose(rotation @ [0, 1, 0], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(rotation @ [0, 0, -1], [0, -1, 0], atol=1e-12)


@pytest.fixture
def scene(tmp_path):
    path = tmp_path / "model.urdf"
    path.write_text(
        '<robot name="office"><link name="office/world"><visual><geometry><box size="1 2 3"/></geometry></visual></link></robot>'
    )
    return prepare_scene({"office": Component(RobotModel.from_file(path))}, ground_plane=False)


def test_authored_level_is_not_duplicated_and_cannot_be_repositioned(scene):
    manifest, visuals, bodies = export_visuals(scene, ("office/world",))
    assert manifest["geoms"] == []
    assert visuals == []
    assert bodies == {"office/world"}


def test_native_visual_replaces_geometry_and_follows_its_body(scene, tmp_path, mocker):
    mocker.patch("dimos.sim2.sensors.camera.renderers.unreal.renderer.RendererProcess")
    asset = "/Game/Props/Bottle.Bottle"
    manifest, visuals, static = export_visuals(scene, (), {"office/world": asset})
    assert manifest["meshes"] == []
    assert len(manifest["geoms"]) == len(visuals) == 1
    assert manifest["geoms"][0]["static_mesh"] == asset
    assert not static
    renderer = UnrealRenderer(
        Path("unreal"), native_visuals={"office/world": asset}, log_path=tmp_path / "render.log"
    )
    try:
        renderer.load(scene, (CameraConfig("office/world"),))
        renderer.client.capture.return_value.rgba = np.zeros((1, 1, 4), dtype=np.uint8)
        renderer.client.capture.return_value.depth = np.zeros((1, 1), dtype=np.float32)
        for z in (1.0, 1.2):
            pose = Pose(position=Point(2, 3, z), orientation=Quaternion(w=1))
            sample = WorldState(
                header=SampleHeader(),
                model_revision=scene.revision,
                bodies_length=1,
                bodies=[BodyState(body_id="office/world", pose=pose)],
            )
            renderer.render(sample, ("office/world",))
            assert renderer.client.capture.call_args.args[0]["poses"][0]["position"] == [
                200,
                -300,
                z * 100,
            ]
    finally:
        renderer.close()


@pytest.mark.parametrize(
    "authored, mapping",
    [
        (("office/world",), {"office/world": "/Game/Props/Bottle.Bottle"}),
        ((), {"missing/body": "/Game/Props/Bottle.Bottle"}),
        ((), {"office/world": "Bottle"}),
    ],
)
def test_native_visual_rejects_unbound_or_invalid_assets(scene, authored, mapping):
    with pytest.raises(ValueError):
        export_visuals(scene, authored, mapping)


def test_renderer_rejects_mismatched_model_and_moving_authored_background(scene, tmp_path, mocker):
    mocker.patch("dimos.sim2.sensors.camera.renderers.unreal.renderer.RendererProcess")
    renderer = UnrealRenderer(
        Path("unreal"),
        level="/Game/Office",
        authored_bodies=("office/world",),
        log_path=tmp_path / "render.log",
    )
    try:
        renderer.load(scene, (CameraConfig("office/world"),))
        sample = WorldState(
            header=SampleHeader(),
            model_revision=scene.revision,
            bodies_length=1,
            bodies=[
                BodyState(
                    body_id="office/world", pose=Pose(position=Point(), orientation=Quaternion(w=1))
                )
            ],
        )
        renderer.set_poses(sample)
        sample.model_revision = "different"
        with pytest.raises(ValueError, match="model revision"):
            renderer.set_poses(sample)
        sample.model_revision = scene.revision
        sample.bodies[0].pose.position.x = 1
        with pytest.raises(ValueError, match="moved"):
            renderer.set_poses(sample)
    finally:
        renderer.close()


def test_shared_views_use_one_scene_and_one_acquisition(scene, tmp_path, mocker):
    mocker.patch("dimos.sim2.sensors.camera.renderers.unreal.renderer.RendererProcess")
    scene = scene.with_frames({"wrist": ("office/world", (0, 0, 0.1), (0, 0, 0))})
    renderer = UnrealRenderer(Path("unreal"), log_path=tmp_path / "render.log")
    try:
        renderer.load(scene, (CameraConfig("office/world"), CameraConfig("wrist")))
        renderer.client.capture.return_value.rgba = np.zeros((1, 1, 4), dtype=np.uint8)
        renderer.client.capture.return_value.depth = np.ones((1, 1), dtype=np.float32)
        sample = WorldState(
            header=SampleHeader(clock_id="shared", epoch=2, time_ns=123456789, sequence=17),
            model_revision=scene.revision,
            bodies_length=2,
            bodies=[
                BodyState(body_id=n, pose=Pose(orientation=Quaternion(w=1)))
                for n in ("office/world", "wrist")
            ],
        )
        images = renderer.render(sample, ("office/world", "wrist"))
        assert set(images) == {"office/world", "wrist"}
        assert renderer.client.load.call_count == 1
        commands = [call.args[0] for call in renderer.client.capture.call_args_list]
        assert [c["camera"]["view_id"] for c in commands] == ["office/world", "wrist"]
        for command in commands:
            assert command["time_ns"] == "123456789"
            assert command["epoch"] == "2"
            assert command["sequence"] == "17"
        assert commands[0]["poses"] is commands[1]["poses"]
        with pytest.raises(ValueError, match="distinct configured"):
            renderer.render(sample, ("wrist", "wrist"))
    finally:
        renderer.close()
