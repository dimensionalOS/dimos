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

from __future__ import annotations

import asyncio
from collections.abc import Callable, Iterator
from pathlib import Path
import threading
from typing import Any, cast
from unittest.mock import patch

import numpy as np
from open3d.core import Tensor
import pinocchio as pin
import pytest
import trimesh

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.msgs.files.File import File
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.protocol.tf.tf import TF
from dimos.robot.assets.git_cache import DEFAULT_ROBOT_ASSET_CACHE_ROOT, GitAssetCache
from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL
from dimos.robot.manipulators.xarm.config import (
    XARM_ROS2_REF,
    XARM_ROS2_REPO,
    make_xarm7_model_config,
)
from dimos.robot.roboplan_body_filter import RoboPlanBodyFilter
from dimos.robot.urdf_manager import UrdfManager
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing

# A 20cm cube on `arm`, with no external assets.
_URDF = """<?xml version="1.0"?>
<robot name="box_robot">
  <link name="base"/>
  <link name="arm">
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <joint name="shoulder" type="fixed">
    <parent link="base"/><child link="arm"/>
  </joint>
</robot>
"""


@pytest.fixture
def make_filter(tmp_path: Path) -> Iterator[Callable[..., UrdfManagerPointClearing]]:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(_URDF)
    modules: list[UrdfManagerPointClearing] = []

    def make(**overrides: Any) -> UrdfManagerPointClearing:
        urdf.write_text(overrides.pop("urdf_xml", _URDF))
        settings: dict[str, Any] = {
            "model": RobotModel.from_file(urdf),
            "padding_m": 0.01,
            "tf_tolerance_s": 0.001,
            "tf_forward_tolerance_s": 0.0,
        }
        settings.update(overrides)
        module = UrdfManagerPointClearing(**settings)
        cast("dict[str, Any]", module.__dict__)["_tf"] = TF()
        modules.append(module)
        return module

    yield make
    for module in modules:
        cast("dict[str, Any]", module.__dict__)["_tf"] = None
        module.dispose()


def _place_arm(module: UrdfManagerPointClearing, at: tuple[float, float, float], ts: float) -> None:
    """Move the rigid robot base at capture time in both frames."""
    for parent in ("camera", "world"):
        module.tfbuffer.receive_transform(
            Transform(
                translation=Vector3(*at),
                frame_id=parent,
                child_frame_id="base",
                ts=ts,
            )
        )


def _cloud(points: list[list[float]], ts: float = 1.0) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float32).reshape((-1, 3)),
        frame_id="camera",
        timestamp=ts,
    )


def test_points_on_the_robot_are_dropped_and_the_rest_survive(
    make_filter: Callable[..., UrdfManagerPointClearing],
) -> None:
    module = make_filter()
    _place_arm(module, (0.0, 0.0, 0.0), 1.0)

    result = module.filter_cloud(_cloud([[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [2.0, 0.0, 0.0]]))

    assert result is not None
    filtered = result
    np.testing.assert_allclose(filtered.points_f32(), [[2.0, 0.0, 0.0]], atol=1e-6)


def test_a_cloud_without_capture_time_tf_is_dropped_whole(
    make_filter: Callable[..., UrdfManagerPointClearing],
) -> None:
    # Half-filtering would let the arm into the map. Better to lose the frame.
    module = make_filter()

    assert module.filter_cloud(_cloud([[2.0, 0.0, 0.0]])) is None


def _joint_robot(kind: str) -> str:
    return _URDF.replace('name="shoulder" type="fixed"', f'name="shoulder" type="{kind}"').replace(
        '<parent link="base"/><child link="arm"/>',
        '<parent link="base"/><child link="arm"/><axis xyz="1 0 0"/>'
        '<limit lower="-3" upper="3" effort="1" velocity="1"/>',
    )


def test_capture_state_is_matched_by_timestamp_instead_of_latest(make_filter):
    module = make_filter(urdf_xml=_joint_robot("prismatic"), state_tolerance_s=0.001)
    _place_arm(module, (0, 0, 0), 1.0)
    module.add_joint_state(JointState(ts=2.0, name=["shoulder"], position=[2.0]))
    module.add_joint_state(JointState(ts=1.0, name=["shoulder"], position=[0.0]))
    module.add_joint_state(JointState(ts=1.0, name=["shoulder"], position=[0.5]))

    result = module.filter_cloud(_cloud([[0.5, 0, 0], [2, 0, 0]], ts=1.0))

    assert result is not None
    np.testing.assert_allclose(result.points_f32(), [[2, 0, 0]])


@pytest.mark.parametrize(
    "state",
    [
        None,
        JointState(ts=2.0, name=["shoulder"], position=[0.5]),
        JointState(ts=1.0, name=["wrong"], position=[0.5]),
        JointState(ts=1.0, name=["shoulder"], position=[float("nan")]),
    ],
)
def test_missing_late_or_invalid_state_drops_the_capture(make_filter, state):
    module = make_filter(urdf_xml=_joint_robot("prismatic"), state_tolerance_s=0.001)
    _place_arm(module, (0, 0, 0), 1.0)
    if state is not None:
        module.add_joint_state(state)

    assert module.filter_cloud(_cloud([[0.5, 0, 0]])) is None


def test_continuous_joint_uses_cos_sin_configuration(make_filter):
    urdf = _joint_robot("continuous").replace('<axis xyz="1 0 0"/>', '<axis xyz="0 0 1"/>')
    urdf = urdf.replace("<collision><geometry>", '<collision><origin xyz="0.5 0 0"/><geometry>')
    module = make_filter(urdf_xml=urdf)
    _place_arm(module, (0, 0, 0), 1.0)
    module.add_joint_state(JointState(ts=1.0, name=["shoulder"], position=[np.pi / 2]))

    result = module.filter_cloud(_cloud([[0, 0.5, 0], [0.5, 0, 0]]))

    assert result is not None
    np.testing.assert_allclose(result.points_f32(), [[0.5, 0, 0]])


@pytest.mark.parametrize("kind", ["planar", "floating"])
def test_multidof_joints_use_capture_time_relative_tf(make_filter, kind):
    urdf = _joint_robot(kind).replace(
        '<axis xyz="1 0 0"/>', '<axis xyz="1 0 0"/><origin xyz="0.2 0.1 0" rpy="0 0 0.3"/>'
    )
    module = make_filter(urdf_xml=urdf)
    _place_arm(module, (0, 0, 0), 1.0)
    assert module.filter_cloud(_cloud([[0.5, 0, 0]])) is None
    module.tfbuffer.receive_transform(
        Transform(translation=Vector3(0.5, 0, 0), frame_id="base", child_frame_id="arm", ts=1.0)
    )

    result = module.filter_cloud(_cloud([[0.5, 0, 0], [0, 0, 0]]))

    assert result is not None
    np.testing.assert_allclose(result.points_f32(), [[0, 0, 0]])


def test_narrowphase_retains_sphere_corner_obstacles_and_ancillary_fields(make_filter):
    urdf = _URDF.replace('<box size="0.2 0.2 0.2"/>', '<sphere radius="0.1"/>')
    module = make_filter(urdf_xml=urdf, padding_m=0.05)
    _place_arm(module, (0, 0, 0), 1.0)
    cloud = PointCloud2.from_numpy(
        np.array([[0.14, 0, 0], [0.12, 0.12, 0], [0.2, 0, 0]], dtype=np.float32),
        frame_id="camera",
        timestamp=1.0,
        intensities=np.array([1, 2, 3], dtype=np.float32),
    )
    cloud.seq = 42
    cloud.pointcloud_tensor.point["labels"] = Tensor(np.array([[10], [20], [30]], dtype=np.int32))

    result = module.filter_cloud(cloud)

    assert result is not None
    filtered = result
    np.testing.assert_allclose(filtered.points_f32(), [[0.12, 0.12, 0], [0.2, 0, 0]])
    np.testing.assert_array_equal(filtered.intensities_f32(), [2, 3])
    np.testing.assert_array_equal(filtered.pointcloud_tensor.point["labels"].numpy(), [[20], [30]])
    assert (filtered.frame_id, filtered.ts, filtered.seq) == ("camera", 1.0, 42)


def test_rotated_base_aligns_camera_points(make_filter):
    module = make_filter()
    module.tfbuffer.receive_transform(Transform(frame_id="world", child_frame_id="camera", ts=1.0))
    module.tfbuffer.receive_transform(
        Transform(
            translation=Vector3(-1, 0, 0),
            rotation=Quaternion(0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)),
            frame_id="world",
            child_frame_id="base",
            ts=1.0,
        )
    )

    result = module.filter_cloud(_cloud([[-1, 0.05, 0], [-1, 1, 0]]))

    assert result is not None
    np.testing.assert_allclose(result.points_f32(), [[-1, 1, 0]])


def test_mesh_filter_removes_surfaces_without_classifying_solid_volume(make_filter, tmp_path):
    mesh_path = tmp_path / "cube.stl"
    trimesh.creation.box(extents=[0.2, 0.2, 0.2]).export(mesh_path)
    urdf = _URDF.replace('<box size="0.2 0.2 0.2"/>', f'<mesh filename="{mesh_path}"/>')
    module = make_filter(urdf_xml=urdf)
    _place_arm(module, (0, 0, 0), 1.0)

    result = module.filter_cloud(_cloud([[0.1, 0, 0], [0.105, 0, 0], [0, 0, 0], [0.2, 0, 0]]))

    assert result is not None
    np.testing.assert_allclose(result.points_f32(), [[0, 0, 0], [0.2, 0, 0]])


def test_late_state_cannot_resurrect_expired_history(make_filter):
    module = make_filter(urdf_xml=_joint_robot("prismatic"), state_history_s=1.0)
    _place_arm(module, (0, 0, 0), 1.0)
    module.add_joint_state(JointState(ts=3.0, name=["shoulder"], position=[0.5]))
    module.add_joint_state(JointState(ts=1.0, name=["shoulder"], position=[0.5]))

    assert module.filter_cloud(_cloud([[0.5, 0, 0]], ts=1.0)) is None


@pytest.mark.self_hosted
def test_cached_xarm_arm_surfaces_need_no_gripper_state(make_filter, monkeypatch):
    key = GitAssetCache._source_key(XARM_ROS2_REPO, XARM_ROS2_REF)
    cached = DEFAULT_ROBOT_ASSET_CACHE_ROOT / "sources" / key / "xarm_ros2"
    if not cached.is_dir():
        pytest.skip("Pinned xArm assets are not cached; this test never fetches them")
    monkeypatch.setattr(GitAssetCache, "resolve", lambda *_: cached)
    config = make_xarm7_model_config(add_gripper=False)
    module = make_filter(model=config.model)
    description = config.model.load()
    model = pin.buildModelFromXML(description.xml, mimic=True)
    geoms = pin.buildGeomFromUrdfString(
        model,
        description.xml,
        pin.GeometryType.COLLISION,
        package_dirs=[str(p) for p in description.package_paths.values()],
    )
    assert model.nq == 7
    assert len(config.joint_names) == 7
    arm = [0.0, -0.04609, 0.0, 1.83940, 0.0, 1.87106, 0.0]
    controls = np.array([[2, 2, 2], [-2, -2, -2]], dtype=np.float32)
    captures = []
    for stamp, shoulder in ((1.0, 0.0), (2.0, 0.5)):
        arm[0] = shoulder
        module.add_joint_state(JointState(ts=stamp, name=config.joint_names, position=arm))
        data, gdata = model.createData(), pin.GeometryData(geoms)
        pin.updateGeometryPlacements(model, data, geoms, gdata, np.array(arm))
        surfaces = []
        for i, geom in enumerate(geoms.geometryObjects):
            local, _ = trimesh.sample.sample_surface(
                trimesh.load_mesh(geom.meshPath), 100, seed=42 + i
            )
            local *= np.asarray(geom.meshScale)
            surfaces.append(local @ gdata.oMg[i].rotation.T + gdata.oMg[i].translation)
        assert surfaces
        captures.append(
            PointCloud2.from_numpy(
                np.concatenate([*surfaces, controls]).astype(np.float32),
                frame_id="world",
                timestamp=stamp,
            )
        )
    # Both captures are processed after the newer state arrived.
    first = module.filter_cloud(captures[0])
    assert first is not None
    second = module.filter_cloud(captures[1])
    assert second is not None
    for cloud, filtered in zip(captures, (first, second), strict=True):
        np.testing.assert_allclose(filtered.points_f32(), controls)
        assert filtered.frame_id == "world" and filtered.ts == cloud.ts


def test_the_joint_state_is_published_as_one_tf_edge_per_joint(make_filter):
    module = make_filter(urdf_xml=_joint_robot("prismatic"))
    module.add_joint_state(JointState(ts=1.0, name=["shoulder"], position=[0.3]))

    edge = module.tfbuffer.get("base", "arm", time_point=1.0, time_tolerance=0.001)

    assert edge is not None
    np.testing.assert_allclose(
        [edge.translation.x, edge.translation.y, edge.translation.z], [0.3, 0, 0], atol=1e-9
    )


@pytest.mark.parametrize(
    "overrides",
    [
        # Multi-DOF joints are read from tf, so publishing them would feed the filter its own output.
        {"urdf_xml": _joint_robot("planar")},
        {"urdf_xml": _joint_robot("prismatic"), "publish_joint_tf": False},
    ],
)
def test_no_tf_edge_is_published(make_filter, overrides):
    module = make_filter(**overrides)
    module.add_joint_state(JointState(ts=1.0, name=["shoulder"], position=[0.3]))

    assert module.tfbuffer.get("base", "arm", time_point=1.0, time_tolerance=0.001) is None


def test_render_file_request_serves_the_usd_and_only_files_it_names(
    make_filter, tmp_path, monkeypatch
):
    package = tmp_path / "box_description"
    (package / "meshes").mkdir(parents=True)
    trimesh.creation.box(extents=[0.2, 0.2, 0.2]).export(package / "meshes" / "cube.stl")
    (package / "textures").mkdir()
    (package / "textures" / "skin.png").write_bytes(b"png bytes")
    urdf = tmp_path / "mesh_robot.urdf"
    urdf.write_text(
        _URDF.replace(
            '<box size="0.2 0.2 0.2"/>', '<mesh filename="package://box/meshes/cube.stl"/>'
        ).replace(
            "</link>",
            '<visual><geometry><box size="0.1 0.1 0.1"/></geometry><material name="skin">'
            '<texture filename="package://box/textures/skin.png"/></material></visual></link>',
        )
    )
    module = make_filter(model=RobotModel.from_file(urdf, package_paths={"box": package}))
    sent: list[File] = []
    monkeypatch.setattr(module.file_info, "publish", sent.append)
    cube = str((package / "meshes" / "cube.stl").resolve())
    texture = str((package / "textures" / "skin.png").resolve())

    # the scene and the files by the full paths it names; the URDF and anything else gets nothing
    pytest.importorskip("pxr")
    for request in (
        "robot.usda",
        cube,
        texture,
        str(urdf),
        str(package / "meshes"),
        "package://box/meshes/cube.stl",
    ):
        asyncio.run(module.handle_render_file_request(String(request)))

    assert [file.uri for file in sent] == ["robot.usda", cube, texture]
    assert sent[0].data.decode().startswith("#usda")
    assert File.lcm_decode(sent[1].lcm_encode()).data == Path(cube).read_bytes()
    assert sent[2].data == b"png bytes"


def test_a_filtered_cloud_keeps_its_fields_and_capture_metadata_and_a_late_capture_is_still_filtered(
    make_filter: Callable[..., UrdfManagerPointClearing],
) -> None:
    module = make_filter()
    _place_arm(module, (0.0, 0.0, 0.0), 2.0)
    cloud = _cloud([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]], ts=2.0)
    cloud.seq = 7
    cloud.pointcloud_tensor.point["intensity"] = Tensor(np.asarray([[10], [20]], dtype=np.float32))

    result = module.filter_cloud(cloud)

    assert result is not None
    assert (result.frame_id, result.ts, result.seq) == ("camera", 2.0, 7)
    np.testing.assert_allclose(result.points_f32(), [[2.0, 0.0, 0.0]], atol=1e-6)
    np.testing.assert_array_equal(result.pointcloud_tensor.point["intensity"].numpy(), [[20]])
    # a second sensor on the same input lags the first: its older capture is matched to its own capture time
    _place_arm(module, (0.0, 0.0, 0.0), 1.0)
    late = module.filter_cloud(_cloud([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]], ts=1.0))
    assert late is not None
    np.testing.assert_allclose(late.points_f32(), [[2.0, 0.0, 0.0]], atol=1e-6)


def test_joint_transforms_match_roboplan_forward_kinematics_on_the_r1pro():
    pytest.importorskip("roboplan.core")
    description = R1PRO_MODEL.load()
    roboplan = RoboPlanBodyFilter(description, padding_m=0.0)
    module = UrdfManager(model=R1PRO_MODEL)
    rng = np.random.default_rng(0)
    positions = {
        joint.name: rng.uniform(
            -np.pi if joint.lower is None else joint.lower,
            np.pi if joint.upper is None else joint.upper,
        )
        for joint in description.joints
        if joint.type in ("revolute", "continuous", "prismatic")
    }
    configuration = roboplan.configuration(positions, {})
    assert configuration is not None
    try:
        transforms = module.joint_transforms(
            JointState(ts=1.0, name=list(positions), position=list(positions.values()))
        )
    finally:
        module.dispose()

    assert len(transforms) == len(description.joints)
    for transform in transforms:
        np.testing.assert_allclose(
            transform.to_matrix(),
            roboplan._context.forwardKinematics(
                configuration, transform.child_frame_id, transform.frame_id
            ),
            atol=1e-9,
        )


class _FakeArm(Module):
    """Stands in for a module bolted to the robot: all a mounted module owes is get_usd."""

    config: ModuleConfig

    @rpc
    def get_usd(self) -> str:
        return _SUBTREE_USD


class _NoUsd(Module):
    config: ModuleConfig


_SUBTREE_USD = """#usda 1.0
(
    defaultPrim = "fake_arm"
)

def Xform "fake_arm"
{
    custom string dimos:link = "fake_arm_base"
}
"""


class _FakeClient:
    """An RPC client that answers get_usd with a scripted sequence; an Exception means 'not up yet'."""

    def __init__(self, *answers: Any) -> None:
        self.answers = list(answers)
        self.asked = 0

    def get_usd(self) -> Any:
        self.asked += 1
        answer = self.answers.pop(0) if len(self.answers) > 1 else self.answers[0]
        if isinstance(answer, Exception):
            raise answer
        return answer

    def stop_rpc_client(self) -> None: ...


@pytest.fixture
def make_manager(tmp_path: Path) -> Iterator[Callable[..., UrdfManager]]:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(_URDF)
    modules: list[UrdfManager] = []

    def make(**overrides: Any) -> UrdfManager:
        settings: dict[str, Any] = {"model": RobotModel.from_file(urdf)}
        settings.update(overrides)
        module = UrdfManager(**settings)
        modules.append(module)
        return module

    yield make
    for module in modules:
        module.dispose()


def _usd_stage(usd: str) -> Any:
    from pxr import Sdf, Usd

    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(usd)
    return Usd.Stage.Open(layer)


def _scene(module: UrdfManager) -> str:
    """get_scene on a loop of this test's own, so its to_thread executor is torn down with it."""
    coroutine = cast("Any", UrdfManager.get_scene).aio.__get__(module, UrdfManager)
    return cast("str", asyncio.run(coroutine()))


def test_get_usd_is_this_robots_own_urdf_with_nothing_attached(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager()

    stage = _usd_stage(module.get_usd())

    assert stage.GetDefaultPrim().GetName() == "box_robot"
    assert stage.GetPrimAtPath("/box_robot/base/arm")


def test_get_usd_is_built_once_and_reused(make_manager: Callable[..., UrdfManager]) -> None:
    pytest.importorskip("pxr")
    module = make_manager()

    assert module.get_usd() is module.get_usd()


def test_get_scene_without_mounts_is_just_the_robot(
    make_manager: Callable[..., UrdfManager],
) -> None:
    # The empty default is what keeps every existing blueprint working untouched.
    pytest.importorskip("pxr")
    module = make_manager()

    assert _scene(module) == module.get_usd()


def test_get_scene_grafts_each_subtree_onto_the_frame_it_is_bolted_to(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    module._subtree_clients[_FakeArm] = cast("Any", _FakeClient(_SUBTREE_USD))

    stage = _usd_stage(_scene(module))

    assert stage.GetPrimAtPath("/box_robot/base/arm/fake_arm")


def test_get_scene_keeps_asking_a_module_that_is_not_serving_yet(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm}, scene_retry_s=0.01)
    client = _FakeClient(RuntimeError("no reply"), None, _SUBTREE_USD)
    module._subtree_clients[_FakeArm] = cast("Any", client)

    stage = _usd_stage(_scene(module))

    assert client.asked == 3
    assert stage.GetPrimAtPath("/box_robot/base/arm/fake_arm")


def test_get_scene_does_not_return_half_a_robot_while_a_mount_is_silent(
    make_manager: Callable[..., UrdfManager],
) -> None:
    """Half a scene would hand a renderer a robot with no arm, so there is no partial answer.

    There is no deadline and so no error to assert; assert instead that it is still waiting.
    wait_for cancels it, which is also what keeps this test from leaking the retry loop.
    """
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm}, scene_retry_s=0.01)
    client = _FakeClient(RuntimeError("never"))
    module._subtree_clients[_FakeArm] = cast("Any", client)

    async def wait_a_moment() -> None:
        coroutine = cast("Any", UrdfManager.get_scene).aio.__get__(module, UrdfManager)
        with pytest.raises(asyncio.TimeoutError):
            await asyncio.wait_for(coroutine(), 0.2)

    asyncio.run(wait_a_moment())

    assert client.asked > 1, "it should have kept asking rather than giving up or answering"


def test_mounts_at_a_frame_the_urdf_does_not_have_is_rejected(
    make_manager: Callable[..., UrdfManager],
) -> None:
    with pytest.raises(ValueError, match="not a link of this URDF"):
        make_manager(mounts={"elbow": _FakeArm})


def test_mounts_to_a_module_that_cannot_be_asked_is_rejected(
    make_manager: Callable[..., UrdfManager],
) -> None:
    with pytest.raises(ValueError, match="no @rpc get_usd"):
        make_manager(mounts={"arm": _NoUsd})


def test_render_file_request_serves_the_whole_scene_once_one_is_assembled(
    make_manager: Callable[..., UrdfManager],
) -> None:
    """A renderer wants the tree it is drawing, not just the robot at the root of it."""
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    module._subtree_clients[_FakeArm] = cast("Any", _FakeClient(_SUBTREE_USD))
    sent: list[File] = []
    module.file_info.publish = sent.append  # type: ignore[method-assign]

    asyncio.run(cast("Any", UrdfManager.get_scene).aio.__get__(module, UrdfManager)())
    asyncio.run(module.handle_render_file_request(String("scene.usd")))

    assert "fake_arm" in sent[0].data.decode()


def test_the_usd_layer_is_built_off_the_event_loop_thread(
    make_manager: Callable[..., UrdfManager],
) -> None:
    """Building it imports usd-core and converts the URDF: ~150 ms on a real robot.

    That has to happen on a worker thread, or a renderer's first request stalls tf and
    every other handler for as long as it takes.
    """
    pytest.importorskip("pxr")
    module = make_manager()
    loop_thread: list[int] = []
    built_on: list[int] = []
    real = UrdfManager.get_usd

    def record(self: UrdfManager) -> str:
        built_on.append(threading.get_ident())
        return real(self)

    async def ask() -> None:
        loop_thread.append(threading.get_ident())
        await module.handle_render_file_request(String("scene.usda"))

    module.file_info.publish = lambda *_: None  # type: ignore[method-assign]
    with patch.object(UrdfManager, "get_usd", record):
        asyncio.run(ask())

    assert built_on, "get_usd was never called"
    assert built_on[0] != loop_thread[0]
