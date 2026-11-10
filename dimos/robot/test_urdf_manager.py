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
from pytest_mock import MockerFixture
import trimesh

from dimos.core.coordination.blueprints import autoconnect
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
from dimos.robot.assets.model import RobotModel, UsdModel
from dimos.robot.assets.usd import usd_frame_names
from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL
from dimos.robot.manipulators.xarm.config import (
    XARM_ROS2_REF,
    XARM_ROS2_REPO,
    make_xarm7_model_config,
)
from dimos.robot.roboplan_body_filter import RoboPlanBodyFilter
import dimos.robot.urdf_manager as urdf_manager_module
from dimos.robot.urdf_manager import (
    USD,
    CircularMountError,
    Mount,
    UrdfManager,
    _is_circular_mount,
)
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing

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

_TWO_ARM_URDF = """<?xml version="1.0"?>
<robot name="box_robot">
  <link name="base"/>
  <link name="left_mount"/>
  <link name="right_mount"/>
  <joint name="left_shoulder" type="fixed">
    <parent link="base"/><child link="left_mount"/>
    <origin xyz="0 0.3 0"/>
  </joint>
  <joint name="right_shoulder" type="fixed">
    <parent link="base"/><child link="right_mount"/>
    <origin xyz="0 -0.3 0"/>
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
    first = module.filter_cloud(captures[0])
    assert first is not None
    second = module.filter_cloud(captures[1])
    assert second is not None
    for cloud, filtered in zip(captures, (first, second), strict=True):
        np.testing.assert_allclose(filtered.points_f32(), controls)
        assert filtered.frame_id == "world" and filtered.ts == cloud.ts


def test_a_description_with_no_joint_state_source_still_offers_its_fixed_edges(make_filter):
    """The office scene has no controller, so nothing ever calls add_joint_state for it."""
    module = make_filter()

    edges = module.static_transforms()

    assert [(e.frame_id, e.child_frame_id) for e in edges] == [("base", "arm")]
    assert module.tfbuffer.get("base", "arm", time_point=1.0, time_tolerance=0.001) is None, (
        "nothing has been published yet; the edge is only offered"
    )


def test_a_moving_joint_is_left_out_of_the_fixed_edges(make_filter):
    module = make_filter(urdf_xml=_joint_robot("prismatic"))

    assert module.static_transforms() == [], "a prismatic joint needs a joint state"


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
    config: ModuleConfig

    @rpc
    def get_usd(self) -> list[str]:
        return ["", _SUBTREE_USD]


class _FakeG1(Module):
    config: ModuleConfig

    @rpc
    def get_scene(self, visited: list[str] | None = None) -> list[str]:
        return ["", _G1_WITH_HEAD_USD]


class _NoUsd(Module):
    config: ModuleConfig


def _subtree_usd(prim: str, link: str, mesh: str | None = None) -> str:
    geometry = (
        ""
        if mesh is None
        else f"""
    def Mesh "visual_0" (
        assetInfo = {{
            asset identifier = @{mesh}@
            string version = "sha256:0000000000000000000000000000000000000000000000000000000000000000"
        }}
        prepend references = @{mesh}@
    )
    {{
    }}
"""
    )
    return f"""#usda 1.0
(
    defaultPrim = "{prim}"
)

def Xform "{prim}" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    displayName = "{link}"
)
{{
{geometry}}}
"""


_SUBTREE_USD = _subtree_usd("fake_arm", "fake_arm_base")

_G1_WITH_HEAD_USD = """#usda 1.0
(
    defaultPrim = "g1"
)

def Xform "g1" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    displayName = "g1/pelvis"
)
{
    def Xform "head_mount" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
        displayName = "g1/head_mount"
    )
    {
    }
}
"""


class _FakeClient:
    def __init__(self, *answers: Any, asset_root: str = "") -> None:
        self.answers = list(answers)
        self.asked = 0
        self.asset_root = asset_root
        self.chains: list[list[str]] = []

    def get_usd(self) -> Any:
        self.asked += 1
        answer = self.answers.pop(0) if len(self.answers) > 1 else self.answers[0]
        if isinstance(answer, Exception):
            raise answer
        return [self.asset_root, answer] if isinstance(answer, str) else answer

    def get_scene(self, visited: list[str] | None = None) -> Any:
        self.chains.append(list(visited or []))
        return self.get_usd()

    def stop_rpc_client(self) -> None: ...


@pytest.fixture
def make_manager(tmp_path: Path) -> Iterator[Callable[..., UrdfManager]]:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(_URDF)
    modules: list[UrdfManager] = []

    def make(**overrides: Any) -> UrdfManager:
        urdf.write_text(overrides.pop("urdf_xml", _URDF))
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


def _link_of(stage: Any, path: str) -> str:
    prim = stage.GetPrimAtPath(path)
    assert prim, f"no prim at {path}"
    return str(prim.GetDisplayName() or prim.GetName())


def _asset(module: UrdfManager, name: str) -> bytes:
    owner = type(module)
    coroutine = cast("Any", owner.get_asset).aio.__get__(module, owner)
    return cast("bytes", asyncio.run(coroutine(name)))


def _scene(module: UrdfManager) -> str:
    return _scene_answer(module)[1]


def _scene_answer(module: UrdfManager) -> list[str]:
    coroutine = cast("Any", UrdfManager.get_scene).aio.__get__(module, UrdfManager)
    return cast("list[str]", asyncio.run(coroutine()))


def test_get_usd_is_this_robots_own_urdf_with_nothing_attached(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager()

    stage = _usd_stage(module.get_usd()[1])

    assert stage.GetDefaultPrim().GetName() == "box_robot"
    assert stage.GetPrimAtPath("/box_robot/base/arm")


def test_get_usd_is_built_once_and_reused(make_manager: Callable[..., UrdfManager]) -> None:
    pytest.importorskip("pxr")
    module = make_manager()

    assert module.get_usd()[1] is module.get_usd()[1]


def test_get_scene_without_mounts_is_just_the_robot(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager()

    assert _scene(module) == module.get_usd()[USD]


def test_get_scene_grafts_each_subtree_onto_the_frame_it_is_bolted_to(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))

    stage = _usd_stage(_scene(module))

    assert stage.GetPrimAtPath("/box_robot/base/arm/fake_arm")


def test_get_scene_keeps_asking_a_module_that_is_not_serving_yet(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm}, scene_retry_s=0.01)
    client = _FakeClient(RuntimeError("no reply"), None, _SUBTREE_USD)
    module._subtree_clients["_FakeArm"] = cast("Any", client)

    stage = _usd_stage(_scene(module))

    assert client.asked == 3
    assert stage.GetPrimAtPath("/box_robot/base/arm/fake_arm")


def test_get_scene_does_not_return_half_a_robot_while_a_mount_is_silent(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm}, scene_retry_s=0.01)
    client = _FakeClient(RuntimeError("never"))
    module._subtree_clients["_FakeArm"] = cast("Any", client)

    async def wait_a_moment() -> None:
        coroutine = cast("Any", UrdfManager.get_scene).aio.__get__(module, UrdfManager)
        with pytest.raises(asyncio.TimeoutError):
            await asyncio.wait_for(coroutine(), 0.2)

    asyncio.run(wait_a_moment())

    assert client.asked > 1, "it should have kept asking rather than giving up or answering"


def test_a_mount_frame_is_not_checked_until_the_scene_is_assembled(
    make_manager: Callable[..., UrdfManager],
) -> None:
    make_manager(mounts={"elbow": _FakeArm}).dispose()


def test_mounts_to_a_module_that_cannot_be_asked_is_rejected(
    make_manager: Callable[..., UrdfManager],
) -> None:
    with pytest.raises(ValueError, match="neither an @rpc get_usd"):
        make_manager(mounts={"arm": _NoUsd})


def test_a_module_class_names_the_only_instance_of_itself() -> None:
    mount = Mount.of(_FakeArm)

    assert mount.module is _FakeArm
    assert mount.remote_name == "_FakeArm"


def test_a_namespaced_blueprint_names_the_instance_it_deploys() -> None:
    mount = Mount.of(_FakeArm.blueprint().namespace("left"))

    assert mount.module is _FakeArm
    assert mount.remote_name == "left/_fakearm"


def test_two_namespaced_copies_of_one_class_are_two_different_mounts() -> None:
    left = Mount.of(_FakeArm.blueprint().namespace("left"))
    right = Mount.of(_FakeArm.blueprint().namespace("right"))

    assert left.remote_name != right.remote_name


def test_a_blueprint_the_config_parser_flattened_still_names_its_instance() -> None:
    from dimos.core.coordination.blueprint_config.values import plain

    flattened = plain(_FakeArm.blueprint(frame_id_prefix="left").namespace("left"))
    assert isinstance(flattened, dict), "the parser turns a pinned blueprint into a dict"

    mount = Mount.of(flattened)

    assert mount.module is _FakeArm
    assert mount.remote_name == "left/_fakearm"
    assert mount.frame_prefix == "left"


def test_two_flattened_copies_of_one_class_are_still_two_mounts() -> None:
    from dimos.core.coordination.blueprint_config.values import plain

    left = Mount.of(plain(_FakeArm.blueprint().namespace("left")))
    right = Mount.of(plain(_FakeArm.blueprint().namespace("right")))

    assert left.remote_name != right.remote_name


def test_a_flattened_blueprint_with_nothing_to_ask_is_rejected() -> None:
    from dimos.core.coordination.blueprint_config.values import plain

    with pytest.raises(ValueError, match="0 answer an @rpc get_usd"):
        Mount.of(plain(_NoUsd.blueprint()))


def test_an_instance_name_on_its_own_is_accepted_with_no_class_to_check() -> None:
    mount = Mount.of("left/_fakearm")

    assert mount.module is None
    assert mount.remote_name == "left/_fakearm"


def test_a_blueprint_with_nothing_to_ask_for_a_subtree_is_rejected() -> None:
    with pytest.raises(ValueError, match="none of which has an @rpc get_usd"):
        Mount.of(_NoUsd.blueprint())


def test_a_blueprint_with_two_modules_answering_get_usd_is_rejected() -> None:
    blueprint = autoconnect(
        _FakeArm.blueprint().namespace("left"), _FakeArm.blueprint().namespace("right")
    )

    with pytest.raises(ValueError, match="more than one module answering get_usd"):
        Mount.of(blueprint)


def test_a_value_that_is_neither_a_module_a_blueprint_nor_a_name_is_rejected() -> None:
    with pytest.raises(TypeError, match="a Module subclass, a Blueprint or an instance name"):
        Mount.of(42)


def test_each_namespaced_arm_is_asked_under_its_own_name(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(
        urdf_xml=_TWO_ARM_URDF,
        mounts={
            "left_mount": _FakeArm.blueprint().namespace("left"),
            "right_mount": _FakeArm.blueprint().namespace("right"),
        },
    )
    clients = {
        "left/_fakearm": _FakeClient(_SUBTREE_USD),
        "right/_fakearm": _FakeClient(_subtree_usd("other_arm", "other_arm_base")),
    }
    module._subtree_clients.update(cast("dict[str, Any]", clients))

    stage = _usd_stage(_scene(module))

    assert clients["left/_fakearm"].asked == 1
    assert clients["right/_fakearm"].asked == 1
    assert stage.GetPrimAtPath("/box_robot/base/left_mount/fake_arm")
    assert stage.GetPrimAtPath("/box_robot/base/right_mount/other_arm")


def test_a_bare_class_has_no_prefix_to_move_its_frames_under() -> None:
    assert Mount.of(_FakeArm).frame_prefix == ""


def test_a_namespaced_blueprint_carries_the_prefix_namespace_gave_it() -> None:
    mount = Mount.of(_FakeArm.blueprint().namespace("left"))

    assert mount.frame_prefix == "left"


def test_a_nested_namespace_carries_the_whole_prefix() -> None:
    blueprint = _FakeArm.blueprint().namespace("robot0").namespace("fleet")

    assert Mount.of(blueprint).frame_prefix == "fleet/robot0"


def test_a_hand_set_frame_id_prefix_wins_over_the_namespace() -> None:
    blueprint = _FakeArm.blueprint(frame_id_prefix="custom").namespace("left")

    mount = Mount.of(blueprint)

    assert mount.remote_name == "left/_fakearm"
    assert mount.frame_prefix == "custom"


@pytest.mark.parametrize(
    ("remote_name", "prefix"),
    [("left/xarm", "left"), ("fleet/robot0/xarm", "fleet/robot0"), ("xarm", "")],
)
def test_an_instance_name_falls_back_to_the_namespace_in_the_name(
    remote_name: str, prefix: str
) -> None:
    assert Mount.of(remote_name).frame_prefix == prefix


def test_each_namespaced_arm_is_asked_under_its_own_name_and_renamed_to_it(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(
        urdf_xml=_TWO_ARM_URDF,
        mounts={
            "left_mount": _FakeArm.blueprint().namespace("left"),
            "right_mount": _FakeArm.blueprint().namespace("right"),
        },
    )
    clients = {
        "left/_fakearm": _FakeClient(_SUBTREE_USD),
        "right/_fakearm": _FakeClient(_SUBTREE_USD),
    }
    module._subtree_clients.update(cast("dict[str, Any]", clients))

    stage = _usd_stage(_scene(module))

    assert clients["left/_fakearm"].asked == 1
    assert clients["right/_fakearm"].asked == 1
    assert _link_of(stage, "/box_robot/base/left_mount/fake_arm") == "left/fake_arm_base"
    assert _link_of(stage, "/box_robot/base/right_mount/fake_arm") == "right/fake_arm_base"


def test_an_un_namespaced_subtree_keeps_the_link_names_its_description_gave_it(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))

    stage = _usd_stage(_scene(module))

    assert _link_of(stage, "/box_robot/base/arm/fake_arm") == "fake_arm_base"


def test_the_roots_own_links_are_never_renamed(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(
        urdf_xml=_TWO_ARM_URDF,
        mounts={"left_mount": _FakeArm.blueprint().namespace("left")},
    )
    module._subtree_clients["left/_fakearm"] = cast("Any", _FakeClient(_SUBTREE_USD))

    stage = _usd_stage(_scene(module))

    assert _link_of(stage, "/box_robot/base") == "base"
    assert _link_of(stage, "/box_robot/base/left_mount") == "left_mount"


def test_get_usd_reports_the_directory_its_asset_paths_resolve_against(
    make_manager: Callable[..., UrdfManager], tmp_path: Path
) -> None:
    pytest.importorskip("pxr")
    module = make_manager()

    assert module.get_usd()[0] == str(tmp_path)


def test_a_scene_naming_an_asset_that_cannot_be_opened_says_so_once(
    make_manager: Callable[..., UrdfManager],
    mocker: MockerFixture,
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    subtree = _subtree_usd("fake_arm", "fake_arm_base", mesh="meshes/arm.stl")
    module._subtree_clients["_FakeArm"] = cast(
        "Any", _FakeClient(subtree, asset_root="/opt/arm/desc")
    )
    warn = mocker.patch.object(urdf_manager_module.logger, "warning")

    _scene(module)

    said = [call.args[0] for call in warn.call_args_list if "cannot open" in call.args[0]]
    assert len(said) == 1
    assert "/opt/arm/desc/meshes/arm.stl" in said[0]


def test_a_scene_whose_mesh_is_an_unfetched_pointer_is_refused(
    make_manager: Callable[..., UrdfManager],
    tmp_path: Path,
) -> None:
    pytest.importorskip("pxr")
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "arm.stl").write_bytes(
        b"version https://git-lfs.github.com/spec/v1\noid sha256:0\nsize 1\n"
    )
    module = make_manager(mounts={"arm": _FakeArm})
    subtree = _subtree_usd("fake_arm", "fake_arm_base", mesh="meshes/arm.stl")
    module._subtree_clients["_FakeArm"] = cast(
        "Any", _FakeClient(subtree, asset_root=str(tmp_path))
    )

    with pytest.raises(FileNotFoundError) as error:
        _scene(module)

    assert "arm.stl" in str(error.value)
    assert "git lfs checkout" in str(error.value)


def test_a_subtree_mesh_is_rebased_onto_its_module_root_then_published(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    subtree = _subtree_usd("fake_arm", "fake_arm_base", mesh="meshes/arm.stl")
    module._subtree_clients["_FakeArm"] = cast(
        "Any", _FakeClient(subtree, asset_root="/opt/arm/desc")
    )

    scene = _scene(module)

    assert "@meshes/arm.stl@" not in scene
    assert "@/opt/arm/desc/meshes/arm.stl@" not in scene, "a real path must not leave"
    assert list(module._published_assets.values()) == ["/opt/arm/desc/meshes/arm.stl"]
    assert next(iter(module._published_assets)) in scene


def test_a_subtree_texture_is_rebased_the_same_as_its_mesh(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    subtree = _subtree_usd("fake_arm", "fake_arm_base", mesh="meshes/arm.stl")
    shader = (
        '    def Shader "skin" (\n'
        "        assetInfo = {\n"
        f'            string version = "sha256:{"0" * 64}"\n'
        "        }\n"
        "    )\n"
        "    {\n"
        '        uniform token info:id = "UsdUVTexture"\n'
        "        asset inputs:file = @textures/skin.png@\n"
        "    }\n"
    )
    subtree = subtree[: subtree.rindex("}")] + shader + "}\n"
    module._subtree_clients["_FakeArm"] = cast(
        "Any", _FakeClient(subtree, asset_root="/opt/arm/desc")
    )

    _scene(module)

    assert sorted(module._published_assets.values()) == [
        "/opt/arm/desc/meshes/arm.stl",
        "/opt/arm/desc/textures/skin.png",
    ]


def test_a_published_asset_is_named_from_where_it_first_appears(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    subtree = _subtree_usd("fake_arm", "fake_arm_base", mesh="meshes/arm.stl")
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(subtree, asset_root="/opt"))

    _scene(module)

    published = next(iter(module._published_assets))
    assert published.startswith("/dimos/assets/")
    assert published.endswith("arm.stl")


def test_get_asset_serves_a_published_mesh_and_nothing_else(
    make_manager: Callable[..., UrdfManager], tmp_path: Path
) -> None:
    pytest.importorskip("pxr")
    mesh = tmp_path / "meshes" / "arm.stl"
    mesh.parent.mkdir()
    mesh.write_bytes(b"solid arm\n")
    module = make_manager(mounts={"arm": _FakeArm})
    subtree = _subtree_usd("fake_arm", "fake_arm_base", mesh="meshes/arm.stl")
    module._subtree_clients["_FakeArm"] = cast(
        "Any", _FakeClient(subtree, asset_root=str(tmp_path))
    )

    _scene(module)
    published = next(iter(module._published_assets))

    assert _asset(module, published) == b"solid arm\n"
    with pytest.raises(KeyError, match="neither a file this description names"):
        _asset(module, "../../../etc/passwd")


def test_one_mesh_on_two_mounts_is_published_once(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(
        urdf_xml=_TWO_ARM_URDF,
        mounts={
            "left_mount": _FakeArm.blueprint().namespace("left"),
            "right_mount": _FakeArm.blueprint().namespace("right"),
        },
    )
    subtree = _subtree_usd("fake_arm", "fake_arm_base", mesh="meshes/arm.stl")
    module._subtree_clients.update(
        {
            "left/_fakearm": cast("Any", _FakeClient(subtree, asset_root="/opt/arm")),
            "right/_fakearm": cast("Any", _FakeClient(subtree, asset_root="/opt/arm")),
        }
    )

    _scene(module)

    assert len(module._published_assets) == 1


def test_a_subtree_with_no_root_keeps_its_paths_untouched(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    subtree = _subtree_usd("fake_arm", "fake_arm_base", mesh="/already/abs.stl")
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(subtree))

    _scene(module)

    assert list(module._published_assets.values()) == ["/already/abs.stl"]


@pytest.mark.parametrize(
    "answer", [[], ["only-a-root"], ["root", "usd", "extra"], ["root", ""], [1, 2]]
)
def test_an_answer_that_is_not_a_root_and_a_usd_is_retried(
    make_manager: Callable[..., UrdfManager], answer: Any
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm}, scene_retry_s=0.01)
    client = _FakeClient(answer, _SUBTREE_USD)
    module._subtree_clients["_FakeArm"] = cast("Any", client)

    stage = _usd_stage(_scene(module))

    assert client.asked == 2, "the malformed answer should have been asked past"
    assert stage.GetPrimAtPath("/box_robot/base/arm/fake_arm")


def _namespaced_arm(make_manager: Callable[..., UrdfManager], **overrides: Any) -> UrdfManager:
    module = make_manager(mounts={"arm": _FakeArm.blueprint().namespace("left")}, **overrides)
    module._subtree_clients["left/_fakearm"] = cast("Any", _FakeClient(_SUBTREE_USD))
    return module


def test_frames_are_not_renamed_by_default(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = _namespaced_arm(make_manager)

    stage = _usd_stage(_scene(module))

    assert _link_of(stage, "/box_robot/base/arm/fake_arm") == "left/fake_arm_base"


def test_a_pre_namespace_rename_happens_before_the_prefix_goes_on(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = _namespaced_arm(make_manager, tf_renames_pre_namespace={"fake_arm_base": "arm_base"})

    stage = _usd_stage(_scene(module))

    assert _link_of(stage, "/box_robot/base/arm/fake_arm") == "left/arm_base"


def test_a_post_namespace_rename_happens_after_the_prefix_goes_on(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = _namespaced_arm(make_manager, tf_renames={"left/fake_arm_base": "left_arm"})

    stage = _usd_stage(_scene(module))

    assert _link_of(stage, "/box_robot/base/arm/fake_arm") == "left_arm"


def test_both_renames_apply_around_the_prefix(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = _namespaced_arm(
        make_manager,
        tf_renames_pre_namespace={"fake_arm_base": "arm_base"},
        tf_renames={"left/arm_base": "the_left_arm"},
    )

    stage = _usd_stage(_scene(module))

    assert _link_of(stage, "/box_robot/base/arm/fake_arm") == "the_left_arm"


def test_a_rename_meant_for_one_subtree_does_not_upset_the_others(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(
        urdf_xml=_TWO_ARM_URDF,
        mounts={
            "left_mount": _FakeArm.blueprint().namespace("left"),
            "right_mount": _FakeArm.blueprint().namespace("right"),
        },
        tf_renames={"left/fake_arm_base": "the_left_one"},
    )
    module._subtree_clients.update(
        {
            "left/_fakearm": cast("Any", _FakeClient(_SUBTREE_USD)),
            "right/_fakearm": cast("Any", _FakeClient(_SUBTREE_USD)),
        }
    )

    stage = _usd_stage(_scene(module))

    assert _link_of(stage, "/box_robot/base/left_mount/fake_arm") == "the_left_one"
    assert _link_of(stage, "/box_robot/base/right_mount/fake_arm") == "right/fake_arm_base"


def test_a_rename_matching_no_subtree_at_all_fails_the_scene(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = _namespaced_arm(make_manager, tf_renames_pre_namespace={"elbow": "wrist"})

    with pytest.raises(ValueError, match="nor of any mounted subtree"):
        _scene(module)


def test_a_lidar_mounts_onto_a_g1_that_is_itself_mounted_on_the_root(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(
        mounts={"arm": _FakeG1, "g1/head_mount": _FakeArm},
    )
    module._subtree_clients["_FakeG1"] = cast("Any", _FakeClient(_G1_WITH_HEAD_USD))
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))

    stage = _usd_stage(_scene(module))

    assert stage.GetPrimAtPath("/box_robot/base/arm/g1")
    assert stage.GetPrimAtPath("/box_robot/base/arm/g1/head_mount/fake_arm")


def test_a_mounted_manager_is_asked_for_its_whole_scene_not_just_its_own_layer(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")

    assert Mount.of(_FakeG1).subtree_rpc == "get_scene"
    assert Mount.of(_FakeArm).subtree_rpc == "get_usd"


def test_an_instance_name_on_its_own_is_asked_for_its_own_layer() -> None:
    assert Mount.of("left/xarm").subtree_rpc == "get_usd"


def test_mounting_at_a_frame_nothing_will_ever_provide_lists_every_frame_there_is(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm, "nowhere/at_all": _FakeArm})
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))

    with pytest.raises(ValueError) as caught:
        _scene(module)

    message = str(caught.value)
    assert "nowhere/at_all" in message
    assert "g1/head_mount" in message or "namespace prefix" in message
    assert "'base'" in message and "'arm'" in message and "'fake_arm_base'" in message


def test_a_frame_that_only_a_later_wave_provides_is_not_reported_as_missing(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"g1/head_mount": _FakeArm, "arm": _FakeG1})
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))
    module._subtree_clients["_FakeG1"] = cast("Any", _FakeClient(_G1_WITH_HEAD_USD))

    stage = _usd_stage(_scene(module))

    assert stage.GetPrimAtPath("/box_robot/base/arm/g1/head_mount/fake_arm")


def test_a_circular_mount_is_refused_with_the_whole_chain(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeG1})

    async def ask_as_if_from_itself() -> None:
        coroutine = cast("Any", UrdfManager.get_scene).aio.__get__(module, UrdfManager)
        with pytest.raises(ValueError, match="circular mount"):
            await coroutine(["UrdfManager", "SomeOther"])

    asyncio.run(ask_as_if_from_itself())


def test_the_circular_mount_error_names_every_manager_in_the_loop(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager()

    async def ask() -> None:
        coroutine = cast("Any", UrdfManager.get_scene).aio.__get__(module, UrdfManager)
        with pytest.raises(ValueError) as caught:
            await coroutine(["A", "UrdfManager", "B"])
        assert "A -> UrdfManager -> B -> UrdfManager" in str(caught.value)

    asyncio.run(ask())


def test_the_mount_chain_travels_to_a_mounted_manager(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeG1})
    client = _FakeClient(_G1_WITH_HEAD_USD)
    module._subtree_clients["_FakeG1"] = cast("Any", client)

    _scene(module)

    assert client.chains == [["UrdfManager"]]


def test_a_manager_with_no_model_stands_on_a_world_link() -> None:
    pytest.importorskip("pxr")
    module = UrdfManager()
    try:
        stage = _usd_stage(module.get_usd()[1])

        assert module._description.root_link == "world"
        assert stage.GetDefaultPrim().GetName() == "world"
    finally:
        module.dispose()


def test_a_world_manager_has_no_asset_root_to_offer() -> None:
    pytest.importorskip("pxr")
    module = UrdfManager()
    try:
        assert module.get_usd()[0] == ""
    finally:
        module.dispose()


def test_a_world_manager_still_publishes_no_joint_tf() -> None:
    module = UrdfManager()
    try:
        assert module.joint_transforms(JointState()) == []
    finally:
        module.dispose()


def test_mounts_hang_off_the_world_link_of_a_manager_with_no_model(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = UrdfManager(mounts={"world": _FakeG1, "g1/head_mount": _FakeArm})
    try:
        module._subtree_clients["_FakeG1"] = cast("Any", _FakeClient(_G1_WITH_HEAD_USD))
        module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))

        stage = _usd_stage(_scene(module))

        assert stage.GetPrimAtPath("/world/world/g1/head_mount/fake_arm")
    finally:
        module.dispose()


def test_tf_renames_reach_this_robots_own_frames_on_tf(
    make_manager: Callable[..., UrdfManager],
) -> None:
    module = make_manager(tf_renames={"base": "world", "arm": "gripper"})

    (transform,) = module.joint_transforms(JointState(ts=1.0, name=["shoulder"], position=[0.0]))

    assert (transform.frame_id, transform.child_frame_id) == ("world", "gripper")


def test_tf_renames_reach_this_robots_own_frames_in_its_usd(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(tf_renames={"arm": "gripper"})

    stage = _usd_stage(module.get_usd()[USD])

    assert _link_of(stage, "/box_robot/base/arm") == "gripper"


def test_a_pre_namespace_rename_also_reaches_this_robots_own_frames(
    make_manager: Callable[..., UrdfManager],
) -> None:
    module = make_manager(
        tf_renames_pre_namespace={"arm": "forearm"}, tf_renames={"forearm": "gripper"}
    )

    (transform,) = module.joint_transforms(JointState(ts=1.0, name=["shoulder"], position=[0.0]))

    assert transform.child_frame_id == "gripper"


def test_renaming_a_joint_leaves_the_name_its_joint_state_arrives_under(
    make_manager: Callable[..., UrdfManager],
) -> None:
    module = make_manager(tf_renames={"shoulder": "elbow"}, urdf_xml=_joint_robot("revolute"))

    (transform,) = module.joint_transforms(
        JointState(ts=1.0, name=["shoulder"], position=[np.pi / 2])
    )

    np.testing.assert_allclose(
        transform.rotation.to_tuple(), [np.sin(np.pi / 4), 0.0, 0.0, np.cos(np.pi / 4)], atol=1e-9
    )


def test_a_rename_of_this_robots_own_frame_needs_no_mounts(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(tf_renames={"arm": "gripper"})

    assert _usd_stage(_scene(module)).GetPrimAtPath("/box_robot/base/arm")


def test_a_rename_matching_neither_this_robot_nor_a_subtree_fails_at_construction(
    make_manager: Callable[..., UrdfManager],
) -> None:
    with pytest.raises(ValueError, match="which is neither a link or joint of this robot"):
        make_manager(tf_renames={"nothing": "here"})


def test_a_tf_lookup_takes_description_names_and_finds_the_published_ones(
    make_filter: Callable[..., UrdfManagerPointClearing],
) -> None:
    module = make_filter(tf_renames={"base": "chassis", "arm": "forearm"})
    module.tfbuffer.receive_transform(
        Transform(
            translation=Vector3(1.0, 0.0, 0.0),
            frame_id="chassis",
            child_frame_id="forearm",
            ts=1.0,
        )
    )

    found = module._tf_at("base", "arm", 1.0)

    assert found is not None
    np.testing.assert_allclose(found.translation.to_tuple(), [1.0, 0.0, 0.0], atol=1e-9)


def test_a_circular_mount_is_recognised_across_a_process_boundary() -> None:
    from dimos.protocol.rpc.rpc_utils import deserialize_exception, serialize_exception

    original = CircularMountError("circular mount: a -> b -> a")

    arrived = deserialize_exception(serialize_exception(original))

    assert not isinstance(arrived, CircularMountError)
    assert _is_circular_mount(arrived)


def test_an_ordinary_rpc_failure_is_not_mistaken_for_a_cycle() -> None:
    assert not _is_circular_mount(RuntimeError("circular mount is only a phrase in here"))


def test_render_file_request_serves_the_whole_scene_once_one_is_assembled(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))
    sent: list[File] = []
    module.file_info.publish = sent.append  # type: ignore[method-assign]
    _scene(module)

    asyncio.run(module.handle_render_file_request(String("scene.usd")))

    assert "fake_arm" in sent[0].data.decode()


def test_the_published_scene_carries_no_asset_root_to_resolve_against(
    make_manager: Callable[..., UrdfManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))

    assert _scene_answer(module)[0] == ""


def test_the_body_mask_still_cuts_the_robot_out_when_its_frames_are_renamed(
    make_filter: Callable[..., UrdfManagerPointClearing],
) -> None:
    module = make_filter(tf_renames={"base": "chassis", "arm": "forearm"})
    for parent in ("camera", "world"):
        module.tfbuffer.receive_transform(
            Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                frame_id=parent,
                child_frame_id="chassis",
                ts=1.0,
            )
        )

    result = module.filter_cloud(_cloud([[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [2.0, 0.0, 0.0]]))

    assert result is not None
    np.testing.assert_allclose(result.points_f32(), [[2.0, 0.0, 0.0]], atol=1e-6)


def test_a_renamed_multidof_joint_is_still_found_on_tf(
    make_filter: Callable[..., UrdfManagerPointClearing],
) -> None:
    urdf = _joint_robot("floating")
    module = make_filter(urdf_xml=urdf, tf_renames={"base": "chassis", "arm": "forearm"})
    for parent in ("camera", "world"):
        module.tfbuffer.receive_transform(
            Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                frame_id=parent,
                child_frame_id="chassis",
                ts=1.0,
            )
        )
    module.tfbuffer.receive_transform(
        Transform(
            translation=Vector3(1.0, 0.0, 0.0),
            frame_id="chassis",
            child_frame_id="forearm",
            ts=1.0,
        )
    )

    result = module.filter_cloud(_cloud([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]))

    assert result is not None
    np.testing.assert_allclose(result.points_f32(), [[2.0, 0.0, 0.0]], atol=1e-6)


_SCENE_USDA = """#usda 1.0
(
    defaultPrim = "room"
)

def Xform "room" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
)
{
    def Xform "table" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    )
    {
        def Mesh "visual_0" (
            assetInfo = {
                asset identifier = @meshes/table.stl@
                string version = "sha256:0000000000000000000000000000000000000000000000000000000000000000"
            }
        )
        {
        }
    }
}
"""


@pytest.fixture
def scene_file(tmp_path: Path) -> Path:
    path = tmp_path / "room.usda"
    path.write_text(_SCENE_USDA)
    return path


def test_a_usd_model_is_served_as_it_is(scene_file: Path) -> None:
    pytest.importorskip("pxr")
    module = UrdfManager(model=UsdModel.from_file(scene_file))
    try:
        root, usd = module.get_usd()

        assert usd == _SCENE_USDA
        assert root == str(scene_file.parent)
    finally:
        module.dispose()


def test_a_usd_model_serves_the_files_its_layer_names(scene_file: Path) -> None:
    pytest.importorskip("pxr")
    (scene_file.parent / "meshes").mkdir()
    (scene_file.parent / "meshes" / "table.stl").write_bytes(b"stl bytes")
    module = UrdfManager(model=UsdModel.from_file(scene_file))
    try:
        sent: list[File] = []
        module.file_info.publish = sent.append  # type: ignore[method-assign]

        asyncio.run(module.handle_render_file_request(String("meshes/table.stl")))

        assert [file.data for file in sent] == [b"stl bytes"]
    finally:
        module.dispose()


def test_a_usd_model_poses_its_own_frames_on_tf(scene_file: Path) -> None:
    pytest.importorskip("pxr")
    module = UrdfManager(model=UsdModel.from_file(scene_file))
    try:
        posed = module.joint_transforms(JointState(ts=1.0, name=["shoulder"], position=[1.0]))

        assert [(t.frame_id, t.child_frame_id) for t in posed] == [("room", "table")]
    finally:
        module.dispose()


def test_a_usd_model_is_renamed(scene_file: Path) -> None:
    pytest.importorskip("pxr")
    module = UrdfManager(model=UsdModel.from_file(scene_file), tf_renames={"table": "desk"})
    try:
        usd = module.get_usd()[USD]

        assert "desk" in usd_frame_names(usd)
    finally:
        module.dispose()


def test_point_clearing_converts_a_usd_model_back_to_urdf(scene_file: Path) -> None:
    pytest.importorskip("pxr")
    module = UrdfManagerPointClearing(model=UsdModel.from_file(scene_file))
    try:
        description = module._filter_description()

        assert "<robot" in description.xml
        assert 'name="room"' in description.xml
    finally:
        module.dispose()


def test_point_clearing_hands_over_the_urdf_it_already_has(tmp_path: Path) -> None:
    source = tmp_path / "robot.urdf"
    source.write_text(_URDF)
    module = UrdfManagerPointClearing(model=RobotModel.from_file(source))
    try:
        assert module._filter_description().xml == RobotModel.from_file(source).load().xml
    finally:
        module.dispose()


def test_a_usd_model_wants_a_usd_file(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="RobotModel.from_file"):
        UsdModel.from_file(tmp_path / "robot.urdf")


def test_a_manager_needs_no_usd_until_a_layer_is_asked_for(
    make_manager: Callable[..., UrdfManager], monkeypatch: pytest.MonkeyPatch
) -> None:
    def refuse(*args: Any, **kwargs: Any) -> str:
        raise AssertionError("the layer was built before anything asked for it")

    monkeypatch.setattr("dimos.robot.assets.urdf_to_usd.urdf_to_usd", refuse)
    module = make_manager(tf_renames={"arm": "gripper"})

    (transform,) = module.joint_transforms(JointState(ts=1.0, name=["shoulder"], position=[0.0]))

    assert transform.child_frame_id == "gripper"
    assert module._file_paths == set()


def test_the_usd_layer_is_built_off_the_event_loop_thread(
    make_manager: Callable[..., UrdfManager],
) -> None:
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


def test_a_usd_native_robot_cuts_itself_out_of_its_own_cloud(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(_URDF)
    scene = tmp_path / "robot.usda"
    scene.write_text(RobotModel.from_file(urdf).to_usd().usd)

    module = UrdfManagerPointClearing(model=UsdModel.from_file(scene), tf_tolerance_s=0.0)
    try:
        cast("dict[str, Any]", module.__dict__)["_tf"] = TF()
        _place_arm(module, (0.0, 0.0, 0.0), 1.0)

        result = module.filter_cloud(_cloud([[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [2.0, 0.0, 0.0]]))

        assert result is not None
        np.testing.assert_allclose(result.points_f32(), [[2.0, 0.0, 0.0]], atol=1e-6)
    finally:
        module.dispose()
