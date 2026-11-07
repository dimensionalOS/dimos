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
from dimos.robot.assets.usd import unhashed_assets, usd_frame_names
from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL
from dimos.robot.manipulators.xarm.config import (
    XARM_ROS2_REF,
    XARM_ROS2_REPO,
    make_xarm7_model_config,
)
from dimos.robot.roboplan_body_filter import RoboPlanBodyFilter
from dimos.robot.usd_manager import (
    USD,
    CircularMountError,
    Mount,
    UsdManager,
    _is_circular_mount,
)
from dimos.robot.usd_manager_point_clearing import UsdManagerPointClearing

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

# The same cube on two symmetric mounts, for the two-of-the-same-arm cases.
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
def make_filter(tmp_path: Path) -> Iterator[Callable[..., UsdManagerPointClearing]]:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(_URDF)
    modules: list[UsdManagerPointClearing] = []

    def make(**overrides: Any) -> UsdManagerPointClearing:
        urdf.write_text(overrides.pop("urdf_xml", _URDF))
        settings: dict[str, Any] = {
            "model": RobotModel.from_file(urdf),
            "padding_m": 0.01,
            "tf_tolerance_s": 0.001,
            "tf_forward_tolerance_s": 0.0,
        }
        settings.update(overrides)
        module = UsdManagerPointClearing(**settings)
        cast("dict[str, Any]", module.__dict__)["_tf"] = TF()
        modules.append(module)
        return module

    yield make
    for module in modules:
        cast("dict[str, Any]", module.__dict__)["_tf"] = None
        module.dispose()


def _place_arm(module: UsdManagerPointClearing, at: tuple[float, float, float], ts: float) -> None:
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
    make_filter: Callable[..., UsdManagerPointClearing],
) -> None:
    module = make_filter()
    _place_arm(module, (0.0, 0.0, 0.0), 1.0)

    result = module.filter_cloud(_cloud([[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [2.0, 0.0, 0.0]]))

    assert result is not None
    filtered = result
    np.testing.assert_allclose(filtered.points_f32(), [[2.0, 0.0, 0.0]], atol=1e-6)


def test_a_cloud_without_capture_time_tf_is_dropped_whole(
    make_filter: Callable[..., UsdManagerPointClearing],
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
    make_filter: Callable[..., UsdManagerPointClearing],
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
    module = UsdManager(model=R1PRO_MODEL)
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
    def get_usd(self) -> list[str]:
        return ["", _SUBTREE_USD]


class _FakeG1(Module):
    """Stands in for a mounted module that is itself a manager: it answers get_scene.

    Its answer already has its own lidar on it, which is what a nested UsdManager would hand
    over, and the frame that lidar hangs from carries the g1's namespace prefix.
    """

    config: ModuleConfig

    @rpc
    def get_scene(self, visited: list[str] | None = None) -> list[str]:
        return ["", _G1_WITH_HEAD_USD]


class _NoUsd(Module):
    config: ModuleConfig


def _subtree_usd(prim: str, link: str, mesh: str | None = None) -> str:
    """A stand-in subtree in the dialect urdf_to_usd emits: displayName plus RigidBodyAPI."""
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

# A g1-shaped subtree: a root plus a head frame to hang something else off. Its names are
# already prefixed, the way a nested manager's scene arrives.
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
    """An RPC client answering get_usd/get_scene from a script; an Exception means 'not up yet'."""

    def __init__(self, *answers: Any, asset_root: str = "") -> None:
        self.answers = list(answers)
        self.asked = 0
        self.asset_root = asset_root
        # Every mount chain it was handed, so a test can assert the chain travels.
        self.chains: list[list[str]] = []

    def get_usd(self) -> Any:
        self.asked += 1
        answer = self.answers.pop(0) if len(self.answers) > 1 else self.answers[0]
        if isinstance(answer, Exception):
            raise answer
        # A bare string in a script means "that USD, rooted nowhere"; anything else goes
        # through as-is so a test can hand over a malformed answer on purpose.
        return [self.asset_root, answer] if isinstance(answer, str) else answer

    def get_scene(self, visited: list[str] | None = None) -> Any:
        """A mounted manager is asked for its whole scene, and handed the mount chain."""
        self.chains.append(list(visited or []))
        return self.get_usd()

    def stop_rpc_client(self) -> None: ...


@pytest.fixture
def make_manager(tmp_path: Path) -> Iterator[Callable[..., UsdManager]]:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(_URDF)
    modules: list[UsdManager] = []

    def make(**overrides: Any) -> UsdManager:
        urdf.write_text(overrides.pop("urdf_xml", _URDF))
        settings: dict[str, Any] = {"model": RobotModel.from_file(urdf)}
        settings.update(overrides)
        module = UsdManager(**settings)
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
    """The frame_id a prim travels under: its displayName, or its prim name without one."""
    prim = stage.GetPrimAtPath(path)
    assert prim, f"no prim at {path}"
    return str(prim.GetDisplayName() or prim.GetName())


def _scene(module: UsdManager) -> str:
    """The USD out of get_scene, on a loop of this test's own so its executor dies with it."""
    return _scene_answer(module)[1]


def _scene_answer(module: UsdManager) -> list[str]:
    coroutine = cast("Any", UsdManager.get_scene).aio.__get__(module, UsdManager)
    return cast("list[str]", asyncio.run(coroutine()))


def test_get_usd_is_this_robots_own_urdf_with_nothing_attached(
    make_manager: Callable[..., UsdManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager()

    stage = _usd_stage(module.get_usd()[1])

    assert stage.GetDefaultPrim().GetName() == "box_robot"
    assert stage.GetPrimAtPath("/box_robot/base/arm")


def test_get_usd_is_built_once_and_reused(make_manager: Callable[..., UsdManager]) -> None:
    pytest.importorskip("pxr")
    module = make_manager()

    assert module.get_usd()[1] is module.get_usd()[1]


def test_get_scene_without_mounts_is_just_the_robot(
    make_manager: Callable[..., UsdManager],
) -> None:
    # The empty default is what keeps every existing blueprint working untouched.
    pytest.importorskip("pxr")
    module = make_manager()

    # Same layer; only the root differs, since a published scene resolves nothing off disk.
    assert _scene(module) == module.get_usd()[USD]


def test_get_scene_grafts_each_subtree_onto_the_frame_it_is_bolted_to(
    make_manager: Callable[..., UsdManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))

    stage = _usd_stage(_scene(module))

    assert stage.GetPrimAtPath("/box_robot/base/arm/fake_arm")


def test_get_scene_keeps_asking_a_module_that_is_not_serving_yet(
    make_manager: Callable[..., UsdManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm}, scene_retry_s=0.01)
    client = _FakeClient(RuntimeError("no reply"), None, _SUBTREE_USD)
    module._subtree_clients["_FakeArm"] = cast("Any", client)

    stage = _usd_stage(_scene(module))

    assert client.asked == 3
    assert stage.GetPrimAtPath("/box_robot/base/arm/fake_arm")


def test_get_scene_does_not_return_half_a_robot_while_a_mount_is_silent(
    make_manager: Callable[..., UsdManager],
) -> None:
    """Half a scene would hand a renderer a robot with no arm, so there is no partial answer.

    With the deadline gone there is no error to assert, so assert that it is still waiting:
    wait_for cancels it, which is also what keeps this test from leaking the retry loop.
    """
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm}, scene_retry_s=0.01)
    client = _FakeClient(RuntimeError("never"))
    module._subtree_clients["_FakeArm"] = cast("Any", client)

    async def wait_a_moment() -> None:
        coroutine = cast("Any", UsdManager.get_scene).aio.__get__(module, UsdManager)
        with pytest.raises(asyncio.TimeoutError):
            await asyncio.wait_for(coroutine(), 0.2)

    asyncio.run(wait_a_moment())

    assert client.asked > 1, "it should have kept asking rather than giving up or answering"


def test_a_mount_frame_is_not_checked_until_the_scene_is_assembled(
    make_manager: Callable[..., UsdManager],
) -> None:
    # It may name a frame another subtree brings, so there is nothing to check at __init__.
    # _graft_in_waves catches it instead, where it can list every frame there is.
    make_manager(mounts={"elbow": _FakeArm}).dispose()


def test_mounts_to_a_module_that_cannot_be_asked_is_rejected(
    make_manager: Callable[..., UsdManager],
) -> None:
    with pytest.raises(ValueError, match="neither an @rpc get_usd"):
        make_manager(mounts={"arm": _NoUsd})


def test_a_module_class_names_the_only_instance_of_itself() -> None:
    mount = Mount.of(_FakeArm)

    assert mount.module is _FakeArm
    # RPCClient.remote's own default for a module with no instance_name.
    assert mount.remote_name == "_FakeArm"


def test_a_namespaced_blueprint_names_the_instance_it_deploys() -> None:
    mount = Mount.of(_FakeArm.blueprint().namespace("left"))

    assert mount.module is _FakeArm
    assert mount.remote_name == "left/_fakearm"


def test_two_namespaced_copies_of_one_class_are_two_different_mounts() -> None:
    # The whole point: before this, both resolved to the class name and collided.
    left = Mount.of(_FakeArm.blueprint().namespace("left"))
    right = Mount.of(_FakeArm.blueprint().namespace("right"))

    assert left.remote_name != right.remote_name


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
    make_manager: Callable[..., UsdManager],
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
    # namespace() keeps a user-set frame_id_prefix, and that override is the answer to "what
    # are this module's frames called" -- so it is read off the module, not off its name.
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
    # Nothing to read a frame_id_prefix off, so the name is all there is to go on.
    assert Mount.of(remote_name).frame_prefix == prefix


def test_each_namespaced_arm_is_asked_under_its_own_name_and_renamed_to_it(
    make_manager: Callable[..., UsdManager],
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
    # Same class, same description, same link names -- so the two copies would have collided
    # on fake_arm_base. The namespace they were deployed under is what tells them apart.
    assert _link_of(stage, "/box_robot/base/left_mount/fake_arm") == "left/fake_arm_base"
    assert _link_of(stage, "/box_robot/base/right_mount/fake_arm") == "right/fake_arm_base"


def test_an_un_namespaced_subtree_keeps_the_link_names_its_description_gave_it(
    make_manager: Callable[..., UsdManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))

    stage = _usd_stage(_scene(module))

    assert _link_of(stage, "/box_robot/base/arm/fake_arm") == "fake_arm_base"


def test_the_roots_own_links_are_never_renamed(
    make_manager: Callable[..., UsdManager],
) -> None:
    # One URDF, so its link names are already unique; only what is grafted on needs moving.
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
    make_manager: Callable[..., UsdManager], tmp_path: Path
) -> None:
    # A layer only says where its meshes are relative to something, and the asker has no way
    # to guess what. The URDF's own directory is that something.
    pytest.importorskip("pxr")
    module = make_manager()

    assert module.get_usd()[0] == str(tmp_path)


def test_a_subtree_mesh_is_rebased_onto_its_module_root_then_published(
    make_manager: Callable[..., UsdManager],
) -> None:
    """Both halves of what a mounted module's mesh path goes through.

    The module's cwd is not this one's, so a relative path means nothing here until it is
    rebased onto the root that module reported. And an absolute path on this machine means
    nothing to whoever ends up asking for the file, so the scene goes out under a name this
    manager answers for, with the real path kept behind it.
    """
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


def test_a_published_asset_is_named_from_where_it_first_appears(
    make_manager: Callable[..., UsdManager],
) -> None:
    # Stable across runs of the same scene, and readable, rather than an opaque number.
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    subtree = _subtree_usd("fake_arm", "fake_arm_base", mesh="meshes/arm.stl")
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(subtree, asset_root="/opt"))

    _scene(module)

    published = next(iter(module._published_assets))
    assert published.startswith("/dimos/assets/")
    assert published.endswith("arm.stl")


def test_one_mesh_on_two_mounts_is_published_once(
    make_manager: Callable[..., UsdManager],
) -> None:
    """Namespaces do not matter to an asset: the same file is the same asset.

    Two copies of one arm are two mount points and one mesh, so they share a published name
    rather than each getting their own.
    """
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
    make_manager: Callable[..., UsdManager],
) -> None:
    # An empty root means "nothing to resolve"; an absolute path needs no rebasing either.
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
    make_manager: Callable[..., UsdManager], answer: Any
) -> None:
    # A malformed answer is treated like a module that is not up yet, not grafted blindly.
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm}, scene_retry_s=0.01)
    client = _FakeClient(answer, _SUBTREE_USD)
    module._subtree_clients["_FakeArm"] = cast("Any", client)

    stage = _usd_stage(_scene(module))

    assert client.asked == 2, "the malformed answer should have been asked past"
    assert stage.GetPrimAtPath("/box_robot/base/arm/fake_arm")


def _namespaced_arm(make_manager: Callable[..., UsdManager], **overrides: Any) -> UsdManager:
    module = make_manager(mounts={"arm": _FakeArm.blueprint().namespace("left")}, **overrides)
    module._subtree_clients["left/_fakearm"] = cast("Any", _FakeClient(_SUBTREE_USD))
    return module


def test_frames_are_not_renamed_by_default(
    make_manager: Callable[..., UsdManager],
) -> None:
    pytest.importorskip("pxr")
    module = _namespaced_arm(make_manager)

    stage = _usd_stage(_scene(module))

    assert _link_of(stage, "/box_robot/base/arm/fake_arm") == "left/fake_arm_base"


def test_a_pre_namespace_rename_happens_before_the_prefix_goes_on(
    make_manager: Callable[..., UsdManager],
) -> None:
    # Written in the subtree's own names, which is what a vendor description gives you.
    pytest.importorskip("pxr")
    module = _namespaced_arm(make_manager, tf_renames_pre_namespace={"fake_arm_base": "arm_base"})

    stage = _usd_stage(_scene(module))

    assert _link_of(stage, "/box_robot/base/arm/fake_arm") == "left/arm_base"


def test_a_post_namespace_rename_happens_after_the_prefix_goes_on(
    make_manager: Callable[..., UsdManager],
) -> None:
    # Written in the prefixed names, and the only way to reach a name without the namespace.
    pytest.importorskip("pxr")
    module = _namespaced_arm(make_manager, tf_renames={"left/fake_arm_base": "left_arm"})

    stage = _usd_stage(_scene(module))

    assert _link_of(stage, "/box_robot/base/arm/fake_arm") == "left_arm"


def test_both_renames_apply_around_the_prefix(
    make_manager: Callable[..., UsdManager],
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
    make_manager: Callable[..., UsdManager],
) -> None:
    """One mapping covers the whole scene, so most keys are meant for one subtree only.

    So a key that does not appear in a given subtree is skipped there rather than refused.
    """
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
    make_manager: Callable[..., UsdManager],
) -> None:
    # Loudly: a misspelt frame is a typo, not a no-op, and it would otherwise only show up as
    # a scene whose frames are not the ones asked for.
    pytest.importorskip("pxr")
    module = _namespaced_arm(make_manager, tf_renames_pre_namespace={"elbow": "wrist"})

    with pytest.raises(ValueError, match="nor of any mounted subtree"):
        _scene(module)


def test_a_lidar_mounts_onto_a_g1_that_is_itself_mounted_on_the_root(
    make_manager: Callable[..., UsdManager],
) -> None:
    """The nested case: root -> g1 -> lidar, named by where each ends up in the scene.

    "g1/head_mount" is not a link of this URDF at all; it only exists once the g1's own scene
    is grafted on. So the two mounts cannot both be grafted in one pass.
    """
    pytest.importorskip("pxr")
    module = make_manager(
        mounts={"arm": _FakeG1, "g1/head_mount": _FakeArm},
    )
    module._subtree_clients["_FakeG1"] = cast("Any", _FakeClient(_G1_WITH_HEAD_USD))
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))

    stage = _usd_stage(_scene(module))

    # The g1 went onto the robot, and the arm went onto the g1's head.
    assert stage.GetPrimAtPath("/box_robot/base/arm/g1")
    assert stage.GetPrimAtPath("/box_robot/base/arm/g1/head_mount/fake_arm")


def test_a_mounted_manager_is_asked_for_its_whole_scene_not_just_its_own_layer(
    make_manager: Callable[..., UsdManager],
) -> None:
    # Otherwise a mounted g1 would arrive without the lidar that is mounted on it.
    pytest.importorskip("pxr")

    assert Mount.of(_FakeG1).subtree_rpc == "get_scene"
    assert Mount.of(_FakeArm).subtree_rpc == "get_usd"


def test_an_instance_name_on_its_own_is_asked_for_its_own_layer() -> None:
    # No class to ask whether it has a get_scene, so the narrower rpc is the safe assumption.
    assert Mount.of("left/xarm").subtree_rpc == "get_usd"


def test_mounting_at_a_frame_nothing_will_ever_provide_lists_every_frame_there_is(
    make_manager: Callable[..., UsdManager],
) -> None:
    """The error has to be enough to find the frame you meant.

    So it names what is missing, says mount frames are post-namespace names, and lists every
    link and joint the scene ended up with.
    """
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm, "nowhere/at_all": _FakeArm})
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))

    with pytest.raises(ValueError) as caught:
        _scene(module)

    message = str(caught.value)
    assert "nowhere/at_all" in message
    assert "g1/head_mount" in message or "namespace prefix" in message
    # Every frame the scene has, so the right one can be picked out of the list.
    assert "'base'" in message and "'arm'" in message and "'fake_arm_base'" in message


def test_a_frame_that_only_a_later_wave_provides_is_not_reported_as_missing(
    make_manager: Callable[..., UsdManager],
) -> None:
    # The whole point of waves: "g1/head_mount" is absent on the first look and present later.
    pytest.importorskip("pxr")
    module = make_manager(mounts={"g1/head_mount": _FakeArm, "arm": _FakeG1})
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))
    module._subtree_clients["_FakeG1"] = cast("Any", _FakeClient(_G1_WITH_HEAD_USD))

    stage = _usd_stage(_scene(module))

    assert stage.GetPrimAtPath("/box_robot/base/arm/g1/head_mount/fake_arm")


def test_a_circular_mount_is_refused_with_the_whole_chain(
    make_manager: Callable[..., UsdManager],
) -> None:
    """A manager assembling a scene that is waiting on itself cannot ever finish.

    The chain travels with the request, so the manager that sees itself in it refuses rather
    than asking back round forever.
    """
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeG1})

    async def ask_as_if_from_itself() -> None:
        coroutine = cast("Any", UsdManager.get_scene).aio.__get__(module, UsdManager)
        with pytest.raises(ValueError, match="circular mount"):
            await coroutine(["UsdManager", "SomeOther"])

    asyncio.run(ask_as_if_from_itself())


def test_the_circular_mount_error_names_every_manager_in_the_loop(
    make_manager: Callable[..., UsdManager],
) -> None:
    pytest.importorskip("pxr")
    module = make_manager()

    async def ask() -> None:
        coroutine = cast("Any", UsdManager.get_scene).aio.__get__(module, UsdManager)
        with pytest.raises(ValueError) as caught:
            await coroutine(["A", "UsdManager", "B"])
        assert "A -> UsdManager -> B -> UsdManager" in str(caught.value)

    asyncio.run(ask())


def test_the_mount_chain_travels_to_a_mounted_manager(
    make_manager: Callable[..., UsdManager],
) -> None:
    # Which is what lets the manager below refuse a cycle instead of asking back round.
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeG1})
    client = _FakeClient(_G1_WITH_HEAD_USD)
    module._subtree_clients["_FakeG1"] = cast("Any", client)

    _scene(module)

    assert client.chains == [["UsdManager"]]


def test_a_manager_with_no_model_stands_on_a_world_link() -> None:
    """A manager whose only job is to hold mounts needs somewhere to hang them from.

    So no model gives a single link called "world" rather than refusing to start.
    """
    pytest.importorskip("pxr")
    module = UsdManager()
    try:
        stage = _usd_stage(module.get_usd()[1])

        assert module._description.root_link == "world"
        assert stage.GetDefaultPrim().GetName() == "world"
    finally:
        module.dispose()


def test_a_world_manager_has_no_asset_root_to_offer() -> None:
    # There is no description file, so there is no directory its paths could resolve against.
    module = UsdManager()
    try:
        assert module.get_usd()[0] == ""
    finally:
        module.dispose()


def test_a_world_manager_still_publishes_no_joint_tf() -> None:
    # One link and no joints, so there is nothing to pose.
    module = UsdManager()
    try:
        assert module.joint_transforms(JointState()) == []
    finally:
        module.dispose()


def test_mounts_hang_off_the_world_link_of_a_manager_with_no_model(
    make_manager: Callable[..., UsdManager],
) -> None:
    # The point of the world root: a tree of sensors and arms with no robot of its own.
    pytest.importorskip("pxr")
    module = UsdManager(mounts={"world": _FakeG1, "g1/head_mount": _FakeArm})
    try:
        module._subtree_clients["_FakeG1"] = cast("Any", _FakeClient(_G1_WITH_HEAD_USD))
        module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))

        stage = _usd_stage(_scene(module))

        assert stage.GetPrimAtPath("/world/world/g1/head_mount/fake_arm")
    finally:
        module.dispose()


def test_tf_renames_reach_this_robots_own_frames_on_tf(
    make_manager: Callable[..., UsdManager],
) -> None:
    """The field is called tf_renames: a robot's own frames are the first thing it should rename."""
    module = make_manager(tf_renames={"base": "world", "arm": "gripper"})

    (transform,) = module.joint_transforms(JointState(ts=1.0, name=["shoulder"], position=[0.0]))

    assert (transform.frame_id, transform.child_frame_id) == ("world", "gripper")


def test_tf_renames_reach_this_robots_own_frames_in_its_usd(
    make_manager: Callable[..., UsdManager],
) -> None:
    # Both sides or neither: a layer naming a frame no transform arrives for is worse than none.
    pytest.importorskip("pxr")
    module = make_manager(tf_renames={"arm": "gripper"})

    stage = _usd_stage(module.get_usd()[USD])

    assert _link_of(stage, "/box_robot/base/arm") == "gripper"


def test_a_pre_namespace_rename_also_reaches_this_robots_own_frames(
    make_manager: Callable[..., UsdManager],
) -> None:
    # This robot takes no namespace prefix, so for its frames the two dicts just run in order.
    module = make_manager(
        tf_renames_pre_namespace={"arm": "forearm"}, tf_renames={"forearm": "gripper"}
    )

    (transform,) = module.joint_transforms(JointState(ts=1.0, name=["shoulder"], position=[0.0]))

    assert transform.child_frame_id == "gripper"


def test_renaming_a_joint_leaves_the_name_its_joint_state_arrives_under(
    make_manager: Callable[..., UsdManager],
) -> None:
    """The description keeps the hardware's names; only what goes out is renamed."""
    module = make_manager(tf_renames={"shoulder": "elbow"}, urdf_xml=_joint_robot("revolute"))

    (transform,) = module.joint_transforms(
        JointState(ts=1.0, name=["shoulder"], position=[np.pi / 2])
    )

    # Rotated about x by the position the state carried, so the rename did not hide it.
    np.testing.assert_allclose(
        transform.rotation.to_tuple(), [np.sin(np.pi / 4), 0.0, 0.0, np.cos(np.pi / 4)], atol=1e-9
    )


def test_a_rename_of_this_robots_own_frame_needs_no_mounts(
    make_manager: Callable[..., UsdManager],
) -> None:
    # It used to be a hard error to rename anything without mounting something first.
    pytest.importorskip("pxr")
    module = make_manager(tf_renames={"arm": "gripper"})

    assert _usd_stage(_scene(module)).GetPrimAtPath("/box_robot/base/arm")


def test_a_rename_matching_neither_this_robot_nor_a_subtree_fails_at_construction(
    make_manager: Callable[..., UsdManager],
) -> None:
    """With no mounts every name is known up front, so the typo need not wait for get_scene."""
    with pytest.raises(ValueError, match="which is neither a link or joint of this robot"):
        make_manager(tf_renames={"nothing": "here"})


def test_a_tf_lookup_takes_description_names_and_finds_the_published_ones(
    make_filter: Callable[..., UsdManagerPointClearing],
) -> None:
    """The one crossing point: this module asks in its URDF's names, tf carries the renamed ones."""
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
    """The transport only rebuilds builtin exception types; everything else names the type it was."""
    from dimos.protocol.rpc.rpc_utils import deserialize_exception, serialize_exception

    original = CircularMountError("circular mount: a -> b -> a")

    arrived = deserialize_exception(serialize_exception(original))

    assert not isinstance(arrived, CircularMountError)
    assert _is_circular_mount(arrived)


def test_an_ordinary_rpc_failure_is_not_mistaken_for_a_cycle() -> None:
    assert not _is_circular_mount(RuntimeError("circular mount is only a phrase in here"))


def test_render_file_request_serves_the_whole_scene_once_one_is_assembled(
    make_manager: Callable[..., UsdManager],
) -> None:
    """A renderer wants the tree it is drawing, not just the robot at the root of it."""
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))
    sent: list[File] = []
    module.file_info.publish = sent.append  # type: ignore[method-assign]
    _scene(module)

    asyncio.run(module.handle_render_file_request(String("scene.usd")))

    assert "fake_arm" in sent[0].data.decode()


def test_the_published_scene_carries_no_asset_root_to_resolve_against(
    make_manager: Callable[..., UsdManager],
) -> None:
    """Every path in it is a name this manager answers for, not a path on anyone's disk."""
    pytest.importorskip("pxr")
    module = make_manager(mounts={"arm": _FakeArm})
    module._subtree_clients["_FakeArm"] = cast("Any", _FakeClient(_SUBTREE_USD))

    assert _scene_answer(module)[0] == ""


def test_trivialize_leaves_the_frames_and_drops_the_geometry(
    make_manager: Callable[..., UsdManager],
) -> None:
    """Real hardware wants the mount points without the shapes; sim wants both."""
    pytest.importorskip("pxr")
    full = make_manager()
    thin = make_manager(trivialize=True)

    assert usd_frame_names(thin.get_usd()[USD]) == usd_frame_names(full.get_usd()[USD])
    assert "Cube" in full.get_usd()[USD]
    assert "Cube" not in thin.get_usd()[USD]


def test_a_trivialized_robot_publishes_the_same_tf_as_a_full_one(
    make_manager: Callable[..., UsdManager],
) -> None:
    # The whole point: the same blueprint wiring works against either.
    state = JointState(ts=1.0, name=["shoulder"], position=[0.0])
    full = make_manager().joint_transforms(state)
    thin = make_manager(trivialize=True).joint_transforms(state)

    assert [(t.frame_id, t.child_frame_id) for t in thin] == [
        (t.frame_id, t.child_frame_id) for t in full
    ]


def test_a_trivialized_robot_names_no_assets_to_serve(
    make_manager: Callable[..., UsdManager], tmp_path: Path
) -> None:
    """Nothing to hash and nothing to publish, since the meshes went with the shapes."""
    pytest.importorskip("pxr")
    mesh = tmp_path / "cube.stl"
    trimesh.creation.box(extents=[0.2, 0.2, 0.2]).export(mesh)
    urdf = _URDF.replace('<box size="0.2 0.2 0.2"/>', f'<mesh filename="{mesh}"/>')
    module = make_manager(trivialize=True, urdf_xml=urdf)

    assert unhashed_assets(module.get_usd()[USD]) == {}
    assert "cube.stl" not in module.get_usd()[USD]


def test_trivialize_leaves_a_mounted_subtree_alone(
    make_manager: Callable[..., UsdManager],
) -> None:
    """Only this robot's own layer is trivialized: what a mounted module answers is its own say."""
    pytest.importorskip("pxr")
    module = make_manager(trivialize=True, mounts={"arm": _FakeArm})
    module._subtree_clients["_FakeArm"] = cast(
        "Any", _FakeClient(_subtree_usd("fake_arm", "fake_arm_base", mesh="/opt/arm/arm.stl"))
    )

    scene = _scene(module)

    # The root's own box went; the arm's mesh stayed and is served under a published name.
    assert "Cube" not in scene
    assert "Mesh" in scene
    assert list(module._published_assets.values()) == ["/opt/arm/arm.stl"]


def test_the_body_mask_still_cuts_the_robot_out_when_its_frames_are_renamed(
    make_filter: Callable[..., UsdManagerPointClearing],
) -> None:
    """The filter works in the description's names and reads tf in the published ones.

    Those are two different sets of names once tf_renames is in play, so the whole mask hangs
    on the lookups going through frame() and the joint keys not.
    """
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
    make_filter: Callable[..., UsdManagerPointClearing],
) -> None:
    """A floating joint's motion comes from tf, so its edge has to be looked up renamed too."""
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

    # The cube moved with the floating joint, so the point at its new home goes and the far one stays.
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
    """A scene that is already USD is not round-tripped through a URDF."""
    pytest.importorskip("pxr")
    module = UsdManager(model=UsdModel.from_file(scene_file))
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
    module = UsdManager(model=UsdModel.from_file(scene_file))
    try:
        sent: list[File] = []
        module.file_info.publish = sent.append  # type: ignore[method-assign]

        asyncio.run(module.handle_render_file_request(String("meshes/table.stl")))

        assert [file.data for file in sent] == [b"stl bytes"]
    finally:
        module.dispose()


def test_a_usd_model_poses_its_own_frames_on_tf(scene_file: Path) -> None:
    """A USD layer's own frame tree goes out on tf, so a renderer can place what it is sent.

    A USD conveys the tree by nesting and a nested link with no joint is welded, so this scene's
    one edge is room -> table. The joint state names something else entirely and is ignored,
    which is the point: the tree is published either way.
    """
    pytest.importorskip("pxr")
    module = UsdManager(model=UsdModel.from_file(scene_file))
    try:
        posed = module.joint_transforms(JointState(ts=1.0, name=["shoulder"], position=[1.0]))

        assert [(t.frame_id, t.child_frame_id) for t in posed] == [("room", "table")]
    finally:
        module.dispose()


def test_a_usd_model_is_still_trivialized_and_renamed(scene_file: Path) -> None:
    pytest.importorskip("pxr")
    module = UsdManager(
        model=UsdModel.from_file(scene_file), trivialize=True, tf_renames={"table": "desk"}
    )
    try:
        usd = module.get_usd()[USD]

        assert "Mesh" not in usd
        assert "desk" in usd_frame_names(usd)
    finally:
        module.dispose()


def test_point_clearing_converts_a_usd_model_back_to_urdf(scene_file: Path) -> None:
    """RoboPlan only reads URDF, so a USD-native robot is converted on the way in."""
    pytest.importorskip("pxr")
    module = UsdManagerPointClearing(model=UsdModel.from_file(scene_file))
    try:
        description = module._filter_description()

        assert "<robot" in description.xml
        assert 'name="room"' in description.xml
    finally:
        module.dispose()


def test_point_clearing_hands_over_the_urdf_it_already_has(tmp_path: Path) -> None:
    """No round trip when the model came from a URDF: the original is still there."""
    source = tmp_path / "robot.urdf"
    source.write_text(_URDF)
    module = UsdManagerPointClearing(model=RobotModel.from_file(source))
    try:
        assert module._filter_description().xml == RobotModel.from_file(source).load().xml
    finally:
        module.dispose()


def test_a_usd_model_wants_a_usd_file(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="RobotModel.from_file"):
        UsdModel.from_file(tmp_path / "robot.urdf")


def test_a_manager_needs_no_usd_until_a_layer_is_asked_for(
    make_manager: Callable[..., UsdManager], monkeypatch: pytest.MonkeyPatch
) -> None:
    """Converting needs usd-core, which has no linux-aarch64 wheel.

    A robot that only poses itself on tf must still come up on a Jetson, so the layer is built
    on demand rather than at __init__.
    """

    def refuse(*args: Any, **kwargs: Any) -> str:
        raise AssertionError("the layer was built before anything asked for it")

    monkeypatch.setattr("dimos.robot.assets.usd.urdf_to_usd", refuse)
    module = make_manager(tf_renames={"arm": "gripper"})

    (transform,) = module.joint_transforms(JointState(ts=1.0, name=["shoulder"], position=[0.0]))

    assert transform.child_frame_id == "gripper"
    assert module._file_paths == set()


def test_the_usd_layer_is_built_off_the_event_loop_thread(
    make_manager: Callable[..., UsdManager],
) -> None:
    """Building it imports usd-core and converts the URDF: ~150 ms on a real robot.

    That has to happen on a worker thread, or a renderer's first request stalls tf and
    every other handler for as long as it takes.
    """
    pytest.importorskip("pxr")
    module = make_manager()
    loop_thread: list[int] = []
    built_on: list[int] = []
    real = UsdManager.get_usd

    def record(self: UsdManager) -> str:
        built_on.append(threading.get_ident())
        return real(self)

    async def ask() -> None:
        loop_thread.append(threading.get_ident())
        await module.handle_render_file_request(String("scene.usda"))

    module.file_info.publish = lambda *_: None  # type: ignore[method-assign]
    with patch.object(UsdManager, "get_usd", record):
        asyncio.run(ask())

    assert built_on, "get_usd was never called"
    assert built_on[0] != loop_thread[0]


def test_a_usd_native_robot_cuts_itself_out_of_its_own_cloud(tmp_path: Path) -> None:
    """The whole point of the converter: masking on a robot described only in USD.

    Nothing about the filter reads USD, so this works only because the layer is converted
    back to the URDF RoboPlan wants.
    """
    pytest.importorskip("pxr")
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(_URDF)
    scene = tmp_path / "robot.usda"
    scene.write_text(RobotModel.from_file(urdf).to_usd().usd)

    module = UsdManagerPointClearing(model=UsdModel.from_file(scene), tf_tolerance_s=0.0)
    try:
        cast("dict[str, Any]", module.__dict__)["_tf"] = TF()
        _place_arm(module, (0.0, 0.0, 0.0), 1.0)

        result = module.filter_cloud(_cloud([[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [2.0, 0.0, 0.0]]))

        # The 20cm cube on `arm` swallows the first two; the far point survives.
        assert result is not None
        np.testing.assert_allclose(result.points_f32(), [[2.0, 0.0, 0.0]], atol=1e-6)
    finally:
        module.dispose()
