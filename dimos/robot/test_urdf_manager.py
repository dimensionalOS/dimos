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
from typing import Any, cast

import numpy as np
import pinocchio as pin
import pytest
import trimesh

from dimos.message_utils import geometry, pointcloud, time as message_time
from dimos_messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_messages.sensor_msgs.msg import JointState, PointCloud2, PointField
from dimos_messages.std_msgs.msg import Header, String
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
from dimos.types.file import File

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

_FIELD_DTYPES = {PointField.FLOAT32: "<f4", PointField.INT32: "<i4", PointField.FLOAT64: "<f8"}


def _header(ts: float, frame_id: str = "camera") -> Header:
    return Header(stamp=message_time.time_from_seconds(ts), frame_id=frame_id)


def _transform(
    parent: str,
    child: str,
    ts: float,
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> TransformStamped:
    return TransformStamped(
        header=_header(ts, parent),
        child_frame_id=child,
        transform=Transform(
            translation=Vector3(*translation),
            rotation=Quaternion(*quaternion),
        ),
    )


def _joint_state(
    ts: float,
    name: list[str],
    position: list[float],
    frame_id: str = "",
) -> JointState:
    values = np.asarray(position, dtype=np.float64)
    return JointState(
        header=_header(ts, frame_id),
        name=list(name),
        position=values,
        velocity=np.zeros(len(values), dtype=np.float64),
        effort=np.zeros(0, dtype=np.float64),
    )


def _cloud(
    points: list[list[float]] | np.ndarray,
    ts: float = 1.0,
    frame_id: str = "camera",
    **fields: tuple[np.ndarray, int],
) -> PointCloud2:
    """XYZ plus one scalar column per keyword, packed as a generated PointCloud2."""
    values = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    columns = [
        ("x", "<f4", PointField.FLOAT32, values[:, 0]),
        ("y", "<f4", PointField.FLOAT32, values[:, 1]),
        ("z", "<f4", PointField.FLOAT32, values[:, 2]),
    ]
    for name, (column, datatype) in fields.items():
        columns.append((name, _FIELD_DTYPES[datatype], datatype, np.asarray(column).reshape(-1)))
    packed = np.empty(len(values), dtype=[(name, dtype) for name, dtype, _, _ in columns])
    point_fields = []
    for name, _, datatype, column in columns:
        packed[name] = column
        point_fields.append(
            PointField(
                name=name,
                offset=packed.dtype.fields[name][1],
                datatype=datatype,
                count=1,
            )
        )
    stride = packed.dtype.itemsize
    return PointCloud2(
        header=_header(ts, frame_id),
        height=1,
        width=len(values),
        fields=point_fields,
        is_bigendian=False,
        point_step=stride,
        row_step=stride * len(values),
        data=packed.view(np.uint8).reshape(-1),
        is_dense=bool(np.isfinite(values).all()),
    )


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
        module.tfbuffer.receive_transform(_transform(parent, "base", ts, translation=at))


def test_points_on_the_robot_are_dropped_and_the_rest_survive(
    make_filter: Callable[..., UrdfManagerPointClearing],
) -> None:
    module = make_filter()
    _place_arm(module, (0.0, 0.0, 0.0), 1.0)

    result = module.filter_cloud(_cloud([[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [2.0, 0.0, 0.0]]))

    assert result is not None
    filtered = result
    np.testing.assert_allclose(pointcloud.xyz(filtered), [[2.0, 0.0, 0.0]], atol=1e-6)


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
    module.add_joint_state(_joint_state(2.0, ["shoulder"], [2.0]))
    module.add_joint_state(_joint_state(1.0, ["shoulder"], [0.0]))
    module.add_joint_state(_joint_state(1.0, ["shoulder"], [0.5]))

    result = module.filter_cloud(_cloud([[0.5, 0, 0], [2, 0, 0]], ts=1.0))

    assert result is not None
    np.testing.assert_allclose(pointcloud.xyz(result), [[2, 0, 0]])


@pytest.mark.parametrize(
    "state",
    [
        None,
        _joint_state(2.0, ["shoulder"], [0.5]),
        _joint_state(1.0, ["wrong"], [0.5]),
        _joint_state(1.0, ["shoulder"], [float("nan")]),
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
    module.add_joint_state(_joint_state(1.0, ["shoulder"], [np.pi / 2]))

    result = module.filter_cloud(_cloud([[0, 0.5, 0], [0.5, 0, 0]]))

    assert result is not None
    np.testing.assert_allclose(pointcloud.xyz(result), [[0.5, 0, 0]])


@pytest.mark.parametrize("kind", ["planar", "floating"])
def test_multidof_joints_use_capture_time_relative_tf(make_filter, kind):
    urdf = _joint_robot(kind).replace(
        '<axis xyz="1 0 0"/>', '<axis xyz="1 0 0"/><origin xyz="0.2 0.1 0" rpy="0 0 0.3"/>'
    )
    module = make_filter(urdf_xml=urdf)
    _place_arm(module, (0, 0, 0), 1.0)
    assert module.filter_cloud(_cloud([[0.5, 0, 0]])) is None
    module.tfbuffer.receive_transform(_transform("base", "arm", 1.0, translation=(0.5, 0, 0)))

    result = module.filter_cloud(_cloud([[0.5, 0, 0], [0, 0, 0]]))

    assert result is not None
    np.testing.assert_allclose(pointcloud.xyz(result), [[0, 0, 0]])


def test_narrowphase_retains_sphere_corner_obstacles_and_ancillary_fields(make_filter):
    urdf = _URDF.replace('<box size="0.2 0.2 0.2"/>', '<sphere radius="0.1"/>')
    module = make_filter(urdf_xml=urdf, padding_m=0.05)
    _place_arm(module, (0, 0, 0), 1.0)
    cloud = _cloud(
        [[0.14, 0, 0], [0.12, 0.12, 0], [0.2, 0, 0]],
        intensities=(np.array([1, 2, 3], dtype=np.float32), PointField.FLOAT32),
        labels=(np.array([10, 20, 30], dtype=np.int32), PointField.INT32),
    )

    result = module.filter_cloud(cloud)

    assert result is not None
    filtered = result
    np.testing.assert_allclose(pointcloud.xyz(filtered), [[0.12, 0.12, 0], [0.2, 0, 0]])
    records = pointcloud.view(filtered)
    np.testing.assert_array_equal(records["intensities"].ravel(), [2, 3])
    np.testing.assert_array_equal(records["labels"].ravel(), [20, 30])
    assert (filtered.header.frame_id, message_time.to_seconds(filtered.header.stamp)) == (
        "camera",
        1.0,
    )
    assert filtered.header.stamp == cloud.header.stamp


def test_rotated_base_aligns_camera_points(make_filter):
    module = make_filter()
    module.tfbuffer.receive_transform(_transform("world", "camera", 1.0))
    module.tfbuffer.receive_transform(
        _transform(
            "world",
            "base",
            1.0,
            translation=(-1, 0, 0),
            quaternion=(0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)),
        )
    )

    result = module.filter_cloud(_cloud([[-1, 0.05, 0], [-1, 1, 0]]))

    assert result is not None
    np.testing.assert_allclose(pointcloud.xyz(result), [[-1, 1, 0]])


def test_mesh_filter_removes_surfaces_without_classifying_solid_volume(make_filter, tmp_path):
    mesh_path = tmp_path / "cube.stl"
    trimesh.creation.box(extents=[0.2, 0.2, 0.2]).export(mesh_path)
    urdf = _URDF.replace('<box size="0.2 0.2 0.2"/>', f'<mesh filename="{mesh_path}"/>')
    module = make_filter(urdf_xml=urdf)
    _place_arm(module, (0, 0, 0), 1.0)

    result = module.filter_cloud(_cloud([[0.1, 0, 0], [0.105, 0, 0], [0, 0, 0], [0.2, 0, 0]]))

    assert result is not None
    np.testing.assert_allclose(pointcloud.xyz(result), [[0, 0, 0], [0.2, 0, 0]])


def test_late_state_cannot_resurrect_expired_history(make_filter):
    module = make_filter(urdf_xml=_joint_robot("prismatic"), state_history_s=1.0)
    _place_arm(module, (0, 0, 0), 1.0)
    module.add_joint_state(_joint_state(3.0, ["shoulder"], [0.5]))
    module.add_joint_state(_joint_state(1.0, ["shoulder"], [0.5]))

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
        module.add_joint_state(_joint_state(stamp, config.joint_names, arm))
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
            _cloud(
                np.concatenate([*surfaces, controls]).astype(np.float32),
                ts=stamp,
                frame_id="world",
            )
        )
    # Both captures are processed after the newer state arrived.
    first = module.filter_cloud(captures[0])
    assert first is not None
    second = module.filter_cloud(captures[1])
    assert second is not None
    for cloud, filtered in zip(captures, (first, second), strict=True):
        np.testing.assert_allclose(pointcloud.xyz(filtered), controls)
        assert filtered.header.frame_id == "world"
        assert filtered.header.stamp == cloud.header.stamp


def test_the_joint_state_is_published_as_one_tf_edge_per_joint(make_filter):
    module = make_filter(urdf_xml=_joint_robot("prismatic"))
    module.add_joint_state(_joint_state(1.0, ["shoulder"], [0.3]))

    edge = module.tfbuffer.get("base", "arm", time_point=1.0, time_tolerance=0.001)

    assert edge is not None
    np.testing.assert_allclose(
        [edge.transform.translation.x, edge.transform.translation.y, edge.transform.translation.z],
        [0.3, 0, 0],
        atol=1e-9,
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
    module.add_joint_state(_joint_state(1.0, ["shoulder"], [0.3]))

    assert module.tfbuffer.get("base", "arm", time_point=1.0, time_tolerance=0.001) is None


def test_render_file_request_serves_the_urdf_and_only_files_it_names(
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

    # the URDF and the files by the full paths it names; anything else on disk gets nothing
    for request in (
        str(urdf),
        cube,
        texture,
        str(package / "meshes"),
        "package://box/meshes/cube.stl",
    ):
        asyncio.run(module.handle_render_file_request(String(request)))

    assert [file.uri for file in sent] == [str(urdf), cube, texture]
    assert f'filename="{cube}"' in sent[0].data.decode()
    assert File.lcm_decode(sent[1].lcm_encode()).data == Path(cube).read_bytes()
    assert sent[2].data == b"png bytes"


def test_a_filtered_cloud_keeps_its_fields_and_capture_metadata_and_a_late_capture_is_still_filtered(
    make_filter: Callable[..., UrdfManagerPointClearing],
) -> None:
    module = make_filter()
    _place_arm(module, (0.0, 0.0, 0.0), 2.0)
    cloud = _cloud(
        [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        ts=2.0,
        intensity=(np.asarray([10, 20], dtype=np.float32), PointField.FLOAT32),
    )

    result = module.filter_cloud(cloud)

    assert result is not None
    assert (result.header.frame_id, message_time.to_seconds(result.header.stamp)) == (
        "camera",
        2.0,
    )
    np.testing.assert_allclose(pointcloud.xyz(result), [[2.0, 0.0, 0.0]], atol=1e-6)
    np.testing.assert_array_equal(pointcloud.view(result)["intensity"].ravel(), [20])
    # a second sensor on the same input lags the first: its older capture is matched to its own capture time
    _place_arm(module, (0.0, 0.0, 0.0), 1.0)
    late = module.filter_cloud(_cloud([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]], ts=1.0))
    assert late is not None
    np.testing.assert_allclose(pointcloud.xyz(late), [[2.0, 0.0, 0.0]], atol=1e-6)


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
            _joint_state(1.0, list(positions), list(positions.values()))
        )
    finally:
        module.dispose()

    assert len(transforms) == len(description.joints)
    for transform in transforms:
        np.testing.assert_allclose(
            geometry.transform_matrix(transform.transform),
            roboplan._context.forwardKinematics(
                configuration, transform.child_frame_id, transform.header.frame_id
            ),
            atol=1e-9,
        )
