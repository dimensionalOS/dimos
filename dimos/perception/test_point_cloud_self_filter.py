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

# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from __future__ import annotations

from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any, cast

import numpy as np
import pytest
import trimesh

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.point_cloud_self_filter import (
    PointCloudSelfFilter,
    _CollisionGeometry,
)
from dimos.protocol.tf.tf import TF, MultiTBuffer
from dimos.robot.manipulators.xarm.config import make_xarm7_sim_robot_config


def _write_box_robot(path: Path) -> None:
    path.write_text(
        """<?xml version="1.0"?>
<robot name="box_robot">
  <link name="base">
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
</robot>
"""
    )


@pytest.fixture
def make_filter(tmp_path: Path) -> Iterator[Callable[[], PointCloudSelfFilter]]:
    urdf = tmp_path / "robot.urdf"
    _write_box_robot(urdf)
    modules: list[PointCloudSelfFilter] = []

    def make() -> PointCloudSelfFilter:
        module = PointCloudSelfFilter(
            robot_model=RobotModelConfig(
                name="robot",
                model_path=urdf,
                joint_names=[],
                base_link="base",
            ),
            padding_m=0.01,
            voxel_size=0.05,
            tf_tolerance_s=0.001,
            tf_forward_tolerance_s=0.0,
        )
        cast("dict[str, Any]", module.__dict__)["_tf"] = MultiTBuffer()
        modules.append(module)
        return module

    yield make
    for module in modules:
        cast("dict[str, Any]", module.__dict__)["_tf"] = None
        module.dispose()


def _cloud(points: list[tuple[float, float, float]], timestamp: float = 12.5) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float32).reshape((-1, 3)),
        frame_id="camera",
        timestamp=timestamp,
        intensities=np.arange(1, len(points) + 1, dtype=np.float32),
    )


def _publish_link_pose(module: PointCloudSelfFilter, timestamp: float, x: float = 0.0) -> None:
    tf = cast("MultiTBuffer", module.__dict__["_tf"])
    tf.receive_transform(
        Transform(
            translation=Vector3(x, 0.0, 0.0),
            frame_id="world",
            child_frame_id="base",
            ts=timestamp,
        ),
        Transform(frame_id="world", child_frame_id="camera", ts=timestamp),
    )


def test_model_filter_removes_robot_surface_and_preserves_external_point(
    make_filter: Callable[[], PointCloudSelfFilter],
) -> None:
    module = make_filter()
    _publish_link_pose(module, 12.5)

    result = module.filter_cloud(_cloud([(0.5, 0.0, 0.0), (0.0, 0.0, 0.0), (2.0, 0.0, 0.0)]))

    assert result is not None
    filtered, clear_mask = result
    np.testing.assert_allclose(filtered.points_f32(), [[2.0, 0.0, 0.0]])
    np.testing.assert_allclose(filtered.intensities_f32(), [3.0])
    assert clear_mask.frame_id == "world"
    assert clear_mask.ts == 12.5
    samples = clear_mask.points_f32()
    assert len(samples) > 0
    np.testing.assert_allclose(
        np.floor(samples / module.filter_config.voxel_size) + 0.5,
        samples / module.filter_config.voxel_size,
        atol=1e-5,
    )


def test_clear_mask_contains_previous_and_current_robot_volumes(
    make_filter: Callable[[], PointCloudSelfFilter],
) -> None:
    module = make_filter()
    _publish_link_pose(module, 12.5, x=0.0)
    first = module.filter_cloud(_cloud([], 12.5))
    assert first is not None
    first_keys = {tuple(key) for key in first[1].points_f32()}

    _publish_link_pose(module, 13.0, x=1.0)
    second = module.filter_cloud(_cloud([], 13.0))
    assert second is not None
    second_keys = {tuple(key) for key in second[1].points_f32()}

    assert first_keys < second_keys
    assert any(key[0] >= 0.5 for key in second_keys)


def test_missing_required_link_tf_drops_whole_capture(
    make_filter: Callable[[], PointCloudSelfFilter],
) -> None:
    module = make_filter()
    assert module.filter_cloud(_cloud([(2.0, 0.0, 0.0)])) is None


def test_self_filter_stream_types_include_independent_clear_mask() -> None:
    assert PointCloudSelfFilter.__annotations__["pointcloud"]
    assert PointCloudSelfFilter.__annotations__["filtered_pointcloud"]
    assert PointCloudSelfFilter.__annotations__["voxel_clear_mask"]


def test_transform_points_applies_rotation_and_translation() -> None:
    points = np.asarray([[1.0, 0.0, 0.0], [0.0, 2.0, 3.0]], dtype=np.float32)
    transform = np.asarray(
        [
            [0.0, -1.0, 0.0, 4.0],
            [1.0, 0.0, 0.0, 5.0],
            [0.0, 0.0, 1.0, 6.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    transformed = PointCloudSelfFilter._transform_points(points, transform)

    np.testing.assert_allclose(transformed, [[4.0, 6.0, 6.0], [2.0, 5.0, 9.0]])


def test_mesh_filter_uses_exact_mesh_surface_with_padding() -> None:
    mesh = trimesh.creation.icosphere(radius=1.0)
    geometry = _CollisionGeometry(
        link="arm",
        link_from_geometry=np.eye(4),
        mesh=mesh,
        shape="mesh",
        dimensions=(),
        clear_samples=np.empty((0, 3)),
    )
    points = np.asarray(
        [
            [0.9, 0.9, 0.0],  # Outside the sphere, inside its bounding box.
            [1.02, 0.0, 0.0],  # Included by padding.
            [1.2, 0.0, 0.0],
        ]
    )

    removed = PointCloudSelfFilter._points_inside_geometry(points, geometry, padding=0.05)

    np.testing.assert_array_equal(removed, [False, True, False])


@pytest.mark.self_hosted
def test_xarm_model_removes_base_arm_and_gripper_surfaces() -> None:
    timestamp = 20.0
    module = PointCloudSelfFilter(
        robot_model=make_xarm7_sim_robot_config(),
        padding_m=0.025,
        voxel_size=0.05,
        tf_tolerance_s=0.001,
        tf_forward_tolerance_s=0.0,
    )
    try:
        collision_links = {geometry.link for geometry in module._collision_geometry}
        assert collision_links == {
            "link_base",
            "link1",
            "link2",
            "link3",
            "link4",
            "link5",
            "link6",
            "link7",
            "xarm_gripper_base_link",
            "left_outer_knuckle",
            "left_finger",
            "left_inner_knuckle",
            "right_outer_knuckle",
            "right_finger",
            "right_inner_knuckle",
        }
        transforms = [
            Transform(frame_id="world", child_frame_id=link, ts=timestamp)
            for link in collision_links
        ]
        transforms.append(Transform(frame_id="world", child_frame_id="camera", ts=timestamp))
        tf = TF()
        tf.receive_transform(*transforms)
        cast("dict[str, Any]", module.__dict__)["_tf"] = tf

        robot_surface_points = []
        for geometry in module._collision_geometry:
            vertex = np.append(np.asarray(geometry.mesh.vertices[0], dtype=np.float64), 1.0)
            robot_surface_points.append((geometry.link_from_geometry @ vertex)[:3])
        external_point = np.array([10.0, 10.0, 10.0], dtype=np.float64)
        cloud = PointCloud2.from_numpy(
            np.asarray([*robot_surface_points, external_point], dtype=np.float32),
            frame_id="camera",
            timestamp=timestamp,
        )

        result = module.filter_cloud(cloud)

        assert result is not None
        filtered, _ = result
        np.testing.assert_allclose(filtered.points_f32(), [external_point])
    finally:
        module.dispose()
