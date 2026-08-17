from __future__ import annotations

from pathlib import Path
from typing import Any, cast

import numpy as np
import pytest

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.point_cloud_self_filter import (
    PointCloudSelfFilter,
    PointCloudSelfFilterConfig,
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


def _filter(tmp_path: Path) -> PointCloudSelfFilter:
    urdf = tmp_path / "robot.urdf"
    _write_box_robot(urdf)
    module = object.__new__(PointCloudSelfFilter)
    state = cast("dict[str, Any]", module.__dict__)
    state["config"] = PointCloudSelfFilterConfig(
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
    state["_tf"] = MultiTBuffer()
    state["_collision_geometry"] = module._load_collision_geometry()
    state["_previous_clear_keys"] = set()
    return module


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


def test_model_filter_removes_robot_surface_and_preserves_external_point(tmp_path: Path) -> None:
    module = _filter(tmp_path)
    _publish_link_pose(module, 12.5)

    result = module.filter_cloud(_cloud([(0.5, 0.0, 0.0), (0.0, 0.0, 0.0), (2.0, 0.0, 0.0)]))

    assert result is not None
    filtered, clear_mask = result
    np.testing.assert_allclose(filtered.points_f32(), [[2.0, 0.0, 0.0]])
    np.testing.assert_allclose(filtered.intensities_f32(), [3.0])
    assert clear_mask.frame_id == "world"
    assert clear_mask.ts == 12.5
    keys = clear_mask.points_f32()
    assert len(keys) > 0
    np.testing.assert_array_equal(keys, np.floor(keys))


def test_clear_mask_contains_previous_and_current_robot_volumes(tmp_path: Path) -> None:
    module = _filter(tmp_path)
    _publish_link_pose(module, 12.5, x=0.0)
    first = module.filter_cloud(_cloud([], 12.5))
    assert first is not None
    first_keys = {tuple(key) for key in first[1].points_f32()}

    _publish_link_pose(module, 13.0, x=1.0)
    second = module.filter_cloud(_cloud([], 13.0))
    assert second is not None
    second_keys = {tuple(key) for key in second[1].points_f32()}

    assert first_keys < second_keys
    assert any(key[0] >= 10 for key in second_keys)


def test_missing_required_link_tf_drops_whole_capture(tmp_path: Path) -> None:
    module = _filter(tmp_path)
    assert module.filter_cloud(_cloud([(2.0, 0.0, 0.0)])) is None


def test_self_filter_stream_types_include_atomic_clear_mask() -> None:
    assert PointCloudSelfFilter.__annotations__["pointcloud"]
    assert PointCloudSelfFilter.__annotations__["filtered_pointcloud"]
    assert PointCloudSelfFilter.__annotations__["robot_clear_mask"]


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
        module.stop()
