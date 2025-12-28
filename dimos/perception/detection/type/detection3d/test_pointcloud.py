# Copyright 2025 Dimensional Inc.
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

import numpy as np
import pytest


def test_detection3dpc(detection3dpc) -> None:
    # def test_oriented_bounding_box(detection3dpc):
    """Test oriented bounding box calculation and values."""
    obb = detection3dpc.get_oriented_bounding_box()
    assert obb is not None, "Oriented bounding box should not be None"

    # Verify OBB center values
    assert obb.center[0] == pytest.approx(-3.36002, abs=0.1)
    assert obb.center[1] == pytest.approx(-0.196446, abs=0.1)
    assert obb.center[2] == pytest.approx(0.220184, abs=0.1)

    # Verify OBB extent values
    assert obb.extent[0] == pytest.approx(0.531275, abs=0.12)
    assert obb.extent[1] == pytest.approx(0.461054, abs=0.1)
    assert obb.extent[2] == pytest.approx(0.155, abs=0.1)

    # def test_bounding_box_dimensions(detection3dpc):
    """Test bounding box dimension calculation."""
    dims = detection3dpc.get_bounding_box_dimensions()
    assert len(dims) == 3, "Bounding box dimensions should have 3 values"
    assert dims[0] == pytest.approx(0.350, abs=0.1)
    assert dims[1] == pytest.approx(0.250, abs=0.1)
    assert dims[2] == pytest.approx(0.550, abs=0.1)

    # def test_axis_aligned_bounding_box(detection3dpc):
    """Test axis-aligned bounding box calculation."""
    aabb = detection3dpc.get_bounding_box()
    assert aabb is not None, "Axis-aligned bounding box should not be None"

    # Verify AABB min values
    assert aabb.min_bound[0] == pytest.approx(-3.575, abs=0.1)
    assert aabb.min_bound[1] == pytest.approx(-0.375, abs=0.1)
    assert aabb.min_bound[2] == pytest.approx(-0.075, abs=0.1)

    # Verify AABB max values
    assert aabb.max_bound[0] == pytest.approx(-3.075, abs=0.1)
    assert aabb.max_bound[1] == pytest.approx(-0.125, abs=0.1)
    assert aabb.max_bound[2] == pytest.approx(0.475, abs=0.1)

    # def test_point_cloud_properties(detection3dpc):
    """Test point cloud data and boundaries."""
    points = detection3dpc.pointcloud.as_numpy()
    assert len(points) > 60
    assert detection3dpc.pointcloud.frame_id == "world", (
        f"Expected frame_id 'world', got '{detection3dpc.pointcloud.frame_id}'"
    )

    min_pt = np.min(points, axis=0)
    max_pt = np.max(points, axis=0)
    center = np.mean(points, axis=0)

    # Verify point cloud boundaries
    assert min_pt[0] == pytest.approx(-3.575, abs=0.1)
    assert min_pt[1] == pytest.approx(-0.375, abs=0.1)
    assert min_pt[2] == pytest.approx(-0.075, abs=0.1)

    assert max_pt[0] == pytest.approx(-3.075, abs=0.1)
    assert max_pt[1] == pytest.approx(-0.125, abs=0.1)
    assert max_pt[2] == pytest.approx(0.475, abs=0.1)

    assert center[0] == pytest.approx(-3.326, abs=0.1)
    assert center[1] == pytest.approx(-0.202, abs=0.1)
    assert center[2] == pytest.approx(0.160, abs=0.1)

    # def test_foxglove_scene_entity_generation(detection3dpc):
    """Test Foxglove scene entity creation and structure."""
    entity = detection3dpc.to_foxglove_scene_entity("test_entity_123")

    # Verify entity metadata
    assert entity.id == "1", f"Expected entity ID '1', got '{entity.id}'"
    assert entity.frame_id == "world", f"Expected frame_id 'world', got '{entity.frame_id}'"
    assert entity.cubes_length == 1, f"Expected 1 cube, got {entity.cubes_length}"
    assert entity.texts_length == 1, f"Expected 1 text, got {entity.texts_length}"

    # def test_foxglove_cube_properties(detection3dpc):
    """Test Foxglove cube primitive properties."""
    entity = detection3dpc.to_foxglove_scene_entity("test_entity_123")
    cube = entity.cubes[0]

    # Verify position
    assert cube.pose.position.x == pytest.approx(-3.325, abs=0.1)
    assert cube.pose.position.y == pytest.approx(-0.250, abs=0.1)
    assert cube.pose.position.z == pytest.approx(0.200, abs=0.1)

    # Verify size
    assert cube.size.x == pytest.approx(0.350, abs=0.1)
    assert cube.size.y == pytest.approx(0.250, abs=0.1)
    assert cube.size.z == pytest.approx(0.550, abs=0.1)

    # Verify color (green with alpha)
    assert cube.color.r == pytest.approx(0.08235294117647059, abs=0.1)
    assert cube.color.g == pytest.approx(0.7176470588235294, abs=0.1)
    assert cube.color.b == pytest.approx(0.28627450980392155, abs=0.1)
    assert cube.color.a == pytest.approx(0.2, abs=0.1)

    # def test_foxglove_text_label(detection3dpc):
    """Test Foxglove text label properties."""
    entity = detection3dpc.to_foxglove_scene_entity("test_entity_123")
    text = entity.texts[0]

    assert text.text in ["1/suitcase (81%)", "1/suitcase (82%)"], (
        f"Expected text '1/suitcase (81%)' or '1/suitcase (82%)', got '{text.text}'"
    )
    assert text.pose.position.x == pytest.approx(-3.325, abs=0.1)
    assert text.pose.position.y == pytest.approx(-0.250, abs=0.1)
    assert text.pose.position.z == pytest.approx(0.575, abs=0.1)
    assert text.font_size == 20.0, f"Expected font size 20.0, got {text.font_size}"

    # def test_detection_pose(detection3dpc):
    """Test detection pose and frame information."""
    assert detection3dpc.pose.x == pytest.approx(-3.327, abs=0.1)
    assert detection3dpc.pose.y == pytest.approx(-0.202, abs=0.1)
    assert detection3dpc.pose.z == pytest.approx(0.160, abs=0.1)
    assert detection3dpc.pose.frame_id == "world", (
        f"Expected frame_id 'world', got '{detection3dpc.pose.frame_id}'"
    )
