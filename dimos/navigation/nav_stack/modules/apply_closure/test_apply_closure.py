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

import math

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.DynamicCloud import DynamicCloud
from dimos.msgs.nav_msgs.Graph3D import Graph3D
from dimos.msgs.nav_msgs.GraphDelta3D import GraphDelta3D
from dimos.navigation.nav_stack.modules.apply_closure.apply_closure import (
    apply_closure_to_cloud,
    graph_delta_to_arrays,
    lbs_warp_positions,
    merge_duplicate_voxels,
    transform_to_matrix,
)


def _pose(ts, x=0.0, y=0.0, z=0.0, yaw=0.0):
    quat = Rotation.from_euler("z", yaw).as_quat()
    return PoseStamped(
        ts=ts,
        frame_id="map",
        position=[x, y, z],
        orientation=[quat[0], quat[1], quat[2], quat[3]],
    )


def _node(ts, x=0.0, y=0.0, z=0.0, yaw=0.0, node_id=0):
    return Graph3D.Node3D(pose=_pose(ts, x, y, z, yaw), id=node_id, metadata_id=0)


def _transform(tx=0.0, ty=0.0, tz=0.0, yaw=0.0):
    quat = Rotation.from_euler("z", yaw).as_quat()
    return GraphDelta3D.Transform(
        translation=Vector3(tx, ty, tz),
        rotation=Quaternion(quat[0], quat[1], quat[2], quat[3]),
    )


def _delta(*pairs):
    """Build a GraphDelta3D from an iterable of (node, transform) pairs."""
    nodes = [pair[0] for pair in pairs]
    transforms = [pair[1] for pair in pairs]
    return GraphDelta3D(ts=1.0, nodes=nodes, transforms=transforms)


class TestTransformHelpers:
    def test_transform_to_matrix_identity(self):
        identity = GraphDelta3D.Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        np.testing.assert_allclose(transform_to_matrix(identity), np.eye(4), atol=1e-12)

    def test_transform_to_matrix_translation_only(self):
        t = GraphDelta3D.Transform(
            translation=Vector3(1.0, 2.0, 3.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        matrix = transform_to_matrix(t)
        np.testing.assert_allclose(matrix[:3, :3], np.eye(3))
        np.testing.assert_allclose(matrix[:3, 3], [1.0, 2.0, 3.0])

    def test_graph_delta_to_arrays(self):
        delta = _delta(
            (_node(1.0, 2.0, 3.0, 4.0), _transform(0.5, 0.0, 0.0)),
            (_node(5.0, 6.0, 7.0, 8.0), _transform(0.0, 0.7, 0.0)),
        )
        timestamps, deltas = graph_delta_to_arrays(delta)
        np.testing.assert_array_equal(timestamps, [1.0, 5.0])
        assert deltas.shape == (2, 4, 4)
        np.testing.assert_allclose(deltas[0, :3, 3], [0.5, 0.0, 0.0])
        np.testing.assert_allclose(deltas[1, :3, 3], [0.0, 0.7, 0.0])


class TestLBSWarp:
    def test_empty_positions_returns_empty(self):
        out = lbs_warp_positions(
            np.zeros((0, 3)),
            np.zeros(0),
            np.array([0.0, 1.0]),
            np.stack([np.eye(4), np.eye(4)]),
        )
        assert out.shape == (0, 3)

    def test_no_nodes_passes_through(self):
        positions = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        out = lbs_warp_positions(positions, np.array([1.0, 2.0]), np.zeros(0), np.zeros((0, 4, 4)))
        np.testing.assert_allclose(out, positions)

    def test_single_node_applies_rigidly(self):
        delta = np.eye(4)
        delta[:3, 3] = [10.0, 0.0, 0.0]
        positions = np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        out = lbs_warp_positions(
            positions, np.array([0.0, 100.0]), np.array([0.0]), delta[None, :, :]
        )
        np.testing.assert_allclose(out, positions + np.array([10.0, 0.0, 0.0]))

    def test_before_range_clips_to_first_node(self):
        deltas = np.stack([np.eye(4), np.eye(4)])
        deltas[0, :3, 3] = [1.0, 0.0, 0.0]
        deltas[1, :3, 3] = [10.0, 0.0, 0.0]
        positions = np.array([[0.0, 0.0, 0.0]])
        # point time well before node[0] should snap to delta[0]
        out = lbs_warp_positions(positions, np.array([-100.0]), np.array([0.0, 1.0]), deltas)
        np.testing.assert_allclose(out, [[1.0, 0.0, 0.0]])

    def test_after_range_clips_to_last_node(self):
        deltas = np.stack([np.eye(4), np.eye(4)])
        deltas[0, :3, 3] = [1.0, 0.0, 0.0]
        deltas[1, :3, 3] = [10.0, 0.0, 0.0]
        positions = np.array([[0.0, 0.0, 0.0]])
        out = lbs_warp_positions(positions, np.array([1e9]), np.array([0.0, 1.0]), deltas)
        np.testing.assert_allclose(out, [[10.0, 0.0, 0.0]])

    def test_midpoint_translation_lerps(self):
        deltas = np.stack([np.eye(4), np.eye(4)])
        deltas[0, :3, 3] = [0.0, 0.0, 0.0]
        deltas[1, :3, 3] = [10.0, 0.0, 0.0]
        positions = np.array([[0.0, 0.0, 0.0]])
        out = lbs_warp_positions(positions, np.array([0.5]), np.array([0.0, 1.0]), deltas)
        np.testing.assert_allclose(out, [[5.0, 0.0, 0.0]])

    def test_midpoint_rotation_slerps(self):
        deltas = np.stack([np.eye(4), np.eye(4)])
        deltas[1, :3, :3] = Rotation.from_euler("z", math.pi / 2).as_matrix()
        # A point at (1, 0, 0) rotated by 45deg should land at (cos45, sin45, 0)
        out = lbs_warp_positions(
            np.array([[1.0, 0.0, 0.0]]),
            np.array([0.5]),
            np.array([0.0, 1.0]),
            deltas,
        )
        np.testing.assert_allclose(
            out, [[math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0]], atol=1e-9
        )


class TestMergeDuplicates:
    def test_no_duplicates_passes_through(self):
        voxels = np.array([[0, 0, 0], [1, 1, 1]], dtype=np.int32)
        quantity = np.array([2, 3], dtype=np.uint32)
        events = np.array([0, 1, 1], dtype=np.uint32)
        unique_voxels, merged_quantity, remapped_events = merge_duplicate_voxels(
            voxels, quantity, events
        )
        # np.unique sorts lexicographically — order may differ but contents must match
        assert unique_voxels.shape == (2, 3)
        assert int(merged_quantity.sum()) == 5
        # Build old → new index map and verify events were remapped correctly
        old_to_new = {}
        for old_i, original in enumerate(voxels):
            matches = np.where((unique_voxels == original).all(axis=1))[0]
            assert matches.size == 1
            old_to_new[old_i] = int(matches[0])
        expected_events = np.array([old_to_new[int(idx)] for idx in events], dtype=np.uint32)
        np.testing.assert_array_equal(remapped_events, expected_events)

    def test_collision_sums_quantity(self):
        voxels = np.array([[0, 0, 0], [0, 0, 0], [1, 0, 0]], dtype=np.int32)
        quantity = np.array([5, 7, 9], dtype=np.uint32)
        events = np.array([0, 1, 2], dtype=np.uint32)
        unique_voxels, merged_quantity, remapped_events = merge_duplicate_voxels(
            voxels, quantity, events
        )
        assert unique_voxels.shape == (2, 3)
        zero_row = np.where((unique_voxels == [0, 0, 0]).all(axis=1))[0][0]
        one_row = np.where((unique_voxels == [1, 0, 0]).all(axis=1))[0][0]
        assert int(merged_quantity[zero_row]) == 12
        assert int(merged_quantity[one_row]) == 9
        # The two events that referenced (0,0,0) should now reference zero_row
        assert int(remapped_events[0]) == zero_row
        assert int(remapped_events[1]) == zero_row
        assert int(remapped_events[2]) == one_row

    def test_empty_inputs(self):
        unique_voxels, merged_quantity, remapped_events = merge_duplicate_voxels(
            np.zeros((0, 3), dtype=np.int32),
            np.zeros(0, dtype=np.uint32),
            np.zeros(0, dtype=np.uint32),
        )
        assert unique_voxels.shape == (0, 3)
        assert merged_quantity.shape == (0,)
        assert remapped_events.shape == (0,)


class TestApplyClosureToCloud:
    def test_empty_graph_delta_returns_input(self):
        cloud = DynamicCloud(
            voxels=np.array([[1, 2, 3]], dtype=np.int32),
            quantity=np.array([1], dtype=np.uint32),
            voxel_size=0.5,
        )
        out = apply_closure_to_cloud(cloud, _delta())
        assert out is cloud

    def test_identity_deltas_preserve_voxels(self):
        cloud = DynamicCloud(
            voxels=np.array([[1, 0, 0], [-2, 3, 1]], dtype=np.int32),
            quantity=np.array([4, 5], dtype=np.uint32),
            voxel_size=0.5,
        )
        delta = _delta(
            (_node(1.0), _transform()),
            (_node(10.0), _transform()),
        )
        out = apply_closure_to_cloud(cloud, delta)
        # Sort both for comparison since merge_duplicates may reorder
        np.testing.assert_array_equal(np.sort(out.voxels, axis=0), np.sort(cloud.voxels, axis=0))
        assert int(out.quantity.sum()) == int(cloud.quantity.sum())

    def test_uniform_translation_shifts_whole_cloud(self):
        """All nodes carry the same translation delta → entire cloud shifts by it."""
        cloud = DynamicCloud(
            voxels=np.array([[2, 0, 0], [4, 0, 0]], dtype=np.int32),
            quantity=np.array([1, 1], dtype=np.uint32),
            voxel_size=0.5,
        )
        delta = _delta(
            (_node(1.0), _transform(tx=1.0)),
            (_node(2.0), _transform(tx=1.0)),
        )
        out = apply_closure_to_cloud(cloud, delta)
        # World positions: (1.0, 0, 0) and (2.0, 0, 0). +1m → (2,0,0), (3,0,0).
        # voxel_size = 0.5, so voxels should be (4, 0, 0) and (6, 0, 0).
        sorted_out = np.sort(out.voxels, axis=0)
        np.testing.assert_array_equal(sorted_out, np.array([[4, 0, 0], [6, 0, 0]]))

    def test_recent_voxel_follows_latest_node_correction(self):
        """A voxel with a recent event timestamp warps by the late-node delta.

        Older voxel (no event, effective ts=0) clips to the early node which has
        zero correction; recent voxel clips to the late node which has a +5m shift.

        Note: PoseStamped maps ts=0 to time.time() as a "missing" sentinel, so
        we use ts >= 1.0 throughout to keep the pose-graph timeline well-defined.
        """
        cloud = DynamicCloud(
            voxels=np.array([[0, 0, 0], [10, 0, 0]], dtype=np.int32),
            quantity=np.array([1, 1], dtype=np.uint32),
            event_indices=np.array([1], dtype=np.uint32),
            event_timestamps=np.array([100 * 1_000_000_000], dtype=np.uint64),
            voxel_size=1.0,
        )
        delta = _delta(
            (_node(1.0), _transform()),
            (_node(100.0), _transform(tx=5.0)),
        )
        out = apply_closure_to_cloud(cloud, delta)
        # Voxel 0 (no event → ts=0 → clipped to first node, ts=1 → identity delta): (0,0,0)
        # Voxel 1 (event_ts=100s → clipped to last node → +5m): (10,0,0) → (15,0,0)
        sorted_out = np.sort(out.voxels, axis=0)
        np.testing.assert_array_equal(sorted_out, np.array([[0, 0, 0], [15, 0, 0]]))
