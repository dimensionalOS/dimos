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

import numpy as np
import pytest

pytest.importorskip("dimos_mls_planner")

from dimos.navigation.global_planner.mls_planner.mls_planner import (
    LOOP_CLOSURE_EVENT_DTYPE,
    POSE_GRAPH_DTYPE,
    MLSPlanner,
)

VOXEL_SIZE = 0.2


def floor_with_wall():
    coords = np.arange(-3.0, 3.0, VOXEL_SIZE, dtype=np.float32) + VOXEL_SIZE / 2
    xs, ys = np.meshgrid(coords, coords)
    floor = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, VOXEL_SIZE / 2, np.float32)], axis=1)
    wall = np.array(
        [(0.1, y, z) for y in coords if abs(y) > 1.0 for z in coords[coords > 0][:8]], np.float32
    )
    return np.concatenate([floor, wall])


def keyframes():
    grid = np.arange(-2.5, 3.0, 1.0)
    records = np.zeros(len(grid) ** 2, POSE_GRAPH_DTYPE)
    records["x"], records["y"] = np.repeat(grid, len(grid)), np.tile(grid, len(grid))
    records["z"] = 0.3
    records["node_id"] = np.arange(len(records))
    records["ts"] = np.arange(len(records), dtype=float)
    records["qw"] = 1.0
    return records


def shift_all(keyframe_records, dx, dy):
    event = np.zeros(len(keyframe_records), LOOP_CLOSURE_EVENT_DTYPE)
    for name in keyframe_records.dtype.names:
        event[name] = keyframe_records[name]
    event["dx"], event["dy"], event["dqw"] = dx, dy, 1.0
    return event.tobytes()


def make_planner(**kwargs):
    return MLSPlanner(voxel_size=VOXEL_SIZE, robot_height=1.0, **kwargs)


def test_a_loop_closure_moves_the_graph_and_the_held_goal():
    world = floor_with_wall()
    planner = make_planner(voxel_times=True)
    planner.update_global_map(world, stamp=0.0)
    keyframe_records = keyframes()
    assert planner.add_keyframes(keyframe_records.tobytes()) == len(keyframe_records)

    result = planner.apply_loop_closure(
        shift_all(keyframe_records, 1.0, 0.4), held=[(2.0, 2.0, 0.2)], blend_sigma_m=1.0
    )
    assert result is not None
    assert result["keyframes_moved"] == len(keyframe_records)
    assert result["voxels_warped"] == planner.voxel_count() > 0
    np.testing.assert_allclose(result["held"][0], (3.0, 2.4, 0.2), atol=1e-4)

    rebuilt = make_planner()
    rebuilt.update_global_map(world + np.array([1.0, 0.4, 0.0], np.float32))
    np.testing.assert_array_equal(
        np.unique(planner.surface_map(), axis=0), np.unique(rebuilt.surface_map(), axis=0)
    )
    path = planner.plan((-1.0, -1.6, 0.2), (2.0, 2.4, 0.2))
    assert path is not None
    np.testing.assert_allclose(path[-1][:2], (2.0, 2.4), atol=0.3)


def test_a_closure_below_min_shift_is_held_for_the_next():
    planner = make_planner()
    planner.update_global_map(floor_with_wall())
    keyframe_records = keyframes()
    planner.add_keyframes(keyframe_records.tobytes())
    assert (
        planner.apply_loop_closure(shift_all(keyframe_records, 0.01, 0.0), min_shift_m=0.04) is None
    )


def test_records_of_the_wrong_size_are_rejected():
    planner = make_planner()
    with pytest.raises(ValueError):
        planner.apply_loop_closure(b"\x00" * (np.dtype(LOOP_CLOSURE_EVENT_DTYPE).itemsize - 1))
    with pytest.raises(ValueError):
        planner.add_keyframes(b"\x00" * (np.dtype(POSE_GRAPH_DTYPE).itemsize + 1))
