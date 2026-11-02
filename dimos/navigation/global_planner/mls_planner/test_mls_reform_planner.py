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

from dimos.navigation.global_planner.mls_planner.mls_planner import MLSPlanner
from dimos.navigation.global_planner.mls_planner.mls_reform_planner_native import (
    MLSReformPlannerNative,
    MLSReformPlannerNativeConfig,
)

VOXEL = 0.2
POSE = np.dtype([("xyz", "<f4", 3), ("id", "<u4"), ("ts", "<f8"), ("q", "<f4", 4)])
EVENT = np.dtype(
    [
        ("xyz", "<f4", 3),
        ("id", "<u4"),
        ("ts", "<f8"),
        ("q", "<f4", 4),
        ("dt", "<f4", 3),
        ("dq", "<f4", 4),
    ]
)


def floor_with_wall() -> np.ndarray:
    coords = np.arange(-3.0, 3.0, VOXEL, dtype=np.float32) + VOXEL / 2
    xs, ys = np.meshgrid(coords, coords)
    floor = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, VOXEL / 2, np.float32)], axis=1)
    wall = np.array(
        [(0.1, y, z) for y in coords if abs(y) > 1.0 for z in coords[coords > 0][:8]], np.float32
    )
    return np.concatenate([floor, wall])


def keyframes() -> np.ndarray:
    grid = np.arange(-2.5, 3.0, 1.0)
    out = np.zeros(len(grid) ** 2, POSE)
    out["xyz"] = [(x, y, 0.3) for x in grid for y in grid]
    out["id"] = np.arange(len(out))
    out["ts"] = np.arange(len(out), dtype=float)
    out["q"] = (0, 0, 0, 1)
    return out


def shift_all(kfs: np.ndarray, dx: float, dy: float) -> bytes:
    event = np.zeros(len(kfs), EVENT)
    for name in ("xyz", "id", "ts", "q"):
        event[name] = kfs[name]
    event["dt"] = (dx, dy, 0.0)
    event["dq"] = (0, 0, 0, 1)
    return event.tobytes()


def make_planner(**kwargs) -> MLSPlanner:
    return MLSPlanner(voxel_size=VOXEL, robot_height=1.0, **kwargs)


def test_a_loop_closure_moves_the_graph_and_the_held_goal() -> None:
    world = floor_with_wall()
    planner = make_planner(voxel_times=True)
    planner.update_global_map(world, stamp=0.0)
    kfs = keyframes()
    assert planner.add_keyframes(kfs.tobytes()) == len(kfs)

    result = planner.apply_loop_closure(
        shift_all(kfs, 1.0, 0.4), held=[(2.0, 2.0, 0.2)], blend_sigma_m=1.0
    )
    assert result is not None
    assert result["keyframes_moved"] == len(kfs)
    assert result["voxels_moved"] == result["voxels_warped"] > 0
    np.testing.assert_allclose(result["held"][0], (3.0, 2.4, 0.2), atol=1e-4)

    rebuilt = make_planner()
    rebuilt.update_global_map(world + np.array([1.0, 0.4, 0.0], np.float32))
    np.testing.assert_array_equal(
        np.unique(planner.surface_map(), axis=0), np.unique(rebuilt.surface_map(), axis=0)
    )
    path = planner.plan((-1.0, -1.6, 0.2), (2.0, 2.4, 0.2))
    assert path is not None
    np.testing.assert_allclose(path[-1][:2], (2.0, 2.4), atol=0.3)


def test_a_closure_below_min_shift_is_held_for_the_next() -> None:
    planner = make_planner()
    planner.update_global_map(floor_with_wall())
    kfs = keyframes()
    planner.add_keyframes(kfs.tobytes())
    assert planner.apply_loop_closure(shift_all(kfs, 0.01, 0.0), min_shift_m=0.04) is None


def test_records_of_the_wrong_size_are_rejected() -> None:
    planner = make_planner()
    with pytest.raises(ValueError):
        planner.apply_loop_closure(b"\x00" * 67)
    with pytest.raises(ValueError):
        planner.add_keyframes(b"\x00" * 41)


def test_the_module_declares_the_closure_inputs() -> None:
    ports = MLSReformPlannerNative.__annotations__
    assert {"pose_graph", "loop_closure_event"} <= set(ports)
    config = MLSReformPlannerNativeConfig()
    assert config.executable.endswith("mls_reform_planner")
    assert {"reform_blend_sigma_m", "reform_blend_time_sigma_s", "reform_min_shift_m"} <= set(
        config.to_config_dict()
    )
