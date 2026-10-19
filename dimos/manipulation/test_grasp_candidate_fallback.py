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

import math

import numpy as np
import pytest

from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def _box_cloud() -> PointCloud2:
    """An oblong cross-section, so the narrow axis is unambiguous."""
    rng = np.random.default_rng(0)
    points = np.column_stack(
        [
            rng.uniform(-0.05, 0.05, 400),
            rng.uniform(-0.01, 0.01, 400),
            rng.uniform(0.0, 0.10, 400),
        ]
    )
    return PointCloud2.from_numpy(points, frame_id="world", timestamp=1.0)


def _yaw_of(candidate: GraspCandidate) -> float:
    return float(candidate.pose.orientation.to_euler().z)


def test_generator_offers_the_wrist_flip_and_keeps_the_narrow_axis_first() -> None:
    single = HeuristicGraspModule()
    many = HeuristicGraspModule(yaw_candidates=4)
    try:
        cloud = _box_cloud()
        one = single.propose_grasps(cloud)
        several = many.propose_grasps(cloud)

        # The default stays exactly what it was, so the xArm is unaffected.
        assert len(one.candidates) == 1
        assert len(several.candidates) > 1
        assert several.candidates[0].score == pytest.approx(1.0)
        assert _yaw_of(several.candidates[0]) == pytest.approx(_yaw_of(one.candidates[0]))
        assert [c.score for c in several.candidates] == sorted(
            (c.score for c in several.candidates), reverse=True
        )
        # A half turn is the same physical grasp for parallel jaws, so the
        # oblong object gets it and nothing that grasps across the wide axis.
        deltas = [
            abs((_yaw_of(c) - _yaw_of(one.candidates[0]) + math.pi) % (2 * math.pi) - math.pi)
            for c in several.candidates[1:]
        ]
        assert deltas and all(delta == pytest.approx(math.pi, abs=1e-6) for delta in deltas)
    finally:
        single.stop()
        many.stop()
