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

from collections.abc import Iterator, Sequence
import math

import numpy as np
import pytest

from dimos.manipulation.grasping.grasp_proposals import (
    GraspProposals,
    ObjectCloudSpec,
    ReachabilitySpec,
)
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.perception.localize.module import PerceptionLocalize
from dimos.spec.utils import spec_annotation_compliance


def _block_cloud() -> PointCloud2:
    """A 4 x 4 x 4 cm block at (0.5, 0.1), its top at z 0.85."""
    rng = np.random.default_rng(0)
    points = rng.uniform(-0.02, 0.02, (300, 3)) + np.asarray([0.5, 0.1, 0.83])
    return PointCloud2.from_numpy(points.astype(np.float32), frame_id="world", timestamp=1.0)


class FakeLocalize:
    def __init__(self, clouds: dict[str, PointCloud2]) -> None:
        self.clouds = clouds

    def object_cloud(self, object_id: str) -> PointCloud2 | None:
        return self.clouds.get(object_id)


class FakeReachability:
    def __init__(self, unreachable_above_z: float = math.inf) -> None:
        self.unreachable_above_z = unreachable_above_z
        self.asked: list[PoseStamped] = []

    def check_reachable(
        self, poses: Sequence[PoseStamped], planning_group: str | None = None
    ) -> list[str]:
        self.asked.extend(poses)
        return [
            "IK failed: NO_SOLUTION" if p.position.z > self.unreachable_above_z else ""
            for p in poses
        ]


class FakeGraspGenX:
    """40 side grasps along +x, ranked; the even-ranked ones sit too high to reach."""

    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        side = Quaternion.from_euler(Vector3(0.0, math.pi / 2, 0.0))  # approach along +x
        return GraspCandidateArray(
            Header(1.0, "world"),
            [
                GraspCandidate(
                    Pose(Vector3(0.48, 0.1, 2.0 if i % 2 == 0 else 0.83), side), 1 - i / 100
                )
                for i in range(40)
            ],
        )


@pytest.fixture
def proposals() -> Iterator[GraspProposals]:
    module = GraspProposals()
    module._localize = FakeLocalize({"blk": _block_cloud()})
    module._reachability = FakeReachability()
    yield module
    module.stop()


def test_the_modules_it_reads_from_meet_its_specs() -> None:
    assert spec_annotation_compliance(PerceptionLocalize, ObjectCloudSpec)
    assert spec_annotation_compliance(ManipulationModule, ReachabilitySpec)


def test_heuristic_lists_a_top_down_grasp_ready_for_move_pose(proposals: GraspProposals) -> None:
    lines = proposals.grasps("blk").splitlines()

    assert lines[0].startswith("heuristic: 1 grasp(s) for blk, the best 1 checked, 1 reachable")
    # 2 cm below the top (z 0.848), just under half way up.
    assert lines[1].startswith("1. position [0.500, 0.100, 0.828]")
    assert "approach straight down" in lines[1]
    assert "object width between the jaws 0.03" in lines[1]  # 4 cm, trimmed 2% each side
    assert lines[1].endswith("pre-grasp [0.500, 0.100, 0.928]")


def test_graspgenx_checks_thirty_and_lists_the_best_reachable(proposals: GraspProposals) -> None:
    proposals._graspgenx = FakeGraspGenX()  # type: ignore[assignment]
    proposals._reachability = FakeReachability(unreachable_above_z=1.5)

    lines = proposals.grasps("blk", proposer="graspgenx").splitlines()

    assert lines[0].startswith("graspgenx: 40 grasp(s) for blk, the best 30 checked, 15 reachable")
    assert len(lines) == 6
    assert [line.split("score ")[1][:4] for line in lines[1:]] == [
        "0.99", "0.97", "0.95", "0.93", "0.91"
    ]  # fmt: skip
    assert "approach along [1.000, 0.000, " in lines[1]
    assert "pre-grasp [0.380, 0.100, 0.830]" in lines[1]
    assert len(proposals._reachability.asked) == 60  # type: ignore[attr-defined]


def test_none_reachable_says_why(proposals: GraspProposals) -> None:
    proposals._reachability = FakeReachability(unreachable_above_z=0.5)

    text = proposals.grasps("blk")

    assert text.endswith("None reachable from the current joints (first: IK failed: NO_SOLUTION).")


def test_graspgenx_not_in_the_stack_points_to_the_heuristic(proposals: GraspProposals) -> None:
    assert 'use proposer="heuristic"' in proposals.grasps("blk", proposer="graspgenx")


def test_an_unknown_proposer_names_both(proposals: GraspProposals) -> None:
    assert 'use "heuristic" or "graspgenx"' in proposals.grasps("blk", proposer="ml")


def test_an_unknown_id_points_back_to_localize(proposals: GraspProposals) -> None:
    assert "Pass an id from a localize result" in proposals.grasps("nope")
