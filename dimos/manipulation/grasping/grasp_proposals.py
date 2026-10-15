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

"""Grasp proposals an agent reads and chooses from, for objects localize has reported."""

from __future__ import annotations

from collections.abc import Sequence
import math
from typing import Protocol

import numpy as np
from pydantic import Field

from dimos.agents.annotation import skill
from dimos.core.module import Module, ModuleConfig
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import top_down_grasps
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.utils import Spec


class ObjectCloudSpec(Spec, Protocol):
    def object_cloud(self, object_id: str) -> PointCloud2 | None: ...


class ReachabilitySpec(Spec, Protocol):
    def check_reachable(
        self,
        poses: Sequence[PoseStamped],
        planning_group: PlanningGroupID | None = None,
    ) -> list[str]: ...


class GraspProposalsConfig(ModuleConfig):
    planning_frame: str = "world"
    # How far back along the approach the pre-grasp pose sits, metres.
    pregrasp_offset: float = Field(default=0.10, gt=0.0)
    # Widest object the open jaws close on, metres, for the heuristic.
    max_jaw_span: float = Field(default=0.085, gt=0.0)
    # Least depth below the object's top a heuristic grasp goes, metres.
    grasp_depth: float = Field(default=0.02, ge=0.0)
    # How many of a proposer's best grasps are checked for reach; the listed ones come from these.
    check_count: int = Field(default=30, gt=0)


class GraspProposals(Module):
    """Ranked, reachability-checked grasp poses for a localized object, from the top-down
    heuristic or GraspGenX when it is in the blueprint. The agent chooses and executes."""

    config: GraspProposalsConfig
    _localize: ObjectCloudSpec
    _reachability: ReachabilitySpec
    _graspgenx: GraspGenXModule | None = None

    @skill
    def grasps(self, object_id: str, proposer: str = "heuristic", count: int = 5) -> str:
        """Grasp poses for an object localize reported: the best ones IK reaches, best first.

        proposer "heuristic": straight down at the centre of the object's footprint, jaws
        across its narrow side; one or two grasps, instant. Suits small box-like objects.
        proposer "graspgenx": a learned model's grasps from any direction, ranked by its
        confidence; suits other shapes, rims and handles. Both read only the surfaces the
        camera saw.

        Each grasp gives the tool centre point (TCP) position and orientation to pass to
        arm.move_pose, the direction the gripper approaches along, a pre-grasp pose back
        along that approach, the object's width between the jaws and the proposer's score.
        IK must reach the grasp and the pre-grasp from the current joints; collisions are not
        checked. The proposals do not know the task; choose the one that suits it.

        Args:
            object_id: The id localize printed for the object.
            proposer: "heuristic" or "graspgenx".
            count: How many reachable grasps to list at most.
        """
        cloud = self._localize.object_cloud(object_id)
        if cloud is None:
            return (
                f"No point cloud for id {object_id!r}. Pass an id from a localize result; "
                "localize keeps the cloud of every object it reports."
            )
        if proposer == "heuristic":
            proposals = top_down_grasps(cloud, self.config.max_jaw_span, self.config.grasp_depth)
        elif proposer == "graspgenx":
            if self._graspgenx is None:
                return 'graspgenx is not running in this robot\'s stack; use proposer="heuristic".'
            proposals = self._graspgenx.propose_grasps(cloud)
        else:
            return f'Unknown proposer {proposer!r}; use "heuristic" or "graspgenx".'
        frame = proposals.header.frame_id
        if frame != self.config.planning_frame:
            return f"The grasps came back in frame {frame!r}, not {self.config.planning_frame!r}."
        if not proposals.candidates:
            return f"{proposer} proposed no grasps for {object_id}."
        return self._describe(object_id, proposer, cloud, proposals, max(count, 1))

    def _describe(
        self,
        object_id: str,
        proposer: str,
        cloud: PointCloud2,
        proposals: GraspCandidateArray,
        count: int,
    ) -> str:
        checked = proposals.candidates[: self.config.check_count]
        grasps = [
            PoseStamped(
                frame_id=proposals.header.frame_id,
                position=c.pose.position,
                orientation=c.pose.orientation,
            )
            for c in checked
        ]
        approaches = [g.orientation.rotate_vector(Vector3(0.0, 0.0, 1.0)) for g in grasps]
        pregrasps = [
            PoseStamped(
                frame_id=g.frame_id,
                position=g.position + a * -self.config.pregrasp_offset,
                orientation=g.orientation,
            )
            for g, a in zip(grasps, approaches, strict=True)
        ]
        reasons = self._reachability.check_reachable([*grasps, *pregrasps])
        n = len(checked)
        reachable = [i for i in range(n) if not reasons[i] and not reasons[n + i]]
        points = cloud.points_f32()
        bottom, top = np.quantile(points[:, 2], [0.05, 0.95])
        header = (
            f"{proposer}: {len(proposals.candidates)} grasp(s) for {object_id}, the best {n} "
            f"checked, {len(reachable)} reachable; cloud of {len(points)} points, surfaces seen "
            f"from z {bottom:.3f} to {top:.3f}."
        )
        if not reachable:
            first = reasons[0] or reasons[n]
            return f"{header} None reachable from the current joints (first: {first})."
        lines = [header + " Reachable, best first:"]
        for rank, i in enumerate(reachable[:count], start=1):
            grasp, approach = grasps[i], approaches[i]
            jaw = grasp.orientation.rotate_vector(Vector3(0.0, 1.0, 0.0))
            along_jaw = points @ np.asarray([jaw.x, jaw.y, jaw.z], dtype=np.float32)
            width = float(np.quantile(along_jaw, 0.98) - np.quantile(along_jaw, 0.02))
            lines.append(
                f"{rank}. position {_xyz(grasp.position)} orientation {_xyzw(grasp)}; "
                f"approach {_approach(approach)}, jaw yaw "
                f"{math.degrees(math.atan2(jaw.y, jaw.x)):.0f}deg, object width between the "
                f"jaws {width:.3f} m, score {checked[i].score:.2f}; "
                f"pre-grasp {_xyz(pregrasps[i].position)}"
            )
        return "\n".join(lines)


def _xyz(v: Vector3) -> str:
    return f"[{v.x:.3f}, {v.y:.3f}, {v.z:.3f}]"


def _xyzw(pose: PoseStamped) -> str:
    return "[" + ", ".join(f"{c:.4f}" for c in pose.orientation.to_tuple()) + "]"


def _approach(v: Vector3) -> str:
    if v.z < -0.97:
        return "straight down"
    return f"along {_xyz(v)}"
