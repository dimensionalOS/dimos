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
from dimos.manipulation.grasping import grasp_filters
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXError, GraspGenXModule
from dimos.manipulation.grasping.grasp_gen_x.remote import RemoteGraspGenX
from dimos.manipulation.grasping.heuristic_grasp import top_down_grasps
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.utils import Spec


class ObjectCloudSpec(Spec, Protocol):
    def object_cloud(self, object_id: str) -> PointCloud2 | None: ...

    def scene_cloud(self, object_id: str, margin: float = 0.15) -> PointCloud2 | None: ...


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
    # A shared GraspGenX server (graspgenx_runtime.server) for proposer "graspgenx" when no
    # GraspGenX module is in the stack, e.g. GRASPPROPOSALS__GRASPGENX_URL=http://host:8701.
    # Empty: no server. The seed makes the same cloud give the same grasps; None leaves it unseeded.
    graspgenx_url: str = ""
    graspgenx_seed: int | None = 0
    # GraspGenX grasps are checked against the scene around the object (localize's scene_cloud):
    # gripper samples within this of a scene point collide (metres).
    collision_m: float = Field(default=grasp_filters.COLLISION_M, gt=0.0)
    # A cloud with fewer points than this gets a hint to look from another angle.
    sparse_cloud_points: int = Field(default=60, gt=0)


class GraspProposals(Module):
    """Ranked, reachability-checked grasp poses for a localized object, from the top-down
    heuristic or GraspGenX when it is in the blueprint. The agent chooses and executes."""

    config: GraspProposalsConfig
    _localize: ObjectCloudSpec
    _reachability: ReachabilitySpec
    _graspgenx: GraspGenXModule | None = None

    @skill
    def grasps(
        self,
        object_id: str,
        proposer: str = "heuristic",
        count: int = 5,
        max_tilt: float = 180.0,
    ) -> str:
        """Grasp poses for an object localize reported: the best ones IK reaches, best first.

        proposer "heuristic": straight down at the centre of the object's footprint, jaws
        across its narrow side; one or two grasps, instant. Suits small box-like objects. An
        open container wider than the jaws (a bin, a tray) is grasped on its walls instead:
        straight down, jaws across one wall, a little below its rim, long walls first.
        proposer "graspgenx": a learned model's grasps from any direction, ranked by its
        confidence; suits other shapes, rims and handles. Its grasps are checked against what
        the camera saw around the object: those whose gripper would hit it, or go below the
        table, are dropped and counted. Both read only the surfaces the camera saw.

        Each grasp gives the tool centre point (TCP) position and orientation to pass to
        arm.move_pose, the direction the gripper approaches along, a pre-grasp pose back
        along that approach, the object's width between the jaws and the proposer's score.
        A note says why each was proposed or what about it is uncertain. IK must reach the
        grasp and the pre-grasp from the current joints. The proposals do not know the task;
        choose the one that suits it.

        Args:
            object_id: The id localize printed for the object.
            proposer: "heuristic" or "graspgenx".
            count: How many reachable grasps to list at most.
            max_tilt: Drop grasps whose approach is more than this many degrees from straight
                down (0 keeps only top-down ones).
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
            if proposer == "heuristic":
                return (
                    f"heuristic proposed no grasps for {object_id}: it is wider than the jaws "
                    f"({self.config.max_jaw_span:.3f} m) on every side and its inside was not "
                    "seen below its rim, so no wall to hold either."
                )
            return f"{proposer} proposed no grasps for {object_id}."
        candidates, dropped, caveat = self._filter(object_id, proposer, cloud, proposals, max_tilt)
        if not candidates:
            return f"{proposer}: {len(proposals.candidates)} grasp(s) for {object_id}, none left: {dropped}."
        proposals = GraspCandidateArray(proposals.header, candidates)
        return self._describe(object_id, proposer, cloud, proposals, max(count, 1), dropped, caveat)

    def _filter(
        self,
        object_id: str,
        proposer: str,
        cloud: PointCloud2,
        proposals: GraspCandidateArray,
        max_tilt: float,
    ) -> tuple[list[GraspCandidate], str, str]:
        """The candidates that pass the tilt limit and, for graspgenx, the scene checks; what
        was dropped and what could not be checked, as text."""
        candidates = list(proposals.candidates)
        if not candidates:
            return [], "", ""
        poses = grasp_filters.pose_matrices(
            np.asarray([c.pose.position.to_numpy() for c in candidates]),
            np.asarray([c.pose.orientation.to_numpy() for c in candidates]),
        )
        keep = np.ones(len(candidates), dtype=bool)
        dropped, caveat = [], ""
        if max_tilt < 180.0:
            tilted = grasp_filters.tilt_degrees(poses) > max_tilt
            dropped.append(f"{int((tilted & keep).sum())} tilted over {max_tilt:.0f}deg")
            keep &= ~tilted
        if proposer == "graspgenx":
            scene = self._localize.scene_cloud(object_id)
            points = scene.points_f32() if scene is not None else np.zeros((0, 3))
            if not len(points):
                caveat = "collisions not checked: no scene cloud around the object"
            else:
                gripper = grasp_filters.box_surface_points()
                support = grasp_filters.support_height(
                    points, below=float(np.quantile(cloud.points_f32()[:, 2], 0.5))
                )
                if support is not None:
                    low = grasp_filters.below_support(poses, support, gripper)
                    dropped.append(f"{int((low & keep).sum())} below the table (z {support:.3f})")
                    keep &= ~low
                hit = grasp_filters.colliding(poses, points, gripper, self.config.collision_m)
                dropped.append(f"{int((hit & keep).sum())} colliding with the scene seen around it")
                keep &= ~hit
        kept = [c for c, k in zip(candidates, keep, strict=True) if k]
        return kept, ("dropped " + ", ".join(dropped)) if dropped else "", caveat

    def _describe(
        self,
        object_id: str,
        proposer: str,
        cloud: PointCloud2,
        proposals: GraspCandidateArray,
        count: int,
        dropped: str = "",
        caveat: str = "",
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
            f"{proposer}: {len(proposals.candidates)} grasp(s) for {object_id}"
            + (f" after the checks ({dropped})" if dropped else "")
            + f", the best {n} checked, {len(reachable)} reachable; cloud of {len(points)} "
            f"points, surfaces seen from z {bottom:.3f} to {top:.3f}."
        )
        if caveat:
            header += f" {caveat[0].upper()}{caveat[1:]}."
        if len(points) < self.config.sparse_cloud_points:
            header += (
                f" Only {len(points)} points: view the object from another angle and localize "
                "again before trusting these."
            )
        elif top - bottom < 0.005:
            header += " The surfaces seen span under 5 mm (the top only): look from the side too."
        if not reachable:
            first = reasons[0] or reasons[n]
            return f"{header} None reachable from the current joints (first: {first})."
        lines = [header + " Reachable, best first:"]
        for rank, i in enumerate(reachable[:count], start=1):
            grasp, approach = grasps[i], approaches[i]
            jaw = grasp.orientation.rotate_vector(Vector3(0.0, 1.0, 0.0))
            width = _jaw_width(points, grasp, jaw, self.config.max_jaw_span)
            lines.append(
                f"{rank}. position {_xyz(grasp.position)} orientation {_xyzw(grasp)}; "
                f"approach {_approach(approach)}, jaw yaw "
                f"{math.degrees(math.atan2(jaw.y, jaw.x)):.0f}deg, "
                + (
                    f"object width between the jaws {width:.3f} m"
                    if width is not None
                    else "object between the jaws not seen"
                )
                + f", score {checked[i].score:.2f}; "
                f"pre-grasp {_xyz(pregrasps[i].position)}"
                + (f" ({checked[i].note})" if checked[i].note else "")
            )
        return "\n".join(lines)


def _jaw_width(
    points: np.ndarray, grasp: PoseStamped, jaw: Vector3, max_jaw_span: float
) -> float | None:
    """Width of the object between the jaws: the points within reach of the closing jaws (half
    their span either side along the jaw axis, a finger's breadth across it, near the grasp's
    height), so a grasp on one wall of a container reports the wall, not the container."""
    axis = np.asarray([jaw.x, jaw.y, jaw.z], dtype=np.float32)
    rel = points - np.asarray([grasp.position.x, grasp.position.y, grasp.position.z], np.float32)
    along_jaw = rel @ axis
    across = np.linalg.norm(rel[:, :2] - np.outer(along_jaw, axis[:2]), axis=1)
    near = (np.abs(along_jaw) < max_jaw_span / 2.0) & (across < 0.02) & (np.abs(rel[:, 2]) < 0.02)
    if near.sum() < 5:
        return None  # e.g. a container's wall the camera saw only edge-on
    return float(np.quantile(along_jaw[near], 0.98) - np.quantile(along_jaw[near], 0.02))


def _xyz(v: Vector3) -> str:
    return f"[{v.x:.3f}, {v.y:.3f}, {v.z:.3f}]"


def _xyzw(pose: PoseStamped) -> str:
    return "[" + ", ".join(f"{c:.4f}" for c in pose.orientation.to_tuple()) + "]"


def _approach(v: Vector3) -> str:
    if v.z < -0.97:
        return "straight down"
    return f"along {_xyz(v)}"
