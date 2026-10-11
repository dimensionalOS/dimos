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

"""Grasps from a grasp provider, with heuristic grasps to try after them."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec, SceneGraspGenSpec
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspConfig, heuristic_grasps
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Source name of the grasps computed with heuristic_grasps.
HEURISTIC = "heuristic"
# A heuristic grasp within both of these of a provider grasp is the same grasp
# and is not tried twice: distance (metres) and angle (radians).
SAME_GRASP_DISTANCE = 0.02
SAME_GRASP_ANGLE = 0.35
# Half a turn about the gripper's approach axis (its local z), as (x, y, z, w).
_HALF_TURN_ABOUT_APPROACH = Quaternion(0.0, 0.0, 1.0, 0.0)


@dataclass(frozen=True)
class SourcedGrasp:
    """One grasp candidate and where it came from.

    Args:
        source: Who proposed it: the provider's name, e.g. "graspgenx", or "heuristic".
        rank: Its place in that source's list, 0 for the best.
        candidate: The grasp, in the frame of the point cloud it was made from.
    """

    source: str
    rank: int
    candidate: GraspCandidate


@dataclass(frozen=True)
class SceneCheck:
    """How to drop the provider's grasps whose gripper would hit the scene.

    Args:
        provider: The grasp provider, able to check its grasps against a scene.
        scene: Points around the object (table, other objects) without the
            object's own points, in the object cloud's frame, metres.
        collision_threshold: A gripper part closer than this (metres) to a scene
            point counts as a collision.
    """

    provider: SceneGraspGenSpec
    scene: PointCloud2
    collision_threshold: float


@dataclass(frozen=True)
class GraspProposals:
    """The grasps for one object from each source.

    Args:
        provider: The grasp provider's name, e.g. "graspgenx".
        sources: Grasp sources in effect, provider first, e.g. ["graspgenx", "heuristic"].
        provided: The provider's grasps, or None when it raised.
        heuristic: The heuristic grasps, or None when there are none to add.
        provider_error: What the provider raised, or None.
        scene_filtered: Per source, grasps dropped because the gripper would hit
            the scene; empty when no scene check ran.
        scene_filter_skipped: Why the scene check did not run ("off" when it is
            switched off), or None when it ran or the provider cannot run it.
    """

    provider: str
    sources: list[str]
    provided: GraspCandidateArray | None
    heuristic: GraspCandidateArray | None
    provider_error: str | None = None
    scene_filtered: dict[str, int] = field(default_factory=dict)
    scene_filter_skipped: str | None = None

    def all(self) -> GraspCandidateArray:
        """Every candidate, provider's first, as one array for the viewer."""
        arrays = [array for array in (self.provided, self.heuristic) if array is not None]
        if not arrays:
            return GraspCandidateArray()
        return GraspCandidateArray(
            arrays[0].header, [c for array in arrays for c in array.candidates]
        )

    def check_frame(self, planning_frame: str) -> None:
        """Raise RuntimeError when the grasps are not in the planning frame.

        The provider's frame is checked whenever it answered; the heuristic's
        only when it has grasps.
        """
        frames = [] if self.provided is None else [self.provided.header.frame_id]
        if self.heuristic is not None and self.heuristic.candidates:
            frames.append(self.heuristic.header.frame_id)
        for frame in frames:
            if frame != planning_frame:
                raise RuntimeError(
                    f"Grasp candidates are in frame {frame!r}; "
                    f"the planning frame is {planning_frame!r}"
                )

    def to_try(
        self,
        limit: int,
        skip: Callable[[GraspCandidate], bool] | None = None,
        distance: float = SAME_GRASP_DISTANCE,
        angle: float = SAME_GRASP_ANGLE,
    ) -> list[SourcedGrasp]:
        """The grasps to try, in order: the provider's best, then heuristic ones.

        Each keeps its rank in its own source's list.

        Args:
            limit: Most grasps taken from each source.
            skip: Returns True for a grasp to leave out, e.g. one already tried.
            distance: A heuristic grasp closer than this (metres) to a provider
                grasp taken here, and within ``angle`` of it, is left out.
            angle: Angle (radians) between orientations for that rule. A grasp
                turned half a turn about its approach axis counts as the same.
        """

        def kept(candidate: GraspCandidate) -> bool:
            return skip is None or not skip(candidate)

        provided = [g for g in _sourced(self.provider, self.provided) if kept(g.candidate)]
        provided = provided[:limit]
        heuristic = [
            g
            for g in _sourced(HEURISTIC, self.heuristic)
            if kept(g.candidate)
            and not any(
                same_grasp(g.candidate.pose, p.candidate.pose, distance, angle) for p in provided
            )
        ]
        return provided + heuristic[:limit]

    def facts(self) -> dict[str, Any]:
        """The sources in effect, how many grasps each gave and lost to the scene check,
        and any provider error."""
        counts = {self.provider: 0 if self.provided is None else len(self.provided.candidates)}
        if self.heuristic is not None:
            counts[HEURISTIC] = len(self.heuristic.candidates)
        facts: dict[str, Any] = {
            "grasp_sources": self.sources,
            "candidates_by_source": counts,
            "scene_filtered_by_source": dict(self.scene_filtered),
        }
        if self.scene_filter_skipped is not None:
            facts["scene_filter_skipped"] = self.scene_filter_skipped
        if self.provider_error is not None:
            facts["grasp_provider_error"] = self.provider_error
        return facts

    def note(self) -> str:
        """A sentence saying the provider failed, with a leading space, or ""."""
        if self.provider_error is None:
            return ""
        return f" The grasp provider {self.provider} failed: {self.provider_error}."


def provider_name(provider: Any) -> str:
    """Short name of a grasp provider for reports, e.g. "graspgenx" for GraspGenXModule.

    HeuristicGraspModule is "heuristic", the same as the heuristic fallback.
    A provider with no module name is "provider".
    """
    name = getattr(provider, "remote_name", None)
    if not isinstance(name, str) or not name:
        return "provider"
    return name.removesuffix("Module").removesuffix("Grasp").lower()


def grasp_sources(provider: Any, heuristic_fallback: bool) -> list[str]:
    """Grasp sources a pick uses, provider first, e.g. ["graspgenx", "heuristic"].

    Args:
        provider: The grasp provider module.
        heuristic_fallback: Whether heuristic grasps are tried after the provider's.
    """
    name = provider_name(provider)
    return [name, HEURISTIC] if heuristic_fallback and name != HEURISTIC else [name]


def propose_with_fallback(
    provider: GraspGenSpec,
    object_pointcloud: PointCloud2,
    heuristic_fallback: bool,
    heuristic_config: HeuristicGraspConfig,
    scene: SceneCheck | None = None,
    scene_skipped: str | None = None,
) -> GraspProposals:
    """Ask the grasp provider for grasps and, with the fallback on, compute heuristic grasps too.

    Args:
        provider: The grasp provider module, e.g. GraspGenXModule.
        object_pointcloud: The object's points, metres, in a frame with Z up.
        heuristic_fallback: Also compute heuristic grasps, and treat a provider
            error as no provider grasps. Off, a provider error is raised. A
            provider that is itself the heuristic gets no second copy.
        heuristic_config: How the heuristic grasps fit the gripper.
        scene: Drop the provider's grasps that would hit this scene, or None to
            take them all. Heuristic grasps are never checked.
        scene_skipped: Why the scene check did not run, for the report.
    """
    name = provider_name(provider)
    sources = grasp_sources(provider, heuristic_fallback)
    error = None
    scene_filtered: dict[str, int] = {}
    provided: GraspCandidateArray | None
    try:
        if scene is None:
            provided = provider.propose_grasps(object_pointcloud)
        else:
            checked = scene.provider.propose_grasps_in_scene(
                object_pointcloud, scene.scene, scene.collision_threshold
            )
            provided = checked.grasps
            scene_filtered[name] = checked.removed
    except Exception as exc:
        if not heuristic_fallback:
            raise
        error = f"{type(exc).__name__}: {exc}"
        logger.warning("Grasp provider failed; trying heuristic grasps", provider=name, error=error)
        provided = None
    heuristic = None
    if heuristic_fallback and name != HEURISTIC:
        try:
            heuristic = heuristic_grasps(object_pointcloud, heuristic_config)
        except ValueError as exc:
            logger.warning("No heuristic grasps for this point cloud", error=str(exc))
    return GraspProposals(name, sources, provided, heuristic, error, scene_filtered, scene_skipped)


def same_grasp(a: Pose, b: Pose, distance: float, angle: float) -> bool:
    """Whether two grasps hold an object the same way.

    Args:
        a: One grasp pose.
        b: The other, in the same frame.
        distance: Most distance (metres) between their positions.
        angle: Most angle (radians) between their orientations. A parallel-jaw
            grasp turned half a turn about its approach axis counts as the same.
    """
    if a.position.distance(b.position) > distance:
        return False
    flipped = b.orientation * _HALF_TURN_ABOUT_APPROACH
    turn = min(a.orientation.angle_to(b.orientation), a.orientation.angle_to(flipped))
    return turn <= angle


def _sourced(source: str, array: GraspCandidateArray | None) -> list[SourcedGrasp]:
    """Each candidate of an array with its source and rank."""
    if array is None:
        return []
    return [SourcedGrasp(source, rank, c) for rank, c in enumerate(array.candidates)]
