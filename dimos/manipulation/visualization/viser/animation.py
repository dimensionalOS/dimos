# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import time

from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.msgs.sensor_msgs.JointState import JointState


@dataclass(frozen=True)
class PreviewTrack:
    """One render track owned by one or more planning groups."""

    robot_id: str
    group_ids: tuple[PlanningGroupID, ...]
    joint_names: tuple[str, ...]
    path: tuple[JointState, ...]


@dataclass(frozen=True)
class GroupPreviewAnimation:
    """Group-native preview transaction for a generated plan."""

    group_ids: tuple[PlanningGroupID, ...]
    tracks: tuple[PreviewTrack, ...]


def interpolate_joint_path(
    path: Sequence[JointState], duration: float, fps: float
) -> list[list[float]]:
    """Interpolate a joint path into visualization frames."""
    waypoints = [list(waypoint.position) for waypoint in path if waypoint.position]
    if not waypoints:
        return []
    if len(waypoints) == 1 or duration <= 0.0:
        return [waypoints[-1]]
    frame_count = max(int(duration * max(fps, 1.0)) + 1, len(waypoints))
    segment_count = len(waypoints) - 1
    frames: list[list[float]] = []
    for frame_index in range(frame_count):
        path_t = frame_index / max(frame_count - 1, 1)
        scaled = path_t * segment_count
        segment_index = min(int(scaled), segment_count - 1)
        local_t = scaled - segment_index
        start = waypoints[segment_index]
        end = waypoints[segment_index + 1]
        if len(start) != len(end):
            continue
        frames.append(
            [
                start_value + (end_value - start_value) * local_t
                for start_value, end_value in zip(start, end, strict=False)
            ]
        )
    if frames and frames[-1] != waypoints[-1]:
        frames.append(waypoints[-1])
    return frames


def sampled_joint_path_frames(
    path: Sequence[JointState], duration: float, fps: float
) -> list[list[float]]:
    """Return animation frames while preserving already sampled trajectories.

    ViserManipulationVisualizer.animate_plan() projects GeneratedPlan waypoints into robot-local
    preview paths. If a path arrives already sampled near the target display rate, Viser should
    play those samples directly instead of re-interpolating by waypoint index. Sparse direct scene
    callers still get local interpolation as a fallback.
    """
    waypoints = [list(waypoint.position) for waypoint in path if waypoint.position]
    if not waypoints:
        return []
    expected_frames = max(int(duration * max(fps, 1.0)) + 1, 1) if duration > 0.0 else 1
    if len(waypoints) >= expected_frames:
        return waypoints
    return interpolate_joint_path(path, duration, fps)


class PreviewAnimator:
    """Blocking preview-ghost path animator with Meshcat-compatible semantics.

    This class is only for transient path playback. Persistent target ghosts are updated
    directly by scene target methods and must not be routed through this animator.
    """

    def __init__(
        self,
        set_joints: Callable[[Sequence[float]], None],
        *,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._set_joints = set_joints
        self._sleep = sleep

    def animate(self, path: Sequence[JointState], duration: float, fps: float) -> bool:
        frames = sampled_joint_path_frames(path, duration, fps)
        if not frames:
            return False
        step_delay = duration / max(len(frames) - 1, 1) if duration > 0.0 else 0.0
        for index, joints in enumerate(frames):
            self._set_joints(joints)
            if index < len(frames) - 1:
                self._sleep(step_delay)
        return True
