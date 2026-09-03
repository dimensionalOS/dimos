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

"""Exploratory pose-trajectory questions for the SF office Go2 recording."""

from pathlib import Path

from dimos.evals.environments.pose_trajectory_dataset import PoseTrajectoryDataset
from dimos.evals.types import EvalCase, Suite


_DATASET = str(
    Path.home() / "Documents/2026-08-27_sf_office_8mins_moshi/go2_SF_office_8mins_moshi.db"
)


def _trajectory(start_s: float | None = None, stop_s: float | None = None) -> PoseTrajectoryDataset:
    return PoseTrajectoryDataset(_DATASET, start_s=start_s, stop_s=stop_s)


SUITE: Suite = [
    EvalCase(
        id="sf_office_pose_max_tilt",
        inputs=(
            "At what point did the robot tilt the most? Return only JSON with seconds from "
            "trajectory start, world-frame position in meters, and tilt in degrees: "
            '{"time_s": number, "position_m": [x, y, z], "tilt_deg": number}.'
        ),
        environment=_trajectory(),
        grade=lambda _: 0.0,
        timeout_s=120.0,
        tags=frozenset({"sf-office", "pose", "trajectory", "tilt", "exploratory"}),
    ),
    EvalCase(
        id="sf_office_pose_travelled_distance",
        inputs=(
            "How long is the robot's complete trajectory, measured as distance traveled from "
            "start to stop rather than straight-line displacement? Return only JSON in meters: "
            '{"travelled_distance_m": number}.'
        ),
        environment=_trajectory(),
        grade=lambda _: 0.0,
        timeout_s=120.0,
        tags=frozenset({"sf-office", "pose", "trajectory", "distance", "exploratory"}),
    ),
    EvalCase(
        id="sf_office_pose_self_intersection",
        inputs=(
            "Does the robot's top-down XY trajectory self-intersect? Count only crossings of "
            "non-adjacent path sections, not consecutive segments sharing an endpoint. Return "
            "only JSON; include approximate world-frame crossing locations if visible: "
            '{"intersects": boolean, "locations_m": [[x, y], ...]}.'
        ),
        environment=_trajectory(),
        grade=lambda _: 0.0,
        timeout_s=120.0,
        tags=frozenset({"sf-office", "pose", "trajectory", "intersection", "exploratory"}),
    ),
    EvalCase(
        id="sf_office_pose_relative_motion",
        inputs=(
            "Between the first and last poses shown, how far did the robot move and how much "
            "did its orientation rotate? Use 3D straight-line displacement and the shortest "
            "quaternion rotation angle. Return only JSON: "
            '{"distance_m": number, "rotation_deg": number}.'
        ),
        environment=_trajectory(60.0, 120.0),
        grade=lambda _: 0.0,
        timeout_s=120.0,
        tags=frozenset({"sf-office", "pose", "relative-motion", "exploratory"}),
    ),
    EvalCase(
        id="sf_office_pose_motion_direction",
        inputs=(
            "Between the first and last poses shown, did the robot move primarily forward, "
            "backward, left, or right relative to its orientation at the first pose? Return "
            'only JSON: {"direction": "forward" | "backward" | "left" | "right", '
            '"local_displacement_m": [forward_x, left_y, up_z]}.'
        ),
        environment=_trajectory(60.0, 120.0),
        grade=lambda _: 0.0,
        timeout_s=120.0,
        tags=frozenset({"sf-office", "pose", "relative-motion", "direction", "exploratory"}),
    ),
    EvalCase(
        id="sf_office_pose_return_to_start",
        inputs=(
            "Did the robot finish close to its starting pose? Treat within 0.5 meters and 15 "
            "degrees as close. Return only JSON with the decision and measured differences: "
            '{"returned_close": boolean, "distance_m": number, "rotation_deg": number}.'
        ),
        environment=_trajectory(),
        grade=lambda _: 0.0,
        timeout_s=120.0,
        tags=frozenset({"sf-office", "pose", "trajectory", "return", "exploratory"}),
    ),
    EvalCase(
        id="sf_office_pose_max_speed",
        inputs=(
            "At what point did the robot achieve its highest one-second windowed speed? Return "
            "only JSON with seconds from trajectory start, world-frame position in meters, and "
            'speed: {"time_s": number, "position_m": [x, y, z], "speed_mps": number}.'
        ),
        environment=_trajectory(),
        grade=lambda _: 0.0,
        timeout_s=120.0,
        tags=frozenset({"sf-office", "pose", "trajectory", "speed", "exploratory"}),
    ),
]
