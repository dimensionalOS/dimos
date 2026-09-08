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

"""PoseStamped autoresearch cases for the SF office Go2 recording."""

import os
from pathlib import Path

from dimos.evals.environments.dataset import Dataset
from dimos.evals.suites.sf_office_pose_grading import grader
from dimos.evals.types import EvalCase, Suite

RECORDING_PATH = Path(
    os.environ.get(
        "SF_OFFICE_POSE_RECORDING",
        Path.home() / "Documents/worktrees/agentencode/recording_go2.db",
    )
)
EXPECTED_RECORDING_SHA256 = "75dab75f22fb19b730ca43121d7601a0460ff7cea533de8f3f9a8d7b88f55fe8"
EXPECTED_POSE_COUNT = 8568
_TIMEOUT_S = 180.0

_EVIDENCE_CONTRACT = (
    "Open the recording listed in the system prompt and use only its odom PoseStamped "
    "observations. Iterate observations in timestamp order and call obs.data.agent_encode() "
    "on each one. The returned dictionaries are the sole evidence. Do not inspect raw pose "
    "attributes or database payloads, skip observations, inspect evaluation files or labels, "
    "construct a Path, or use repository pose, mapping, navigation, or evaluation utilities. "
    "Pass the complete list of returned dictionaries to "
    "dimos.evals.suites.sf_office_pose_preprocessing.preprocess_encoded_poses and use its "
    "time_s, position_m, yaw_rad, velocity_xy_m_s, and speed_m_s arrays for analysis. This "
    "frozen helper is the only repository analysis utility you may use. You may use generic "
    "Python, NumPy, and SciPy calculations on its output. "
)

_PREPROCESSING = (
    "Unless the question says otherwise, analyze world-frame XY motion and express times as "
    "seconds from the first pose. The frozen helper already performs timestamp sorting, duplicate "
    "retention, quaternion normalization, yaw unwrapping, 10 Hz interpolation, Hampel and "
    "Savitzky-Golay filtering, and centered velocity calculation; do not repeat preprocessing. "
)


def _poses() -> Dataset:
    return Dataset(str(RECORDING_PATH), select=(lambda store: store.streams.odom,))


def _case(id: str, question: str, *tags: str) -> EvalCase:
    return EvalCase(
        id=id,
        inputs=_EVIDENCE_CONTRACT + _PREPROCESSING + question,
        environment=_poses(),
        grade=grader(id),
        timeout_s=_TIMEOUT_S,
        tags=frozenset({"sf-office", "pose", "autoresearch", *tags}),
    )


SUITE: Suite = [
    _case(
        "sf_office_pose_return_distance",
        "What minimum additional straight-line XY distance must the robot travel from its final "
        "processed position to return to its initial processed position? Ignore obstacles. Return "
        'only JSON: {"remaining_distance_m": number}.',
        "kinematics",
        "return",
    ),
    _case(
        "sf_office_pose_stationary_percentage",
        "Stationary means planar speed below 0.08 m/s. Bridge nonstationary gaps no longer than "
        "1.0 s and discard stationary runs shorter than 2.0 s. What percentage of total elapsed "
        'time was stationary? Return only JSON: {"stationary_time_s": number, '
        '"stationary_percent": number}.',
        "kinematics",
        "stationary",
    ),
    _case(
        "sf_office_pose_backward_intervals",
        "Body-forward speed is XY velocity dotted with [cos(yaw), sin(yaw)]. Backward requires "
        "total speed above 0.10 m/s and body-forward speed below -0.08 m/s. Bridge gaps no longer "
        "than 0.5 s and retain intervals at least 1.0 s long. During which intervals did the robot "
        'walk backward? Return only JSON: {"backward_intervals_s": [{"start_s": number, '
        '"end_s": number}, ...]}.',
        "kinematics",
        "backward",
    ),
    _case(
        "sf_office_pose_self_intersections",
        "Simplify XY with standard recursive Ramer-Douglas-Peucker epsilon 0.10 m. Count proper "
        "interior intersections between nonadjacent segments separated by at least 5 s. Group "
        "successive crossings when later times differ by at most 2 s and locations by at most "
        "0.30 m; report each group's mean later crossing time. Return only JSON: "
        '{"intersection_count": integer, "crossing_times_s": [number, ...]}.',
        "topology",
        "intersection",
    ),
    _case(
        "sf_office_pose_path_compression",
        "Build a collision-unaware replay plan by simplifying XY with Ramer-Douglas-Peucker "
        "epsilon 0.20 m. Report original and simplified polyline lengths. Count turns with heading "
        "change at least 20 degrees. A simplified leg is reverse when its median cosine alignment "
        "with body yaw is negative. An in-place rotation is a stationary interval from the prior "
        "rule with at least 30 degrees net unwrapped yaw change. Return only JSON: "
        '{"original_length_m": number, "optimized_length_m": number, '
        '"reduction_percent": number, "line_segments": integer, "turns": integer, '
        '"reverse_segments": integer, "in_place_rotations": integer}.',
        "planning",
        "compression",
    ),
    _case(
        "sf_office_pose_opposite_retrace",
        "Use centered 2 s displacement windows with speed at least 0.10 m/s. A window retraces an "
        "earlier path when the earlier window is at least 10 s older, midpoint separation is at "
        "most 0.30 m, and travel directions differ by at least 150 degrees. Bridge gaps no longer "
        "than 0.5 s, retain runs at least 1.0 s, and report the longest run, its most-recent "
        "matching earlier span, and processed-path length during the later run. Return only JSON: "
        '{"interval_s": [start, end], "earlier_interval_s": [start, end], '
        '"retrace_length_m": number}.',
        "topology",
        "retrace",
    ),
    _case(
        "sf_office_pose_longest_elapsed_return",
        "Among pose pairs at least 10 s apart and within 0.30 m in XY, require the intervening "
        "trajectory to depart at least 1.0 m from the earlier location. Which location was "
        "revisited after the greatest elapsed time? Report the midpoint of the paired positions. "
        'Return only JSON: {"location_m": [x, y], "previous_time_s": number, '
        '"return_time_s": number, "elapsed_s": number}.',
        "topology",
        "return",
    ),
    _case(
        "sf_office_pose_least_aligned",
        "For each centered 2 s window with displacement speed at least 0.10 m/s, compute the "
        "unsigned wrapped angle between midpoint body yaw and the window displacement direction. "
        "During which window was facing least aligned with travel? Return only JSON: "
        '{"interval_s": [start, end], "center_time_s": number, "misalignment_deg": number, '
        '"position_m": [x, y]}.',
        "kinematics",
        "alignment",
    ),
    _case(
        "sf_office_pose_repeated_patrol_cycle",
        "Determine whether timestamps t0 < t1 < t2 at whole-second boundaries define two "
        "consecutive, non-overlapping traversals [t0, t1] and [t1, t2] around one anchor. The "
        "traversals share only boundary t1; arbitrary overlapping intervals do not qualify. Each "
        "traversal must last at least 30 s and cover at least 10 m. Positions at t0, t1, and t2 "
        "must be pairwise within 0.50 m. Traversal durations and lengths must agree within 20%, "
        "using the larger value as denominator. Resample each world-frame trajectory independently "
        "to 100 normalized-phase points; their pointwise XY Euclidean distances must have root-mean-"
        "square at most 0.75 m. Return null metrics when no cycle exists. For a cycle, start_time_s "
        "is t0, duration_s is the mean of the two traversal durations, and length_m is the mean of "
        "their processed polyline lengths. Return only JSON: "
        "If multiple triples qualify, select the lexicographically earliest (t0, t1, t2). "
        '{"repeated_cycle": boolean, "start_time_s": number | null, '
        '"duration_s": number | null, "length_m": number | null}.',
        "patterns",
        "cycle",
    ),
]
