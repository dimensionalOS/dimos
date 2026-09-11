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

"""Human-reviewed overall-recording QA, independent of encoder autoresearch.

Uses the named dataset recording_go2. Set SF_OFFICE_RECORDING to a local .db
path to override it. Every case exposes the complete recording.
"""

import os

from dimos.evals.environments.dataset import Dataset
from dimos.evals.scorers import boolean, exact, multi_select, numeric, point
from dimos.evals.types import EvalCase, Suite

_RECORDING = os.environ.get("SF_OFFICE_RECORDING", "recording_go2")

SUITE: Suite = [
    EvalCase(
        id="sf_office_recording_every_desk_has_monitor",
        inputs=(
            "Does every desk have a monitor on it? "
            'Return JSON with "every_desk_has_monitor": true if yes, false if no.'
        ),
        environment=Dataset(_RECORDING),
        # Human-reviewed reference: no.
        grade=lambda o: boolean(False, o.trajectory.final_answer, key="every_desk_has_monitor"),
        timeout_s=600.0,
        tags=frozenset({"sf-office", "recording", "desk", "monitor", "boolean"}),
    ),
    EvalCase(
        id="sf_office_recording_opposite_retrace",
        inputs=(
            "During which of these intervals did the robot retrace an earlier path "
            "in the opposite direction? Select all that apply. Times are seconds "
            "from the earliest odometry timestamp. "
            "A: 112.5-121.6 s; B: 239.4-252.4 s; C: 404.6-426.8 s; D: 27.8-50.1 s. "
            'Return only JSON with an "options" list of selected uppercase labels, '
            "or an empty list if none qualify."
        ),
        environment=Dataset(_RECORDING),
        # Intervals from opposite-retrace-review.json; B and C selected by the user.
        grade=lambda o: multi_select(
            ("B", "C"), o.trajectory.final_answer, options=("A", "B", "C", "D")
        ),
        timeout_s=600.0,
        tags=frozenset({"sf-office", "recording", "pose", "retrace", "multi-select"}),
    ),
    EvalCase(
        id="sf_office_recording_return_distance",
        inputs=(
            "How much additional straight-line distance must the robot travel from its "
            "final position to return to its initial position? Ignore obstacles and "
            'height differences. Return JSON with "distance_m": a number in meters.'
        ),
        environment=Dataset(_RECORDING),
        # Timestamp-ordered raw odometry endpoints: XY separation 0.21429127105414264 m.
        grade=lambda o: numeric(
            0.2143, o.trajectory.final_answer, key="distance_m", tolerance=0.05, band=0.30
        ),
        timeout_s=600.0,
        tags=frozenset({"sf-office", "recording", "pose", "return", "distance"}),
    ),
    EvalCase(
        id="sf_office_recording_forward_clearance",
        inputs=(
            "Using the latest map, can a circular robot 0.20 m wide, centered at "
            "(0.00, -4.20) m and facing "
            "90 degrees (world +Y), move 1 meter straight ahead without collision? "
            "Coordinates are world-frame meters. Unknown space is not clear. "
            'Return JSON with "clear": true if it can, false otherwise.'
        ),
        environment=Dataset(_RECORDING),
        # Raw map #714: swept 0.10 m-radius disk is clear, including endpoints.
        # Verified at <= 5 mm travel steps with an extra 2.5 mm clearance margin.
        grade=lambda o: boolean(True, o.trajectory.final_answer, key="clear"),
        timeout_s=600.0,
        tags=frozenset({"sf-office", "recording", "occupancy", "clearance", "boolean"}),
    ),
    EvalCase(
        id="sf_office_recording_twist_collision_object",
        inputs=(
            "Using the latest map, starting at (-0.75, -4.35) m, facing 0 degrees, "
            "a circular robot of radius "
            "0.25 m moves at a constant forward speed of 0.5 m/s and turns "
            "counterclockwise at 15 degrees/s for 8 seconds. What will it collide "
            "with first? Coordinates are world-frame meters; 0 degrees faces world +X. "
            "A: Table; B: Wall; C: Bean bag; D: Nothing within that time. "
            "Answer with only one letter: A, B, C, or D."
        ),
        environment=Dataset(_RECORDING),
        # Human-labeled table from twist-collision-review.json; map contact at ~6.13 s.
        grade=lambda o: exact("A", o.trajectory.final_answer.strip().upper()),
        timeout_s=600.0,
        tags=frozenset({"sf-office", "recording", "collision", "motion-model", "multiple-choice"}),
    ),
    EvalCase(
        id="sf_office_recording_largest_free_circle",
        inputs=(
            "Using the latest map, where is the largest circle that fits entirely "
            "within known-free floor "
            "space across the entire mapped area? Return its center in world-frame "
            'meters and radius in meters: {"center_m": [x, y], "radius_m": number}.'
        ),
        environment=Dataset(_RECORDING),
        # Computed from raw map #714, blocked-cell boundaries, then visually approved.
        # Subcell search radius 1.9524 m, optimality gap <= 0.005 m; rounded safe circle below.
        # Independent fixed-reference scores; no map/validity check during grading.
        grade=lambda o: 0.7
        * numeric(1.95, o.trajectory.final_answer, key="radius_m", tolerance=0.10, band=0.75)
        + 0.3
        * point((0.02, -4.02), o.trajectory.final_answer, key="center_m", tolerance=0.25, band=2.0),
        timeout_s=600.0,
        tags=frozenset({"sf-office", "recording", "occupancy", "clearance", "circle"}),
    ),
    EvalCase(
        id="sf_office_recording_hiding_place",
        inputs=(
            "Using the latest map, which of these locations would be the best place "
            "to hide during a game "
            "of hide-and-seek? Coordinates are world-frame meters. "
            "A: (0.75, -5.05); B: (4.50, 6.55); C: (-4.90, -2.15); "
            "D: (4.00, 0.30); E: (-5.90, -10.35). "
            "Answer with only one letter: A, B, C, D, or E."
        ),
        environment=Dataset(_RECORDING),
        # Human preference from hiding-place-review.json: B, not a computed optimum.
        grade=lambda o: exact("B", o.trajectory.final_answer.strip().upper()),
        timeout_s=600.0,
        tags=frozenset({"sf-office", "recording", "occupancy", "concealment", "multiple-choice"}),
    ),
    EvalCase(
        id="sf_office_recording_three_point_loop",
        inputs=(
            "Using the latest map, what is the approximate length in meters of the "
            "shortest collision-free "
            "closed loop through (1.25, 1.80), (-4.50, -9.25), and (3.35, -6.10) "
            "in any order, for a circular robot of radius 0.15 m? "
            "Return to the starting point. Coordinates are world-frame meters. "
            'Return JSON with "length_m": a number.'
        ),
        environment=Dataset(_RECORDING),
        # User-reviewed three-point-loop export: 34.61909140176568 m.
        # Conservative eight-connected grid route, not an exact continuous-space optimum.
        grade=lambda o: numeric(
            34.62, o.trajectory.final_answer, key="length_m", tolerance=1.5, band=10.0
        ),
        timeout_s=600.0,
        tags=frozenset({"sf-office", "recording", "occupancy", "routing", "distance"}),
    ),
    EvalCase(
        id="sf_office_recording_path_intersections",
        inputs=(
            "How many distinct times did the robot cross a path it had already traveled? "
            "Ignore brief back-and-forth motion: the two passages must be more than "
            "5 seconds apart. Count nearby crossing detections within 0.2 m and "
            "5 seconds of the same crossing event only once. "
            'Return JSON with "intersection_count": an integer.'
        ),
        environment=Dataset(_RECORDING),
        # Human-reviewed approximate count: 19 marks; full credit for 18-20.
        # Linear partial credit outside that range, zero at 11 or 27 and beyond.
        grade=lambda o: numeric(
            19, o.trajectory.final_answer, key="intersection_count", tolerance=1, band=8
        ),
        timeout_s=600.0,
        tags=frozenset({"sf-office", "recording", "pose", "intersection", "count"}),
    ),
    EvalCase(
        id="sf_office_recording_strafing_time",
        inputs=(
            "At what time is the robot moving approximately sideways relative to its "
            "facing direction, rather than backward? Return the time in seconds from "
            "the earliest odometry timestamp and the unsigned angle between facing "
            'and travel direction: {"time_s": number, "angle_deg": number}.'
        ),
        environment=Dataset(_RECORDING),
        # Pose-derived reference: 313.1 s, 90.02 degrees (rounded target: 90).
        # Centered 2 s displacement, speed >= 0.10 m/s, frozen pose preprocessing.
        # Fixed-reference grading only; angle and time credits are independent.
        grade=lambda o: 0.7
        * numeric(90.0, o.trajectory.final_answer, key="angle_deg", tolerance=3.0, band=10.0)
        + 0.3 * numeric(313.1, o.trajectory.final_answer, key="time_s", tolerance=1.0, band=5.0),
        timeout_s=600.0,
        tags=frozenset({"sf-office", "recording", "pose", "strafing", "time", "angle"}),
    ),
    EvalCase(
        id="sf_office_recording_country_flag",
        inputs=(
            "Which country's flag is visible in the recording? "
            "A: United States; B: Canada; C: India; D: Liberia; "
            "E: None (no country flag is visible). "
            "Answer with only one letter: A, B, C, D, or E."
        ),
        environment=Dataset(_RECORDING),
        # Human-provided reference: United States.
        grade=lambda o: exact("A", o.trajectory.final_answer.strip().upper()),
        timeout_s=600.0,
        tags=frozenset({"sf-office", "recording", "flag", "recognition", "multiple-choice"}),
    ),
    EvalCase(
        id="sf_office_recording_kitchen_floor_elevation",
        inputs=(
            "How much higher is the kitchen floor than the rest of the floor, if elevated? "
            "A: 0 inches; B: 5-10 inches; C: 15-20 inches; D: more than 25 inches. "
            "Answer with only one letter: A, B, C, or D."
        ),
        environment=Dataset(_RECORDING),
        # Human-provided reference: 8 inches, within option B.
        grade=lambda o: exact("B", o.trajectory.final_answer.strip().upper()),
        timeout_s=600.0,
        tags=frozenset({"sf-office", "recording", "height", "kitchen", "multiple-choice"}),
    ),
    EvalCase(
        id="sf_office_recording_kitchen_floor_material",
        inputs=(
            "What is the material of the kitchen floor? "
            "A: wood; B: marble; C: plastic; D: ceramic tile. "
            "Answer with only one letter: A, B, C, or D."
        ),
        environment=Dataset(_RECORDING),
        # Human-provided reference: wood.
        grade=lambda o: exact("A", o.trajectory.final_answer.strip().upper()),
        timeout_s=600.0,
        tags=frozenset({"sf-office", "recording", "material", "kitchen", "multiple-choice"}),
    ),
    EvalCase(
        id="sf_office_recording_g1_ground_contact",
        inputs=(
            "Are the feet of the G1 robot held by the gantry touching the ground? "
            'Return JSON with "feet_touching_ground": true if yes, false if no.'
        ),
        environment=Dataset(_RECORDING),
        # Human-provided reference: not touching the ground.
        grade=lambda o: boolean(False, o.trajectory.final_answer, key="feet_touching_ground"),
        timeout_s=600.0,
        tags=frozenset({"sf-office", "recording", "spatial-relation", "g1", "boolean"}),
    ),
    EvalCase(
        id="sf_office_recording_oven_state",
        inputs=(
            "Is the oven door open or closed? "
            'Return JSON with "oven_open": true if open, false if closed.'
        ),
        environment=Dataset(_RECORDING),
        # Human-provided reference: closed.
        grade=lambda o: boolean(False, o.trajectory.final_answer, key="oven_open"),
        timeout_s=600.0,
        tags=frozenset({"sf-office", "recording", "object-state", "oven", "boolean"}),
    ),
    EvalCase(
        id="sf_office_recording_clear_corridors",
        inputs=(
            "Using the latest map, which of the following corridors have a clear "
            "passage wider than 1.0 m? "
            "Select all that apply. Coordinates are world-frame meters. "
            "A: (-1.55, -4.35) to (-1.25, -0.75); "
            "B: (-1.30, 0.35) to (1.60, 2.20); "
            "C: (0.65, -5.90) to (0.75, -3.55); "
            "D: (5.95, -1.65) to (3.15, 1.05). "
            'Return only JSON with an "options" list of selected uppercase labels, '
            "or an empty list if none qualify."
        ),
        environment=Dataset(_RECORDING),
        # Reviewed raw-map widths: A=1.69, B=0.43, C=2.49, D=0.78 meters.
        grade=lambda o: multi_select(
            ("A", "C"), o.trajectory.final_answer, options=("A", "B", "C", "D")
        ),
        timeout_s=600.0,
        tags=frozenset(
            {"sf-office", "recording", "occupancy", "clearance", "multi-select", "static"}
        ),
    ),
]
