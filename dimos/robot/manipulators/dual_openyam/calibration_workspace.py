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

"""Where each Dual OpenYAM arm may go while it is calibrated on the bench.

The arms stand on a table about 30 mm below their base frames, their bases
``DUAL_OPENYAM_BENCH_BASE_SPACING_M`` apart: the bench measures 0.468 m, while
the URDF puts the bases at y = +-0.31 m, 0.62 m apart. During calibration an
arm's fingertip stays in front of its base, every checked point stays 150 mm
above the table at a pose (110 mm on the way between poses), and nothing
crosses the rig midline, halfway between the bases on the bench, by more than
``DUAL_OPENYAM_MIDLINE_MARGIN_M``. The pose grid uses the bench's hand-set
steps, so the poses are those the bench has run.
"""

from __future__ import annotations

import math

from dimos.robot.manipulators.calibration.spec import (
    ArmCalibrationSpec,
    HalfSpace,
    WorkspaceGuard,
    specs_from_damiao_robot,
)
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM, DUAL_OPENYAM_SIDES

# The bases' spacing in the URDF (y = +-0.31 m) and on the bench.
DUAL_OPENYAM_URDF_BASE_SPACING_M = 0.62
DUAL_OPENYAM_BENCH_BASE_SPACING_M = 0.468
DUAL_OPENYAM_TABLE_Z_M = -0.03
DUAL_OPENYAM_POSE_CLEARANCE_M = 0.15
DUAL_OPENYAM_TRANSITION_CLEARANCE_M = 0.11
DUAL_OPENYAM_MIN_FRONT_X_M = 0.15
DUAL_OPENYAM_MIDLINE_MARGIN_M = 0.05
# The fingertip, 46 mm past the grasp frame along the gripper's -z.
DUAL_OPENYAM_FINGERTIP_OFFSET_M = (0.0, 0.0, -0.046)
# Pose-generation boxes, deg by joint number, well inside the URDF limits
# (j2 0..210, j3 0..180, j4 -97..90, j5 -90..90); the yaw and the wrist roll
# stay at home. Identical for both arms; they keep the pose set of the bench.
DUAL_OPENYAM_CALIBRATION_RANGES_DEG: dict[int, tuple[float, float]] = {
    1: (0.0, 0.0),
    2: (25.0, 150.0),
    3: (25.0, 160.0),
    4: (-75.0, 75.0),
    5: (-75.0, 75.0),
    6: (0.0, 0.0),
}
# Pose grid steps, deg by measured joint number: the bench's hand-set grid.
DUAL_OPENYAM_GRID_STEP_DEG: dict[int, float] = {2: 12.5, 3: 13.5, 4: 25.0, 5: 37.5}


def dual_openyam_bench_workspace(
    side: str, *, base_spacing_m: float = DUAL_OPENYAM_BENCH_BASE_SPACING_M
) -> WorkspaceGuard:
    """The bench guard for one arm; the midline is a keep-out half-space toward the other arm."""
    if side not in DUAL_OPENYAM_SIDES:
        raise ValueError(f"side must be one of {DUAL_OPENYAM_SIDES}, got {side!r}")
    # +y points from the right arm to the left, so the midline lies at -y of the left base.
    toward_midline = -1.0 if side == "left" else 1.0
    return WorkspaceGuard(
        floor_z_m=DUAL_OPENYAM_TABLE_Z_M,
        pose_clearance_m=DUAL_OPENYAM_POSE_CLEARANCE_M,
        transition_clearance_m=DUAL_OPENYAM_TRANSITION_CLEARANCE_M,
        min_x_m=DUAL_OPENYAM_MIN_FRONT_X_M,
        keep_out=(
            HalfSpace(
                normal=(0.0, toward_midline, 0.0),
                offset_m=base_spacing_m / 2.0 + DUAL_OPENYAM_MIDLINE_MARGIN_M,
                name="rig midline",
            ),
        ),
        point_offset_m=DUAL_OPENYAM_FINGERTIP_OFFSET_M,
    )


def dual_openyam_calibration_ranges(side: str) -> dict[str, tuple[float, float]]:
    """The pose-generation boxes for one arm, rad by joint name."""
    return {
        f"{side}_joint{number}": (math.radians(low), math.radians(high))
        for number, (low, high) in DUAL_OPENYAM_CALIBRATION_RANGES_DEG.items()
    }


def dual_openyam_grid_steps(side: str) -> dict[str, float]:
    """The bench's pose grid steps for one arm, rad by joint name."""
    return {
        f"{side}_joint{number}": math.radians(step)
        for number, step in DUAL_OPENYAM_GRID_STEP_DEG.items()
    }


def dual_openyam_calibration_specs(
    *, base_spacing_m: float = DUAL_OPENYAM_BENCH_BASE_SPACING_M
) -> list[ArmCalibrationSpec]:
    """Both arms' specs with the bench guard, boxes and grid: "dual_openyam/left" and ".../right"."""
    ranges: dict[str, tuple[float, float]] = {}
    steps: dict[str, float] = {}
    for side in DUAL_OPENYAM_SIDES:
        ranges.update(dual_openyam_calibration_ranges(side))
        steps.update(dual_openyam_grid_steps(side))
    return specs_from_damiao_robot(
        DUAL_OPENYAM,
        workspace={
            side: dual_openyam_bench_workspace(side, base_spacing_m=base_spacing_m)
            for side in DUAL_OPENYAM_SIDES
        },
        joint_ranges=ranges,
        grid_step_rad=steps,
    )
