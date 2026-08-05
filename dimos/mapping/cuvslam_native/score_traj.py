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

"""Score a standalone-harness TUM trajectory against the Point-LIO reference.

    python -m dimos.mapping.cuvslam_native.score_traj <recording> <traj.tum> [...]

cuVSLAM silently restarts its world frame rather than reporting tracking loss, so a
raw trajectory contains teleports. Those are rebased here before scoring -- otherwise
a single restart dominates the ATE and the number says nothing about drift.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

from dimos.mapping.topdown_html import align, pointlio_trajectory, segment_error
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# A handheld rig does not move faster than this; anything above it is a world-frame
# restart rather than motion.
RESTART_SPEED_M_S = 5.0
DATASETS = Path.home() / "datasets" / "d455"


def load_rebased(path: Path) -> tuple[np.ndarray, int]:
    """Load a TUM file and undo cuVSLAM's world-frame restarts."""
    raw = np.loadtxt(path)
    seconds = raw[:, 0] / 1e9
    elapsed = seconds - seconds[0]
    positions = raw[:, 1:4].copy()

    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    intervals = np.maximum(np.diff(elapsed), 1e-3)
    restarts = set(np.flatnonzero(steps > RESTART_SPEED_M_S * intervals).tolist())

    rebased = positions.copy()
    offset = np.zeros(3)
    for index in range(1, len(positions)):
        if (index - 1) in restarts:
            offset += rebased[index - 1] - (positions[index] + offset)
        rebased[index] = positions[index] + offset
    return np.column_stack([elapsed, rebased]), len(restarts)


def score(recording: str, path: Path) -> dict[str, object]:
    reference = pointlio_trajectory(DATASETS / recording / f"{recording}.db")
    trajectory, restarts = load_rebased(path)
    fitted = align(trajectory, reference, False)
    segments = segment_error(trajectory, reference, False)
    return {
        "recording": recording,
        "trajectory": str(path),
        "restarts": restarts,
        "poses": len(trajectory),
        "ate_rmse_m": fitted["ate_rmse_m"],
        "drift_pct": fitted["drift_pct"],
        "segment_median_m": segments["segment_ate_median_m"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording")
    parser.add_argument("trajectories", type=Path, nargs="+")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--save-as", default=None, help="method name; writes <rec>/<name>_traj.npy")
    args = parser.parse_args()

    if args.save_as:
        trajectory, restarts = load_rebased(args.trajectories[0])
        destination = DATASETS / args.recording / f"{args.save_as}_traj.npy"
        np.save(destination, trajectory)
        logger.info("wrote %s (%d poses, %d restarts)", destination, len(trajectory), restarts)

    results = [score(args.recording, path) for path in args.trajectories]
    if args.json:
        print(json.dumps(results, indent=2))
        return 0
    for result in results:
        print(
            f"{Path(str(result['trajectory'])).name:34} "
            f"{result['restarts']:>5} restarts  "
            f"ATE {result['ate_rmse_m']:>8}m  "
            f"drift {result['drift_pct']:>6}%  "
            f"seg {result['segment_median_m']:>7}m"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
