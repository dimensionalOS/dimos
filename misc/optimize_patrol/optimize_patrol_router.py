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

"""Parent process: matrix-test saturation_threshold and clearance_radius_m."""

from concurrent.futures import ProcessPoolExecutor, as_completed
import itertools
import subprocess
import sys

import matplotlib.pyplot as plt
import numpy as np

N_POINTS_SAT = 9
N_POINTS_CLR = 10
N_ITERATIONS = 5
TOTAL_DISTANCE = 4000.0
MAX_WORKERS = 32

SAT_MIN, SAT_MAX = 0.1, 0.9
CLR_MIN, CLR_MAX = 0.1, 1.0


def run_child(saturation_threshold: float, clearance_radius_m: float) -> tuple[float, float, float]:
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "misc.optimize_patrol.optimize_patrol_router_child",
            "--saturation_threshold",
            str(saturation_threshold),
            "--clearance_radius_m",
            str(clearance_radius_m),
            "--n_iterations",
            str(N_ITERATIONS),
            "--total_distance",
            str(TOTAL_DISTANCE),
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"FAILED sat={saturation_threshold} clr={clearance_radius_m}", file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        return (saturation_threshold, clearance_radius_m, float("nan"))
    score = float(result.stdout.strip())
    return (saturation_threshold, clearance_radius_m, score)


def main() -> None:
    sat_values = np.linspace(SAT_MIN, SAT_MAX, N_POINTS_SAT)
    clr_values = np.linspace(CLR_MIN, CLR_MAX, N_POINTS_CLR)
    combos = list(itertools.product(sat_values, clr_values))

    print(f"Running {len(combos)} combinations with up to {MAX_WORKERS} workers...")

    results: dict[tuple[float, float], float] = {}

    with ProcessPoolExecutor(max_workers=MAX_WORKERS) as pool:
        futures = {pool.submit(run_child, sat, clr): (sat, clr) for sat, clr in combos}
        for i, future in enumerate(as_completed(futures), 1):
            sat, clr, score = future.result()
            results[(sat, clr)] = score
            print(f"[{i}/{len(combos)}] sat={sat:.3f} clr={clr:.3f} -> score={score:.4f}")

    # Build matrix for plotting.
    matrix = np.zeros((N_POINTS_SAT, N_POINTS_CLR))
    for i, sat in enumerate(sat_values):
        for j, clr in enumerate(clr_values):
            matrix[i, j] = results.get((sat, clr), float("nan"))

    fig, ax = plt.subplots(figsize=(8, 6))
    im = ax.imshow(matrix, origin="lower", aspect="auto", cmap="viridis")
    ax.set_xticks(range(N_POINTS_CLR))
    ax.set_xticklabels([f"{v:.2f}" for v in clr_values])
    ax.set_yticks(range(N_POINTS_SAT))
    ax.set_yticklabels([f"{v:.2f}" for v in sat_values])
    ax.set_xlabel("clearance_radius_m")
    ax.set_ylabel("saturation_threshold")
    ax.set_title(f"Coverage score (median of {N_ITERATIONS} iters, {TOTAL_DISTANCE}m walk)")
    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label("Coverage (fraction of free cells visited)")

    # Annotate cells with values.
    for i in range(N_POINTS_SAT):
        for j in range(N_POINTS_CLR):
            val = matrix[i, j]
            if not np.isnan(val):
                ax.text(
                    j,
                    i,
                    f"{val:.3f}",
                    ha="center",
                    va="center",
                    fontsize=8,
                    color="white" if val < (np.nanmax(matrix) + np.nanmin(matrix)) / 2 else "black",
                )

    out_path = "patrol_router_optimization.png"
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Chart saved to {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
