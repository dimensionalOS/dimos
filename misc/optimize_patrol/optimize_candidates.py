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

"""Find the optimal _candidates_to_consider value for CoveragePatrolRouter.

For each candidate count, runs until TARGET_COVERAGE is reached and measures:
  - Average next_goal() call duration (the planning cost)
  - Distance traveled to reach the target coverage (path quality)

Produces a single dual-axis chart with both metrics.
"""

from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import subprocess
import sys

import matplotlib.pyplot as plt
import numpy as np

CANDIDATE_VALUES = list(range(1, 16))
N_ITERATIONS = 9
TARGET_COVERAGE = 0.25
MAX_WORKERS = 32


def run_child(candidates: int) -> tuple[int, float, float]:
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "misc.optimize_patrol.optimize_candidates_child",
            "--candidates",
            str(candidates),
            "--target_coverage",
            str(TARGET_COVERAGE),
            "--n_iterations",
            str(N_ITERATIONS),
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"FAILED candidates={candidates}", file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        return (candidates, float("nan"), float("nan"))
    data = json.loads(result.stdout.strip())
    return (candidates, data["avg_next_goal_time"], data["distance"])


def main() -> None:
    print(f"Sweeping candidates_to_consider in {CANDIDATE_VALUES}")
    print(
        f"  {N_ITERATIONS} iterations each, target coverage={TARGET_COVERAGE}, up to {MAX_WORKERS} workers"
    )

    results: dict[int, tuple[float, float]] = {}

    with ProcessPoolExecutor(max_workers=MAX_WORKERS) as pool:
        futures = {pool.submit(run_child, c): c for c in CANDIDATE_VALUES}
        for i, future in enumerate(as_completed(futures), 1):
            cand, avg_time, distance = future.result()
            results[cand] = (avg_time, distance)
            print(
                f"[{i}/{len(CANDIDATE_VALUES)}] candidates={cand}"
                f" -> avg_next_goal={avg_time * 1000:.1f}ms  distance={distance:.0f}m"
            )

    xs = sorted(results.keys())
    avg_times_ms = np.array([results[x][0] * 1000 for x in xs])  # Convert to ms.
    distances = np.array([results[x][1] for x in xs])

    fig, ax1 = plt.subplots(figsize=(9, 5))
    color_time = "#FF5722"
    color_dist = "#2196F3"

    ax1.set_xlabel("candidates_to_consider")
    ax1.set_ylabel("Avg next_goal() duration (ms)", color=color_time)
    ax1.plot(
        xs,
        avg_times_ms,
        "s-",
        color=color_time,
        linewidth=2,
        markersize=6,
        label="Avg planning time",
    )
    ax1.tick_params(axis="y", labelcolor=color_time)
    ax1.set_xticks(xs)

    ax2 = ax1.twinx()
    ax2.set_ylabel("Distance to reach target (m)", color=color_dist)
    ax2.plot(xs, distances, "o-", color=color_dist, linewidth=2, markersize=6, label="Distance")
    ax2.tick_params(axis="y", labelcolor=color_dist)

    fig.suptitle(
        f"Planning cost vs path quality to reach {TARGET_COVERAGE:.0%} coverage"
        f"  (median of {N_ITERATIONS} iters)"
    )
    fig.tight_layout()
    out = "candidates_optimization.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"\nChart saved to {out}")
    plt.close(fig)

    # Summary table.
    print("\n--- Summary ---")
    print(f"{'candidates':>12} {'avg_time(ms)':>14} {'distance(m)':>14}")
    for x in xs:
        avg_t, dist = results[x]
        print(f"{x:>12} {avg_t * 1000:>14.1f} {dist:>14.0f}")


if __name__ == "__main__":
    main()
