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

"""Lidar-frame-to-planner latency from a DIMOS_LOG_LEVEL=DEBUG run log.

    python dimos/mapping/gpu_mls/demo_region_latency.py <run.log> [--skip-s 30]

Joins the ray tracer's "region sent" lines (when the lidar frame arrived) with
the planner's "region applied" lines (when its region update finished) by
stamp, and prints the latency and per-stage medians and p95s. Cells retuned
come from the planner's throttled stage lines, one per few seconds.
"""

from __future__ import annotations

import argparse
import re
import statistics

FIELD = re.compile(r"(\w+)=(\S+)")
PERCENTILE_95 = 0.95


def fields(line: str) -> dict[str, str]:
    return dict(FIELD.findall(line))


def summary(name: str, values: list[float]) -> str:
    if not values:
        return f"{name:>18}: none"
    ordered = sorted(values)
    p95 = ordered[round((len(ordered) - 1) * PERCENTILE_95)]
    return (
        f"{name:>18}: n {len(ordered):6d} median {statistics.median(ordered):8.2f}"
        f" p95 {p95:8.2f} max {ordered[-1]:8.2f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("log")
    parser.add_argument("--skip-s", type=float, default=30.0, help="startup seconds left out")
    args = parser.parse_args()

    sent: dict[str, dict[str, str]] = {}
    applied: dict[str, dict[str, str]] = {}
    errors = 0
    retuned: list[float] = []
    with open(args.log, errors="replace") as log:
        for line in log:
            if "[err]" in line or " ERROR " in line:
                errors += 1
            if "region sent" in line:
                f = fields(line)
                sent[f["stamp"]] = f
            elif "gpu surface region update stages" in line:
                retuned.append(float(fields(line).get("cells_retuned", "0")))
            elif "region applied" in line:
                f = fields(line)
                applied[f["stamp"]] = f

    if not sent:
        raise SystemExit("no 'region sent' lines; was DIMOS_LOG_LEVEL=DEBUG set?")
    start = min(float(f["received_unix_s"]) for f in sent.values()) + args.skip_s
    latency, gpu_latency, tracer_ms, planner_ms = [], [], [], []
    dirty, bricks, region_bricks, rows = [], [], [], []
    for stamp, s in sent.items():
        if float(s["received_unix_s"]) < start:
            continue
        tracer_ms.append(float(s["process_ms"]))
        if "region_bricks" in s and float(s["region_bricks"]) > 0:
            dirty.append(float(s["dirty_columns"]))
            bricks.append(float(s["surface_bricks"]))
            region_bricks.append(float(s["region_bricks"]))
        a = applied.get(stamp)
        if a is None:
            continue
        ms = (float(a["applied_unix_s"]) - float(s["received_unix_s"])) * 1e3
        latency.append(ms)
        planner_ms.append(float(a["process_ms"]))
        if a.get("gpu_surface", "").lower() == "true":
            gpu_latency.append(ms)
            rows.append(float(a.get("surface_rows", "0")))

    print(f"{args.log}: {len(sent)} regions sent, {len(applied)} applied, {errors} error lines")
    print(summary("latency ms", latency))
    print(summary("  on gpu surfaces", gpu_latency))
    print(summary("ray tracer ms", tracer_ms))
    print(summary("planner ms", planner_ms))
    print(summary("surface rows", rows))
    print(summary("dirty columns", dirty))
    print(summary("window bricks", bricks))
    print(summary("region bricks", region_bricks))
    print(summary("cells retuned", retuned))


if __name__ == "__main__":
    main()
