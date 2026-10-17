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

"""Summarize mls_load.py runs -> <name>.mls_summary.json

usage: mls_report.py <name>
- whole: MLS full-rebuild cost per published map, the publish rate, and the backlog that implies.
- region: MLS cost per incremental region update.
- patch: MLS cost of applying gsc's corrections in place (mls_load.py region --patch-from).
- goal drift: Ivan's map is re-anchored and re-corrected on every rebuild, and nothing tells a consumer how
  to move a goal it holds. For a goal set at a keyframe's place, how far that place has moved in the
  published map by the time later rebuilds land.
"""

import json
import sys

import numpy as np
from paths import OUT

PUBLISH_PERIOD_S = 0.5  # PGOVoxelMapper emit_every=5 at 10 Hz


def stats(values) -> dict:
    values = np.asarray(values, float)
    return {
        "n": len(values),
        "mean": round(float(values.mean()), 2),
        "median": round(float(np.median(values)), 2),
        "p90": round(float(np.percentile(values, 90)), 2),
        "p99": round(float(np.percentile(values, 99)), 2),
        "max": round(float(values.max()), 2),
    }


def goal_drift(rebuilds: list, horizons_s=(60, 300, 900)) -> dict:
    times = np.array([r["ts"] for r in rebuilds])
    xyz = [np.array(r["keyframe_xyz"]) for r in rebuilds]
    out = {}
    for horizon in horizons_s:
        shifts = []
        for i, set_at in enumerate(times):
            later = np.searchsorted(times, set_at + horizon)
            if later >= len(times):
                break
            known = len(xyz[i])
            shifts.append(np.linalg.norm(xyz[later][:known] - xyz[i], axis=1))
        if shifts:
            out[f"after_{horizon}s"] = stats(np.concatenate(shifts))
    next_jump = [
        np.linalg.norm(xyz[i + 1][: len(xyz[i])] - xyz[i], axis=1) for i in range(len(xyz) - 1)
    ]
    out["per_rebuild"] = stats(np.concatenate(next_jump))
    out["rebuilds"] = len(rebuilds)
    return out


def patch_summary(name: str, duration_s: float, label: str = "patch") -> dict | None:
    path = OUT / f"{name}.mls_{label}.json"
    if not path.exists():
        return None
    patches = np.array(json.loads(path.read_text())["patches"] or [[0, 0, 0, 0]])
    return {
        "patches": len(patches),
        "ms_per_patch": stats(patches[:, 1]),
        "cells_per_patch": stats(patches[:, 3]),
        "points_per_patch": stats(patches[:, 2]),
        "mls_seconds_per_hour": round(float(patches[:, 1].sum()) / 1e3 * 3600 / duration_s),
    }


def main() -> None:
    name = sys.argv[1]
    whole = json.loads((OUT / f"{name}.mls_whole.json").read_text())
    region = json.loads((OUT / f"{name}.mls_region.json").read_text())
    timed = np.array(whole["timed"])
    duration_s = region["timed"][-1][0] - region["timed"][0][0]
    rebuild_rows = timed[timed[:, 3] == 1]
    closure = np.array(region.get("closure_rebuilds") or [[0, 0, 0, 0]])
    t0 = timed[0, 0]
    ms, points = timed[:, 1], timed[:, 2]
    per_million = ms / np.maximum(points, 1) * 1e6
    summary = {
        "whole": {
            "publishes": whole["publishes"],
            "timed": len(timed),
            "publish_period_s": PUBLISH_PERIOD_S,
            "ms_per_publish": stats(ms),
            "map_points": stats(points),
            "ms_per_million_points": stats(per_million),
            "ms_per_publish_last_10min": stats(ms[timed[:, 0] >= timed[-1, 0] - 600]),
            "seconds_of_mls_per_second_of_driving": round(
                float(np.median(ms[timed[:, 0] >= timed[-1, 0] - 600])) / 1e3 / PUBLISH_PERIOD_S, 1
            ),
            "first_minute_over_budget": round(
                float((timed[np.argmax(ms > PUBLISH_PERIOD_S * 1e3), 0] - t0) / 60), 1
            ),
            "peak_rss_mb": round(whole["peak_rss_mb"]),
            "downsampled_to_planner_voxel": {
                "ms_per_publish": stats(timed[:, 4]),
                "map_points": stats(timed[:, 5]),
                "seconds_of_mls_per_second_of_driving": round(
                    float(np.median(timed[timed[:, 0] >= timed[-1, 0] - 600, 4]))
                    / 1e3
                    / PUBLISH_PERIOD_S,
                    1,
                ),
            },
            # a consumer that skips the every-5th-frame publishes and only takes his rebuilds
            "rebuilds_only": {
                "rebuilds": len(rebuild_rows),
                "ms_per_rebuild": stats(rebuild_rows[:, 1]),
                "ms_per_rebuild_downsampled": stats(rebuild_rows[:, 4]),
                "mls_seconds_per_hour": round(
                    float(rebuild_rows[:, 1].sum()) / 1e3 * 3600 / duration_s
                ),
                "mls_seconds_per_hour_downsampled": round(
                    float(rebuild_rows[:, 4].sum()) / 1e3 * 3600 / duration_s
                ),
            },
        },
        "region": {
            "ms_per_frame": stats([row[1] for row in region["timed"]]),
            "region_points": stats([row[2] for row in region["timed"]]),
            "peak_rss_mb": round(region["peak_rss_mb"]),
            "mls_seconds_per_hour": round(
                sum(row[1] for row in region["timed"]) / 1e3 * 3600 / duration_s
            ),
        },
        # gsc's closures charged a full rebuild of the ray tracer map (10 s cooldown): upper bound
        "gsc_closure_full_rebuilds": {
            "closures": region.get("closures", 0),
            "rebuilds": len(region.get("closure_rebuilds") or []),
            "ms_per_rebuild": stats(closure[:, 1]),
            "map_points": stats(closure[:, 2]),
            "mls_seconds_per_hour": round(float(closure[:, 1].sum()) / 1e3 * 3600 / duration_s),
        },
        # gsc's corrections applied in place: only cells whose keyframes moved go to update_region
        "gsc_patch_in_place": patch_summary(name, duration_s),
        "goal_drift_m": goal_drift(whole["rebuilds"]),
        # the same for gsc's map frame, from its keyframes after each closure (pgo-eval --loop-snapshots)
        "goal_drift_gsc_m": goal_drift(
            [
                {"ts": ts, "keyframe_xyz": xyz}
                for ts, xyz in json.loads(
                    (OUT / "timing" / f"{name}.gsc_snapshots.json").read_text()
                )["loop_snapshots"]
            ]
        ),
        # Ivan's rebuilds patched the same way (mls_load.py --patch-from <name>.mls_whole.json)
        "ivan_patch_in_place": patch_summary(name, duration_s, "patch_ivan"),
        "curve_whole": [
            [round((t - t0) / 60, 2), round(m), int(p)] for t, m, p, *_ in timed.tolist()
        ],
    }
    (OUT / f"{name}.mls_summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps({k: v for k, v in summary.items() if k != "curve_whole"}, indent=1))


if __name__ == "__main__":
    main()
