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

"""Synthetic fixture runs for the mapping-localization benchmark devtool.

Mirrors dimos/protocol/pubsub/benchmark/testdata.py's shape (a registry list
of cases, appended to as each is defined) but generates RunRecord sequences
shaped like ../../../trial/scripts/metrics_logger.py's JSONL output instead of
pubsub messages. Each case carries its own known-correct `expected` metrics
dict (verified by hand against the ported computations in tool_benchmark.py)
so test_benchmark_metrics.py can assert the port reproduces them exactly.
"""

from dataclasses import dataclass
from typing import Any

from dimos.mapping.benchmark.type import RunRecord

# Identity rotation for every synthetic pose -- rotation isn't exercised by
# most of the ported metrics (loop-closure/ATE-proxy are translation-only),
# so a fixed placeholder keeps most fixtures focused on what's actually
# measured. The start/end referee fixtures below (Cases 4/5) are the
# exception -- they exercise _quat_angle_deg too, so they pass an explicit
# rotation instead of relying on this default.
_IDENTITY_ROTATION = (0.0, 0.0, 0.0, 1.0)

# marker_id shared by both start/end referee fixtures (Cases 4/5) -- matches
# trial/scripts/out/fixtures/holdout-referee_*.jsonl's marker_id=42.
_START_END_TAG_ID = 42

_POSE_FRAMES: dict[str, tuple[str, str]] = {
    "odom_pose": ("world", "base_link"),
    "corrected_pose": ("map", "base_link"),
    "correction_new": ("world", "map"),
    "correction_hold": ("world", "map"),
}


def _pose(
    ts: float,
    kind: str,
    xyz: tuple[float, float, float],
    magnitude_m: float | None = None,
    rotation: tuple[float, float, float, float] = _IDENTITY_ROTATION,
) -> RunRecord:
    frame_id, child_frame_id = _POSE_FRAMES[kind]
    return RunRecord(
        type=kind,
        ts=ts,
        frame_id=frame_id,
        child_frame_id=child_frame_id,
        translation=xyz,
        rotation=rotation,
        magnitude_m=magnitude_m,
    )


def _log_event(ts: float, logger: str, event: str) -> RunRecord:
    return RunRecord(type="log_event", ts=ts, level="WARNING", logger=logger, event=event)


def _tag_sighting(
    ts: float,
    xyz: tuple[float, float, float],
    rotation: tuple[float, float, float, float],
    reprojection_error_px: float,
    marker_id: int = _START_END_TAG_ID,
) -> RunRecord:
    """A tag_sighting record -- ported from metrics_logger.py's --holdout-tag
    path: one independent-solvePnP reading (marker_<id> -> camera_optical,
    i.e. the camera's pose IN the withheld tag's own fixed frame) per frame
    the tag was seen, logged regardless of which mode (odom/marker/lidar) is
    under test."""
    return RunRecord(
        type="tag_sighting",
        ts=ts,
        frame_id=f"marker_{marker_id}",
        child_frame_id="camera_optical",
        translation=xyz,
        rotation=rotation,
        marker_id=marker_id,
        reprojection_error_px=reprojection_error_px,
    )


@dataclass
class FixtureCase:
    name: str
    route: str
    mode: str
    duration_s: float
    records: list[RunRecord]
    expected: dict[str, Any]
    start_end_tag: int | None = None


testcases: list[FixtureCase] = []


# -- Case 1: odom-only, zero corrections ------------------------------------
# A perfect physical loop (start == end) with no map correction landing at
# all -- the "nothing to fuse yet" baseline every route starts from.

_odom_only_records = [
    _pose(0.0, "odom_pose", (0.0, 0.0, 0.0)),
    _pose(1.0, "odom_pose", (1.0, 0.0, 0.0)),
    _pose(2.0, "odom_pose", (0.0, 0.0, 0.0)),
]

testcases.append(
    FixtureCase(
        name="odom-only-no-corrections",
        route="baseline-loop",
        mode="odom",
        duration_s=2.0,
        records=_odom_only_records,
        expected={
            "odom_ticks": 3,
            "corrected_ticks": 0,
            "corrected_pose_coverage": 0.0,
            "corrections_accepted": 0,
            "corrections_held": 0,
            "corrections_rejected": 0,
            "correction_mag_mean_m": None,
            "correction_mag_max_m": None,
            "loop_closure_error_odom_m": 0.0,
            "loop_closure_error_corrected_m": None,
            "ate_proxy_rmse_m": None,
            "ate_proxy_n_matched": 0,
            "marker_log_events": 0,
            "lidar_log_events": 0,
        },
    )
)


# -- Case 2: marker mode, drift + a landed correction + one rejected gate ---
# Odom drifts 0.1118m short of closing the loop; corrected_pose tracks the
# same square but snaps exactly back to the start on the last tick -- the
# "corrections tighten the loop" story head-to-head reports look for. Also
# carries one accepted correction, one held (unchanged) republish, and one
# gate-rejected detection, to exercise every corrections_* counter and the
# marker-side of the log_event source breakdown.

_marker_drift_records = [
    _pose(0.0, "odom_pose", (0.0, 0.0, 0.0)),
    _pose(1.0, "odom_pose", (1.0, 0.0, 0.0)),
    _pose(2.0, "odom_pose", (2.0, 0.0, 0.0)),
    _pose(3.0, "odom_pose", (2.0, 1.0, 0.0)),
    _pose(4.0, "odom_pose", (1.0, 1.0, 0.0)),
    _pose(5.0, "odom_pose", (0.1, 0.05, 0.0)),  # 0.1118m short of the start
    _pose(0.0, "corrected_pose", (0.0, 0.0, 0.0)),
    _pose(1.0, "corrected_pose", (1.0, 0.0, 0.0)),
    _pose(2.0, "corrected_pose", (2.0, 0.0, 0.0)),
    _pose(3.0, "corrected_pose", (2.0, 1.0, 0.0)),
    _pose(4.0, "corrected_pose", (1.0, 1.0, 0.0)),
    _pose(5.0, "corrected_pose", (0.0, 0.0, 0.0)),  # snaps back to the start
    _pose(0.0, "correction_new", (0.0, 0.0, 0.0), magnitude_m=None),  # first-ever, no prior
    _pose(2.5, "correction_new", (0.0, 0.0, 0.05), magnitude_m=0.05),  # accepted
    _pose(4.0, "correction_hold", (0.0, 0.0, 0.05), magnitude_m=0.005),  # unchanged republish
    _log_event(1.5, "MarkerLocalizationModule", "gate rejected (2 tags seen)"),
]

testcases.append(
    FixtureCase(
        name="marker-loop-drift-corrected",
        route="drift-recovery",
        mode="marker",
        duration_s=12.5,
        records=_marker_drift_records,
        expected={
            "odom_ticks": 6,
            "corrected_ticks": 6,
            "corrected_pose_coverage": 1.0,
            "corrections_accepted": 2,
            "corrections_held": 1,
            "corrections_rejected": 1,
            "correction_mag_mean_m": 0.05,
            "correction_mag_max_m": 0.05,
            "loop_closure_error_odom_m": 0.1118,
            "loop_closure_error_corrected_m": 0.0,
            "ate_proxy_rmse_m": 0.0456,
            "ate_proxy_n_matched": 6,
            "marker_log_events": 1,
            "lidar_log_events": 0,
        },
    )
)


# -- Case 3: lidar mode, every gate attempt rejected ------------------------
# RelocalizationModule sees tags/features but never clears the acceptance
# gate -- zero corrected_pose samples, three rejects, all lidar-sourced. Tests
# the acceptance-rate-of-zero path and the lidar side of the source
# breakdown (Case 2 already covers marker).

_lidar_rejected_records = [
    _pose(0.0, "odom_pose", (0.0, 0.0, 0.0)),
    _pose(1.0, "odom_pose", (2.0, 0.0, 0.0)),
    _pose(2.0, "odom_pose", (2.0, 2.0, 0.0)),
    _pose(3.0, "odom_pose", (0.05, -0.05, 0.0)),  # 0.0707m short of the start
    _log_event(0.5, "RelocalizationModule", "gate rejected (1 tags seen)"),
    _log_event(1.5, "RelocalizationModule", "gate rejected (1 tags seen)"),
    _log_event(2.5, "RelocalizationModule", "gate rejected (3 tags seen)"),
]

testcases.append(
    FixtureCase(
        name="lidar-all-rejected",
        route="glass-lobby",
        mode="lidar",
        duration_s=9.8,
        records=_lidar_rejected_records,
        expected={
            "odom_ticks": 4,
            "corrected_ticks": 0,
            "corrected_pose_coverage": 0.0,
            "corrections_accepted": 0,
            "corrections_held": 0,
            "corrections_rejected": 3,
            "correction_mag_mean_m": None,
            "correction_mag_max_m": None,
            "loop_closure_error_odom_m": 0.0707,
            "loop_closure_error_corrected_m": None,
            "ate_proxy_rmse_m": None,
            "ate_proxy_n_matched": 0,
            "marker_log_events": 0,
            "lidar_log_events": 3,
        },
    )
)


# -- Cases 4/5: start/end referee (a withheld marker tag) -------------------
# Ported from trial/scripts/out/fixtures/holdout-referee_{odom,marker}.jsonl,
# hand-verified against trial/scripts/bench.py's _holdout_referee_metrics()
# in trial/scripts/tests/test_holdout_fixture_metrics.py:
#   odom-claim run:   start/end closure error 0.2959m / 10.0deg (claim falls
#                      back to odom_pose -- no corrections landed this run)
#   marker-claim run: start/end closure error 0.0014m /  1.0deg (claim =
#                      corrected_pose)
# Both runs see the SAME withheld-tag (marker_42) sighting stream -- two
# 10-reading clusters near drill-start (ts 0.0-4.5) and drill-end
# (ts 15.5-20.0), the tag rotated ~10deg about z between clusters. Only the
# claim source (odom vs. corrected) differs between the two cases, which is
# the whole point of the referee: it scores each mode's own claim against the
# same map-independent reference.

_TEN_DEG_Z = (0.0, 0.0, 0.0871557427, 0.9961946981)  # ~10deg about z
_NINE_DEG_Z = (0.0, 0.0, 0.0784590957, 0.9969173337)  # ~9deg about z

# odom_pose ticks every 0.5s for 20s, drifting +0.01m/tick along x -- shared
# by both cases (the physical motion under test is identical either way).
_start_end_odom_ticks = [_pose(0.5 * i, "odom_pose", (0.01 * i, 0.0, 0.02)) for i in range(41)]

# The withheld tag's own sightings -- identical in both cases (same physical
# tag, same drill). First cluster near drill-start, second near drill-end,
# rotated ~10deg about z from the first (the true physical displacement the
# referee measures).
_start_end_sightings = [
    _tag_sighting(0.5 * i, (0.02, 0.0, 0.3), _IDENTITY_ROTATION, 0.8) for i in range(10)
] + [_tag_sighting(15.5 + 0.5 * i, (0.03, 0.01, 0.3), _TEN_DEG_Z, 0.9) for i in range(10)]

testcases.append(
    FixtureCase(
        name="start-end-referee-odom-claim",
        route="holdout-referee",
        mode="odom",
        duration_s=21.0,
        records=[*_start_end_odom_ticks, *_start_end_sightings],
        start_end_tag=_START_END_TAG_ID,
        expected={
            "start_end_tag_id": _START_END_TAG_ID,
            "start_end_readings_start": 10,
            "start_end_readings_end": 10,
            "start_end_closure_error_m": 0.2959,
            "start_end_closure_error_deg": 10.0,
        },
    )
)

# corrected_pose ticks tracking the same drill, +0.0005m/tick, rotated
# ~9deg about z from ts=5.0 onward -- closes the loop far tighter than raw
# odom against the same withheld-tag reference.
_start_end_corrected_ticks = [
    _pose(
        0.5 * i,
        "corrected_pose",
        (0.02 + 0.0005 * i, 0.0, 0.02),
        rotation=_IDENTITY_ROTATION if i < 10 else _NINE_DEG_Z,
    )
    for i in range(41)
]

testcases.append(
    FixtureCase(
        name="start-end-referee-marker-claim",
        route="holdout-referee",
        mode="marker",
        duration_s=21.0,
        records=[*_start_end_odom_ticks, *_start_end_corrected_ticks, *_start_end_sightings],
        start_end_tag=_START_END_TAG_ID,
        expected={
            "start_end_tag_id": _START_END_TAG_ID,
            "start_end_readings_start": 10,
            "start_end_readings_end": 10,
            "start_end_closure_error_m": 0.0014,
            "start_end_closure_error_deg": 1.0,
        },
    )
)
