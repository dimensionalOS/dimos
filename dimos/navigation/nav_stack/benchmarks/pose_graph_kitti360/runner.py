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

"""Generic KITTI-360 loop-closure benchmark for any module satisfying
``LoopClosure`` (see ``dimos/navigation/nav_stack/specs.py``).

The playback + scoring modules wire into the producer via ``autoconnect``;
the runner doesn't care which implementation it is.
"""

from __future__ import annotations

from pathlib import Path
import time
from typing import Any

import numpy as np

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.navigation.nav_stack.benchmarks.pose_graph_kitti360.kitti360_loader import (
    load_kitti360_sequence,
)
from dimos.navigation.nav_stack.benchmarks.pose_graph_kitti360.loop_groundtruth import (
    DEFAULT_MAX_LOOP_DISTANCE_M,
    DEFAULT_MIN_FRAME_GAP,
    compute_loop_groundtruth,
)
from dimos.navigation.nav_stack.benchmarks.pose_graph_kitti360.playback import (
    Kitti360PlaybackModule,
    compute_send_timestamps,
)
from dimos.navigation.nav_stack.benchmarks.pose_graph_kitti360.scoring import (
    PoseGraphScoringModule,
)
from dimos.navigation.nav_stack.specs import LoopClosure
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def run_benchmark(
    module_under_test: type[LoopClosure],
    kitti360_root: Path,
    module_kwargs: dict[str, Any] | None = None,
    sequence_id: int = 2,
    max_scans: int | None = None,
    publish_interval_sec: float = 0.02,
    min_frame_gap: int = DEFAULT_MIN_FRAME_GAP,
    max_loop_distance_m: float = DEFAULT_MAX_LOOP_DISTANCE_M,
    drain_sec: float = 10.0,
    poll_interval_sec: float = 0.5,
) -> dict[str, Any]:
    """Run a pose-graph SLAM blueprint against KITTI-360 and return scores.

    ``module_under_test`` is any Blueprint whose module exposes the
    pose-graph interface (in: ``registered_scan``, ``odometry``;
    out: ``pose_graph``, ``loop_correction_delta``). The runner adds a
    ``Kitti360PlaybackModule`` (publishes the inputs) and a
    ``PoseGraphScoringModule`` (subscribes to the outputs + scores),
    then auto-connects everything into one blueprint.

    Returns a dict with TP/FP/FN, precision, recall, F1, and the raw
    detected-edge / loop-event counts.
    """
    sequence = load_kitti360_sequence(kitti360_root, sequence_id)
    frame_ids = sequence.frame_ids
    if max_scans is not None:
        frame_ids = frame_ids[:max_scans]
    positions = np.array([sequence.lidar_pose(frame_id)[:3, 3] for frame_id in frame_ids])
    groundtruth = compute_loop_groundtruth(
        frame_ids,
        positions,
        min_frame_gap=min_frame_gap,
        max_distance_m=max_loop_distance_m,
    )
    send_timestamps = compute_send_timestamps(sequence.timestamps, frame_ids)

    logger.info(
        f"KITTI-360 seq {sequence_id}: {len(frame_ids)} frames, "
        f"{groundtruth.queries_with_loop} GT queries with loops, "
        f"{groundtruth.total_loop_pairs} GT pairs."
    )

    playback_blueprint = Kitti360PlaybackModule.blueprint(
        kitti360_root=str(kitti360_root),
        sequence_id=sequence_id,
        max_scans=max_scans,
        publish_interval_sec=publish_interval_sec,
    )
    scoring_blueprint = PoseGraphScoringModule.blueprint(
        frame_ids=frame_ids,
        send_timestamps=send_timestamps,
        valid_loops_per_query={
            frame_id: list(valid) for frame_id, valid in groundtruth.valid_loops_per_query.items()
        },
    )

    sut_blueprint = module_under_test.blueprint(**(module_kwargs or {}))
    blueprint = autoconnect(playback_blueprint, scoring_blueprint, sut_blueprint)

    wallclock_start = time.monotonic()
    coordinator = ModuleCoordinator.build(blueprint)
    try:
        playback = coordinator.get_instance(Kitti360PlaybackModule)
        scoring = coordinator.get_instance(PoseGraphScoringModule)

        # Wait for the playback module to finish publishing all scans.
        while not playback.is_finished():
            published = playback.frames_published()
            logger.info(
                f"  playback {published}/{len(frame_ids)} "
                f"({published / max(len(frame_ids), 1) * 100:.0f}%)"
            )
            time.sleep(poll_interval_sec)

        playback_error = playback.playback_error()
        if playback_error is not None:
            raise RuntimeError(
                f"Kitti360PlaybackModule aborted at frame "
                f"{playback.frames_published()}/{len(frame_ids)}: {playback_error}"
            )

        # Drain remaining loop-closure / edge messages from PGO's backlog.
        logger.info(f"playback done, draining for {drain_sec:.1f}s")
        time.sleep(drain_sec)

        results: dict[str, Any] = scoring.get_results()
    finally:
        coordinator.stop()

    results["wallclock_seconds"] = time.monotonic() - wallclock_start
    results["sequence_id"] = sequence_id
    return results
