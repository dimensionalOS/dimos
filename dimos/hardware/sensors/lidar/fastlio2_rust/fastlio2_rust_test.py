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

"""End-to-end harness: replay the Livox ``.db`` through the Rust FAST-LIO2 module
and check that the estimated trajectory stays physically plausible.

This is the regression test for the FAST-LIO2 divergence: on the recorded
``fastlio_stairwell_odom_divergence`` slice the pipeline currently blows the odometry up to
hundreds of meters within ~50s. The test asserts a bounded per-frame jump, so it
fails while the divergence is present and passes once the pipeline is fixed.

Marked ``tool`` because it needs the nix-built native binary and the LFS-hosted
recording, so it is excluded from the fast and slow CI lanes. Run it directly::

    uv run pytest dimos/hardware/sensors/lidar/fastlio2_rust/fastlio_rust_test.py
"""

from __future__ import annotations

from pathlib import Path
import time

import numpy as np
import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.hardware.sensors.lidar.fastlio2_rust.fastlio2_rust_replay import LivoxDbReplay
from dimos.hardware.sensors.lidar.fastlio2_rust.module import FastLio2Rust
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.utils.data import get_data

# NOTE: because of fastlio's non-deterministic design
# we need to replay the data at the same real-time
# rate that the data was recorded on.
# Even worse: we need to use a CPU/machine that is
# at least roughly the same as the machine that the live
# data was recorded on, because it makes a difference
# how many loops complete before a new packet comes in
REPLAY_DURATION_SECONDS = 90.0
SHUTDOWN_MARGIN_SECONDS = 5.0
# Hardcoded recording this regression test replays (LFS-pulled on a cache miss).
RECORDING_DATASET = "fastlio_stairwell_odom_divergence.db"
# Odometry runs at ~10Hz; >5m between consecutive frames means ~50 m/s, which is
# divergence rather than real Mid360 motion.
MAX_FRAME_JUMP_M = 5.0


class TrajectoryProbeConfig(ModuleConfig):
    output_file: str = ""


class TrajectoryProbe(Module):
    """Append each odometry position to a file so the test process can read it."""

    config: TrajectoryProbeConfig
    odometry: In[Odometry]

    async def handle_odometry(self, value: Odometry) -> None:
        position = value.pose.pose.position
        with open(self.config.output_file, "a") as handle:
            handle.write(f"{position.x} {position.y} {position.z}\n")


def _read_positions(path: Path) -> np.ndarray:
    rows = [line.split() for line in path.read_text().splitlines() if line.strip()]
    return np.array(rows, dtype=np.float64)


@pytest.mark.tool
@pytest.mark.heavy
def test_fastlio_rust_replay_trajectory_is_bounded(tmp_path: Path) -> None:
    output_file = tmp_path / "trajectory.txt"
    output_file.touch()

    coordinator = ModuleCoordinator.build(
        autoconnect(
            LivoxDbReplay.blueprint(
                dataset=get_data(RECORDING_DATASET), duration=REPLAY_DURATION_SECONDS
            ),
            FastLio2Rust.blueprint(),
            TrajectoryProbe.blueprint(output_file=str(output_file)),
        )
    )
    try:
        time.sleep(REPLAY_DURATION_SECONDS + SHUTDOWN_MARGIN_SECONDS)
    finally:
        coordinator.stop()

    positions = _read_positions(output_file)
    assert len(positions) > 100, f"pipeline produced too few odometry samples: {len(positions)}"
    assert np.isfinite(positions).all(), "odometry produced non-finite positions"

    frame_jumps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    worst_jump = float(frame_jumps.max())
    assert worst_jump < MAX_FRAME_JUMP_M, (
        f"FAST-LIO2 trajectory diverged: largest single-frame jump was {worst_jump:.1f}m "
        f"(limit {MAX_FRAME_JUMP_M}m), peak displacement "
        f"{float(np.linalg.norm(positions, axis=1).max()):.1f}m"
    )
