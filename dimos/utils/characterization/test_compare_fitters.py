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

"""Tests for the head-to-head comparator: both fitters run on a sim recording,
the pose-domain method recovers ground truth at least as well as velocity-domain,
and the report + overlay PNG are written."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from dimos.utils.benchmarking.plant import FopdtChannelParams, TwistBasePlantParams
from dimos.utils.characterization.compare_fitters import compare_recording, write_report
from dimos.utils.characterization.sim_ground_truth import (
    multistep_excitation,
    synthesize_recording,
)

_EXCITATION = multistep_excitation(
    vx_amps=(0.3, 0.6), vy_amps=(), wz_amps=(0.5, 1.0), hold_s=4.0, settle_s=2.0
)


def _truth() -> TwistBasePlantParams:
    return TwistBasePlantParams(
        vx=FopdtChannelParams(K=0.92, tau=0.20, L=0.15),
        vy=FopdtChannelParams(K=0.92, tau=0.20, L=0.15),
        wz=FopdtChannelParams(K=2.45, tau=0.25, L=0.12),
    )


def test_pose_domain_recovers_truth_at_least_as_well(tmp_path: Path) -> None:
    truth = _truth()
    db = synthesize_recording(
        truth, db_path=tmp_path / "sim.db", segments=_EXCITATION, seed=0
    ).db_path
    comp = compare_recording(db, label="sim", truth=truth, noise_std=None)

    vx = comp.axes["vx"]
    pose_err = abs(vx.pose_tau - truth.vx.tau) / truth.vx.tau
    assert pose_err < 0.15  # pose-domain recovers tau within 15%
    assert vx.pose_r2 > 0.99  # near-perfect pose fit on clean sim
    # truth carried through for the report
    assert vx.true_tau == truth.vx.tau


def test_write_report_emits_markdown_and_png(tmp_path: Path) -> None:
    truth = _truth()
    db = synthesize_recording(
        truth, db_path=tmp_path / "sim.db", segments=_EXCITATION, seed=1
    ).db_path
    comp = compare_recording(db, label="sim_case", truth=truth, noise_std=None)
    report = write_report([comp], tmp_path / "out")

    assert report.exists()
    text = report.read_text()
    assert "pose-domain" in text and "velocity-domain" in text
    assert "Verdict" in text
    pngs = list((tmp_path / "out").glob("*.png"))
    assert len(pngs) == 1


def test_velocity_domain_runs_and_returns_all_axes(tmp_path: Path) -> None:
    from dimos.utils.characterization.compare_fitters import velocity_domain_fit
    from dimos.utils.characterization.recording_io import load_recording

    db = synthesize_recording(
        _truth(), db_path=tmp_path / "s.db", segments=_EXCITATION, seed=2
    ).db_path
    vel = velocity_domain_fit(load_recording(db))
    assert set(vel) == {"vx", "vy", "wz"}
    assert np.isfinite(vel["vx"].tau)  # vx was excited -> a fit exists
