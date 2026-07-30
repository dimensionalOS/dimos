# Copyright 2025-2026 Dimensional Inc.
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

"""Autotune orchestration: episodes -> FOPDT fit -> bandwidth -> tune -> emit.

The offline pipeline (:func:`autotune_offline`) is the heart of autotune and is
fully testable: given recorded step segments per channel, it fits the FOPDT with
the user-selected fitter, derives bandwidth, lambda-tunes each channel, and
populates the profile's measured slot plus the tuned artifact + characterization
report. It needs no robot - a collection recorded once can be re-fit any number
of times.

The live wrapper (drive the battery, record, read episodes back) lives in the
entry point; it feeds this function the segments it reads from the store.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from dimos.control.autotune.bandwidth import bandwidth_hz
from dimos.control.autotune.fit.pose_fopdt import estimate_deadtime, fit_pose_fopdt_multi
from dimos.control.autotune.fit.velocity_fopdt import fit_fopdt
from dimos.control.autotune.profile import ChannelFopdt, RobotProfile
from dimos.control.autotune.properties import direction_asymmetry
from dimos.control.autotune.report import (
    build_characterization_report,
    build_tuned_artifact,
)
from dimos.control.autotune.tune import ChannelTuning, tune_channel_full

# One recorded excitation segment: time relative to the step edge, the measured
# channel (velocity for the velocity fitter, pose for the pose fitter), and the
# signed commanded amplitude.
Segment = tuple[np.ndarray, np.ndarray, float]


def _pool_median(values: list[float]) -> float:
    finite = [v for v in values if np.isfinite(v)]
    return float(np.median(finite)) if finite else float("nan")


def fit_channel(profile: RobotProfile, channel: str, segments: list[Segment]) -> ChannelFopdt:
    """Fit one channel's FOPDT from its recorded segments, using the profile's
    declared fitter. Pools K/tau/L across amplitudes and records per-direction K
    so direction asymmetry is visible downstream."""
    if not segments:
        raise ValueError(f"channel {channel!r}: no segments to fit")

    K_fwd = K_rev = None
    if profile.fitter == "velocity":
        # Keep each fit paired with its signed amplitude so the main pool and the
        # per-direction split apply the SAME plausibility gate (mismatched gates
        # let boundary-landing fits bias the directional medians and mask real
        # asymmetry).
        fits = [(fit_fopdt(t, y, amp), amp) for t, y, amp in segments]
        usable = [(f, amp) for f, amp in fits if f.converged and f.plausible]
        if not usable:
            usable = [(f, amp) for f, amp in fits if f.converged]
        if not usable:
            return ChannelFopdt(K=float("nan"), tau=float("nan"), L=float("nan"), plausible=False)
        K = _pool_median([f.K for f, _ in usable])
        tau = _pool_median([f.tau for f, _ in usable])
        L = _pool_median([f.L for f, _ in usable])
        r2 = _pool_median([f.r_squared for f, _ in usable])
        plausible = all(f.plausible for f, _ in usable)
        # Per-direction K from the same usable set.
        fwd = [f.K for f, amp in usable if amp > 0]
        rev = [f.K for f, amp in usable if amp < 0]
        if fwd and rev:
            kf, kr = _pool_median(fwd), _pool_median(rev)
            if direction_asymmetry(kf, kr).asymmetric:
                K_fwd, K_rev = kf, kr
    else:  # pose output-error
        # Decouple L by profiling on the largest-amplitude segment, then joint-fit.
        biggest = max(segments, key=lambda s: abs(s[2]))
        L, _, _ = estimate_deadtime(biggest[0], biggest[1], biggest[2])
        joint = fit_pose_fopdt_multi(segments, L)
        K, tau, r2, plausible = joint.K, joint.tau, joint.r_squared, joint.valid

    return ChannelFopdt(
        K=K, tau=tau, L=L, K_forward=K_fwd, K_reverse=K_rev, r2=r2, plausible=plausible
    )


@dataclass
class AutotuneOutputs:
    """Everything an autotune run produces: the populated profile, the tuned
    artifact dict, the characterization report dict, and the per-channel
    tuning records."""

    profile: RobotProfile
    artifact: dict[str, Any]
    report: dict[str, Any]
    tunings: dict[str, ChannelTuning]


def autotune_offline(
    profile: RobotProfile,
    segments_by_channel: dict[str, list[Segment]],
    *,
    robot_id: str,
    sim_or_hw: str,
    fit_quality_gate: float = 0.8,
    **artifact_kwargs: Any,
) -> AutotuneOutputs:
    """Fit, derive bandwidth, tune, and emit - the full offline pipeline.

    Populates ``profile.measured`` in place and returns the artifact + report.
    Channels whose fit is implausible are tuned-skipped (no gains emitted) but
    still reported. ``sim_or_hw`` gates the artifact's ``valid_for_tuning``."""
    tunings: dict[str, ChannelTuning] = {}
    for channel in profile.channel_names:
        segments = segments_by_channel.get(channel, [])
        fopdt = fit_channel(profile, channel, segments)
        profile.measured.fopdt[channel] = fopdt

        bw = bandwidth_hz(fopdt.K, fopdt.tau, fopdt.L, r2=fopdt.r2, quality_gate=fit_quality_gate)
        if bw is not None:
            profile.measured.bandwidth_hz[channel] = bw

        # Tune only channels with a trustworthy fit.
        if fopdt.plausible and np.isfinite([fopdt.K, fopdt.tau, fopdt.L]).all() and fopdt.tau > 0:
            vmax = profile.channel(channel).vmax
            tunings[channel] = tune_channel_full(
                channel, fopdt.K, fopdt.tau, fopdt.L, saturation_limits=(-vmax, vmax)
            )

    artifact = build_tuned_artifact(
        profile, tunings, robot_id=robot_id, sim_or_hw=sim_or_hw, **artifact_kwargs
    )
    report = build_characterization_report(profile, tunings)
    return AutotuneOutputs(profile=profile, artifact=artifact, report=report, tunings=tunings)
