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

"""Passive timing/noise probe.

Measures how fast data *arrives* and how clean it is, with the robot
stationary and no command motion. This is distinct from plant bandwidth:

  * topic frequency  - how fast odom/sensor messages arrive (Hz). Measured
    here from inter-arrival times.
  * plant bandwidth  - frequency where the plant response rolls off, DERIVED
    from the fitted FOPDT (see bandwidth.py). Not measured here.

The probe's job is to surface the numbers that make the user's fitter choice
obvious - especially the samples-per-tau ratio - WITHOUT deciding for them.
At low odom rate relative to tau, velocity differentiation inflates tau and
the pose output-error fitter is appropriate; but the user owns that call. The
probe only advises (RobotProfile.fitter is never modified here).

The arrival-time math is a pure function (:func:`compute_timing`) so it is
unit-testable independent of any transport; the live collector is a thin
wrapper that records arrival times and stationary sample values.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from dimos.control.autotune.profile import StreamTiming


@dataclass(frozen=True)
class TimingAdvice:
    """Advisory readout for one channel/stream. Informs the user's fitter
    choice; it does not change it."""

    stream: StreamTiming
    nyquist_hz: float
    samples_per_tau: float | None  # None when expected_tau unknown
    recommendation: str  # human-readable, advisory only


def compute_timing(
    name: str,
    arrival_times_s: Sequence[float],
    values: Sequence[float] | None = None,
) -> StreamTiming:
    """Rate, jitter, and noise floor from passively observed arrivals.

    ``arrival_times_s`` are monotonic receive timestamps (seconds). ``values``
    are concurrent scalar samples of a stationary signal (robot at rest) for
    the noise floor; pass ``None`` to skip (noise floor reported as 0.0).

    Rate is the inverse mean inter-arrival time; jitter is the std of the
    inter-arrival times. Both require at least two arrivals.
    """
    t = np.asarray(arrival_times_s, dtype=float)
    if t.size < 2:
        raise ValueError(f"stream {name!r}: need >=2 arrivals, got {t.size}")
    dt = np.diff(t)
    if np.any(dt <= 0):
        raise ValueError(f"stream {name!r}: arrival times must be increasing")
    rate_hz = float(1.0 / dt.mean())
    jitter_s = float(dt.std())
    noise_floor = float(np.asarray(values, dtype=float).std()) if values is not None else 0.0
    return StreamTiming(name=name, rate_hz=rate_hz, jitter_s=jitter_s, noise_floor=noise_floor)


def advise(stream: StreamTiming, expected_tau_s: float | None) -> TimingAdvice:
    """Wrap a timing measurement with Nyquist + samples-per-tau guidance.

    samples_per_tau = rate * tau is how many feedback samples land within one
    time constant. Low values (a handful) mean velocity differentiation will
    smear the step; the pose output-error fitter avoids differentiating. This
    is surfaced as a recommendation string only.
    """
    nyquist_hz = stream.rate_hz / 2.0
    if expected_tau_s is None:
        return TimingAdvice(
            stream=stream,
            nyquist_hz=nyquist_hz,
            samples_per_tau=None,
            recommendation=(
                "no expected_tau declared; cannot advise fitter from sampling. "
                "User chooses velocity vs pose fitter."
            ),
        )
    spt = stream.rate_hz * expected_tau_s
    if spt < 8.0:
        rec = (
            f"~{spt:.1f} samples per tau (low): differentiating odom will inflate "
            f"tau. Pose output-error fitter is typically appropriate here - your call."
        )
    else:
        rec = (
            f"~{spt:.1f} samples per tau (ample): velocity-domain fitting is well "
            f"sampled. Either fitter is defensible - your call."
        )
    return TimingAdvice(
        stream=stream, nyquist_hz=nyquist_hz, samples_per_tau=spt, recommendation=rec
    )


class StreamObserver:
    """Records arrival timestamps (and optional scalar samples) for one stream.

    Transport-agnostic: feed it a monotonic clock reading per message via
    :meth:`on_message`. The live probe subscribes the profile's streams and
    routes callbacks here; tests drive :meth:`on_message` directly.
    """

    def __init__(self, name: str) -> None:
        self.name = name
        self._arrivals: list[float] = []
        self._values: list[float] = []

    def on_message(self, monotonic_s: float, value: float | None = None) -> None:
        self._arrivals.append(monotonic_s)
        if value is not None:
            self._values.append(value)

    def timing(self) -> StreamTiming:
        values = self._values if self._values else None
        return compute_timing(self.name, self._arrivals, values)
