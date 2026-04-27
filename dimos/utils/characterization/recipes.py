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

"""Test recipes and signal generators for the characterization harness.

A ``TestRecipe`` bundles a name, a signal function ``t -> (vx, vy, wz)``,
a duration, a sample rate, and pre/post-roll. The runner evaluates the
signal function at each tick during the active window and pads zeros
around it. Define a recipe in ~5 lines::

    step_vx_1 = TestRecipe(
        name="step_vx_1.0",
        test_type="step",
        duration_s=3.0,
        signal_fn=step(amplitude=1.0, channel="vx"),
    )
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any, Literal

import numpy as np

Channel = Literal["vx", "vy", "wz"]
SignalFn = Callable[[float], tuple[float, float, float]]
TestType = Literal["step", "ramp", "chirp", "constant", "composite"]

_CHANNEL_INDEX: dict[str, int] = {"vx": 0, "vy": 1, "wz": 2}


@dataclass(frozen=True)
class TestRecipe:
    """A characterization test, as data.

    The signal function receives ``t`` in seconds *relative to the start
    of the active window* (not relative to pre-roll). Pre-roll and
    post-roll are always zero commands.
    """

    # Tell pytest this dataclass is not a test collection target.
    __test__ = False

    name: str
    test_type: TestType
    duration_s: float
    signal_fn: SignalFn
    sample_rate_hz: float = 50.0
    pre_roll_s: float = 0.5
    post_roll_s: float = 1.0
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def serialize(self) -> dict[str, Any]:
        """Metadata-safe dict. Excludes ``signal_fn`` since it's a callable."""
        return {
            "name": self.name,
            "test_type": self.test_type,
            "duration_s": self.duration_s,
            "sample_rate_hz": self.sample_rate_hz,
            "pre_roll_s": self.pre_roll_s,
            "post_roll_s": self.post_roll_s,
            "metadata": dict(self.metadata),
        }


def _one_channel(channel: Channel, value: float) -> tuple[float, float, float]:
    out = [0.0, 0.0, 0.0]
    out[_CHANNEL_INDEX[channel]] = value
    return (out[0], out[1], out[2])


def step(amplitude: float, channel: Channel = "vx", *, t_start: float = 0.0) -> SignalFn:
    """Zero until ``t_start``, then ``amplitude`` on ``channel`` until the end."""

    def fn(t: float) -> tuple[float, float, float]:
        if t < t_start:
            return (0.0, 0.0, 0.0)
        return _one_channel(channel, amplitude)

    return fn


def ramp(start: float, end: float, duration: float, channel: Channel = "vx") -> SignalFn:
    """Linear ramp from ``start`` to ``end`` on ``channel`` over ``duration``."""

    def fn(t: float) -> tuple[float, float, float]:
        if t <= 0.0:
            return _one_channel(channel, start)
        if t >= duration:
            return _one_channel(channel, end)
        frac = t / duration
        return _one_channel(channel, start + (end - start) * frac)

    return fn


def chirp(
    f_min_hz: float,
    f_max_hz: float,
    duration: float,
    amplitude: float,
    mean: float = 0.0,
    channel: Channel = "vx",
    *,
    method: Literal["linear", "logarithmic"] = "logarithmic",
) -> SignalFn:
    """Exponential or linear frequency sweep on ``channel``, offset by ``mean``.

    Uses ``scipy.signal.chirp`` for the phase. Returns a callable that
    can be evaluated at any ``t`` in ``[0, duration]``.
    """
    from scipy.signal import chirp as _scipy_chirp

    def fn(t: float) -> tuple[float, float, float]:
        t_clamped = min(max(t, 0.0), duration)
        sample = float(
            _scipy_chirp(
                np.asarray([t_clamped]),
                f0=f_min_hz,
                t1=duration,
                f1=f_max_hz,
                method=method,
            )[0]
        )
        return _one_channel(channel, mean + amplitude * sample)

    return fn


def constant(vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> SignalFn:
    """Hold a constant Twist for the whole active window."""

    def fn(_t: float) -> tuple[float, float, float]:
        return (vx, vy, wz)

    return fn


def composite(*signals: SignalFn) -> SignalFn:
    """Element-wise sum of multiple signals. Useful for vx-step + wz-chirp, etc."""

    def fn(t: float) -> tuple[float, float, float]:
        vx = vy = wz = 0.0
        for s in signals:
            a, b, c = s(t)
            vx += a
            vy += b
            wz += c
        return (vx, vy, wz)

    return fn


__all__ = [
    "Channel",
    "SignalFn",
    "TestRecipe",
    "TestType",
    "chirp",
    "composite",
    "constant",
    "ramp",
    "step",
]
