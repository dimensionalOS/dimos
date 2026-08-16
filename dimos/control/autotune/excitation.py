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

"""Excitation signal generators.

Pure ``(t) -> command`` functions that the drive layer plays through the
coordinator. They carry NO recording concern (capture is the learning
recorder's job) and NO timing concern beyond the signal shape. One excited
channel at a time; the value is the command for that channel (other channels
are commanded zero by the driver).

Amplitudes are profile-driven (fractions of the channel envelope) and applied
in both directions, so the same battery scales to any robot and exposes
direction asymmetry. A run is one ``ExcitationRun`` = one episode.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from dimos.control.autotune.profile import RobotProfile

SignalFn = Callable[[float], float]
"""Time (s, relative to run start) -> command value for the excited channel."""


def step(amplitude: float, t_start: float = 0.5) -> SignalFn:
    """Zero until ``t_start``, then constant ``amplitude``. The canonical FOPDT
    excitation; the pre-step dwell gives a clean baseline."""

    def _fn(t: float) -> float:
        return amplitude if t >= t_start else 0.0

    return _fn


def ramp(amplitude: float, duration_s: float, t_start: float = 0.5) -> SignalFn:
    """Linear rise from 0 to ``amplitude`` over ``duration_s`` after ``t_start``,
    then hold. Used for rate-limit probing and as the deadzone sweep base."""
    if duration_s <= 0:
        raise ValueError("duration_s must be > 0")

    def _fn(t: float) -> float:
        if t < t_start:
            return 0.0
        frac = (t - t_start) / duration_s
        if frac >= 1.0:
            return amplitude
        return amplitude * frac

    return _fn


def deadzone_ramp(amplitude: float, duration_s: float, t_start: float = 0.5) -> SignalFn:
    """Slow ramp from 0 to ``amplitude`` to find the minimum command that moves
    the base. Identical shape to :func:`ramp`; named distinctly because the
    intent (and the slow rate) differ — keep ``duration_s`` long so the deadzone
    crossing is well resolved."""
    return ramp(amplitude, duration_s, t_start)


@dataclass(frozen=True)
class ExcitationRun:
    """One excitation episode: which channel, signed amplitude, signal, repeat
    index, and how long to play it."""

    channel: str
    amplitude: float
    direction: int  # +1 or -1
    repeat: int
    signal: SignalFn
    duration_s: float
    kind: str  # "step" | "ramp" | "deadzone_ramp"

    @property
    def label(self) -> str:
        sign = "fwd" if self.direction >= 0 else "rev"
        return f"{self.kind}_{self.channel}_{abs(self.amplitude):.3g}_{sign}_r{self.repeat}"


def step_battery(
    profile: RobotProfile,
    *,
    duration_s: float = 4.0,
    t_start: float = 0.5,
    bidirectional: bool = True,
) -> list[ExcitationRun]:
    """Build the step battery for every channel: each profile amplitude, both
    directions (to expose asymmetry), repeated ``profile.battery.repeats`` times.

    Order is channel-major then amplitude then direction then repeat; the driver
    may randomize this to decorrelate drift."""
    runs: list[ExcitationRun] = []
    directions = (1, -1) if bidirectional else (1,)
    for ch in profile.channel_names:
        for amp in profile.battery_amplitudes(ch):
            for direction in directions:
                signed = direction * amp
                for rep in range(profile.battery.repeats):
                    runs.append(
                        ExcitationRun(
                            channel=ch,
                            amplitude=signed,
                            direction=direction,
                            repeat=rep,
                            signal=step(signed, t_start),
                            duration_s=duration_s,
                            kind="step",
                        )
                    )
    return runs


def deadzone_battery(
    profile: RobotProfile,
    *,
    duration_s: float = 8.0,
    t_start: float = 0.5,
    bidirectional: bool = True,
) -> list[ExcitationRun]:
    """One slow ramp to full ``vmax`` per channel per direction. Long duration so
    the deadzone crossing is well sampled."""
    runs: list[ExcitationRun] = []
    directions = (1, -1) if bidirectional else (1,)
    for ch in profile.channel_names:
        vmax = profile.channel(ch).vmax
        for direction in directions:
            signed = direction * vmax
            runs.append(
                ExcitationRun(
                    channel=ch,
                    amplitude=signed,
                    direction=direction,
                    repeat=0,
                    signal=deadzone_ramp(signed, duration_s - t_start, t_start),
                    duration_s=duration_s,
                    kind="deadzone_ramp",
                )
            )
    return runs
