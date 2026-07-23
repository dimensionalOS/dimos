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

"""RobotProfile - the declared characterization input.

The profile is what the *user* asserts about a robot before autotune runs:
the command interface, odometry domain, controllable channels and their
saturation envelope, which FOPDT fitter to use, and which controller form to
tune. Autotune never auto-detects or overrides these declarations - the
passive probe may *advise* (e.g. "odom is slow relative to your expected
tau"), but the user owns the fitter choice (the robot cannot judge for
itself without knowledge the user has).

The profile also carries a measured-properties slot that characterization
fills in. Declared fields are inputs; :class:`MeasuredProperties` is the
output the report and tuned artifact draw from.

Symbols follow the fitter convention throughout autotune:
  K   - steady-state gain (measured output per unit command)
  tau - first-order time constant (s)
  L   - lumped dead-time (s); includes transport + sensor pipeline + true
        plant deadtime and is not decomposed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

OdomType = Literal["pose", "velocity"]
"""Odometry domain the robot reports feedback in.

``velocity`` - body-frame twist feedback; the velocity-domain fitter
reconstructs body velocity from odom and fits it.
``pose`` - world-frame pose feedback (e.g. slow legged-base odom); the
pose-domain output-error fitter forward-models pose and never differentiates
the measurement.
"""

FitterKind = Literal["velocity", "pose"]
"""Which FOPDT fitter the user selected. Mirrors :data:`OdomType` but is a
separate, explicit choice: a base may report both, and the user decides which
domain to identify in."""


@dataclass(frozen=True)
class Channel:
    """One controllable axis and its saturation envelope.

    ``vmax`` is the operational saturation limit (m/s for linear axes, rad/s
    for angular). ``amax`` is an optional declared rate/acceleration limit;
    when ``None`` the rate limit is left to be derived from the step rise.
    """

    name: str
    vmax: float
    amax: float | None = None


@dataclass(frozen=True)
class BatteryConfig:
    """Excitation battery shape - profile-driven, not a fixed m/s grid.

    Amplitudes are fractions of each channel's ``vmax`` so the same battery
    scales to any robot. A hardcoded absolute grid is meaningless for a base
    whose ``vmax`` is much smaller or larger; fractions of the declared
    envelope are robot-agnostic.
    """

    amplitude_fractions: tuple[float, ...] = (0.25, 0.5, 0.75)
    repeats: int = 3

    def __post_init__(self) -> None:
        if not self.amplitude_fractions:
            raise ValueError("amplitude_fractions must be non-empty")
        for frac in self.amplitude_fractions:
            if not 0.0 < frac <= 1.0:
                raise ValueError(f"amplitude fraction {frac} out of (0, 1]; fractions are of vmax")
        if self.repeats < 1:
            raise ValueError("repeats must be >= 1")


@dataclass
class ChannelFopdt:
    """Per-channel FOPDT fit result. Filled by the fitter, read by the report,
    bandwidth derivation, and tuner."""

    K: float
    tau: float
    L: float
    # Direction-asymmetric gains, populated only when forward/reverse K differ
    # beyond the pooling threshold (otherwise both fall back to ``K``).
    K_forward: float | None = None
    K_reverse: float | None = None
    # Lowest command magnitude that produces sustained motion (deadzone).
    deadzone: float | None = None
    r2: float | None = None
    plausible: bool = True


@dataclass
class StreamTiming:
    """Passive-probe output for one stream: arrival rate and quality. No motion
    is required to measure these."""

    name: str
    rate_hz: float
    jitter_s: float  # std of inter-arrival dt
    noise_floor: float  # stationary signal std at rest


@dataclass
class MeasuredProperties:
    """Everything characterization measures. This is the output slot on the
    profile; the report and tuned artifact are projections of it.

    Intentionally NOT measured (marginal benefit, see design.md): isolated
    measurement latency (already lumped into ``L``), hysteresis, and automatic
    controller-structure selection.
    """

    fopdt: dict[str, ChannelFopdt] = field(default_factory=dict)
    bandwidth_hz: dict[str, float] = field(default_factory=dict)
    cross_axis_coupling: dict[str, float] = field(default_factory=dict)
    streams: dict[str, StreamTiming] = field(default_factory=dict)
    # samples-per-tau advisory per channel (informs, never forces, fitter pick).
    samples_per_tau: dict[str, float] = field(default_factory=dict)


@dataclass
class RobotProfile:
    """User-declared description of a robot for autotune.

    The drive layer is stream-name agnostic: ``command_stream`` and
    ``feedback_stream`` name the coordinator topics so the same module works
    across robots without hardcoded topic names.
    """

    name: str
    command_interface: str  # e.g. "twist", "joint", "pose_setpoint"
    odom_type: OdomType
    channels: list[Channel]
    fitter: FitterKind
    controller_form: str  # e.g. "velocity_pi"
    command_stream: str
    feedback_stream: str
    # Optional prior used only for the probe's samples-per-tau advisory.
    expected_tau_s: float | None = None
    battery: BatteryConfig = field(default_factory=BatteryConfig)
    measured: MeasuredProperties = field(default_factory=MeasuredProperties)

    def __post_init__(self) -> None:
        if not self.channels:
            raise ValueError("profile must declare at least one channel")
        names = [c.name for c in self.channels]
        if len(names) != len(set(names)):
            raise ValueError(f"duplicate channel names: {names}")
        # The fitter is user-owned; we only sanity-check it is a known kind.
        if self.fitter not in ("velocity", "pose"):
            raise ValueError(f"unknown fitter {self.fitter!r}")

    def channel(self, name: str) -> Channel:
        for ch in self.channels:
            if ch.name == name:
                return ch
        raise KeyError(f"no channel named {name!r}")

    @property
    def channel_names(self) -> list[str]:
        return [c.name for c in self.channels]

    def battery_amplitudes(self, channel: str) -> list[float]:
        """Signed-magnitude excitation amplitudes for a channel: each fraction
        of ``vmax``. Direction (forward/reverse) is applied by the excitation
        generator, not here."""
        vmax = self.channel(channel).vmax
        return [frac * vmax for frac in self.battery.amplitude_fractions]
