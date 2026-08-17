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

"""Trajectory statistics that survive chaos, and the filtering they need.

NEVER COMPARE TRAJECTORIES. A legged gait with contacts amplifies a 3-degree
initial-pose difference into more than a metre of divergence in 12 s,
non-monotonically — position error measures how long ago two runs diverged,
not whether the plant is right. What survives is distributional: speed gain,
gait frequency, body-height statistics. Chaos scrambles the phase, not the
distribution. :func:`summarize` computes the eleven; :func:`spread_of`
measures how much each one moves across repeated runs, so a statistic is only
trusted when a difference clearly exceeds its own noise.

Filtering matters more than it looks. Tracker samples arrive at a nominal
~250 Hz with dt jitter as large as the interval, so differentiating raw
samples amplifies noise enormously — and doing it with a fixed *sample*
window applies wildly different smoothing to a 250 Hz recording and a 50 Hz
rollout. Everything here resamples onto a uniform grid first and smooths by a
window in *seconds*.

Ported from the frozen ``motion/simulation/metrics.py`` — the numerical
conventions are kept verbatim so numbers remain comparable with that
instrument's.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

RESAMPLE_HZ = 100.0
VELOCITY_WINDOW_S = 0.4

# Gait band. HIMLoco free_walk has no clocked gait, so this brackets a
# plausible trot rather than targeting a known value. The floor matters: below
# ~1 Hz the estimate locks onto the robot's slow drift around the room.
GAIT_BAND_HZ = (1.0, 6.0)

# Longest policy->body lag to search for when fitting a command gain.
MAX_LAG_S = 0.6


def resample(
    t: np.ndarray, x: np.ndarray, rate: float = RESAMPLE_HZ
) -> tuple[np.ndarray, np.ndarray]:
    """Linear-interpolate ``x(t)`` onto a uniform grid at ``rate`` Hz."""
    grid = np.arange(float(t[0]), float(t[-1]), 1.0 / rate)
    if x.ndim == 1:
        return grid, np.interp(grid, t, x)
    return grid, np.stack([np.interp(grid, t, x[:, i]) for i in range(x.shape[1])], axis=1)


def _moving_average(x: np.ndarray, n: int) -> np.ndarray:
    if n < 2:
        return x
    pad = n // 2
    ker = np.ones(n) / n
    if x.ndim == 1:
        return np.convolve(np.pad(x, pad, mode="edge"), ker, mode="same")[pad : pad + len(x)]
    return np.stack([_moving_average(x[:, i], n) for i in range(x.shape[1])], axis=1)


def velocity(
    t: np.ndarray,
    pos: np.ndarray,
    *,
    window_s: float = VELOCITY_WINDOW_S,
    rate: float = RESAMPLE_HZ,
) -> tuple[np.ndarray, np.ndarray]:
    """Smoothed derivative of ``pos(t)``, on a uniform grid.

    The window is in seconds so a 250 Hz recording and a 50 Hz rollout get the
    same treatment — the mistake that once reported a Go2 walking at 3.9 m/s.
    """
    grid, p = resample(t, pos, rate)
    p = _moving_average(p, max(2, round(window_s * rate)))
    return grid, np.gradient(p, 1.0 / rate, axis=0)


def yaw_of(quat: np.ndarray) -> np.ndarray:
    """Heading angle from (n, 4) wxyz quaternions."""
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    yaw: np.ndarray = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return yaw


def pitch_roll_of(quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Pitch and roll from (n, 4) wxyz quaternions, ZYX convention.

    Only their *std* is comparable sim-to-real: a constant error in the room
    calibration or the mount shifts the mean but not the spread. Unlike
    anything derived from position, these are immune to the tracker
    translation — rotation carries no lever arm.
    """
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    pitch = -np.arcsin(np.clip(2 * (x * z - w * y), -1.0, 1.0))
    roll = np.arctan2(2 * (y * z + w * x), 1 - 2 * (x * x + y * y))
    return pitch, roll


# A candidate first peak yields to a peak this much stronger (normalised-ac
# units). Measured on the freewalk grounding: the pathological lock chose a
# -0.010 ripple at 3.85 Hz over a +0.171 peak at 1.72 Hz (margin 0.18), while
# every benign ambiguity measured sits under 0.05 — so 0.1 separates them.
GAIT_PEAK_MARGIN = 0.1


def gait_frequency(z: np.ndarray, *, rate: float = RESAMPLE_HZ) -> float:
    """Bob frequency, from the autocorrelation of the high-passed height.

    An FFT peak is not usable here: on a 40 s window a harmonic can outscore
    the fundamental. Autocorrelation asks a steadier question — what delay
    does this signal most resemble itself at — and the first peak inside the
    gait band is the stride, not a harmonic of it.

    The first peak is kept UNLESS another peak beats it by
    :data:`GAIT_PEAK_MARGIN`: under some closed-loop probes the first local
    maximum is a noise ripple at the step harmonic (~2x the stride reading)
    an order of magnitude weaker than the true stride peak behind it, which
    made single-rollout ``gait_hz`` bimodal — a draw, not a measurement. Two
    comparable peaks (a genuine fundamental/harmonic pair) still resolve to
    the first, exactly as before.
    """
    bob = z - _moving_average(z, int(rate))  # drop the drift around the room
    bob = bob - bob.mean()
    if not np.any(bob):
        return 0.0
    ac = np.correlate(bob, bob, mode="full")[len(bob) - 1 :]
    ac = ac / ac[0]
    lo = int(rate / GAIT_BAND_HZ[1])  # shortest plausible stride, samples
    hi = min(int(rate / GAIT_BAND_HZ[0]), len(ac) - 1)
    if hi <= lo:
        return 0.0
    window = ac[lo:hi]
    # first local maximum, not the global one: at low frequencies the envelope
    # can rise again and outscore the true stride peak.
    peaks = np.where((window[1:-1] > window[:-2]) & (window[1:-1] >= window[2:]))[0] + 1
    if len(peaks):
        idx = int(peaks[0])
        strongest = int(peaks[np.argmax(window[peaks])])
        if window[strongest] > window[idx] + GAIT_PEAK_MARGIN:
            idx = strongest
    else:
        idx = int(np.argmax(window))
    return float(rate / (lo + idx))


def _best_lag(achieved: np.ndarray, commanded: np.ndarray, rate: float) -> int:
    """Samples of delay that best aligns the response with the command.

    The body does not turn the instant the command changes — the policy has
    its own history and the robot has inertia. Regressing at zero lag against
    a command that alternates faster than that delay reads as no response at
    all, which is how a simulator once appeared to turn backwards.
    """
    a = achieved - achieved.mean()
    c = commanded - commanded.mean()
    if not np.any(c) or not np.any(a):
        return 0
    span = int(MAX_LAG_S * rate)
    scores = [float(c[: len(c) - k] @ a[k:]) for k in range(span)]
    return int(np.argmax(scores))


def _gain(
    achieved: np.ndarray, commanded: np.ndarray, threshold: float, *, rate: float = RESAMPLE_HZ
) -> tuple[float, float]:
    """Least-squares slope of achieved against commanded, at the best lag.

    Returns ``(gain, lag_seconds)``. Not ``mean(achieved / commanded)``: in a
    real recording the command flips sign constantly, so per-sample ratios
    blow up near zero and opposite-sign turns cancel.
    """
    lag = _best_lag(achieved, commanded, rate)
    a = achieved[lag:] if lag else achieved
    c_all = commanded[: len(commanded) - lag] if lag else commanded
    m = np.abs(c_all) > threshold
    if not m.any():
        return 0.0, 0.0
    c = c_all[m]
    return float((c @ a[m]) / (c @ c)), lag / rate


# Carried for context, never scored. Both entries failed the same way: the
# statistic measured its own instrument rather than the robot. `height_mean`
# lives in the tracker room frame, whose floor is unknown — its value is the
# frame calibration. `gait_hz` is the bob autocorrelation's chosen peak, and
# on the freewalk grounding it reads 1.33 (sim) vs 1.67 (real) while the legs
# of BOTH cycle at 1.93/1.96 Hz (measured directly by ``sysid.gait``) — its
# sim-real gap was a property of the estimator, not of either gait. The
# stride pair below is the cadence claim now.
NOT_COMPARABLE = ("height_mean", "gait_hz")


@dataclass
class Summary:
    """Chaos-tolerant description of one run — the eleven statistics."""

    speed: float  # mean planar speed while commanded to move, m/s
    speed_gain: float  # achieved / commanded speed
    yaw_rate_gain: float  # achieved / commanded turn rate
    height_mean: float  # mean height, m — NOT comparable sim-to-real (room frame)
    height_std: float  # bob amplitude: std of the detrended height, m
    gait_hz: float  # dominant frequency of the detrended vertical bob
    speed_lag: float  # policy->body delay fitted for the speed gain, s
    yaw_lag: float  # same, for the turn gain
    pitch_std: float  # gait-driven pitch oscillation, rad
    roll_std: float  # same, about the roll axis
    tilt_p99: float  # near-worst-case body tilt, rad — the stability tail
    # The stride pair (sysid.gait): measured from foot FK + touchdown events,
    # not from any body-signal estimator. NaN when the run carries no joint
    # trajectory (an old caller) or no travel reference for the length.
    stride_hz: float = float("nan")  # leg cadence: 1 / median touchdown interval
    stride_len: float = float("nan")  # body planar travel per stride, m
    # Instrument provenance ("pos:tracker att:imu", "sim", ...): part of any
    # claim, never scored. A claim whose instrument changed is a new claim.
    source: str = ""

    def as_dict(self) -> dict[str, float]:
        return {
            "speed": self.speed,
            "speed_gain": self.speed_gain,
            "yaw_rate_gain": self.yaw_rate_gain,
            "height_mean": self.height_mean,
            "height_std": self.height_std,
            "gait_hz": self.gait_hz,
            "speed_lag": self.speed_lag,
            "yaw_lag": self.yaw_lag,
            "pitch_std": self.pitch_std,
            "roll_std": self.roll_std,
            "tilt_p99": self.tilt_p99,
            "stride_hz": self.stride_hz,
            "stride_len": self.stride_len,
        }


def summarize(
    t: np.ndarray,
    pos: np.ndarray | None,
    quat: np.ndarray,
    cmd: np.ndarray,
    *,
    t_att: np.ndarray | None = None,
    moving_threshold: float = 0.25,
) -> Summary:
    """Distributional statistics for one run.

    ``cmd`` is the vx/vy/vyaw in force at each sample of ``t``; ``pos`` is
    sampled at ``t`` too. ``quat`` is the attitude at ``t_att`` — a SEPARATE
    timeline (default ``t``), because position and attitude may come from
    different instruments: the real side reads position from the tracker and
    attitude from the IMU (README 6 — the tracker's flexing mount invents
    ~2x the roll rate, so it is unusable for attitude and precise for
    position). Both timelines share an epoch; each is resampled onto its own
    uniform grid, and the yaw-gain regression truncates to the shorter.

    ``pos=None`` marks every position-derived statistic NaN — a tracker-less
    recording still scores its attitude, and NaN pairs drop out of the SNR
    (they are not comparable on this recording, which is different from
    matching).
    """
    _, c = resample(t, cmd)
    cmd_speed = np.linalg.norm(c[:, :2], axis=1)
    moving = cmd_speed > moving_threshold

    ta = t if t_att is None else t_att
    yaw = np.unwrap(yaw_of(quat))
    _grid_y, yaw_u = resample(ta, yaw)
    yaw_rate = np.gradient(
        _moving_average(yaw_u, max(2, int(VELOCITY_WINDOW_S * RESAMPLE_HZ))), 1.0 / RESAMPLE_HZ
    )
    n = min(len(yaw_rate), len(c))
    yaw_gain, yaw_lag = _gain(yaw_rate[:n], c[:n, 2], 0.2)

    # Detrended before the std, like gait_frequency: a slow height sag or a
    # tilted room frame leaking planar travel into z would otherwise read as
    # "bob".
    pitch, roll = pitch_roll_of(quat)
    _, pitch_u = resample(ta, pitch)
    _, roll_u = resample(ta, roll)

    if pos is not None:
        grid, vel = velocity(t, pos)
        _, z = resample(t, pos[:, 2])
        speed = np.linalg.norm(vel[:, :2], axis=1)
        speed_gain, speed_lag = _gain(speed, cmd_speed, moving_threshold)
        bob = z - _moving_average(z, int(RESAMPLE_HZ))
        speed_v = float(speed[moving].mean()) if moving.any() else 0.0
        height_mean, height_std = float(z.mean()), float(bob.std())
        gait_hz = gait_frequency(z)
    else:
        nan = float("nan")
        speed_v = speed_gain = speed_lag = nan
        height_mean = height_std = gait_hz = nan

    return Summary(
        speed=speed_v,
        speed_gain=speed_gain,
        yaw_rate_gain=yaw_gain,
        height_mean=height_mean,
        height_std=height_std,
        gait_hz=gait_hz,
        speed_lag=speed_lag,
        yaw_lag=yaw_lag,
        pitch_std=float((pitch_u - _moving_average(pitch_u, int(RESAMPLE_HZ))).std()),
        roll_std=float((roll_u - _moving_average(roll_u, int(RESAMPLE_HZ))).std()),
        tilt_p99=float(np.percentile(np.sqrt(pitch_u**2 + roll_u**2), 99)),
    )


def median_summary(summaries: list[Summary]) -> Summary:
    """Per-statistic median across replicate runs — the verdict's point estimate.

    README 4a's shape ("point = per-parameter median of the pool") applied to
    loop 2: one chaotic rollout is a draw, not a verdict, so the point
    estimate is the per-statistic median over replicates and the spread rides
    beside it (:func:`spread_of`). NaN statistics stay NaN — a statistic no
    replicate can measure is still not comparable.
    """
    keys = summaries[0].as_dict()
    med = {k: float(np.median([s.as_dict()[k] for s in summaries])) for k in keys}
    return Summary(**med, source=summaries[0].source)


def spread_of(summaries: list[Summary]) -> dict[str, float]:
    """Peak-to-peak of each statistic across repeated runs — a noise floor.

    THE SOURCE OF THE REPEATS IS A PARAMETER OF THE CLAIM, not of this
    function. Perturbed SIM reruns measure what chaos alone does
    (:func:`~dimos.robot.unitree.go2.sim.sysid.ground.sim_noise`); two REAL
    recordings of the same walk measure what the robot does against itself —
    battery sag and motor temperature included — and are the better yardstick
    (:func:`~dimos.robot.unitree.go2.sim.sysid.ground.robot_noise`). Either
    way the floor is this spread, and a statistic is only usable when the
    sim-real difference clearly exceeds it.
    """
    keys = summaries[0].as_dict()
    # np.max/np.min, not the builtins: NaN (a statistic one run cannot measure)
    # must poison the spread, not silently drop out of it.
    return {
        k: float(np.max(vals) - np.min(vals))
        for k in keys
        if (vals := [s.as_dict()[k] for s in summaries])
    }
