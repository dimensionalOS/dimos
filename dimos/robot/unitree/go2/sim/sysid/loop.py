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

"""Measure the CONTROL LOOP, not the plant.

    python -m dimos.robot.unitree.go2.sim.sysid.loop REC.mcap [--sweep]

This module is where each candidate loop mechanism gets a MEASURED value
instead of a fitted one (README 9: the mechanisms stay default-off). A mechanism fitted against the referee moves the right statistics
for possibly the wrong reason; one parameterised from a measurement is a
claim about the robot. Four instruments:

* :func:`transport_leg` — the recordings carry the same command on two
  channels (``policy/lowcmd``, our executor's output; ``rt/lowcmd``, the DDS
  channel the motors listen to). Matching by exact 12-float payload times the
  command transport directly — a DISTRIBUTION, not a constant, and it refuses
  rather than guesses when a recording lacks either channel.
* :func:`control_timing` — the real executor's inter-command intervals, from
  ``policy/lowcmd`` log times. The sim ticks the policy on a perfect 20 ms
  grid; the measured executor does not (22.3 ms median on every 2026-08-16
  policy recording, with a dropout tail) — and unlike a latency, this has
  ZERO free parameters. Feeds ``rollout_policy(control_intervals=...)``.
* :func:`sensor_noise` — the >20 Hz residual of each observation channel
  (clear of every gait harmonic), i.e. the measured floor that should
  parameterise :class:`~dimos.robot.unitree.go2.sim.sysid.ground.ObsNoise`
  instead of the training levels (which are 2-3x larger on dq/gyro and ~70x
  larger on gravity — the attitude filter smooths gravity almost clean).
* :func:`command_shift_sweep` — shifting the recorded command timeline and
  replaying Mode A is a latency knob scored WITHOUT any referee: the target
  -> plant leg is identifiable open loop. Measured on the freewalk recording
  it answers ~0 ms (optimum -5..+2 ms, sharply resolved), which is what rules
  a large ``action_latency`` out of the target->plant leg and pushes the
  question to the legs open loop cannot see.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from dimos.robot.unitree.go2.sim.ranges import Preset
from dimos.robot.unitree.go2.sim.sysid.ground import ObsNoise
from dimos.robot.unitree.go2.sim.sysid.ingest import command_coverage
from dimos.simulation.sysid.backend import Backend
from dimos.simulation.sysid.recording import Streams

# The band every gait harmonic stays under; noise is what lives above it.
NOISE_HIGHPASS_HZ = 20.0

# An interval this far past nominal means at least one control frame never
# went out — a much bigger perturbation than jitter around the mean.
DROPOUT_S = 0.030

# Where a loop latency could hide, measured leg by leg on the 2026-08-16
# freewalk session: the command transport is 1.34 ms (two-channel payload
# matching, :func:`transport_leg`); the state side is bounded by the lowstate
# batch spacing (~3.9 ms median arrival gaps, so a freshest-sample age of
# ~0-4 ms) plus ~0-2 ms of executor software (verify_net --state-offset); and
# the target->plant leg is ~0 (:func:`command_shift_sweep`). Anything above
# the top of this band is NOT a latency the loop can physically contain.
LATENCY_BAND_S = (0.0013, 0.006)


def _uniform_level(rms: float) -> float:
    """The ±level of a uniform distribution with this RMS (level = rms·√3)."""
    return float(rms * np.sqrt(3.0))


# --------------------------------------------------------- the two channels


def _read_lowcmd_channels(path: str | Path) -> dict[str, tuple[np.ndarray, list[bytes]]]:
    """Log times and exact 12-float payload keys for both lowcmd channels."""
    from mcap.reader import make_reader

    from dimos.robot.unitree.go2.dds import cdr
    from dimos.robot.unitree.go2.dds.msgs.LowCmd import LowCmd

    raw: dict[str, list[tuple[float, bytes]]] = {"policy/lowcmd": [], "rt/lowcmd": []}
    with Path(path).open("rb") as f:
        for _s, ch, msg in make_reader(f).iter_messages(topics=list(raw)):
            lc, _end = cdr.decode(msg.data, LowCmd)
            key = np.array([m.q for m in lc.motor_cmd[:12]], dtype=np.float32).tobytes()
            raw[ch.topic].append((msg.log_time / 1e9, key))
    return {
        k: (np.array([t for t, _ in v]), [b for _, b in v]) if v else (np.zeros(0), [])
        for k, v in raw.items()
    }


def _executor_drove(pt: np.ndarray, rt: np.ndarray) -> bool:
    """Did our executor drive this recording? Same >=50% coverage rule as
    ingest — a sport file can carry a policy/lowcmd STUB of idle seconds."""
    if len(pt) < 3:
        return False
    span = float(rt[-1] - rt[0]) if len(rt) > 1 else float(pt[-1] - pt[0])
    return command_coverage(float(pt[0]), float(pt[-1]), len(pt), span) >= 0.5


def match_commands(
    src_t: np.ndarray,
    src_keys: list[bytes],
    dst_t: np.ndarray,
    dst_keys: list[bytes],
) -> tuple[np.ndarray, int]:
    """Per-command ``dst - src`` delay for payload-identical pairs, in order.

    Each source command is matched to the FIRST destination occurrence of the
    same payload at or after its own time — repeated payloads cannot steal an
    earlier match. Returns ``(delays_s, n_unmatched)``.
    """
    where: dict[bytes, list[float]] = {}
    for t, k in zip(dst_t, dst_keys, strict=True):
        where.setdefault(k, []).append(float(t))
    deltas: list[float] = []
    unmatched = 0
    for t, k in zip(src_t, src_keys, strict=True):
        times = where.get(k)
        i = bisect.bisect_left(times, float(t)) if times else 0
        if times and i < len(times):
            deltas.append(times[i] - float(t))
        else:
            unmatched += 1
    return np.array(deltas), unmatched


@dataclass(frozen=True)
class TransportLeg:
    """The command-transport delay distribution, ``rt/lowcmd - policy/lowcmd``."""

    delta: np.ndarray  # seconds, one per matched command, emission order
    n_commands: int  # policy/lowcmd messages seen
    n_unmatched: int

    @property
    def median_ms(self) -> float:
        return float(np.median(self.delta) * 1e3)

    def percentile_ms(self, p: float) -> float:
        return float(np.percentile(self.delta, p) * 1e3)

    def describe(self) -> str:
        return (
            f"transport leg (rt/lowcmd - policy/lowcmd): "
            f"{len(self.delta)}/{self.n_commands} matched, "
            f"median {self.median_ms:+.2f} ms, "
            f"p10-p90 {self.percentile_ms(10):.2f}-{self.percentile_ms(90):.2f} ms"
        )


def transport_leg(recording: str | Path) -> TransportLeg:
    """Measure the command transport by two-channel payload matching.

    REFUSES (raises) when the recording lacks either channel — a sport
    recording has no ``policy/lowcmd``, and guessing a transport delay would
    be exactly the fitted-constant habit this module exists to replace.
    """
    ch = _read_lowcmd_channels(recording)
    pt, pk = ch["policy/lowcmd"]
    rt, rk = ch["rt/lowcmd"]
    if len(rt) == 0 or not _executor_drove(pt, rt):
        raise ValueError(
            f"{recording}: transport_leg needs BOTH policy/lowcmd and rt/lowcmd "
            f"across the recording (got {len(pt)} and {len(rt)} messages) — a "
            f"sport recording's policy stub times a couple of idle seconds, "
            f"not the loop, so it refuses rather than guesses"
        )
    delta, unmatched = match_commands(pt, pk, rt, rk)
    if len(delta) == 0:
        raise ValueError(f"{recording}: no payload-identical command pairs at all")
    return TransportLeg(delta=delta, n_commands=len(pt), n_unmatched=unmatched)


# ------------------------------------------------------------ control timing


@dataclass(frozen=True)
class ControlTiming:
    """The executor's measured inter-command intervals, in emission order.

    This is the ``control_intervals`` mechanism's parameterisation: replayed
    AS A SEQUENCE (not resampled) it carries the mean rate, the jitter and
    the dropouts with zero free parameters.
    """

    intervals: np.ndarray  # seconds

    @property
    def mean_ms(self) -> float:
        return float(np.mean(self.intervals) * 1e3)

    @property
    def median_ms(self) -> float:
        return float(np.median(self.intervals) * 1e3)

    @property
    def std_ms(self) -> float:
        return float(np.std(self.intervals) * 1e3)

    @property
    def rate_hz(self) -> float:
        return float(1.0 / np.mean(self.intervals))

    @property
    def n_dropouts(self) -> int:
        """Intervals long enough that at least one control frame never went out."""
        return int(np.sum(self.intervals > DROPOUT_S))

    def describe(self) -> str:
        return (
            f"control timing: {len(self.intervals)} intervals, "
            f"mean {self.mean_ms:.2f} ms ({self.rate_hz:.1f} Hz), "
            f"median {self.median_ms:.2f}, std {self.std_ms:.2f}, "
            f"{self.n_dropouts} dropouts (> {DROPOUT_S * 1e3:.0f} ms, "
            f"max {np.max(self.intervals) * 1e3:.1f} ms)"
        )


def control_timing(recording: str | Path) -> ControlTiming:
    """Measure the executor's control-interval sequence from ``policy/lowcmd``.

    Refuses on a sport recording: ``rt/lowcmd`` alone times the builtin
    controller, not our executor, and the mechanism this feeds simulates OUR
    loop.
    """
    ch = _read_lowcmd_channels(recording)
    pt, _pk = ch["policy/lowcmd"]
    rt, _rk = ch["rt/lowcmd"]
    if len(pt) < 3 or not _executor_drove(pt, rt):
        raise ValueError(
            f"{recording}: control_timing needs policy/lowcmd covering the "
            f"recording (executor-driven), got {len(pt)} messages"
        )
    return ControlTiming(intervals=np.diff(pt))


def timing_of(st: Streams, *, t0: float | None = None, t1: float | None = None) -> ControlTiming:
    """The executor's intervals from an already-ingested recording's ``ct``.

    ``Streams.ct`` IS ``policy/lowcmd``'s log times whenever the executor
    drove (ingest's coverage rule), so no second MCAP pass is needed, and a
    window in recording time selects one span's timing — a mixed recording's
    non-freewalk modes carry their own stall pattern. Refuses when ``ct``
    ticks far faster than a policy rate: that is ``rt/lowcmd`` (a sport
    recording), the builtin controller's clock, not our executor's.
    """
    t = st.ct
    if t0 is not None or t1 is not None:
        lo = -np.inf if t0 is None else t0
        hi = np.inf if t1 is None else t1
        t = t[(t >= lo) & (t < hi)]
    if len(t) < 3:
        raise ValueError("timing_of: fewer than 3 commands in the window")
    iv = np.diff(t)
    if float(np.median(iv)) < 0.010:
        raise ValueError(
            "timing_of: ct ticks at sub-10 ms — this is rt/lowcmd (sport-driven), "
            "not the executor's clock"
        )
    return ControlTiming(intervals=iv)


# -------------------------------------------------------------- sensor noise


@dataclass(frozen=True)
class SensorNoise:
    """Per-channel RMS of the >``NOISE_HIGHPASS_HZ`` observation residual."""

    dq: np.ndarray  # (12,) rad/s, per joint
    gyro: np.ndarray  # (3,) rad/s, per axis
    q: np.ndarray  # (12,) rad
    gravity: np.ndarray  # (3,) unit-vector components

    @property
    def dq_rms(self) -> float:
        return float(np.sqrt(np.mean(self.dq**2)))

    @property
    def gyro_rms(self) -> float:
        return float(np.sqrt(np.mean(self.gyro**2)))

    @property
    def q_rms(self) -> float:
        return float(np.sqrt(np.mean(self.q**2)))

    @property
    def gravity_rms(self) -> float:
        return float(np.sqrt(np.mean(self.gravity**2)))

    def obs_noise(self) -> ObsNoise:
        """The measured :class:`ObsNoise`: uniform levels with the measured RMS.

        ``ObsNoise`` injects uniform ±level (RMS = level/√3), so each level is
        the measured RMS times √3. Per-channel on purpose — the training
        levels overshoot dq by ~3x and gyro by ~1.6x, and the two disagree, so
        one scalar scale cannot express the measurement.
        """
        return ObsNoise(
            gyro=_uniform_level(self.gyro_rms),
            gravity=_uniform_level(self.gravity_rms),
            q=_uniform_level(self.q_rms),
            dq=_uniform_level(self.dq_rms),
        )

    def describe(self) -> str:
        train = ObsNoise()
        mine = self.obs_noise()
        return (
            f"sensor noise > {NOISE_HIGHPASS_HZ:.0f} Hz (RMS, and ±level vs training level):\n"
            f"  dq      {self.dq_rms:.3f} rad/s   ±{mine.dq:.3f} vs ±{train.dq:g} "
            f"(x{mine.dq / train.dq:.2f})\n"
            f"  gyro    {self.gyro_rms:.4f} rad/s  ±{mine.gyro:.4f} vs ±{train.gyro:g} "
            f"(x{mine.gyro / train.gyro:.2f})\n"
            f"  q       {self.q_rms:.5f} rad    ±{mine.q:.5f} vs ±{train.q:g} "
            f"(x{mine.q / train.q:.2f})\n"
            f"  gravity {self.gravity_rms:.5f}        ±{mine.gravity:.5f} vs ±{train.gravity:g} "
            f"(x{mine.gravity / train.gravity:.3f})"
        )


def sensor_noise(
    st: Streams,
    *,
    t0: float,
    t1: float,
    highpass_hz: float = NOISE_HIGHPASS_HZ,
) -> SensorNoise:
    """Measure each observation channel's noise floor above ``highpass_hz``.

    The cut sits clear of every gait harmonic, so what survives is sensor
    content the policy sees but the gait does not put there. Gravity is the
    PROJECTED-gravity observation (from the IMU attitude), matching what the
    net consumes — the attitude filter smooths it nearly clean, which is why
    its measured level is ~70x under the training level.
    """
    from scipy import signal

    sel = (st.lt >= t0) & (st.lt < t1)
    if sel.sum() < 100:
        raise ValueError(f"sensor_noise: only {sel.sum()} lowstate samples in [{t0}, {t1})")
    fs = 1.0 / float(np.median(np.diff(st.lt[sel])))
    sos = signal.butter(4, highpass_hz, "highpass", fs=fs, output="sos")

    def rms(x: np.ndarray) -> np.ndarray:
        y = signal.sosfiltfilt(sos, x, axis=0)
        return np.asarray(np.sqrt(np.mean(y**2, axis=0)))

    qw = st.lquat[sel]
    w, x, y, z = qw[:, 0], qw[:, 1], qw[:, 2], qw[:, 3]
    grav = np.stack([-2 * (x * z - w * y), -2 * (y * z + w * x), -(1 - 2 * (x * x + y * y))], 1)
    return SensorNoise(
        dq=rms(st.ldq[sel]),
        gyro=rms(st.lgyro[sel]),
        q=rms(st.lq[sel]),
        gravity=rms(grav),
    )


# -------------------------------------------------- open-loop latency probe


@dataclass(frozen=True)
class ShiftPoint:
    """One command-shift replay: the open-loop latency knob at one value."""

    shift_ms: float
    joint_mean: float  # mean |joint error|, rad
    accel_rms: float  # RMS accel residual, m/s^2


def command_shift_sweep(
    st: Streams,
    backend: Backend,
    shifts_ms: tuple[float, ...],
    *,
    t0: float,
    duration: float,
    preset: Preset,
    window: float | tuple[float, float] | None = 0.4,
    seed: int = 0,
) -> list[ShiftPoint]:
    """Replay Mode A with the command timeline shifted by each value.

    A positive shift delays every command — exactly what an
    ``action_latency`` would do to the target->plant leg — and the residuals
    score it against the recording with NO referee involved. On the freewalk
    recording the sweep is sharply resolved (44% depth over ±30 ms) with its
    optimum at -5..+2 ms: the target->plant leg holds no room for a 10 ms
    latency. What this instrument structurally cannot see is the
    obs->policy->cmd legs — there is no policy in the loop — and the ingest
    clock rebase absorbs constant one-way transports.
    """
    from dimos.robot.unitree.go2.sim.sysid.replay import replay

    out = []
    for ms in shifts_ms:
        shifted = replace(st, ct=st.ct + ms / 1e3)
        r = replay(shifted, t0, duration, backend, preset=preset, window=window, seed=seed)
        out.append(
            ShiftPoint(
                shift_ms=float(ms),
                joint_mean=float(r.joint_err().mean()),
                accel_rms=float(np.sqrt(np.mean((r.prediction.imu_accel - r.a_real) ** 2))),
            )
        )
    return out


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(prog="go2.sim.sysid.loop", description=__doc__)
    ap.add_argument("recording")
    ap.add_argument("--noise-t0", type=float, default=20.0)
    ap.add_argument("--noise-t1", type=float, default=50.0)
    ap.add_argument("--sweep", action="store_true", help="run the open-loop command-shift sweep")
    ap.add_argument(
        "--shifts",
        type=float,
        nargs="+",
        default=[-10, -5, -2, 0, 2, 5, 10, 30],
        metavar="MS",
    )
    ap.add_argument("--start", type=float, default=20.0, help="sweep replay start, s")
    ap.add_argument("--seconds", type=float, default=30.0, help="sweep replay duration, s")
    ap.add_argument("--preset", default="measured")
    args = ap.parse_args()

    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams

    for probe in (transport_leg, control_timing):
        try:
            print(probe(args.recording).describe())  # type: ignore[operator]
        except ValueError as e:
            print(f"({e})")
    st = read_streams(args.recording)
    print(sensor_noise(st, t0=args.noise_t0, t1=args.noise_t1).describe())

    if args.sweep:
        from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend
        from dimos.robot.unitree.go2.sim.ranges import load_preset

        print(f"\ncommand-shift sweep  t={args.start:g}-{args.start + args.seconds:g}s")
        pts = command_shift_sweep(
            st,
            MujocoBackend(),
            tuple(args.shifts),
            t0=args.start,
            duration=args.seconds,
            preset=load_preset(args.preset),
        )
        best_j = min(pts, key=lambda p: p.joint_mean)
        best_a = min(pts, key=lambda p: p.accel_rms)
        for p in pts:
            marks = ("  <- joint optimum" if p is best_j else "") + (
                "  <- accel optimum" if p is best_a else ""
            )
            print(
                f"  {p.shift_ms:+6.1f} ms  joint {p.joint_mean:.6f} rad  "
                f"accel {p.accel_rms:.3f} m/s^2{marks}"
            )


if __name__ == "__main__":
    main()
