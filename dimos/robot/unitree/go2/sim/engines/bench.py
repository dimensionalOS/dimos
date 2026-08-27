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

"""Does the batched engine beat the CPU pool? The measurement that decides it.

MJX earns its place only batched: a single env costs 90 s of JIT to do what
CPU MuJoCo does in 1 s. The bar is the pool that already exists — CPU MuJoCo
at ~5000 steps/s per core times the cores on the box — and clearing it on 8 GB
of consumer VRAM is not a given, which is why this runs before any env is
written on top.

Two axes, both load-bearing:

* **batch** — throughput per env only pays off once the GPU is saturated, and
  VRAM is the ceiling that stops it.
* **precision** — the plant is defined in float64 and the cross-engine parity
  was measured there, but consumer GPUs run fp64 at 1/32 (Turing) to 1/64
  (Blackwell) of fp32. Training will be float32 whatever this says; the number
  that matters is what that costs, and the referee answers that, not this.

The stepped body is the env's, not a bare ``mjx.step``: PD law, torque clip,
the measured envelope and the first-order lag all run inside the scan, because
that is what a training step actually costs.
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any

import mujoco
import numpy as np

from dimos.robot.unitree.go2.sim.engines import mjx as go2_mjx, model as go2_model
from dimos.robot.unitree.go2.sim.plant import TORQUE_ENVELOPES, TORQUE_LIMITS, TorqueEnvelope
from dimos.robot.unitree.go2.sim.ranges import Preset, load_preset

STAND_Q = np.tile([0.0, 0.9, -1.8], 4)
KP, KD = 40.0, 2.0


def _envelope(preset: Preset) -> TorqueEnvelope | None:
    """The preset's own envelope: a plant fitted with it must run with it."""
    return TORQUE_ENVELOPES[preset.envelope] if preset.envelope is not None else None


@dataclass(frozen=True)
class Result:
    label: str
    envs: int
    steps: int
    compile_s: float
    run_s: float
    peak_mib: float
    finite: bool = True
    """False = the batch went NaN. Throughput of a diverged plant is still a
    NUMBER (it bounds what the solver setting could buy), but never a claim
    the plant survives there — which is why it rides the result instead of
    killing the sweep."""

    @property
    def steps_per_s(self) -> float:
        return self.envs * self.steps / self.run_s


def standing_state(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
    """The stand pose with the lowest foot on the floor — contacts live.

    A floating robot measures the wrong thing: contact is where both the cost
    and the engines' disagreement live.
    """
    feet = go2_model.foot_geom_ids(model)
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[3] = 1.0
    data.qpos[7:19] = STAND_Q
    data.qpos[2] = 0.30
    mujoco.mj_forward(model, data)
    data.qpos[2] -= float(np.min(data.geom_xpos[feet, 2])) - go2_model.FOOT_RADIUS
    mujoco.mj_forward(model, data)
    return data.qpos.copy(), data.qvel.copy()


def cpu_baseline(preset: Preset, steps: int = 5000) -> Result:
    """CPU MuJoCo, one env, the same stepped body. The bar to clear."""
    model, data = go2_model.load()
    go2_model.apply_physics(model, preset.physics)
    envelope = _envelope(preset)
    qpos, qvel = standing_state(model, data)
    data.qpos[:], data.qvel[:] = qpos, qvel
    dt = float(model.opt.timestep)
    alpha = dt / (preset.actuator_tau + dt) if preset.actuator_tau > 0.0 else 1.0
    applied = np.zeros(12)

    t0 = time.perf_counter()
    for _ in range(steps):
        q, dq = data.qpos[7:19], data.qvel[6:18]
        tau = np.clip(KP * (STAND_Q - q) - KD * dq, -TORQUE_LIMITS, TORQUE_LIMITS)
        if envelope is not None:
            tau = envelope.deliverable(tau, dq)
        applied = applied + alpha * (tau - applied)
        data.ctrl[:] = applied
        mujoco.mj_step(model, data)
    return Result("mujoco cpu", 1, steps, 0.0, time.perf_counter() - t0, 0.0)


def _stepper(mx: Any, envelope: TorqueEnvelope | None, alpha: float, steps: int) -> Any:
    """One env's scanned rollout, closed over the compiled model."""
    import jax
    import jax.numpy as jnp

    limits = jnp.asarray(TORQUE_LIMITS)
    target = jnp.asarray(STAND_Q)
    from mujoco import mjx  # type: ignore[attr-defined]

    def one(carry, _):  # type: ignore[no-untyped-def]
        d, applied = carry
        q, dq = d.qpos[7:19], d.qvel[6:18]
        tau = jnp.clip(KP * (target - q) - KD * dq, -limits, limits)
        if envelope is not None:
            tau = go2_mjx.deliverable_jax(envelope, tau, dq)
        applied = applied + alpha * (tau - applied)
        return (mjx.step(mx, d.replace(ctrl=applied)), applied), None

    def run(d):  # type: ignore[no-untyped-def]
        (out, _), _ = jax.lax.scan(one, (d, jnp.zeros(12)), None, length=steps)
        return out.qpos

    return run


def mjx_throughput(preset: Preset, envs: int, steps: int, *, x64: bool) -> Result:
    """Batched MJX: compile once, then time the steady state."""
    import jax
    import jax.numpy as jnp

    jax.config.update("jax_enable_x64", x64)  # type: ignore[no-untyped-call]
    from mujoco import mjx  # type: ignore[attr-defined]

    model, data = go2_model.load()
    go2_model.apply_physics(model, preset.physics)
    go2_mjx.prepare(model)
    qpos, qvel = standing_state(model, data)
    dt = float(model.opt.timestep)

    mx = mjx.put_model(model)
    dx = mjx.make_data(mx)
    dx = dx.replace(qpos=jnp.asarray(qpos), qvel=jnp.asarray(qvel))
    # A jittered batch, not N copies of one state: identical envs are not what
    # training runs and not what the solver's branch behaviour sees.
    key = jax.random.PRNGKey(0)
    jitter = jax.random.uniform(key, (envs, 12), minval=-0.02, maxval=0.02)
    batch = jax.tree.map(lambda x: jnp.broadcast_to(x, (envs, *x.shape)).copy(), dx)
    batch = batch.replace(qpos=batch.qpos.at[:, 7:19].add(jitter))

    alpha = dt / (preset.actuator_tau + dt) if preset.actuator_tau > 0.0 else 1.0
    run = jax.jit(jax.vmap(_stepper(mx, _envelope(preset), alpha, steps)))
    t0 = time.perf_counter()
    out = jax.block_until_ready(run(batch))  # type: ignore[no-untyped-call]
    compile_s = time.perf_counter() - t0

    t0 = time.perf_counter()
    out = jax.block_until_ready(run(batch))  # type: ignore[no-untyped-call]
    run_s = time.perf_counter() - t0
    finite = bool(np.isfinite(np.asarray(out)).all())

    peak = 0.0
    stats = jax.local_devices()[0].memory_stats()
    if stats:
        peak = stats.get("peak_bytes_in_use", 0) / 1024**2
    return Result(
        f"mjx {'f64' if x64 else 'f32'}", envs, steps, compile_s, run_s, peak, finite=finite
    )


def main() -> None:
    import argparse
    import os

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--preset", default="measured", help="plant preset name or JSON path")
    ap.add_argument("--envs", type=int, nargs="+", default=[1, 64, 256, 1024, 4096])
    ap.add_argument("--steps", type=int, default=1000, help="physics steps per env")
    ap.add_argument("--x64", action="store_true", help="also measure float64")
    args = ap.parse_args()

    import jax

    preset = load_preset(args.preset)
    print(
        f"device: {jax.local_devices()[0]}  |  cores: {os.cpu_count()}  |  "
        f"preset: {preset.name} (envelope {preset.envelope or 'none'})"
    )
    base = cpu_baseline(preset)
    cores = os.cpu_count() or 1
    print(
        f"\n{'engine':>10s} {'envs':>6s} {'steps/s':>12s} {'compile':>9s} "
        f"{'run':>8s} {'peak VRAM':>11s}"
    )
    print(
        f"{base.label:>10s} {base.envs:6d} {base.steps_per_s:12,.0f} {'-':>9s} "
        f"{base.run_s:7.2f}s {'-':>11s}"
    )
    print(
        f"{'(pool)':>10s} {cores:6d} {base.steps_per_s * cores:12,.0f} {'-':>9s} "
        f"{'-':>8s} {'-':>11s}   <- the bar: one core x cores"
    )

    for x64 in [False, True] if args.x64 else [False]:
        for envs in args.envs:
            try:
                r = mjx_throughput(preset, envs, args.steps, x64=x64)
            except Exception as exc:  # OOM is a RESULT, not a crash
                print(
                    f"{'mjx f64' if x64 else 'mjx f32':>10s} {envs:6d} "
                    f"{'FAILED':>12s}   {type(exc).__name__}: {str(exc)[:60]}"
                )
                break
            print(
                f"{r.label:>10s} {r.envs:6d} {r.steps_per_s:12,.0f} {r.compile_s:8.1f}s "
                f"{r.run_s:7.2f}s {r.peak_mib:10,.0f}M"
                + ("" if r.finite else "   <- went NaN: a speed, not a survivable plant")
            )


if __name__ == "__main__":
    main()
