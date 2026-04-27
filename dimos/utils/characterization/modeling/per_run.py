# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Per-run FOPDT fitter.

Loads one run via ``analyze_run.load_run``, parses channel / amplitude /
direction from the recipe name, then fits TWO FOPDTs:

  - **rise** (``params``):     cmd 0 → ``amplitude``, slice = [step_t, active_end_t]
  - **fall** (``params_down``): cmd ``amplitude`` → 0, slice = [active_end_t, end_of_post_roll]

The two fits use the same FOPDT primitive but different baselines and
opposite ``u_step`` sign. Comparing rise vs fall surfaces accel-vs-decel
asymmetry, which is common on legged plants (the robot can ramp up
faster than it can stop, or vice versa).

Recipe-name convention (from ``experiments.py``): ``e<num>_<channel>_<sign><amp>``,
e.g. ``e1_vx_+1.0`` or ``e2_wz_-0.3``. Step recipes only — runs whose
``test_type != "step"`` are returned with ``skip_reason`` set.
"""

from __future__ import annotations

import re
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from dimos.utils.characterization.modeling.fopdt import FopdtParams, fit_fopdt

_RECIPE_RE = re.compile(r"^e\d+_(vx|vy|wz)_([+-]?\d+(?:\.\d+)?)$")


@dataclass
class RunFit:
    run_id: str
    run_dir: str                  # absolute path to the run directory
    recipe: str
    channel: str | None
    amplitude: float | None       # signed; e.g. +1.0 or -0.3
    direction: str | None         # "forward" if amplitude > 0 else "reverse"
    mode: str                     # "default" or "rage", from session.json
    split: str | None             # "train" if direction=="forward" else "validate"
    params: FopdtParams | None    # rise fit (cmd 0 → amplitude)
    params_down: FopdtParams | None = None   # fall fit (cmd amplitude → 0)
    skip_reason: str | None = None
    extra: dict[str, Any] = field(default_factory=dict)         # rise fit metadata
    extra_down: dict[str, Any] = field(default_factory=dict)    # fall fit metadata

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


def parse_recipe_name(name: str) -> tuple[str, float] | None:
    """Return (channel, signed_amplitude) for parseable step recipe names.

    Returns ``None`` when the name doesn't match the convention. E8
    recipes (``e8_vx_short_step`` / ``e8_vx_short_step_neg``) don't carry
    the amplitude in the name; ``fit_run`` falls back to detecting it
    from the cmd data for those.
    """
    m = _RECIPE_RE.match(name)
    if m is None:
        return None
    return (m.group(1), float(m.group(2)))


def _detect_channel_and_amplitude(run) -> tuple[str, float] | None:
    """Fallback: read channel and signed amplitude directly from the cmd
    arrays. Used when the recipe name doesn't match the regex (e.g. E8).
    """
    from dimos.utils.characterization.scripts.analyze_run import _dominant_channel
    import numpy as np

    channel = _dominant_channel(run)
    cmd = {"vx": run.cmd_vx, "vy": run.cmd_vy, "wz": run.cmd_wz}[channel]
    if cmd.size == 0:
        return None
    nonzero = np.flatnonzero(np.abs(cmd) > 1e-6)
    if nonzero.size == 0:
        return None
    # The active hold's signed amplitude is the cmd value at the step edge.
    amp = float(cmd[nonzero[0]])
    if abs(amp) < 1e-6:
        return None
    return (channel, amp)


def fit_run(run_dir: Path, *, mode: str) -> RunFit:
    """Load one run, fit FOPDT, return a ``RunFit``.

    Skips with ``skip_reason`` set when the run is not a step recipe,
    when the recipe name doesn't parse, or when measured data is too
    short. Fit failures land as ``params.converged == False`` with a
    populated ``params.reason``.
    """
    from dimos.utils.characterization.scripts.analyze_run import (
        _channel_arrays,
        load_run,
    )

    run_dir = Path(run_dir).expanduser().resolve()
    run_id = run_dir.name

    try:
        run = load_run(run_dir)
    except Exception as e:
        return RunFit(
            run_id=run_id, run_dir=str(run_dir), recipe="<unknown>",
            channel=None, amplitude=None, direction=None,
            mode=mode, split=None, params=None,
            skip_reason=f"load_run failed: {type(e).__name__}: {e}",
        )

    recipe = run.metadata["recipe"]["name"]
    test_type = run.metadata["recipe"]["test_type"]
    if test_type != "step":
        return RunFit(
            run_id=run_id, run_dir=str(run_dir), recipe=recipe,
            channel=None, amplitude=None, direction=None,
            mode=mode, split=None, params=None,
            skip_reason=f"not a step recipe (test_type={test_type})",
        )

    parsed = parse_recipe_name(recipe)
    if parsed is None:
        # Fallback for recipes whose names don't carry the amplitude
        # (e.g. E8 ``e8_vx_short_step``). Read it from the cmd data.
        parsed = _detect_channel_and_amplitude(run)
    if parsed is None:
        return RunFit(
            run_id=run_id, run_dir=str(run_dir), recipe=recipe,
            channel=None, amplitude=None, direction=None,
            mode=mode, split=None, params=None,
            skip_reason=f"could not infer channel/amplitude for recipe {recipe!r}",
        )
    channel, amplitude = parsed
    direction = "forward" if amplitude > 0 else "reverse"
    split = "train" if direction == "forward" else "validate"

    cmd_arr, meas_arr = _channel_arrays(run, channel)
    cmd_ts_rel = run.cmd_ts_rel
    meas_ts_rel = run.meas_ts_rel

    if meas_ts_rel.size < 4:
        return RunFit(
            run_id=run_id, run_dir=str(run_dir), recipe=recipe, channel=channel,
            amplitude=amplitude, direction=direction, mode=mode, split=split,
            params=None,
            skip_reason="fewer than 4 measured samples",
        )

    # Step edge: first commanded sample where |cmd| > 1e-6 on the parsed channel.
    nonzero = np.flatnonzero(np.abs(cmd_arr) > 1e-6)
    if nonzero.size == 0:
        return RunFit(
            run_id=run_id, run_dir=str(run_dir), recipe=recipe, channel=channel,
            amplitude=amplitude, direction=direction, mode=mode, split=split,
            params=None,
            skip_reason="no nonzero command on parsed channel",
        )
    step_t = float(cmd_ts_rel[nonzero[0]])
    duration = float(run.metadata["recipe"]["duration_s"])
    active_end_t = step_t + duration

    # Pre-step baseline (mean of measured during pre-roll, channel-specific).
    pre_mask = meas_ts_rel < step_t
    if pre_mask.any():
        baseline = float(np.mean(meas_arr[pre_mask]))
    else:
        baseline = 0.0

    # Fit window in absolute relative-time: [step_t, active_end_t]. Slice
    # both ts and meas, re-zero ts so step is at t=0, baseline-correct y.
    fit_mask = (meas_ts_rel >= step_t) & (meas_ts_rel <= active_end_t)
    if int(fit_mask.sum()) < 4:
        return RunFit(
            run_id=run_id, run_dir=str(run_dir), recipe=recipe, channel=channel,
            amplitude=amplitude, direction=direction, mode=mode, split=split,
            params=None,
            skip_reason=f"fewer than 4 samples in fit window ({int(fit_mask.sum())})",
        )

    t_fit = meas_ts_rel[fit_mask] - step_t
    y_fit = meas_arr[fit_mask] - baseline

    noise_std = _noise_std_for_channel(run.metadata.get("noise_floor"), channel)

    params = fit_fopdt(
        t_fit, y_fit, u_step=amplitude,
        noise_std=noise_std,
        fit_window_s=(0.0, float(t_fit[-1])),
    )

    # ---- Step-down (fall) fit -------------------------------------------------
    # Fit window: from active_end_t through the end of post-roll (where data
    # exists). Pre-fall baseline = mean of meas during the last 30% of the
    # active window — i.e. the steady-state right before the cmd drops.
    # u_step for the fall = -amplitude (the change in command).
    post_roll_s = float(run.metadata["recipe"].get("post_roll_s", 1.0))
    fall_end_t = active_end_t + post_roll_s

    ss_window_lo = step_t + 0.7 * duration
    ss_mask = (meas_ts_rel >= ss_window_lo) & (meas_ts_rel <= active_end_t)
    if ss_mask.any():
        baseline_down = float(np.mean(meas_arr[ss_mask]))
    else:
        baseline_down = baseline  # fallback
    fall_mask = (meas_ts_rel >= active_end_t) & (meas_ts_rel <= fall_end_t)

    params_down: FopdtParams | None
    extra_down: dict[str, Any]
    if int(fall_mask.sum()) < 4:
        params_down = None
        extra_down = {
            "active_end_t": active_end_t,
            "fall_end_t": fall_end_t,
            "baseline_down": baseline_down,
            "skip_reason": f"fewer than 4 samples in fall window ({int(fall_mask.sum())})",
        }
    else:
        t_fall = meas_ts_rel[fall_mask] - active_end_t
        y_fall = meas_arr[fall_mask] - baseline_down
        params_down = fit_fopdt(
            t_fall, y_fall, u_step=-amplitude,
            noise_std=noise_std,
            fit_window_s=(0.0, float(t_fall[-1])),
        )
        extra_down = {
            "active_end_t": active_end_t,
            "fall_end_t": fall_end_t,
            "baseline_down": baseline_down,
            "n_samples_fall": int(t_fall.size),
        }

    return RunFit(
        run_id=run_id, run_dir=str(run_dir), recipe=recipe, channel=channel,
        amplitude=amplitude, direction=direction, mode=mode, split=split,
        params=params,
        params_down=params_down,
        skip_reason=None,
        extra={
            "step_t": step_t,
            "active_end_t": active_end_t,
            "baseline": baseline,
            "noise_std": noise_std,
            "n_meas_total": int(meas_ts_rel.size),
        },
        extra_down=extra_down,
    )


def _noise_std_for_channel(
    noise_floor: dict[str, Any] | None, channel: str
) -> float | None:
    """Pull per-channel σ from ``run.json["noise_floor"]``. Returns None when missing."""
    if not noise_floor or "_unavailable" in noise_floor:
        return None
    entry = noise_floor.get(channel)
    if not isinstance(entry, dict):
        return None
    std = entry.get("std")
    if std is None:
        return None
    try:
        v = float(std)
    except (TypeError, ValueError):
        return None
    return v if v > 0 else None


def select_edge(run_fits: list[RunFit], edge: str) -> list[RunFit]:
    """Return a list of RunFits projected onto one edge (rise or fall).

    For ``edge == "rise"`` the originals are returned. For ``edge == "fall"``
    each RunFit is shallow-copied with ``params := params_down`` and
    ``extra := extra_down`` so downstream aggregate/pool code can be
    edge-agnostic.
    """
    if edge == "rise":
        return run_fits
    if edge != "fall":
        raise ValueError(f"edge must be 'rise' or 'fall', got {edge!r}")
    out: list[RunFit] = []
    for rf in run_fits:
        out.append(RunFit(
            run_id=rf.run_id, run_dir=rf.run_dir, recipe=rf.recipe,
            channel=rf.channel, amplitude=rf.amplitude, direction=rf.direction,
            mode=rf.mode, split=rf.split,
            params=rf.params_down,
            params_down=None,
            skip_reason=rf.skip_reason,
            extra=rf.extra_down or {},
            extra_down={},
        ))
    return out


__all__ = ["RunFit", "fit_run", "parse_recipe_name", "select_edge"]
