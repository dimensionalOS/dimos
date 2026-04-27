# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Full E1–E8 recipe set (E9 skipped — gait mode is changed manually).

Layout:
    E1 — vx step response (4 amps × 2 directions)
    E2 — wz step response (4 amps × 2 directions)
    E3 — vx saturation ramp (3 sizes for different runway lengths)
    E4 — wz saturation ramp (in place)
    E7 — cross-coupling
        E7a: pure wz, measure vx leak
        E7b: pure vx, measure wz / yaw drift
    E8 — deadtime precision (sharp short steps, run with high :N for many repeats)

E5 / E6 chirps are intentionally not here — they're the most timing-sensitive
tests and need a cleaner space; defer until E1–E4 are in hand.
"""

from __future__ import annotations

from dimos.utils.characterization.recipes import (
    TestRecipe,
    constant,
    ramp,
    step,
)


# ──────────────────────────────────────────────────────────────────────────────
# Helpers

def _step(amp: float, channel: str, name: str) -> TestRecipe:
    return TestRecipe(
        name=name, test_type="step", duration_s=3.0,
        signal_fn=step(amplitude=amp, channel=channel),
    )


def _const_hold(vx: float, vy: float, wz: float, name: str, duration: float = 5.0) -> TestRecipe:
    return TestRecipe(
        name=name, test_type="constant", duration_s=duration,
        signal_fn=constant(vx=vx, vy=vy, wz=wz),
        pre_roll_s=2.0, post_roll_s=2.0,
    )


# ──────────────────────────────────────────────────────────────────────────────
# E1 — vx step response

e1_vx_pos_0p3 = _step( 0.3, "vx", "e1_vx_+0.3")
e1_vx_neg_0p3 = _step(-0.3, "vx", "e1_vx_-0.3")
e1_vx_pos_0p6 = _step( 0.6, "vx", "e1_vx_+0.6")
e1_vx_neg_0p6 = _step(-0.6, "vx", "e1_vx_-0.6")
e1_vx_pos_1p0 = _step( 1.0, "vx", "e1_vx_+1.0")
e1_vx_neg_1p0 = _step(-1.0, "vx", "e1_vx_-1.0")
e1_vx_pos_1p5 = _step( 1.5, "vx", "e1_vx_+1.5")
e1_vx_neg_1p5 = _step(-1.5, "vx", "e1_vx_-1.5")


# ──────────────────────────────────────────────────────────────────────────────
# E2 — wz step response

e2_wz_pos_0p3 = _step( 0.3, "wz", "e2_wz_+0.3")
e2_wz_neg_0p3 = _step(-0.3, "wz", "e2_wz_-0.3")
e2_wz_pos_0p6 = _step( 0.6, "wz", "e2_wz_+0.6")
e2_wz_neg_0p6 = _step(-0.6, "wz", "e2_wz_-0.6")
e2_wz_pos_1p0 = _step( 1.0, "wz", "e2_wz_+1.0")
e2_wz_neg_1p0 = _step(-1.0, "wz", "e2_wz_-1.0")
e2_wz_pos_1p5 = _step( 1.5, "wz", "e2_wz_+1.5")
e2_wz_neg_1p5 = _step(-1.5, "wz", "e2_wz_-1.5")


# ──────────────────────────────────────────────────────────────────────────────
# E3 — vx saturation ramp (three runway sizes)
# Slope ≈ 0.15–0.20 m/s² so the robot has time to track each instantaneous cmd.

# Big runway (≥10 m): 0 → 3 m/s over 15 s, ~22 m travel.
e3_vx_ramp_0_to_3 = TestRecipe(
    name="e3_vx_ramp_0_to_3", test_type="ramp", duration_s=15.0,
    signal_fn=ramp(start=0.0, end=3.0, duration=15.0, channel="vx"),
    pre_roll_s=2.0, post_roll_s=2.0,
)

# Medium runway (~7 m): 0 → 1.5 m/s over 10 s, ~7.5 m travel.
e3_vx_ramp_0_to_1p5 = TestRecipe(
    name="e3_vx_ramp_0_to_1.5", test_type="ramp", duration_s=10.0,
    signal_fn=ramp(start=0.0, end=1.5, duration=10.0, channel="vx"),
    pre_roll_s=2.0, post_roll_s=2.0,
)

# 5 m runway: 0 → 1.2 m/s over 7 s, ~4.2 m peak travel.
e3_vx_ramp_0_to_1p2_5m = TestRecipe(
    name="e3_vx_ramp_0_to_1.2_5m", test_type="ramp", duration_s=7.0,
    signal_fn=ramp(start=0.0, end=1.2, duration=7.0, channel="vx"),
    pre_roll_s=1.5, post_roll_s=1.5,
)

# Tight runway (~3–5 m): 0 → 1.0 m/s over 6 s, ~3 m travel.
e3_vx_ramp_0_to_1p0_tight = TestRecipe(
    name="e3_vx_ramp_0_to_1.0_tight", test_type="ramp", duration_s=6.0,
    signal_fn=ramp(start=0.0, end=1.0, duration=6.0, channel="vx"),
    pre_roll_s=1.5, post_roll_s=1.5,
)


# ──────────────────────────────────────────────────────────────────────────────
# E4 — wz saturation ramp (in place)

e4_wz_ramp_0_to_4 = TestRecipe(
    name="e4_wz_ramp_0_to_4", test_type="ramp", duration_s=15.0,
    signal_fn=ramp(start=0.0, end=4.0, duration=15.0, channel="wz"),
    pre_roll_s=2.0, post_roll_s=2.0,
)


# ──────────────────────────────────────────────────────────────────────────────
# E7 — cross-coupling

# E7a: pure wz (in place), three amplitudes × two directions
e7a_wz_pos_0p3 = _const_hold(0, 0,  0.3, "e7a_wz_+0.3")
e7a_wz_neg_0p3 = _const_hold(0, 0, -0.3, "e7a_wz_-0.3")
e7a_wz_pos_0p8 = _const_hold(0, 0,  0.8, "e7a_wz_+0.8")
e7a_wz_neg_0p8 = _const_hold(0, 0, -0.8, "e7a_wz_-0.8")
e7a_wz_pos_1p5 = _const_hold(0, 0,  1.5, "e7a_wz_+1.5")
e7a_wz_neg_1p5 = _const_hold(0, 0, -1.5, "e7a_wz_-1.5")

# E7b: pure vx (forward motion), three amplitudes × two directions
# 1.0 m/s × 5 s = 5 m travel — drop to 0.8 if your space is tighter.
e7b_vx_pos_0p3 = _const_hold( 0.3, 0, 0, "e7b_vx_+0.3")
e7b_vx_neg_0p3 = _const_hold(-0.3, 0, 0, "e7b_vx_-0.3")
e7b_vx_pos_0p6 = _const_hold( 0.6, 0, 0, "e7b_vx_+0.6")
e7b_vx_neg_0p6 = _const_hold(-0.6, 0, 0, "e7b_vx_-0.6")
e7b_vx_pos_1p0 = _const_hold( 1.0, 0, 0, "e7b_vx_+1.0")
e7b_vx_neg_1p0 = _const_hold(-1.0, 0, 0, "e7b_vx_-1.0")


# ──────────────────────────────────────────────────────────────────────────────
# E8 — deadtime precision

# Sharp 1.0 m/s vx step, short hold. Run with high :N for many repeats.
# 1.0 m/s × 0.5 s = 0.5 m per pulse — alternate +/- to stay in place.
e8_vx_short_step = TestRecipe(
    name="e8_vx_short_step", test_type="step", duration_s=0.5,
    signal_fn=step(amplitude=1.0, channel="vx"),
    pre_roll_s=1.0, post_roll_s=2.0,
)
e8_vx_short_step_neg = TestRecipe(
    name="e8_vx_short_step_neg", test_type="step", duration_s=0.5,
    signal_fn=step(amplitude=-1.0, channel="vx"),
    pre_roll_s=1.0, post_roll_s=2.0,
)
