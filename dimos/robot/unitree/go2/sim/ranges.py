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

"""Knob set and the two shipped presets for the measured Go2 plant.

Everything here is a measurement, a derivation from one, a fitted value, or
a declaration — and each value on a shipped preset says which
(:attr:`Preset.provenance`). Derived values are COMPUTED from their sources
at import, never copied: the discarded early plant shipped a torsional
friction implying mu = 0.349 against its own declared 0.635, a stale copy
nobody could have caught by reading either number alone. The knob SET is
data, not code: a backend exposes its own (see
:mod:`~dimos.simulation.sysid.backend`), and these are the MuJoCo ones.

The single most important finding about fitting this plant: the data
localises it to a REGION, not a point. Four fits differing only in seed
agree on the loss to within 3 points and disagree on the parameters by up
to 8.8x — so a preset's values are a CENTRE, and no single number here
should be read as identified. Randomization spreads for training must be
re-measured under the LOO restart rule (README 4a) before anything trains:
the old 4-seed table belonged to a discarded fit and is gone with it.
"""

from __future__ import annotations

from dimos.robot.unitree.go2.sim.anchors import RobotSpec, derive
from dimos.robot.unitree.go2.sim.plant import TORQUE_ENVELOPES
from dimos.simulation.sysid.presets import Knob, Preset, load_preset as _load_preset

# The MuJoCo knob set. Ranges come from ~/recordings/DR_RANGES.md: cross-regime
# fit disagreement plus the measured 4-seed procedure variance.
KNOBS: dict[str, Knob] = {
    "armature": Knob(
        0.001,
        0.05,
        log=True,
        unit="kg*m^2",
        why="seed spread 0.0015-0.0131 (8.8x); cross-regime fits up to 0.049",
    ),
    "damping": Knob(
        0.01,
        0.6,
        log=True,
        unit="N*m*s/rad",
        why="only the suspended regime resolves it (330x information vs walking); fits 0.04-0.5",
    ),
    "frictionloss": Knob(
        0.1, 2.0, log=True, unit="N*m", why="fits 0.21-1.60 across regimes and seeds"
    ),
    "actuator_tau": Knob(
        0.0005, 0.02, log=True, unit="s", why="seed spread 0.0009-0.0066; loaded data wanted 0.016"
    ),
    "trunk_mass_scale": Knob(
        0.8, 1.4, why="anchored by the kitchen scale in practice; range covers battery options"
    ),
    "trunk_inertia_scale": Knob(
        0.8,
        1.3,
        why="hard ceiling ~1.3: free searches run to 3.25-3.46 as a fudge factor, buying ~5% on walking and losing 7-9% on jumps",
    ),
    "trunk_com_x": Knob(
        -0.03, 0.03, unit="m", why="parallel-axis consequence of the payload; anchored"
    ),
    "leg_mass_scale": Knob(
        0.7,
        1.7,
        why="single-regime fits 1.31-1.66 absorbed run error; joint fit 0.96; seed spread 1.06-1.44",
    ),
    "foot_friction": Knob(
        0.5,
        1.0,
        why="a floor property, not a robot one; 0.8-1.0 covers rubber pads on both "
        "measured surfaces, neither slips — and closed loop it does not matter: "
        "sweeping 0.6-1.6 moves speed 0.4% against an 18% deficit (README 9), "
        "so the shipped 0.635 sitting below DR_FLOOR's (0.8, 1.0) is inert",
    ),
    "foot_friction_torsional": Knob(
        0.001,
        0.02,
        log=True,
        unit="m",
        why="(3pi/16)*mu*a; fitted 0.00583 == derived for an 11 mm patch. Menagerie's 0.02 implies a 37.7 mm patch, wider than the foot",
    ),
    "foot_solref_time": Knob(
        0.002,
        0.02,
        log=True,
        unit="s",
        why="seed spread 0.0066-0.0163; Hertz prediction ~0.010 not confirmed (one draw landing on it was a sample, not a measurement)",
    ),
    "foot_solref_damp": Knob(0.5, 2.0, why="seed spread 1.10-1.43"),
    "foot_solimp_dmin": Knob(
        0.005, 0.5, log=True, why="never resolved; menagerie default 0.015 stands"
    ),
    "foot_solimp_width": Knob(
        0.001,
        0.08,
        log=True,
        unit="m",
        why="stiff-solver seed spread 0.0021-0.0069; hi widened 0.03->0.08 by the "
        "cheap-solver contact refit (2026-08-19), whose referee-selected draws "
        "sit at 0.03-0.07 — the two solvers want different contact shapes",
    ),
    # Contact SOLVER settings. Model options, not fitted quantities — but a
    # preset must fully determine the plant, and these were inherited silently
    # from menagerie's scene until 2026-08-19, when the batched engine made
    # them a CHOICE: walking on the shipped contact is bit-identical down to
    # Newton 15/20 (so 100/50 is slack), explodes intermittently at 10/10 and
    # below, and the cheap 1/5 the batched engine wants needs a RE-IDENTIFIED
    # contact (the `fast` preset). Never in DEFAULT_SEARCH: a solver is chosen
    # and then the contact is identified UNDER it, not the other way around.
    "solver_iterations": Knob(
        1,
        200,
        log=True,
        why="Newton iteration cap; on the shipped plant a PD-held stand converges "
        "by 2, but WALKING needs ~15 (impacts) — 4/10 explodes mid-recording",
    ),
    "solver_ls_iterations": Knob(
        1,
        100,
        log=True,
        why="line-search cap per Newton iteration; walking on the shipped plant "
        "needs ~16-20 (10 explodes where 16 holds; the cap binds before Newton's)",
    ),
    "solver_cone": Knob(
        0,
        1,
        why="categorical, mjtCone: 0 = pyramidal, 1 = elliptic. The scene's "
        "impratio 100 was chosen for elliptic friction; pyramidal linearises "
        "the cone and must be re-identified, not toggled",
    ),
}

# Foot-floor contact solver keys. No early preset carries a value for these:
# an absent key is never written, the menagerie default stands, and every
# rollout scored before contact modelling reproduces bit-for-bit.
CONTACT_DEFAULTS: dict[str, float] = {
    # go2.xml foot class: solref="0.02 1", solimp="0.015 1 0.022 0.5 2".
    "foot_solref_time": 0.02,
    "foot_solref_damp": 1.0,
    "foot_solimp_dmin": 0.015,
    "foot_solimp_width": 0.022,
}
CONTACT_KEYS = frozenset(CONTACT_DEFAULTS)

# Contact-solver keys and the values the vendored scene compiles to (go2.xml
# declares none of them, so these are MuJoCo's own defaults plus the scene's
# cone choice). Same discipline as CONTACT_DEFAULTS: an absent key is never
# written and every pre-schema preset reproduces bit-for-bit.
SOLVER_DEFAULTS: dict[str, float] = {
    "solver_iterations": 100.0,
    "solver_ls_iterations": 50.0,
    "solver_cone": 1.0,  # mjCONE_ELLIPTIC
}
SOLVER_KEYS = frozenset(SOLVER_DEFAULTS)

# What the engine would do with no preset at all: the scene's own values for
# every key a preset MAY carry but early presets did not.
ENGINE_DEFAULTS: dict[str, float] = {**CONTACT_DEFAULTS, **SOLVER_DEFAULTS}

# Model-override keys apply_physics accepts: every knob except the actuator lag,
# which lives in the stepping loop rather than on the compiled model.
PHYSICS_KEYS = frozenset(KNOBS) - {"actuator_tau"}


# The robot the shipped plant is derived for: weighed 16.500 kg on a kitchen
# scale, 2026-08-16 — an anchor no amount of fitting ever found (the model
# was 1.3 kg light). Change the robot, change THIS, and every derived anchor
# follows.
GO2 = RobotSpec(mass_kg=16.500)

# The floor the anchors assume: the centre of DR_FLOOR and the mu the fit
# pins (fit.default_plan). A declaration, not a measurement — and a nearly
# inert one closed loop: sweeping mu 0.6-1.6 moved speed 0.4% (README 9).
FLOOR_MU = 0.90

# trunk_mass_scale, trunk_com_x, trunk_inertia_scale, foot_friction_torsional
# — computed from the weighing and the contact-patch formula at import, so a
# stale copy cannot exist. This is the same derive() call the fit uses for
# its pins: the shipped plant and the fitting discipline agree by
# construction.
_ANCHORS = derive(GO2, floor_mu=FLOOR_MU)

# THE shipped plant: the measured trunk (weighed and derived), and the
# searched knobs of draw 54 — SELECTED, not just fitted (2026-08-18). The
# open-loop fit pins a region, not a point (README 4a), and refit
# stochasticity beats every judge-weighting scheme (README 4: the outer
# study) — so the shipping step is DRAW SELECTION: 161 joint draws from the
# incumbent objective's pooled cloud, each grounded at 16 replicates on the
# selection recording (195401), the top three quoted once on the reserve
# (195539), and the reserve's winner ships. Quoted: loss 1.93 vs the
# previous plant's 2.14, with all three finalists beating it on both
# recordings. HYBRID PROVENANCE still deliberate: knobs fitted open loop
# with the envelope OFF, run with it ON — the referee prefers the hybrid
# everywhere and the envelope-consistent refit grounds WORSE. DO NOT "fix"
# the inconsistency by refitting: that was tried, and lost.
MEASURED_PHYSICS: dict[str, float] = {
    "armature": 0.026235469296468247,
    "damping": 0.03808,
    "frictionloss": 0.6997397096456963,
    "leg_mass_scale": 1.3275068262275753,
    "foot_friction": FLOOR_MU,
    "foot_solref_time": 0.014832125244436329,
    "foot_solref_damp": 1.1398535892980237,
    "foot_solimp_dmin": 0.015,
    "foot_solimp_width": 0.0016142451913904524,
    # The solver the knobs were identified UNDER, recorded rather than
    # inherited: these are exactly what the scene compiles to, so writing
    # them down changes nothing (test_backend.py holds that).
    **SOLVER_DEFAULTS,
    **_ANCHORS,
}
MEASURED_ACTUATOR_TAU = 0.005010955656476395

# The experimental CONTROL, not a leftover: every claim this package makes
# is comparative — "the tuned plant matches the robot better than bare
# menagerie does" — and the eventual sim-to-real transfer experiment is
# stock-vs-tuned by design. Delete this and the claim becomes unverifiable.
# Since the assets were vendored (model.MENAGERIE_COMMIT), "bare menagerie"
# is a fixed object: stock = the pinned scene, untouched.
STOCK = Preset(name="stock", builtin=True)

MEASURED = Preset(
    builtin=True,
    name="measured",
    physics=dict(MEASURED_PHYSICS),
    actuator_tau=MEASURED_ACTUATOR_TAU,
    envelope="central",
    provenance={
        "armature": "selected: draw 54 of the 2026-08-18 incumbent-objective cloud",
        "damping": "fitted: suspended recording's joint channel — the one knob accel cannot see",
        "frictionloss": "selected: draw 54 (trades against the contact knobs; see cloud spread)",
        "leg_mass_scale": "selected: draw 54",
        "foot_friction": "declared: DR_FLOOR centre, the mu the fit pins; closed-loop inert (README 9)",
        "foot_solref_time": "selected: draw 54",
        "foot_solref_damp": "selected: draw 54",
        "foot_solimp_dmin": "declared: the fit's contact base value, carried by every graded draw",
        "foot_solimp_width": "selected: draw 54",
        "foot_friction_torsional": "derived: torsional_friction(FLOOR_MU, foot radius) via anchors.derive",
        "trunk_mass_scale": "derived: 16.500 kg kitchen-scale weighing (anchors.derive)",
        "trunk_com_x": "derived: parallel-axis payload analysis (anchors.derive)",
        "trunk_inertia_scale": "derived: parallel-axis payload analysis (anchors.derive)",
        "actuator_tau": "selected: draw 54 of the 2026-08-18 incumbent-objective cloud",
        "envelope": "measured: sysid.drive demanded-vs-delivered transfer, zero free parameters",
        "solver_iterations": "declared: menagerie scene default, recorded explicitly 2026-08-19 "
        "(walking replay is bit-identical down to 15/20, so 100/50 is slack, not load-bearing)",
        "solver_ls_iterations": "declared: menagerie scene default, recorded explicitly 2026-08-19",
        "solver_cone": "declared: the scene's elliptic cone, chosen with its impratio 100",
    },
)

# The batched-training plant: `measured` with its contact RE-IDENTIFIED for
# the cheap solver MJX is fast at. Newton 1/5 elliptic explodes the measured
# contact (foot_solimp_width 1.6 mm needs ~15 iterations through impacts), so
# the four contact knobs were refit UNDER 1/5 on the same recordings, pinned
# everywhere else to `measured` — isolating what the cheap solver costs.
# Shipped by draw selection (README 5): 78 pooled draws, 37 stable through
# both fit recordings open-loop, all 37 grounded at 16 replicates on the
# selection recording (195401), the top three quoted once on the
# speeds-strafe reserve (200750, SPENT 2026-08-19) — this is the reserve's
# winner. Quoted: selection 2.29 vs measured's 2.79 (MDD 0.26); reserve 3.81
# vs 4.10 — the cheap solver costs NOTHING measurable on the referee.
# Pyramidal (4x faster still) was killed by measurement: 2 of 103 draws
# survive open loop and its best explodes 1 in 16 closed-loop rollouts.
# Throughput: engines/bench.py — the reason this preset exists.
FAST_REFIT: dict[str, float] = {
    "solver_iterations": 1.0,
    "solver_ls_iterations": 5.0,
    "solver_cone": 1.0,  # elliptic: keeps the friction model impratio 100 was chosen for
    "foot_solref_time": 0.00419376600991049,
    "foot_solref_damp": 0.7847549255682962,
    "foot_solimp_dmin": 0.48066205059077133,
    "foot_solimp_width": 0.0707702018228441,
}

FAST = Preset(
    builtin=True,
    name="fast",
    physics={**MEASURED_PHYSICS, **FAST_REFIT},
    actuator_tau=MEASURED_ACTUATOR_TAU,
    envelope="central",
    provenance={
        **MEASURED.provenance,
        "solver_iterations": "declared: the cheap solver this plant is identified FOR "
        "(Newton 1; MJX's unrolled fast path)",
        "solver_ls_iterations": "declared: chosen with the iteration cap (1/5)",
        "solver_cone": "declared: elliptic — pyramidal is 4x faster batched but 101 of 103 "
        "fitted draws explode open-loop and its best survivor explodes closed loop",
        "foot_solref_time": "selected: draw 18 of the 2026-08-19 cheap-solver refit cloud "
        "(fit 194142+174724, selected on 195401 x16, quoted once on reserve 200750)",
        "foot_solref_damp": "selected: draw 18 of the 2026-08-19 cheap-solver refit cloud",
        "foot_solimp_dmin": "selected: draw 18 — near its range's hi bound (0.48 of [0.005, "
        "0.5]); the cheap solver wants a stiff-start wide ramp the stiff solver never did",
        "foot_solimp_width": "selected: draw 18 of the 2026-08-19 cheap-solver refit cloud",
    },
)

BUILTIN_PRESETS: dict[str, Preset] = {p.name: p for p in (STOCK, MEASURED, FAST)}
DEFAULT_PRESET = "measured"


def load_preset(name: str | None = None) -> Preset:
    """A built-in Go2 preset by name, or a JSON file a fit wrote."""
    p = _load_preset(DEFAULT_PRESET if name is None else name, BUILTIN_PRESETS)
    if p.envelope is not None and p.envelope not in TORQUE_ENVELOPES:
        raise ValueError(
            f"{p.name!r}: unknown envelope {p.envelope!r}; expected one of {sorted(TORQUE_ENVELOPES)}"
        )
    return p


# Measured or derived, never randomized: randomizing a measurement adds
# noise, not robustness. Same derive() call as the shipped preset, so the
# two cannot drift apart. Spreads for the SEARCHED knobs must be re-measured
# under the LOO restart rule (README 4a) before anything trains — the old
# 4-seed table belonged to the discarded accel fit.
DR_PINNED: dict[str, float] = {
    "trunk_mass_scale": _ANCHORS["trunk_mass_scale"],
    "trunk_com_x": _ANCHORS["trunk_com_x"],
    "foot_friction_torsional": _ANCHORS["foot_friction_torsional"],
}

DR_FLOOR: dict[str, tuple[float, float]] = {"foot_friction": (0.8, 1.0)}
