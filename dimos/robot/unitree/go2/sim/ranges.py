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
:mod:`~dimos.robot.unitree.go2.sim.backend`), and these are the MuJoCo ones.

The single most important finding about fitting this plant: the data
localises it to a REGION, not a point. Four fits differing only in seed
agree on the loss to within 3 points and disagree on the parameters by up
to 8.8x — so a preset's values are a CENTRE, and no single number here
should be read as identified. Randomization spreads for training must be
re-measured under the LOO restart rule (README 4a) before anything trains:
the old 4-seed table belonged to a discarded fit and is gone with it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path

from dimos.robot.unitree.go2.sim.anchors import RobotSpec, derive
from dimos.robot.unitree.go2.sim.plant import TORQUE_ENVELOPES


@dataclass(frozen=True)
class Knob:
    """One engine parameter's declared range, judged in its OWN metric."""

    lo: float
    hi: float
    log: bool = False  # bounds are judged in the knob's own metric
    unit: str = ""
    why: str = ""  # where the range came from; required for anything shipped

    def position(self, v: float) -> float:
        """Where ``v`` sits in [0, 1] across the range, in the knob's metric.

        A linear test on a log-scaled range produced four false "at bound"
        warnings, one of which was used to argue a fit had not converged —
        which is why this is a method on the type and not arithmetic at call
        sites.
        """
        if self.log:
            return (math.log(v) - math.log(self.lo)) / (math.log(self.hi) - math.log(self.lo))
        return (v - self.lo) / (self.hi - self.lo)

    def at_bound(self, v: float, tol: float = 0.02) -> bool:
        p = self.position(v)
        return p < tol or p > 1.0 - tol


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
    "foot_solimp_width": Knob(0.001, 0.03, log=True, unit="m", why="seed spread 0.0021-0.0069"),
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

# Model-override keys apply_physics accepts: every knob except the actuator lag,
# which lives in the stepping loop rather than on the compiled model.
PHYSICS_KEYS = frozenset(KNOBS) - {"actuator_tau"}


@dataclass(frozen=True)
class Preset:
    """A named physics configuration: everything a rollout needs to be reproducible.

    Built-ins are immutable; a fit writes its winner to JSON under a NEW name
    (:meth:`save` refuses built-in names), so a refit against messy data can
    never cost a validated tune.

    ``envelope`` names the torque envelope the plant was FITTED under
    (:data:`~dimos.robot.unitree.go2.sim.plant.TORQUE_ENVELOPES`), or ``None``
    for the ideal actuator. It travels with the preset because the two are one
    claim: knobs fitted with the envelope on absorb a different share of the
    drive, and running such a plant without its envelope silently changes the
    physics (README 9: the reverse mistake — stacking the envelope on knobs
    fitted without it — double-counts). Every built-in carries ``None`` except
    ``measured``, whose envelope IS the claim (README 9).
    """

    name: str
    physics: dict[str, float] = field(default_factory=dict)
    actuator_tau: float = 0.0
    envelope: str | None = None
    # Per-value origin, one line each: "fitted: ...", "derived: ...",
    # "declared: ...", "measured: ...". README 3a's "everything is a knob;
    # only the provenance differs", carried in the data instead of in prose.
    provenance: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.envelope is not None and self.envelope not in TORQUE_ENVELOPES:
            raise ValueError(
                f"{self.name!r}: unknown envelope {self.envelope!r}; "
                f"expected one of {sorted(TORQUE_ENVELOPES)}"
            )

    def save(self, path: str | Path) -> Path:
        if self.name in BUILTIN_PRESETS:
            raise ValueError(
                f"{self.name!r} is a built-in preset and cannot be overwritten; "
                "name the result something new"
            )
        out = Path(path)
        d: dict[str, object] = {
            "name": self.name,
            "physics": self.physics,
            "actuator_tau": self.actuator_tau,
        }
        if self.envelope is not None:
            d["envelope"] = self.envelope
        if self.provenance:
            d["provenance"] = self.provenance
        out.write_text(json.dumps(d, indent=2))
        return out

    @classmethod
    def load(cls, path: str | Path) -> Preset:
        d = json.loads(Path(path).read_text())
        return cls(
            name=d.get("name", Path(path).stem),
            physics=dict(d.get("physics", {})),
            actuator_tau=float(d.get("actuator_tau", 0.0)),
            envelope=d.get("envelope"),
            provenance=dict(d.get("provenance", {})),
        )


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

# THE shipped plant: the measured trunk (weighed and derived), the six knobs
# the 2026-08-16 open-loop fit resolved, and the measured torque envelope.
# HYBRID PROVENANCE, deliberately: the knobs were fitted open loop with the
# envelope OFF, so running them with it ON is formally the double-counting
# README 9 warns about. It is kept because the referee prefers the hybrid
# everywhere (README 8-9: the envelope's loss gain is replicated, and the
# envelope-consistent refit grounds WORSE — the anti-transfer). DO NOT
# "fix" the inconsistency by refitting: that was tried, and lost.
MEASURED_PHYSICS: dict[str, float] = {
    "armature": 0.02899,
    "damping": 0.03808,
    "frictionloss": 1.46585,
    "leg_mass_scale": 1.0,
    "foot_friction": FLOOR_MU,
    **_ANCHORS,
}
MEASURED_ACTUATOR_TAU = 0.00525

# The experimental CONTROL, not a leftover: every claim this package makes
# is comparative — "the tuned plant matches the robot better than bare
# menagerie does" — and the eventual sim-to-real transfer experiment is
# stock-vs-tuned by design. Delete this and the claim becomes unverifiable.
# Since the assets were vendored (model.MENAGERIE_COMMIT), "bare menagerie"
# is a fixed object: stock = the pinned scene, untouched.
STOCK = Preset(name="stock")

MEASURED = Preset(
    name="measured",
    physics=dict(MEASURED_PHYSICS),
    actuator_tau=MEASURED_ACTUATOR_TAU,
    envelope="central",
    provenance={
        "armature": "fitted: R8-FIT2 open-loop multiple shooting, 2026-08-16 markers",
        "damping": "fitted: suspended recording's joint channel — the one knob accel cannot see",
        "frictionloss": "fitted: R8-FIT2 open-loop multiple shooting, 2026-08-16 markers",
        "leg_mass_scale": "fitted: joint fit ~0.96; kept at 1.0 (CAD trusted)",
        "foot_friction": "declared: DR_FLOOR centre, the mu the fit pins; closed-loop inert (README 9)",
        "foot_friction_torsional": "derived: torsional_friction(FLOOR_MU, foot radius) via anchors.derive",
        "trunk_mass_scale": "derived: 16.500 kg kitchen-scale weighing (anchors.derive)",
        "trunk_com_x": "derived: parallel-axis payload analysis (anchors.derive)",
        "trunk_inertia_scale": "derived: parallel-axis payload analysis (anchors.derive)",
        "actuator_tau": "fitted: R8-FIT2 open-loop multiple shooting, 2026-08-16 markers",
        "envelope": "measured: sysid.drive demanded-vs-delivered transfer, zero free parameters",
    },
)

BUILTIN_PRESETS: dict[str, Preset] = {p.name: p for p in (STOCK, MEASURED)}
DEFAULT_PRESET = "measured"


def load_preset(name: str | None = None) -> Preset:
    """A preset by built-in name, or from a JSON file a fit wrote."""
    if name is None:
        name = DEFAULT_PRESET
    if name in BUILTIN_PRESETS:
        return BUILTIN_PRESETS[name]
    path = Path(name)
    if path.is_file():
        return Preset.load(path)
    raise ValueError(
        f"unknown preset {name!r}: expected one of {sorted(BUILTIN_PRESETS)} or a JSON path"
    )


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
