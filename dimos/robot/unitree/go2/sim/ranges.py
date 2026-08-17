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

"""Knob set, presets and DR spreads for the measured Go2 plant.

Everything here is either a direct measurement with provenance, or a fitted
value with a stated uncertainty range. The knob SET is data, not code: a
backend exposes its own (see :mod:`~dimos.robot.unitree.go2.sim.backend`), and
these are the MuJoCo ones.

The single most important finding about fitting this plant: the data localises
it to a REGION, not a point. Four fits differing only in seed agree on the
loss to within 3 points and disagree on the parameters by up to 8.8x — so a
preset's values are a CENTRE, the spreads in :data:`DR_ACCEL` are the measured
procedure variance, and no single number here should be read as identified.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path

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
        "sweeping 0.6-1.6 moves speed 0.4% against an 18% deficit (README 5g), "
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
    physics (§5b: the reverse mistake — stacking the envelope on knobs fitted
    without it — double-counts). Every built-in carries ``None``.
    """

    name: str
    physics: dict[str, float] = field(default_factory=dict)
    actuator_tau: float = 0.0
    envelope: str | None = None

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
        )


# The measured-trunk plant (R8-FIT2). Anchors, not fits: the robot was WEIGHED
# (16.500 kg against a 15.206 kg model — a bigger battery no fitting ever
# found), trunk_inertia_scale and trunk_com_x are parallel-axis consequences of
# that payload (see anchors.py), and the remaining six were fitted by open-loop
# multiple shooting on the 2026-08-16 marker session. `damping` is the one
# parameter the accelerometer cannot see; its value comes from the suspended
# recording's joint channel and nothing else.
MEASURED_PHYSICS = {
    "armature": 0.02899,
    "damping": 0.03808,
    "frictionloss": 1.46585,
    "trunk_mass_scale": 1.18693,
    "trunk_inertia_scale": 1.118,
    "foot_friction": 0.63548,
    "foot_friction_torsional": 0.00226,
    "trunk_com_x": -0.01204,
    "leg_mass_scale": 1.0,
}
MEASURED_ACTUATOR_TAU = 0.00525

# The accel-channel fit (R9-ACCEL): same anchors, contact modelled, virtual IMU
# read at the sensor site, remaining knobs = per-parameter MEDIAN of four
# seeded fits (never the best draw — picking it would spend the held-out set).
# All four draws beat `measured` on held-out jumps (-11.5 to -18.4% accel).
ACCEL_PHYSICS = {
    "armature": 0.00595,
    "damping": 0.03808,
    "frictionloss": 1.06436,
    "trunk_mass_scale": 1.18693,
    "trunk_inertia_scale": 1.118,
    "foot_friction": 0.90,
    "foot_friction_torsional": 0.00583,
    "trunk_com_x": -0.01204,
    "leg_mass_scale": 1.32236,
    "foot_solref_time": 0.00853,
    "foot_solref_damp": 1.17897,
    "foot_solimp_width": 0.00244,
    "foot_solimp_dmin": CONTACT_DEFAULTS["foot_solimp_dmin"],
}
ACCEL_ACTUATOR_TAU = 0.00271

STOCK = Preset(name="stock")  # no overrides at all: bare menagerie
MEASURED = Preset(
    name="measured", physics=dict(MEASURED_PHYSICS), actuator_tau=MEASURED_ACTUATOR_TAU
)
ACCEL = Preset(name="accel", physics=dict(ACCEL_PHYSICS), actuator_tau=ACCEL_ACTUATOR_TAU)

BUILTIN_PRESETS: dict[str, Preset] = {p.name: p for p in (STOCK, MEASURED, ACCEL)}
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


# What training should randomize over. min/max over n draws covers (n-1)/(n+1)
# of the distribution — 60% at n=4 — so these 4-seed spreads are too NARROW
# and should be recomputed under the LOO-stability restart rule before anything
# trains against them. Under-randomizing is the dangerous direction.
DR_ACCEL: dict[str, tuple[float, float]] = {
    "armature": (0.0015, 0.0131),  # 8.8x spread
    "actuator_tau": (0.0009, 0.0066),  # 7.2x
    "foot_solimp_width": (0.0021, 0.0069),  # 3.3x
    "foot_solref_time": (0.0066, 0.0163),  # 2.5x
    "frictionloss": (0.948, 1.601),  # 1.7x
    "leg_mass_scale": (1.055, 1.441),  # 1.4x
    "foot_solref_damp": (1.103, 1.432),  # 1.3x
}

# Measured or derived, never randomized: randomizing a measurement adds noise,
# not robustness.
DR_PINNED: dict[str, float] = {
    "trunk_mass_scale": 1.18693,
    "trunk_com_x": -0.01204,
    "foot_friction_torsional": 0.00583,
}

DR_FLOOR: dict[str, tuple[float, float]] = {"foot_friction": (0.8, 1.0)}
