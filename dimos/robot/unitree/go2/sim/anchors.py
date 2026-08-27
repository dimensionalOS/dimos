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

"""robot.json -> pinned knob values. Anchor what physics knows; fit the rest.

Every field of :class:`RobotSpec` is a measurement that was once wrong and
cost us: ``mass_kg`` by 1.3 kg (a bigger battery no amount of fitting found; a
kitchen scale did), the tracker mount by 178.9 deg, the lever arm by 38.6 mm.
Anchoring earns its keep empirically, not as a principle: left free,
``trunk_inertia_scale`` goes to 2.218 and buys 1.6 points in-sample, but the
physical 1.118 generalises better (-10.4% vs -6.6% on held-out jumps).

THE PARALLEL-AXIS FORM, in full because the proposal left it unspelled. The
stock menagerie trunk has mass ``m0``, CoM ``c0`` (``body_ipos``) and diagonal
inertia ``I0`` about ``c0`` along the body axes. Weighing the robot gives a
surplus ``mp = mass_kg - model_total`` carried as a point mass at the payload
position ``p``. Then:

    m  = m0 + mp
    c  = (m0*c0 + mp*p) / m                      (new CoM)
    I_k = I0_k + m0*(|d0|^2 - d0_k^2) + mp*(|dp|^2 - dp_k^2)
          with d0 = c0 - c,  dp = p - c          (parallel axis, per body axis)

``apply_physics`` only exposes a UNIFORM inertia scale, so the anchor is the
least-squares uniform fit  s = sum_k(I_k*I0_k) / sum_k(I0_k^2),  and:

    trunk_mass_scale    = m / m0
    trunk_com_x         = c_x - c0_x
    trunk_inertia_scale = s

WHAT THIS IS, said plainly: ``trunk_mass_scale`` is a derivation from the
weighing, exact at any mass. The CoM and inertia anchors are an
INTERPOLATION of the prior R8-FIT2 analysis, not a derivation from
measurement — the payload position ``p`` was SOLVED to hit the published
com_x (-0.01204) and inertia scale (1.118): two unknowns (payload x, z), two
targets, so exact agreement is guaranteed by construction and the
reproduction test can only catch broken algebra, never wrong physics. The
solved point lands at z = -0.102, which is 45 mm BELOW the trunk's underside
(the collision box is 0.114 m tall about the base frame): it is an effective
parameter absorbing the point-mass simplification, not a place the battery
is. That is why :func:`derive` refuses masses away from the 16.500 kg
calibration point — extrapolating projects anchors from a point outside the
robot, calibrated at a surplus that no longer applies.

The physics-facing checks live elsewhere: applying the derived anchors to
the compiled model must reproduce the WEIGHED total mass (see
test_backend), and held-out generalisation is what validated 1.118 in the
first place. Across a ±2 cm neighbourhood of the solved point the scale
stays in 1.05-1.22 and the CoM shift stays negative; the CoM RISE is
<= 2.7 mm, which is why no trunk_com_z knob exists.

The torsional friction anchor is the classical solid-contact result
``(3*pi/16) * mu * a`` for patch radius ``a`` — half the foot radius, an 11 mm
patch on the 22 mm hemisphere: 0.00583, where the fit lands independently.
Menagerie's 0.02 default implies a 37.7 mm patch, wider than the whole foot.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

# Stock menagerie unitree_go2 (commit 4c358ef9d9d7), read off the compiled
# model; a go2sim-marked test keeps them honest against the checkout.
STOCK_TRUNK_MASS = 6.921
STOCK_TRUNK_IPOS = (0.021112, 0.0, -0.005366)
# Principal inertia mapped to body (roll, pitch, yaw) axes.
STOCK_TRUNK_INERTIA = (0.0244531, 0.0980771, 0.107027)
STOCK_MODEL_TOTAL_MASS = 15.206408

# The payload's EFFECTIVE position in the base frame: SOLVED so the
# parallel-axis form reproduces the R8-FIT2 anchors at the 16.500 kg
# calibration mass — not tape-measured, and not physical: z = -0.102 is 45 mm
# below the trunk's underside (the collision box ends at z = -0.057). The
# excess depth is the point-mass simplification absorbing the battery's own
# inertia and the tracker's top-side mass. Valid only as an interpolation
# near the calibration point; derive() enforces that.
PAYLOAD_SITES: dict[str, tuple[float, float, float]] = {
    "battery_bay": (-0.055345, 0.0, -0.102136),
}

# The weighing the solved payload point is calibrated against, and how far a
# declared mass may drift from it before the CoM/inertia interpolation stops
# meaning anything: ±0.25 kg is ~20% of the 1.29 kg surplus. A new battery
# needs a new analysis, not an extrapolation from a point outside the robot.
CALIBRATED_MASS_KG = 16.500
MASS_INTERPOLATION_TOL_KG = 0.25


@dataclass(frozen=True)
class FootSpec:
    material: str = "rubber"
    radius_m: float = 0.022


@dataclass(frozen=True)
class PayloadSpec:
    mass_kg: float
    where: str = "battery_bay"


@dataclass(frozen=True)
class TrackerSpec:
    lever_arm_m: tuple[float, float, float]
    mount_yaw_deg: float
    flip: bool


@dataclass(frozen=True)
class RobotSpec:
    """Things about THIS robot that no recording can reveal."""

    mass_kg: float
    payload: PayloadSpec | None = None
    foot: FootSpec = FootSpec()
    tracker: TrackerSpec | None = None
    floors: dict[str, dict[str, float]] | None = None

    @classmethod
    def from_json(cls, path: str | Path) -> RobotSpec:
        d = json.loads(Path(path).read_text())
        return cls.from_dict(d)

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> RobotSpec:
        payload = None
        if "payload" in d:
            payload = PayloadSpec(
                mass_kg=float(d["payload"]["mass_kg"]),
                where=str(d["payload"].get("where", "battery_bay")),
            )
        foot = FootSpec(**d["foot"]) if "foot" in d else FootSpec()
        tracker = None
        if "tracker" in d:
            t = d["tracker"]
            tracker = TrackerSpec(
                lever_arm_m=tuple(t["lever_arm_m"]),
                mount_yaw_deg=float(t["mount_yaw_deg"]),
                flip=bool(t.get("flip", False)),
            )
        return cls(
            mass_kg=float(d["mass_kg"]),
            payload=payload,
            foot=foot,
            tracker=tracker,
            floors=d.get("floors"),
        )


def torsional_friction(mu: float, foot_radius_m: float) -> float:
    """``(3*pi/16) * mu * a`` for a contact patch of radius half the foot's."""
    return (3.0 * math.pi / 16.0) * mu * (foot_radius_m / 2.0)


def derive(spec: RobotSpec, *, floor_mu: float = 0.9) -> dict[str, float]:
    """Pinned knob values from a robot.json — the anchoring discipline as code.

    Weighing the robot pins trunk mass exactly, at any mass. The CoM shift
    and inertia scale are an interpolation of the R8-FIT2 analysis (see the
    module docstring) and are only served near its 16.500 kg calibration
    point: further away this refuses rather than extrapolate from an
    effective payload point that sits outside the robot.
    """
    m0 = STOCK_TRUNK_MASS
    c0 = STOCK_TRUNK_IPOS
    i0 = STOCK_TRUNK_INERTIA

    mp = spec.mass_kg - STOCK_MODEL_TOTAL_MASS
    out: dict[str, float] = {
        "trunk_mass_scale": (m0 + mp) / m0,
        "foot_friction_torsional": torsional_friction(floor_mu, spec.foot.radius_m),
    }
    if mp <= 0:
        raise ValueError(
            f"measured mass {spec.mass_kg} kg is not above the {STOCK_MODEL_TOTAL_MASS} kg "
            "model: an anchor below the CAD mass means the wrong robot.json or the wrong model"
        )
    if abs(spec.mass_kg - CALIBRATED_MASS_KG) > MASS_INTERPOLATION_TOL_KG:
        raise ValueError(
            f"mass {spec.mass_kg} kg is too far from the {CALIBRATED_MASS_KG} kg the CoM/inertia "
            "anchors are calibrated at: their payload position was SOLVED against the R8-FIT2 "
            "analysis (it sits outside the robot) and does not extrapolate to a different "
            f"battery. trunk_mass_scale would still be exact ({out['trunk_mass_scale']:.5f} = "
            "(trunk + surplus) / trunk); pin it yourself and redo the payload analysis for "
            "trunk_com_x and trunk_inertia_scale"
        )
    where = spec.payload.where if spec.payload is not None else "battery_bay"
    if where not in PAYLOAD_SITES:
        raise ValueError(f"unknown payload site {where!r}; have {sorted(PAYLOAD_SITES)}")
    p = PAYLOAD_SITES[where]

    m = m0 + mp
    c = tuple((m0 * c0[k] + mp * p[k]) / m for k in range(3))
    d0 = tuple(c0[k] - c[k] for k in range(3))
    dp = tuple(p[k] - c[k] for k in range(3))
    d0_2 = sum(x * x for x in d0)
    dp_2 = sum(x * x for x in dp)
    i_new = tuple(
        i0[k] + m0 * (d0_2 - d0[k] * d0[k]) + mp * (dp_2 - dp[k] * dp[k]) for k in range(3)
    )
    scale = sum(a * b for a, b in zip(i_new, i0, strict=True)) / sum(b * b for b in i0)

    out["trunk_com_x"] = c[0] - c0[0]
    out["trunk_inertia_scale"] = scale
    return out
