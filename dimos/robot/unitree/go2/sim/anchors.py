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

THE PARALLEL-AXIS DERIVATION, in full because the proposal left it unspelled.
The stock menagerie trunk has mass ``m0``, CoM ``c0`` (``body_ipos``) and
diagonal inertia ``I0`` about ``c0`` along the body axes. Weighing the robot
gives a surplus ``mp = mass_kg - model_total`` carried as a point mass at the
payload position ``p`` (battery bay + tracker, lumped). Then:

    m  = m0 + mp
    c  = (m0*c0 + mp*p) / m                      (new CoM)
    I_k = I0_k + m0*(|d0|^2 - d0_k^2) + mp*(|dp|^2 - dp_k^2)
          with d0 = c0 - c,  dp = p - c          (parallel axis, per body axis)

``apply_physics`` only exposes a UNIFORM inertia scale, so the anchor is the
least-squares uniform fit  s = sum_k(I_k*I0_k) / sum_k(I0_k^2),  and:

    trunk_mass_scale    = m / m0
    trunk_com_x         = c_x - c0_x
    trunk_inertia_scale = s

For the weighed 16.500 kg robot this reproduces the published R8-FIT2 anchors
(1.18693 / -0.01204 / 1.118) exactly — see the test. Across every plausible
battery-bay position the scale stays in 1.07-1.21 and the CoM shift stays
negative; the CoM RISE is <= 2.7 mm, which is why no trunk_com_z knob exists.

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
# model; a mujoco-marked test keeps them honest against the checkout.
STOCK_TRUNK_MASS = 6.921
STOCK_TRUNK_IPOS = (0.021112, 0.0, -0.005366)
# Principal inertia mapped to body (roll, pitch, yaw) axes.
STOCK_TRUNK_INERTIA = (0.0244531, 0.0980771, 0.107027)
STOCK_MODEL_TOTAL_MASS = 15.206408

# The payload's EFFECTIVE position in the base frame: the point-mass location
# that reproduces the R8-FIT2 parallel-axis analysis of the weighed battery
# (+ tracker), solved, not tape-measured. It sits behind and below the trunk
# CoM; the extra depth is the point-mass simplification absorbing the
# battery's own inertia and the tracker's top-side mass.
PAYLOAD_SITES: dict[str, tuple[float, float, float]] = {
    "battery_bay": (-0.055345, 0.0, -0.102136),
}


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

    Weighing the robot re-pins three parameters at once: trunk mass directly,
    and by parallel axis the CoM shift and the inertia scale.
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
