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

"""A knob's declared range and a named physics configuration.

Robot-agnostic: the knob SET and the shipped presets are a robot's data
(:mod:`dimos.robot.unitree.go2.sim.ranges`); these are the two types they
are made of. ``Knob.log`` exists because a bound must be judged in the
parameter's own metric, ``Knob.why`` because a range without provenance is
a guess.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
import json
import math
from pathlib import Path


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
        warnings, one of which was used to argue a fit had not converged,
        which is why this is a method on the type and not arithmetic at call
        sites.
        """
        if self.log:
            return (math.log(v) - math.log(self.lo)) / (math.log(self.hi) - math.log(self.lo))
        return (v - self.lo) / (self.hi - self.lo)

    def at_bound(self, v: float, tol: float = 0.02) -> bool:
        p = self.position(v)
        return p < tol or p > 1.0 - tol


@dataclass(frozen=True)
class Preset:
    """A named physics configuration: everything a rollout needs to be reproducible.

    ``envelope`` names the torque envelope the plant was FITTED under, or
    ``None`` for the ideal actuator. It travels with the preset because the
    two are one claim: knobs fitted with the envelope on absorb a different
    share of the drive, and running such a plant without its envelope
    silently changes the physics. The name is resolved by the robot that
    owns the envelope table, at use.
    """

    name: str
    physics: dict[str, float] = field(default_factory=dict)
    actuator_tau: float = 0.0
    envelope: str | None = None
    # Per-value origin, one line each: "fitted: ...", "derived: ...",
    # "declared: ...", "measured: ...". "Everything is a knob; only the
    # provenance differs", carried in the data instead of in prose.
    provenance: dict[str, str] = field(default_factory=dict)
    # A shipped preset is immutable: a fit writes its winner under a NEW name,
    # so a refit against messy data can never cost a validated tune.
    builtin: bool = field(default=False, compare=False)

    def save(self, path: str | Path) -> Path:
        if self.builtin:
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


def load_preset(name: str, builtins: Mapping[str, Preset]) -> Preset:
    """A preset by built-in name, or from a JSON file a fit wrote."""
    if name in builtins:
        return builtins[name]
    path = Path(name)
    if path.is_file():
        return Preset.load(path)
    raise ValueError(f"unknown preset {name!r}: expected one of {sorted(builtins)} or a JSON path")
