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

"""Loop 1 on the Go2: the anchoring plan, and the fit CLI on the Go2 pieces.

The method is :mod:`dimos.simulation.sysid.fit`; this module says what the
Go2 pins (a weighed trunk, a damping only the hanging regime resolves, the
floor) and what it searches.

    python -m dimos.robot.unitree.go2.sim.sysid.fit REC.mcap JUMPS.mcap --workers 20 --out results/freewalk
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

from dimos.robot.unitree.go2.sim.anchors import RobotSpec, derive
from dimos.robot.unitree.go2.sim.ranges import DEFAULT_PRESET, ENGINE_DEFAULTS, load_preset
from dimos.simulation.sysid.fit import KnobPlan, Pin, main as _main, start_values
from dimos.simulation.sysid.presets import Knob

# The default search set: the knobs the 2026-08-16 fits searched — the ones
# the identifiability spectrum resolves. Everything else is anchored by a
# measurement (see default_plan).
DEFAULT_SEARCH: tuple[str, ...] = (
    "armature",
    "actuator_tau",
    "frictionloss",
    "leg_mass_scale",
    "foot_solref_time",
    "foot_solref_damp",
    "foot_solimp_width",
)


def default_plan(
    backend_knobs: Mapping[str, Knob],
    robot: RobotSpec | None = None,
    *,
    floor_mu: float = 0.9,
    search: Sequence[str] | None = None,
) -> KnobPlan:
    """The anchoring discipline as code: derive pins from the robot spec,
    search the rest of the measured-informative set.

    ``damping`` is pinned because only the suspended regime resolves it (330x
    the information of walking) — fit it there, on the joint channel, never
    here. ``foot_friction`` is a floor property, not a robot one.
    """
    spec = robot or RobotSpec(mass_kg=16.500)
    derived = derive(spec, floor_mu=floor_mu)
    why = {
        "trunk_mass_scale": f"(trunk + weighed surplus)/trunk at {spec.mass_kg} kg",
        "trunk_com_x": "parallel-axis consequence of the weighed payload",
        "trunk_inertia_scale": "parallel-axis interpolation at the weighed mass",
        "foot_friction_torsional": f"(3pi/16)*mu*a for the {spec.foot.radius_m * 1e3:.0f} mm foot",
    }
    pinned = {k: Pin(v, why[k]) for k, v in derived.items()}
    pinned["damping"] = Pin(
        0.03808, "only the suspended regime resolves it; fitted there on the joint channel"
    )
    pinned["foot_friction"] = Pin(
        floor_mu, "a floor property, not a robot one; rubber pads grip both measured surfaces"
    )
    names = tuple(search) if search is not None else DEFAULT_SEARCH
    missing = [n for n in names if n not in backend_knobs]
    if missing:
        raise KeyError(f"backend exposes no knob(s) {missing}")
    searched = {n: backend_knobs[n] for n in names}
    for n in searched:
        pinned.pop(n, None)  # an explicit search wins over a default pin
    return KnobPlan(pinned=pinned, searched=searched)


def base_values(preset: str = DEFAULT_PRESET) -> dict[str, float]:
    """The incumbent's complete knob values, by preset name."""
    return start_values(load_preset(preset), ENGINE_DEFAULTS)


def _plan(
    knobs: Mapping[str, Knob], robot_json: str | None, search: Sequence[str] | None
) -> KnobPlan:
    robot = RobotSpec.from_json(robot_json) if robot_json else None
    return default_plan(knobs, robot, search=search)


def main(argv: Sequence[str] | None = None) -> None:
    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend
    from dimos.robot.unitree.go2.sim.plant import TORQUE_ENVELOPES
    from dimos.robot.unitree.go2.sim.sysid.ingest import GO2_READER

    _main(
        argv,
        reader=GO2_READER,
        backend_cls=MujocoBackend,
        load_preset=load_preset,
        default_preset=DEFAULT_PRESET,
        envelopes=TORQUE_ENVELOPES,
        engine_defaults=ENGINE_DEFAULTS,
        default_plan=_plan,
    )


if __name__ == "__main__":
    main()
