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

"""Loop 1 on the G1: what is pinned, what is searched, and the fit CLI.

The method is :mod:`dimos.simulation.sysid.fit`. Nothing on the G1 has been
weighed, so every pin here is a declaration at the stock value, said so in
its ``why``; the search is the set walking can resolve.

    python -m dimos.robot.unitree.g1.sim.sysid.fit REC.db --preset stock --trials 30 --out g1/sim/presets/measured
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

from dimos.robot.unitree.g1.sim.ranges import DEFAULT_PRESET, ENGINE_DEFAULTS, load_preset
from dimos.simulation.sysid.fit import KnobPlan, Pin, main as _main, start_values
from dimos.simulation.sysid.presets import Knob

DEFAULT_SEARCH: tuple[str, ...] = (
    "armature",
    "frictionloss",
    "actuator_tau",
    "foot_solref_time",
    "foot_solref_damp",
)

# Declared, not measured: the stock value with the reason it stays there.
DEFAULT_PINS: dict[str, Pin] = {
    "damping": Pin(0.001, "unresolved without a hanging recording; the fork agrees it is small"),
    "trunk_mass_scale": Pin(1.0, "declared: not weighed (model mass 35.112 kg)"),
    "trunk_inertia_scale": Pin(1.0, "declared: not weighed"),
    "trunk_com_x": Pin(0.0, "declared: not weighed"),
    "leg_mass_scale": Pin(1.0, "declared: CAD masses from Unitree's URDF"),
    "foot_friction": Pin(1.0, "a floor property; the recording's floor is undeclared"),
    "foot_friction_torsional": Pin(
        0.005, "the patch formula is meaningless for a four-sphere foot"
    ),
    "foot_solimp_dmin": Pin(0.9, "contact shape; not resolved by walking, stock stands"),
    "foot_solimp_width": Pin(0.001, "contact shape; not resolved by walking, stock stands"),
}


def default_plan(
    backend_knobs: Mapping[str, Knob], robot_json: str | None, search: Sequence[str] | None
) -> KnobPlan:
    if robot_json is not None:
        raise ValueError("the G1 has no robot.json anchors yet; weigh it first")
    names = tuple(search) if search is not None else DEFAULT_SEARCH
    missing = [n for n in names if n not in backend_knobs]
    if missing:
        raise KeyError(f"backend exposes no knob(s) {missing}")
    pinned = dict(DEFAULT_PINS)
    searched = {n: backend_knobs[n] for n in names}
    for n in searched:
        pinned.pop(n, None)  # an explicit search wins over a default pin
    return KnobPlan(pinned=pinned, searched=searched)


def base_values(preset: str = DEFAULT_PRESET) -> dict[str, float]:
    """The incumbent's complete knob values, by preset name."""
    return start_values(load_preset(preset), ENGINE_DEFAULTS)


def main(argv: Sequence[str] | None = None) -> None:
    from dimos.robot.unitree.g1.sim.engines.mujoco import MujocoBackend
    from dimos.robot.unitree.g1.sim.sysid.ingest import G1_READER

    _main(
        argv,
        reader=G1_READER,
        backend_cls=MujocoBackend,
        load_preset=load_preset,
        default_preset=DEFAULT_PRESET,
        envelopes={},
        engine_defaults=ENGINE_DEFAULTS,
        default_plan=default_plan,
    )


if __name__ == "__main__":
    main()
