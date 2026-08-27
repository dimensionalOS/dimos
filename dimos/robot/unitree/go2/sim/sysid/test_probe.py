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

"""The loop-2 probe: spectrum construction and its verdict table."""

from __future__ import annotations

from dimos.robot.unitree.go2.sim.ranges import KNOBS
from dimos.robot.unitree.go2.sim.sysid.probe import (
    FOCUS,
    Probe,
    Spectrum,
    default_probes,
)
from dimos.robot.unitree.go2.sim.sysid.stats import Summary


def _summary(**over: float) -> Summary:
    base = dict.fromkeys(Summary.__dataclass_fields__, 0.0)
    base.update(speed=0.5, roll_std=0.02, pitch_std=0.02, tilt_p99=0.13, height_std=0.005)
    base.update(gait_hz=1.3)
    base.update(over)
    return Summary(**base)  # type: ignore[arg-type]


def test_default_probes_cover_both_ends_of_every_knob_plus_the_mechanisms():
    probes = default_probes({"armature": 0.02}, 0.005)
    names = [p.name for p in probes]
    # every knob appears at both range ends
    for knob, k in KNOBS.items():
        assert f"{knob}={k.lo:g} (lo)" in names and f"{knob}={k.hi:g} (hi)" in names
    # actuator_tau probes move the LAG, not a physics key
    taus = [p for p in probes if p.name.startswith("actuator_tau")]
    assert {p.actuator_tau for p in taus} == {KNOBS["actuator_tau"].lo, KNOBS["actuator_tau"].hi}
    assert all("actuator_tau" not in p.physics for p in taus)
    # a knob probe changes exactly that knob and keeps the base elsewhere
    lo = next(p for p in probes if p.name == f"armature={KNOBS['armature'].lo:g} (lo)")
    assert lo.physics["armature"] == KNOBS["armature"].lo and lo.actuator_tau == 0.005
    # the absent mechanisms are probed too
    assert any(p.action_latency > 0 for p in probes)
    assert any(p.noise_scale > 0 for p in probes)
    assert any(p.envelope for p in probes)


def test_spectrum_table_flags_floor_and_names_unmovable_statistics():
    base = _summary()
    real = _summary(roll_std=0.044, pitch_std=0.031, tilt_p99=0.25, speed=0.57, gait_hz=1.67)
    floor = dict.fromkeys(Summary.__dataclass_fields__, 0.01)
    moved = _summary(roll_std=0.04)  # +0.02 exceeds the floor, toward real
    still = _summary(roll_std=0.021)  # +0.001 is chaos
    spec = Spectrum(
        base=base,
        real=real,
        floor=floor,
        results=[(Probe("mover"), moved), (Probe("noop"), still)],
        start=6.0,
        seconds=10.0,
        preset="measured",
    )
    out = spec.table()
    assert "mover" in out and "noop" in out
    # the mover's roll_std cell is starred, the noop's is dotted
    mover_row = next(line for line in out.splitlines() if line.startswith("mover"))
    noop_row = next(line for line in out.splitlines() if line.startswith("noop"))
    assert "*" in mover_row.split()[1] and "." in noop_row.split()[1]
    # roll_std has a best mover; pitch_std has none past the floor
    assert "mover" in next(
        line for line in out.splitlines() if "roll_std  " in line and "closes" in line
    )
    assert any("pitch_std" in line and "NOTHING moves it" in line for line in out.splitlines())
    assert all(k in out for k in FOCUS)
