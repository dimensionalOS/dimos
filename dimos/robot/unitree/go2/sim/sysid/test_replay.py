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

"""ACCEPTANCE: the port reproduces the frozen instrument's numbers exactly.

The reference values were produced by the frozen
``dimos/navigation/motion/simulation`` package on ``feat/ivan/go2_terrain``
(segment 4, ``measured`` preset, fixed 0.4 s clips) and both code paths are
deterministic, so the comparison is to full float precision — not
approximately. A drift here means the port changed the physics.

Needs the recordings under ``~/recordings`` and a menagerie checkout, so these
run locally (``-m mujoco``), not in CI.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")
pytest.importorskip("mcap")


# NOT `pytest.mark.mujoco`: the repo's addopts deselects that marker wholesale,
# which is how a run once reported 70 passed while the acceptance test was
# failing. These skip on a missing RECORDING only (recordings are not
# vendored); the go2 assets are (data/go2_menagerie), so a missing scene is a
# real failure, never a skip.
pytestmark = [pytest.mark.go2sim]

MIXED = Path.home() / "recordings/rubber_floor/20260816-015155_policy-mixed_vive.mcap"
HANGING = Path.home() / "recordings/hard_floor/20260816-185220_sport-hanging_novive.mcap"

needs_mixed = pytest.mark.skipif(not MIXED.is_file(), reason=f"no recording at {MIXED}")
needs_hanging = pytest.mark.skipif(not HANGING.is_file(), reason=f"no recording at {HANGING}")


def acceptance_plant():
    """The FROZEN plant the acceptance numbers were measured under.

    Deliberately not ``ranges.MEASURED``: these tests pin the replay
    PIPELINE bit-for-bit, and the shipped plant is allowed to improve
    (draw selection re-ships it) without this file conflating "the plant
    changed" with "the pipeline regressed". These are the 2026-08-17
    shipped values, frozen here the day draw054 replaced them; update
    them only when re-basing the pipeline acceptance itself.
    """
    from dimos.robot.unitree.go2.sim.ranges import Preset

    return Preset(
        name="acceptance-frozen-20260817",
        physics={
            "armature": 0.02899,
            "damping": 0.03808,
            "frictionloss": 1.46585,
            "leg_mass_scale": 1.0,
            "foot_friction": 0.9,
            "trunk_mass_scale": 1.1869082502528536,
            "foot_friction_torsional": 0.005831581363226053,
            "trunk_com_x": -0.012040057929109567,
            "trunk_inertia_scale": 1.1180010114682968,
        },
        actuator_tau=0.00525,
        envelope="central",
    )


@pytest.fixture(scope="module")
def mixed_streams():
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams

    return read_streams(MIXED)


@pytest.fixture(scope="module")
def hanging_streams():
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams

    return read_streams(HANGING)


@needs_mixed
def test_segment_4_reproduces_the_shipped_plants_numbers(mixed_streams):
    """Acceptance for the SHIPPED plant, envelope honoured (a preset that
    carries one must never silently run bare). Re-based 2026-08-17 with the
    preset consolidation, decomposed and predicted before measurement: from
    the frozen instrument's 0.03095355487877684 / 0.024648439602975667 /
    5.188627404101316, the friction correction (0.635->0.90 derived pair)
    moved +0.08% / +0.23% / +0.75% — friction is closed-loop-inert and
    open-loop-nearly-inert, as README 9 measured — and the envelope moved
    joint -3.96%, the direction 5d's drive measurement predicts (the bare
    sim OVER-drives tau 1.1-1.35x; derating toward the measured drive
    tightens the joint residual). Same bit-identical discipline forward."""
    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend
    from dimos.robot.unitree.go2.sim.plant import TORQUE_ENVELOPES
    from dimos.robot.unitree.go2.sim.sysid.replay import replay

    st = mixed_streams
    _i, mode, a, b = st.segments()[4]
    assert mode == "freewalk"
    t0 = a + 0.2
    plant = acceptance_plant()
    backend = MujocoBackend(envelope=TORQUE_ENVELOPES[plant.envelope])
    r = replay(st, t0, b - t0 - 0.2, backend, preset=plant, window=0.4)

    je = r.joint_err()
    dp, da = r.body_err()
    # The three headline numbers ...
    assert f"{je.mean():.4f}" == "0.0298"
    assert f"{dp.mean():.4f}" == "0.0248"
    assert f"{np.degrees(da.mean()):.2f}" == "5.30"
    # ... and to full precision, because both paths are deterministic.
    assert je.mean() == pytest.approx(0.029752463648512955, rel=1e-9)
    assert dp.mean() == pytest.approx(0.02480154961255685, rel=1e-9)
    assert np.degrees(da.mean()) == pytest.approx(5.302594638346394, rel=1e-9)
    assert len(r.prediction.reinit_t) - 1 == 36


@needs_mixed
def test_the_tracker_is_detected_and_scores_the_body_channels(mixed_streams):
    assert mixed_streams.has_markers


@needs_hanging
def test_a_suspended_recording_replays_with_the_trunk_pinned(hanging_streams):
    """No tracker, sport-driven, hanging 70-85 deg off level: loads, replays,
    and the trunk stays pinned to the measured pose, clear of any floor."""
    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend
    from dimos.robot.unitree.go2.sim.plant import TORQUE_ENVELOPES
    from dimos.robot.unitree.go2.sim.sysid.replay import replay

    st = hanging_streams
    plant = acceptance_plant()
    backend = MujocoBackend(envelope=TORQUE_ENVELOPES[plant.envelope])
    r = replay(st, 10.0, 20.0, backend, preset=plant, window=0.4, suspended=True)
    # Reference for the WELD mechanism (trunk held DURING the step, attitude
    # tracking the measurement). The frozen instrument's numbers (joint mean
    # 0.05685969199042827, max 2.4079772840531506) were measured in a
    # WEIGHTLESS plant — the old post-step snap let the whole robot free-fall
    # through mj_step, so gravity cancelled out of the leg dynamics — and
    # their movement here is the bug fix, not a regression.
    # Re-based 2026-08-17 (consolidation): from 0.05482657148294461 /
    # 2.2805870870306104. The friction part is +0.06% — feet DO meet other
    # leg geoms while hanging (measured: 194 of 1369 contacts in a pinned
    # flail involve a foot) — and the rest is the envelope derating the
    # 20 rad/s strokes this sport recording is made of (+19% mean, +9% max).
    je = r.joint_err()
    assert je.mean() == pytest.approx(0.06526516273674912, rel=1e-9)
    assert je.max() == pytest.approx(2.4907904937643277, rel=1e-9)
    assert len(r.prediction.reinit_t) - 1 == 49
    # no tracker: the body channels are simply absent, not a special mode
    assert r.p_real is None and r.r_real is None
    # the trunk never met a floor and, within the weld's hold error, never
    # left the snapped position between snaps
    assert np.all(r.prediction.body_pos[:, 2] >= 1.999)


@needs_hanging
def test_the_builtin_controller_commands_nonzero_dq_des(hanging_streams):
    """dq_des is part of the PD law: Unitree's built-ins command up to
    20.7 rad/s. A port that drops cdq silently halves the damping term."""
    assert np.abs(hanging_streams.cdq).max() > 15.0


@needs_hanging
def test_the_sport_recording_falls_back_to_rt_lowcmd_by_coverage(hanging_streams):
    # rt/lowcmd runs at 500 Hz; a policy/lowcmd stub could never reach this
    assert len(hanging_streams.ct) > 15000


@needs_hanging
def test_the_hanging_file_needs_a_declaration_and_honours_one(hanging_streams):
    from dimos.simulation.sysid.recording import Declarations
    from dimos.simulation.sysid.regimes import propose_suspended, regimes

    assert propose_suspended(hanging_streams)  # unloaded legs: tau p50 ~0.45
    with pytest.raises(ValueError, match="declares"):
        regimes(hanging_streams)
    spans = regimes(hanging_streams, Declarations(suspended=True))
    assert all(s.kind in ("suspended", "contaminated") for s in spans)
    assert any(s.kind == "suspended" for s in spans)
