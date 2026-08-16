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


def _menagerie_available() -> bool:
    """A missing checkout is an ENVIRONMENT gap, not a failure of this package."""
    from dimos.robot.unitree.go2.sim.model import scene_path

    try:
        scene_path()
    except FileNotFoundError:
        return False
    return True


# NOT `pytest.mark.mujoco`: the repo's addopts deselects that marker wholesale,
# which is how a run once reported 70 passed while the acceptance test was
# failing. These skip on a missing recording or menagerie, with the reason
# printed — a test that vanishes silently is worse than one that fails.
pytestmark = [
    pytest.mark.go2sim,
    pytest.mark.skipif(
        not _menagerie_available(),
        reason="no mujoco_menagerie checkout: set MUJOCO_MENAGERIE",
    ),
]

MIXED = Path.home() / "recordings/rubber_floor/20260816-015155_policy-mixed_vive.mcap"
HANGING = Path.home() / "recordings/hard_floor/20260816-185220_sport-hanging_novive.mcap"

needs_mixed = pytest.mark.skipif(not MIXED.is_file(), reason=f"no recording at {MIXED}")
needs_hanging = pytest.mark.skipif(not HANGING.is_file(), reason=f"no recording at {HANGING}")


@pytest.fixture(scope="module")
def mixed_streams():
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams

    return read_streams(MIXED)


@pytest.fixture(scope="module")
def hanging_streams():
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams

    return read_streams(HANGING)


@needs_mixed
def test_segment_4_reproduces_the_frozen_instruments_numbers(mixed_streams):
    from dimos.robot.unitree.go2.sim.backend import MujocoBackend
    from dimos.robot.unitree.go2.sim.ranges import MEASURED
    from dimos.robot.unitree.go2.sim.sysid.replay import replay

    st = mixed_streams
    _i, mode, a, b = st.segments()[4]
    assert mode == "freewalk"
    t0 = a + 0.2
    r = replay(st, t0, b - t0 - 0.2, MujocoBackend(), preset=MEASURED, window=0.4)

    je = r.joint_err()
    dp, da = r.body_err()
    # The three headline numbers, as the frozen tool printed them ...
    assert f"{je.mean():.4f}" == "0.0310"
    assert f"{dp.mean():.4f}" == "0.0246"
    assert f"{np.degrees(da.mean()):.2f}" == "5.19"
    # ... and to full precision, because both paths are deterministic.
    assert je.mean() == pytest.approx(0.03095355487877684, rel=1e-9)
    assert dp.mean() == pytest.approx(0.024648439602975667, rel=1e-9)
    assert np.degrees(da.mean()) == pytest.approx(5.188627404101316, rel=1e-9)
    assert len(r.prediction.reinit_t) - 1 == 36


@needs_mixed
def test_the_tracker_is_detected_and_scores_the_body_channels(mixed_streams):
    assert mixed_streams.has_markers


@needs_hanging
def test_a_suspended_recording_replays_with_the_trunk_pinned(hanging_streams):
    """No tracker, sport-driven, hanging 70-85 deg off level: loads, replays,
    and the trunk stays pinned to the measured pose, clear of any floor."""
    from dimos.robot.unitree.go2.sim.backend import MujocoBackend
    from dimos.robot.unitree.go2.sim.ranges import MEASURED
    from dimos.robot.unitree.go2.sim.sysid.replay import replay

    st = hanging_streams
    r = replay(st, 10.0, 20.0, MujocoBackend(), preset=MEASURED, window=0.4, suspended=True)
    # frozen reference: joint mean 0.05685969199042827, max 2.4079772840531506
    je = r.joint_err()
    assert je.mean() == pytest.approx(0.05685969199042827, rel=1e-9)
    assert je.max() == pytest.approx(2.4079772840531506, rel=1e-9)
    assert len(r.prediction.reinit_t) - 1 == 49
    # no tracker: the body channels are simply absent, not a special mode
    assert r.p_real is None and r.r_real is None
    # the trunk never met a floor and never moved between snaps
    assert np.all(r.prediction.body_pos[:, 2] >= 2.0)


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
    from dimos.robot.unitree.go2.sim.sysid.ingest import Declarations
    from dimos.robot.unitree.go2.sim.sysid.regimes import propose_suspended, regimes

    assert propose_suspended(hanging_streams)  # unloaded legs: tau p50 ~0.45
    with pytest.raises(ValueError, match="declares"):
        regimes(hanging_streams)
    spans = regimes(hanging_streams, Declarations(suspended=True))
    assert all(s.kind in ("suspended", "contaminated") for s in spans)
    assert any(s.kind == "suspended" for s in spans)
