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

"""Net identity is checked, never assumed — and the check discriminates."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.sysid.ground import COMMAND_SLEW
from dimos.robot.unitree.go2.sim.sysid.test_real import _streams
from dimos.robot.unitree.go2.sim.sysid.verify_net import slewed_commands

FREEWALK = Path.home() / "recordings/hard_floor/20260816-194142_policy-freewalk-hard_vive.mcap"
DATA = Path(__file__).parents[6] / "data/ml-trajectory-research"


def test_slew_ramps_the_operator_target_at_the_executor_rate():
    """The policy never saw the step command the control_log carries.

    The slew state starts CONVERGED on the first operator sample (the frozen
    convention: mid-run steady state), so the ramp shows on the second one.
    """
    st = _streams(np.array([0.0, 0.1]), np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))
    got = slewed_commands(st, np.array([0.0, 0.12, 0.3, 10.0]))
    assert got[0, 0] == pytest.approx(0.0)  # first target, converged
    # per 20 ms tick the ramp moves at most COMMAND_SLEW[0] = 0.05 in vx
    assert 0.0 < got[1, 0] <= 2 * COMMAND_SLEW[0] + 1e-9
    assert got[1, 0] < got[2, 0] < 1.0  # still ramping at 0.3 s
    assert got[3, 0] == pytest.approx(1.0)  # converged


@pytest.mark.go2sim
@pytest.mark.skipif(not FREEWALK.is_file(), reason="needs the freewalk recording")
@pytest.mark.skipif(not (DATA / "freewalk_mcf.bin").is_file(), reason="needs the policy blobs")
def test_the_producing_net_explains_the_recording_and_a_stranger_does_not():
    """THE prerequisite of loop 2: grounding against the wrong net produces
    confident, meaningless numbers. Verified, not assumed."""
    from dimos.robot.unitree.go2.sim.sysid.verify_net import verify

    check, control = verify(
        FREEWALK,
        DATA / "freewalk_mcf.bin",
        DATA / "v11_final.bin" if (DATA / "v11_final.bin").is_file() else None,
    )
    assert (check.obs_per_frame, check.hist) == (45, 6)
    assert check.kp_match and check.kd_match
    assert check.ratio < 0.25  # residual well under the signal's own spread
    if control is not None:
        assert check.residual_rms < 0.5 * control.residual_rms
