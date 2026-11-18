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

"""The Dual Metal's MuJoCo rig: its physics running in real time, with no DimOS transport."""

import numpy as np
import pytest

from dimos.robot.manipulators.dual_makermods_metal.sim_connection import (
    DUAL_METAL_SIM,
    DUAL_METAL_WORKCELL,
)
from dimos.robot.manipulators.dual_openyam.sim_rig import DamiaoSimRig
from dimos.utils.testing.waiting import wait_until


@pytest.mark.self_hosted
def test_both_arms_and_jaws_follow_position_commands() -> None:
    rig = DamiaoSimRig(DUAL_METAL_SIM, DUAL_METAL_WORKCELL, camera_size=None)
    travel = DUAL_METAL_SIM.gripper_travel()

    def positions() -> dict[str, float]:
        return dict(zip(rig.joint_names, rig.read().position, strict=True))

    def fingers_apart(side: str) -> float:
        link7, link8 = (rig.body_position(f"{side}_link{index}") for index in (7, 8))
        return float(np.linalg.norm(link7 - link8))

    rig.connect()
    try:
        wait_until(lambda: rig.sim_time > 0.5, timeout=10.0)
        # Each finger's frame stands 2.75 mm off the middle of the jaw when it is closed.
        assert fingers_apart("left") == pytest.approx(0.0055, abs=1e-3)

        # The base joints carry no weight, so they reach their targets exactly.
        targets = {
            "left_joint1": 0.5,
            "right_joint1": -0.5,
            "left_joint7": travel["left_joint7"][1],
        }
        rig.command(targets)
        wait_until(
            lambda: max(abs(positions()[name] - goal) for name, goal in targets.items()) < 2e-3,
            timeout=10.0,
        )
        # joint8 copies joint7 mirrored: the left fingers part 5 cm each way.
        wait_until(lambda: fingers_apart("left") == pytest.approx(0.0945, abs=1e-3), timeout=5.0)
        assert positions()["right_joint7"] == pytest.approx(travel["right_joint7"][0], abs=1e-3)
        assert fingers_apart("right") == pytest.approx(0.0055, abs=1e-3)
    finally:
        rig.disconnect()
