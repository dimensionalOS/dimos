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

"""The OpenYAM's MuJoCo rig: its physics running in real time, with no DimOS transport."""

import numpy as np
import pytest

from dimos.robot.manipulators.dual_openyam.sim_rig import DamiaoSimRig
from dimos.robot.manipulators.openyam.config import OPENYAM, OPENYAM_HOME_JOINTS
from dimos.robot.manipulators.openyam.sim_connection import OPENYAM_WORKCELL
from dimos.utils.testing.waiting import wait_until

# A posture away from home, within every joint's range.
REACHING = [0.4, 0.8, 1.3, 0.3, -0.3, 0.5]


@pytest.mark.self_hosted
def test_arm_and_jaw_follow_position_commands() -> None:
    rig = DamiaoSimRig(OPENYAM, OPENYAM_WORKCELL, camera_size=None)
    closed, opened = OPENYAM.gripper_travel()["joint7"]

    def positions() -> dict[str, float]:
        return dict(zip(rig.joint_names, rig.read().position, strict=True))

    def finger_frames_apart() -> float:
        # Each finger's frame lies on the other finger's side of the jaw, so
        # the frames close in as the fingertips open: 85 mm closed, 12 mm open.
        right, left = (rig.body_position(f"hand_{side}_finger") for side in ("right", "left"))
        return float(np.linalg.norm(right - left))

    rig.connect()
    try:
        wait_until(lambda: rig.sim_time > 0.5, timeout=10.0)
        start = positions()
        assert [start[name] for name in OPENYAM.arm_joint_names] == pytest.approx(
            OPENYAM_HOME_JOINTS, abs=1e-3
        )
        assert start["joint7"] == pytest.approx(closed, abs=1e-3)
        assert finger_frames_apart() == pytest.approx(0.085, abs=2e-3)

        targets = {**dict(zip(OPENYAM.arm_joint_names, REACHING, strict=True)), "joint7": opened}
        rig.command(targets)

        wait_until(
            lambda: max(abs(positions()[name] - goal) for name, goal in targets.items()) < 2e-3,
            timeout=10.0,
        )
        assert finger_frames_apart() == pytest.approx(0.012, abs=2e-3)
    finally:
        rig.disconnect()
