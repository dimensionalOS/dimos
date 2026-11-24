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


import pytest

pytest.importorskip("pxr")


def test_the_layer_carries_the_gains_the_policy_runs_on() -> None:
    from dimos.robot.unitree.go2.sim2 import GO2_USD
    from dimos.sim2.mujoco.drives import usd_drives
    from dimos.simulation.go2_legged.policy import load_policy

    policy = load_policy(None)
    gains = zip(policy.joint_names, policy.kp, policy.kd, policy.default_pose, strict=True)
    assert usd_drives(GO2_USD.usd) == {
        joint: pytest.approx((kp, kd, q)) for joint, kp, kd, q in gains
    }


def test_every_sensor_mount_the_rig_has_is_a_frame_of_the_layer() -> None:
    from dimos.robot.unitree.go2.sim2 import GO2_USD, Go2Sim

    assert set(Go2Sim.mounts) == {"l1", "lidar", "camera"}
    assert set(Go2Sim.mounts) <= GO2_USD.frame_names()
