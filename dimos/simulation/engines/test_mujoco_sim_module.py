# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

import numpy as np

from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule


class _FakeData:
    qpos = np.array([0.0, 0.0, 0.75, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    sensordata = np.array([0.1, 0.2, 0.3, 1.0, 2.0, 3.0], dtype=np.float64)


class _FakeEngine:
    data = _FakeData()
    joint_names = ["joint_a", "joint_b"]


def test_ready_signal_happens_after_joint_state_and_imu_write() -> None:
    events: list[str] = []
    module = object.__new__(MujocoSimModule)
    module._shm_ready_signaled = False
    module._root_base_qpos_adr = 0
    module._imu_quat_slice = None
    module._imu_base_qpos_slice = slice(3, 7)
    module._imu_gyro_slice = slice(0, 3)
    module._imu_accel_slice = slice(3, 6)
    module.odom = MagicMock()
    module.imu = MagicMock()

    class _FakeHooks:
        def post_step(self, engine: Any) -> None:
            assert engine is _FakeEngine
            events.append("joint_state")

    class _FakeShm:
        def write_imu(self, **_: Any) -> None:
            events.append("imu")

        def signal_ready(self, *, num_joints: int) -> None:
            assert num_joints == 2
            events.append("ready")

    module._sim_hooks = _FakeHooks()
    module._shm = _FakeShm()

    module._publish_shm_and_lcm(_FakeEngine)

    assert events == ["joint_state", "imu", "ready"]
