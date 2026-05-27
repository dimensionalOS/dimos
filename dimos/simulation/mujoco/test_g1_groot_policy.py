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

from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

from dimos.simulation.mujoco.policy import G1_GROOT_KPS, G1GrootOnnxController


class _InputDevice:
    def get_command(self) -> np.ndarray[Any, Any]:
        return np.zeros(3, dtype=np.float32)


class _FakeSensor:
    def __init__(self, data: list[float]) -> None:
        self.data = np.array(data, dtype=np.float32)


class _FakeData:
    def __init__(self, n_joints: int = 29, n_actuators: int | None = None) -> None:
        self.qpos = np.zeros(7 + n_joints, dtype=np.float32)
        self.qpos[3] = 1.0
        self.qvel = np.zeros(6 + n_joints, dtype=np.float32)
        self.ctrl = np.zeros(n_actuators if n_actuators is not None else n_joints, dtype=np.float32)
        self._sensors = {
            "local_linvel_pelvis": _FakeSensor([0.1, 0.2, 0.3]),
        }

    def sensor(self, name: str) -> _FakeSensor:
        return self._sensors[name]


def _make_controller() -> G1GrootOnnxController:
    controller = G1GrootOnnxController.__new__(G1GrootOnnxController)
    controller._default_angles = np.zeros(29, dtype=np.float32)
    controller._target_angles = np.ones(29, dtype=np.float32)
    controller._last_action = np.zeros(15, dtype=np.float32)
    controller._counter = 0
    controller._n_substeps = 999
    controller._input_controller = _InputDevice()
    controller._torque_actuators_configured = True
    return controller


def test_g1_groot_obs_includes_body_frame_linear_velocity() -> None:
    controller = _make_controller()

    obs, _command = controller._get_obs(_FakeData())

    np.testing.assert_allclose(obs[4:7], [0.1, 0.2, 0.3])


def test_g1_groot_control_uses_actuator_count_explicitly() -> None:
    controller = _make_controller()
    model = SimpleNamespace(nu=10)
    data = _FakeData(n_actuators=model.nu)

    controller.get_control(model, data)  # type: ignore[arg-type]

    np.testing.assert_allclose(data.ctrl, G1_GROOT_KPS[: model.nu])


def test_g1_groot_control_rejects_more_actuators_than_targets() -> None:
    controller = _make_controller()
    model = SimpleNamespace(nu=30)
    data = _FakeData(n_actuators=model.nu)

    with pytest.raises(ValueError, match="target angle count"):
        controller.get_control(model, data)  # type: ignore[arg-type]
