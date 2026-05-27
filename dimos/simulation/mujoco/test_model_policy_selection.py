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

from dimos.core.global_config import GlobalConfig
from dimos.simulation.mujoco import model as model_module


class _InputDevice:
    def get_command(self) -> np.ndarray[Any, Any]:
        return np.zeros(3, dtype=np.float32)


class _PolicySpy:
    instances: list[dict[str, Any]] = []

    def __init__(self, **kwargs: Any) -> None:
        self.instances.append(kwargs)

    def get_control(self, *_args: Any, **_kwargs: Any) -> None:
        pass


class _GrootPolicySpy(_PolicySpy):
    instances: list[dict[str, Any]] = []


class _DefaultPolicySpy(_PolicySpy):
    instances: list[dict[str, Any]] = []


class _FakeKeyframe:
    qpos = np.array([0.0] * 7 + [0.1, 0.2], dtype=np.float32)


class _FakeModel:
    opt = SimpleNamespace(timestep=0.0)

    def keyframe(self, _name: str) -> _FakeKeyframe:
        return _FakeKeyframe()


class _FakeMjModel:
    @staticmethod
    def from_xml_string(_xml: str, assets: dict[str, bytes]) -> _FakeModel:
        assert assets == {}
        return _FakeModel()


class _FakeMjData:
    def __init__(self, model: _FakeModel) -> None:
        self.model = model


@pytest.fixture
def fake_mujoco_model(monkeypatch: pytest.MonkeyPatch) -> None:
    _GrootPolicySpy.instances.clear()
    _DefaultPolicySpy.instances.clear()

    fake_mujoco = SimpleNamespace(
        MjModel=_FakeMjModel,
        MjData=_FakeMjData,
        mj_resetDataKeyframe=lambda *_args: None,
        set_mjcb_control=lambda *_args: None,
    )

    monkeypatch.setattr(model_module, "mujoco", fake_mujoco)
    monkeypatch.setattr(model_module, "get_assets", lambda: {})
    monkeypatch.setattr(model_module, "get_model_xml", lambda *_args: "<mujoco/>")
    monkeypatch.setattr(model_module, "G1GrootOnnxController", _GrootPolicySpy)
    monkeypatch.setattr(model_module, "G1OnnxController", _DefaultPolicySpy)


def test_unitree_g1_uses_default_mujoco_policy_by_default(fake_mujoco_model: None) -> None:
    model_module.load_model(
        _InputDevice(),
        robot="unitree_g1",
        scene_xml="<mujoco/>",
        config=GlobalConfig(),
    )

    assert len(_DefaultPolicySpy.instances) == 1
    assert _GrootPolicySpy.instances == []


def test_unitree_g1_groot_policy_is_opt_in(fake_mujoco_model: None) -> None:
    model_module.load_model(
        _InputDevice(),
        robot="unitree_g1",
        scene_xml="<mujoco/>",
        config=GlobalConfig(mujoco_g1_policy="groot", mujoco_groot_policy_dir="data/groot"),
    )

    assert len(_GrootPolicySpy.instances) == 1
    assert _GrootPolicySpy.instances[0]["policy_dir"] == "data/groot"
    assert _DefaultPolicySpy.instances == []
