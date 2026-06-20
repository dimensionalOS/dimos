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

import importlib

import pytest

from dimos.hardware.manipulators.openarm_rs.adapter import (
    OpenArmRSAdapter,
    OpenArmRSBindingUnavailableError,
)
import dimos.hardware.whole_body.openarm.adapter as openarm_dual_adapter
from dimos.hardware.whole_body.openarm.adapter import (
    OpenArmDualBindingUnavailableError,
    OpenArmDualWholeBodyAdapter,
    register,
)
from dimos.hardware.whole_body.registry import WholeBodyAdapterRegistry
from dimos.hardware.whole_body.spec import WholeBodyAdapter


def test_openarm_dual_constructs_without_binding() -> None:
    adapter = OpenArmDualWholeBodyAdapter(use_mock_bus=True)

    assert isinstance(adapter, WholeBodyAdapter)
    assert adapter.joint_names == (
        "openarm_left_joint1",
        "openarm_left_joint2",
        "openarm_left_joint3",
        "openarm_left_joint4",
        "openarm_left_joint5",
        "openarm_left_joint6",
        "openarm_left_joint7",
        "openarm_right_joint1",
        "openarm_right_joint2",
        "openarm_right_joint3",
        "openarm_right_joint4",
        "openarm_right_joint5",
        "openarm_right_joint6",
        "openarm_right_joint7",
    )


def test_openarm_dual_configures_side_gravity_models() -> None:
    adapter = OpenArmDualWholeBodyAdapter(use_mock_bus=True)

    left = adapter._robot_spec.groups["left_arm"]
    right = adapter._robot_spec.groups["right_arm"]
    assert str(left.gravity_model_path).endswith("openarm_v10_left.urdf")
    assert str(right.gravity_model_path).endswith("openarm_v10_right.urdf")


def test_openarm_dual_rejects_non_14_dof() -> None:
    with pytest.raises(ValueError, match="supports 14 DOF"):
        _ = OpenArmDualWholeBodyAdapter(dof=7, use_mock_bus=True)


def test_register_preserves_openarm_dual_key() -> None:
    registry = WholeBodyAdapterRegistry()

    register(registry)

    assert registry.available() == ["openarm_dual"]
    assert isinstance(
        registry.create("openarm_dual", use_mock_bus=True), OpenArmDualWholeBodyAdapter
    )


def test_openarm_dual_module_import_does_not_probe_binding(mocker) -> None:
    mocker.patch(
        "dimos.hardware.damiao.runtime.importlib.import_module",
        side_effect=AssertionError("binding import must stay lazy"),
    )

    reloaded = importlib.reload(openarm_dual_adapter)
    registry = WholeBodyAdapterRegistry()
    reloaded.register(registry)

    assert registry.available() == ["openarm_dual"]


def test_openarm_rs_preserves_constructor_deployment_options() -> None:
    adapter = OpenArmRSAdapter(
        address="can9",
        config_path="robot.toml",
        arm_name="right_arm",
        bus_name="right_can",
        fd=False,
        canfd=True,
        side="right",
        use_mock_bus=True,
        gravity_model_path="override.urdf",
    )

    assert adapter._group_name == "right_arm"
    assert adapter._config_path == "robot.toml"
    assert adapter._gravity_model_path == "override.urdf"
    assert adapter._robot_spec.buses["right_can"].address == "can9"
    assert adapter._robot_spec.buses["right_can"].fd is False
    assert adapter.get_limits().position_lower[0] == -1.35


def test_openarm_dual_connect_raises_specific_binding_error(mocker) -> None:
    adapter = OpenArmDualWholeBodyAdapter(use_mock_bus=True)
    mocker.patch(
        "dimos.hardware.damiao.runtime.importlib.import_module",
        side_effect=ImportError("missing binding"),
    )

    with pytest.raises(OpenArmDualBindingUnavailableError):
        adapter.connect()


def test_openarm_rs_connect_raises_specific_binding_error(mocker) -> None:
    adapter = OpenArmRSAdapter(use_mock_bus=True)
    mocker.patch(
        "dimos.hardware.damiao.runtime.importlib.import_module",
        side_effect=ImportError("missing binding"),
    )

    with pytest.raises(OpenArmRSBindingUnavailableError):
        adapter.connect()
