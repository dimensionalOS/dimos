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

import pytest

from dimos.hardware.manipulators.openarm_rs.adapter import OpenArmRSAdapter, register
from dimos.hardware.manipulators.registry import AdapterRegistry
from dimos.hardware.manipulators.spec import ManipulatorAdapter


def test_implements_manipulator_adapter() -> None:
    assert isinstance(OpenArmRSAdapter(use_mock_bus=True), ManipulatorAdapter)


def test_register_preserves_openarm_rs_key() -> None:
    registry = AdapterRegistry()

    register(registry)

    adapter = registry.create("openarm_rs", use_mock_bus=True)
    assert isinstance(adapter, OpenArmRSAdapter)


def test_side_selects_openarm_joint_limits() -> None:
    left = OpenArmRSAdapter(side="left", use_mock_bus=True)
    right = OpenArmRSAdapter(side="right", use_mock_bus=True)

    assert left.get_limits().position_lower[:2] == [-3.45, -3.30]
    assert right.get_limits().position_lower[:2] == [-1.35, -0.15]


def test_invalid_side_is_rejected() -> None:
    with pytest.raises(ValueError, match="side must be 'left' or 'right'"):
        _ = OpenArmRSAdapter(side="middle", use_mock_bus=True)


def test_non_openarm_dof_is_rejected() -> None:
    with pytest.raises(ValueError, match="only supports 7 DOF"):
        _ = OpenArmRSAdapter(dof=2, use_mock_bus=True)


def test_custom_non_openarm_metadata_is_rejected() -> None:
    with pytest.raises(ValueError, match="does not accept custom motor_specs"):
        _ = OpenArmRSAdapter(
            use_mock_bus=True,
            motor_specs=[
                {"name": "shoulder", "type": "DM4310", "send_id": 1, "recv_id": 17},
            ],
        )

    with pytest.raises(ValueError, match="fixed OpenArm limits"):
        _ = OpenArmRSAdapter(
            use_mock_bus=True,
            position_lower=[-0.5] * 7,
        )


def test_openarm_rs_reports_openarm_specific_identity() -> None:
    adapter = OpenArmRSAdapter(
        use_mock_bus=True,
        kp=[3.0] * 7,
        kd=[0.3] * 7,
    )

    assert adapter.get_info().vendor == "Enactic"
    assert adapter.get_info().model == "OpenArm RS v10"
    assert adapter.get_dof() == 7
