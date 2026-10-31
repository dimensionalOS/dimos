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

from collections.abc import Iterator
import runpy

import can_motor_control
from can_motor_control import damiao
import pytest
from pytest_mock import MockerFixture

from dimos.hardware.whole_body.damiao.adapter import DamiaoWholeBodyAdapter
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.makermods_metal import adapter as adapter_module
from dimos.hardware.whole_body.makermods_metal.adapter import MakerModsMetalAdapter
from dimos.hardware.whole_body.registry import whole_body_adapter_registry
from dimos.hardware.whole_body.spec import MotorState
from dimos.robot.manipulators.makermods_metal.config import METAL_JOINTS


@pytest.fixture
def adapter(mocker: MockerFixture) -> Iterator[MakerModsMetalAdapter]:
    mocker.patch.object(
        MakerModsMetalAdapter,
        "_make_can_bus",
        side_effect=lambda _name: can_motor_control.MockCanBus("metal"),
    )
    adapter = MakerModsMetalAdapter(
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
        gripper_current=0.1,
    )
    yield adapter
    adapter.disconnect()


def test_metal_reuses_shared_driver_and_lifecycle(adapter) -> None:
    assert isinstance(adapter, DamiaoWholeBodyAdapter), (
        "Damiao arms must reuse DamiaoWholeBodyAdapter and can-motor-control; "
        "extend shared profiles/transports rather than introduce a vendor SDK driver"
    )
    for method in (
        "connect",
        "activate",
        "deactivate",
        "read_motor_states",
        "write_motor_commands",
    ):
        assert getattr(MakerModsMetalAdapter, method) is getattr(DamiaoWholeBodyAdapter, method)


def test_topology_uses_metal_firmware_profiles(adapter, mocker) -> None:
    profile = mocker.patch.object(damiao, "MotorProfile", create=True)
    profile.side_effect = ["low", "shoulder", "high"]
    codec = mocker.patch.object(damiao, "DamiaoCodec")
    specs = mocker.patch.object(can_motor_control, "MotorSpec")
    builder = mocker.MagicMock()
    builder.add_bus.return_value = builder
    builder.add_arm.return_value = builder
    builder.add_gripper.return_value = builder
    mocker.patch.object(can_motor_control.Robot, "builder", return_value=builder)
    adapter._build_robot()
    assert [call.args[1:] for call in specs.call_args_list] == [
        ("low", 0x01, 0x11),
        ("shoulder", 0x02, 0x12),
        ("low", 0x03, 0x13),
        ("low", 0x04, 0x14),
        ("high", 0x05, 0x15),
        ("high", 0x06, 0x16),
        ("high", 0x07, 0x17),
    ]
    assert [(call.args, call.kwargs) for call in profile.call_args_list] == [
        ((0x8000,), {"p_max": 6.28, "v_max": 10.0, "t_max": 30.0}),
        ((0x8001,), {"p_max": 6.28, "v_max": 10.0, "t_max": 120.0}),
        ((0x8002,), {"p_max": 6.28, "v_max": 30.0, "t_max": 20.0}),
    ]
    codec.assert_called_once_with(profiles=["low", "shoulder", "high"])
    assert builder.add_arm.call_args.kwargs["bus"] == "metal"
    assert builder.add_gripper.call_args.kwargs["opening_direction"] == "increasing_position"
    assert builder.add_gripper.call_args.kwargs["default_current"] == 0.1
    assert list(adapter.joint_names) == list(METAL_JOINTS)


def test_missing_profiles_fail_before_opening_usb(adapter, mocker) -> None:
    mocker.patch.object(damiao, "MotorProfile", None, create=True)
    bus = mocker.spy(adapter, "_make_can_bus")
    with pytest.raises(RuntimeError, match="requires can-motor-control>=0.0.11"):
        adapter._build_robot()
    bus.assert_not_called()


def test_import_does_not_download_model(mocker) -> None:
    checkout = mocker.patch("dimos.robot.assets.source.RobotDescriptionSource.checkout_path")
    runpy.run_path(adapter_module.__file__)
    checkout.assert_not_called()


@pytest.mark.parametrize("current", [0.0, -0.1, 1.1, float("nan"), float("inf")])
def test_invalid_gripper_current_is_rejected(current) -> None:
    with pytest.raises(ValueError, match="current"):
        MakerModsMetalAdapter(gripper_current=current)


def test_measured_metal_zero_feedback_is_normalized(adapter) -> None:
    adapter._arm_position_limits = ((-3.14, 3.14), (-3.14, 0.0), (0.0, 3.14))
    states = [MotorState(q=q) for q in (-0.00488716, 0.00316228, -0.05050065)]

    normalized = adapter._normalize_arm_feedback(states)

    assert [state.q for state in normalized] == [-0.00488716, 0.0, 0.0]
    assert adapter._feedback_fault is None


def test_metal_gross_feedback_violation_still_latches(adapter) -> None:
    adapter._arm_position_limits = ((0.0, 3.14),)

    with pytest.raises(RuntimeError, match="feedback fault"):
        adapter._normalize_arm_feedback([MotorState(q=-0.061)])

    assert not adapter.activate()


def test_registry_accepts_fixed_topology_from_common_coordinator(mocker) -> None:
    mocker.patch.object(
        MakerModsMetalAdapter, "_make_can_bus", return_value=can_motor_control.MockCanBus("metal")
    )
    adapter = whole_body_adapter_registry.create(
        "makermods_metal",
        dof=7,
        hardware_id="metal",
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
    )
    try:
        assert adapter.connect()
        assert len(adapter.read_motor_states()) == 7
    finally:
        adapter.disconnect()


def test_registry_rejects_incompatible_joint_count_before_connect() -> None:
    with pytest.raises(ValueError, match="Metal requires 7 joints"):
        whole_body_adapter_registry.create("makermods_metal", dof=6)
