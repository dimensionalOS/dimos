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

"""Focused OpenYAM adapter tests.

The shared Damiao runtime is an optional hardware dependency in this checkout;
the tests become active when that runtime is installed (as they are in the
hardware test environment).
"""

from pathlib import Path
from unittest.mock import Mock

import pytest

pytest.importorskip("can_motor_control")

from dimos.hardware.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.damiao.runtime import DamiaoGroupState
import dimos.hardware.manipulators.openyam_damiao.adapter as adapter_module
from dimos.hardware.manipulators.openyam_damiao.adapter import (
    ARM_MOTOR_CONFIGS,
    GRIPPER_MOTOR_CONFIG,
    OPENYAM_GRIPPER_CONFIG,
    OpenYamDamiaoAdapter,
    make_openyam_damiao_arm_config,
)
from dimos.robot.model_parser import JointDescription, ModelDescription
from dimos.utils.data import LfsPath

GRAVITY_MODEL_PATH = Path(LfsPath("yam_description")) / "urdf/yam_gripper_gravity.urdf"


def test_openyam_motor_topology() -> None:
    assert [motor.name for motor in ARM_MOTOR_CONFIGS] == [f"yam_joint{i}" for i in range(1, 7)]
    assert [motor.send_id for motor in ARM_MOTOR_CONFIGS] == list(range(1, 7))
    assert [motor.type for motor in ARM_MOTOR_CONFIGS] == ["DM4340"] * 3 + ["DM4310"] * 3
    assert GRIPPER_MOTOR_CONFIG.send_id == 0x08
    assert GRIPPER_MOTOR_CONFIG.effective_recv_id == 0x18
    assert GRIPPER_MOTOR_CONFIG.type == "DM4310"
    assert OPENYAM_GRIPPER_CONFIG.opening_direction == "decreasing_position"
    assert OPENYAM_GRIPPER_CONFIG.default_current == 0.15
    assert make_openyam_damiao_arm_config().gripper is OPENYAM_GRIPPER_CONFIG


def test_openyam_gripper_delegates_normalized_io() -> None:
    adapter = OpenYamDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(
            gravity_model_path=GRAVITY_MODEL_PATH,
            use_mock_bus=True,
        )
    )
    runtime = Mock()
    runtime.read_gripper_opening.return_value = 0.4
    runtime.write_gripper_opening.return_value = True
    adapter._runtime = runtime

    assert adapter.read_gripper_position() == 0.4
    assert adapter.write_gripper_position(0.75)
    runtime.read_gripper_opening.assert_called_once_with()
    runtime.write_gripper_opening.assert_called_once_with(0.75)


def test_openyam_limits_are_loaded_from_active_model() -> None:
    adapter = OpenYamDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(
            gravity_model_path=GRAVITY_MODEL_PATH,
            use_mock_bus=True,
        )
    )

    limits = adapter.get_limits()
    assert limits.position_lower == pytest.approx([-3.92699, 0.0, 0.0, -1.65806, -1.5708, -2.35619])
    assert limits.position_upper == pytest.approx(
        [1.5708, 3.66519, 4.01426, 1.65806, 1.5708, 1.8326]
    )
    assert limits.velocity_max == pytest.approx([3.0, 10.0, 3.0, 10.0, 3.0, 10.0])
    assert adapter._kp == pytest.approx([80.0, 80.0, 80.0, 10.0, 10.0, 10.0])
    assert adapter._kd == pytest.approx([5.0, 5.0, 5.0, 1.5, 1.5, 1.5])
    assert adapter._gravity_comp


def test_openyam_allows_gravity_comp_to_be_disabled() -> None:
    adapter = OpenYamDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False, use_mock_bus=True)
    )

    assert not adapter._gravity_comp

    with pytest.raises(ValueError, match="gravity compensation"):
        OpenYamDamiaoAdapter(runtime_config=DamiaoRuntimeConfig(use_mock_bus=True))


def test_openyam_activation_holds_exact_feedback_position() -> None:
    adapter = OpenYamDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False, use_mock_bus=True)
    )
    runtime = Mock()
    feedback = [-1.2, 0.1, 0.2, -0.3, 0.4, -0.5]
    runtime.refresh_state.return_value = DamiaoGroupState(q=feedback, dq=[0.0] * 6, tau=[0.0] * 6)
    runtime.enable.return_value = True
    runtime.write_mit_commands.return_value = True
    adapter._runtime = runtime

    assert adapter.activate()

    runtime.write_mit_commands.assert_called_once_with(
        q=feedback,
        dq=[0.0] * 6,
        kp=[80.0, 80.0, 80.0, 10.0, 10.0, 10.0],
        kd=[5.0, 5.0, 5.0, 1.5, 1.5, 1.5],
        tau=[0.0] * 6,
    )
    assert runtime.refresh_state.call_count >= 2


def test_openyam_normal_enable_and_error_recovery() -> None:
    adapter = OpenYamDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(
            gravity_model_path=GRAVITY_MODEL_PATH,
            use_mock_bus=True,
        ),
    )
    runtime = Mock()
    runtime.enable.return_value = True
    adapter._runtime = runtime
    adapter._preflight_gravity = Mock()
    adapter.read_joint_positions = Mock(return_value=[0.0] * 6)
    adapter.write_joint_positions = Mock(return_value=True)

    assert adapter.activate()
    runtime.enable.assert_called_once_with()

    runtime.reset_mock()
    runtime.disable.return_value = True
    assert adapter.write_clear_errors()
    runtime.disable.assert_called_once_with()
    runtime.enable.assert_called_once_with()


def test_openyam_forwards_mit_commands() -> None:
    adapter = OpenYamDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(
            gravity_model_path=GRAVITY_MODEL_PATH,
            use_mock_bus=True,
        ),
    )
    runtime = Mock()
    runtime.refresh_state.return_value = DamiaoGroupState(q=[0.25] * 6, dq=[0.0] * 6, tau=[0.0] * 6)
    runtime.write_mit_commands.return_value = True
    adapter._runtime = runtime
    adapter._enabled = True

    assert adapter.refresh_state(force=True)[0] == [0.25] * 6

    assert adapter.write_mit_commands(
        q=[1.0] * 6,
        dq=[2.0] * 6,
        kp=[3.0] * 6,
        kd=[4.0] * 6,
        tau=[5.0] * 6,
    )

    runtime.write_mit_commands.assert_called_once_with(
        q=[1.0] * 6,
        dq=[2.0] * 6,
        kp=[3.0] * 6,
        kd=[4.0] * 6,
        tau=[5.0] * 6,
    )

    assert not adapter.write_mit_commands(
        q=[1.0] * 6, dq=[2.0] * 6, kp=[3.0] * 6, kd=[4.0] * 6, tau=[5.0] * 6
    )
    runtime.write_mit_commands.assert_called_once()


def test_openyam_failed_read_revokes_write_permission() -> None:
    adapter = OpenYamDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False, use_mock_bus=True)
    )
    runtime = Mock()
    runtime.refresh_state.return_value = DamiaoGroupState(q=[0.25] * 6, dq=[0.0] * 6, tau=[0.0] * 6)
    adapter._runtime = runtime
    adapter._enabled = True

    adapter.refresh_state(force=True)
    runtime.refresh_state.side_effect = RuntimeError("feedback unavailable")
    with pytest.raises(RuntimeError, match="feedback unavailable"):
        adapter.refresh_state(force=True)

    assert not adapter.write_joint_positions([0.25] * 6)
    runtime.write_mit_commands.assert_not_called()


def test_openyam_xacro_limits_reject_duplicate_joint_names(monkeypatch: pytest.MonkeyPatch) -> None:
    joints = [JointDescription(f"yam_joint{i}", "revolute", -1.0, 1.0, 1.0) for i in range(1, 7)]
    joints.append(joints[0])
    monkeypatch.setattr(
        adapter_module, "parse_model", lambda *args, **kwargs: ModelDescription(joints=joints)
    )

    with pytest.raises(ValueError, match="duplicate"):
        adapter_module._active_arm_limits()


@pytest.mark.parametrize(
    ("lower", "upper", "velocity"),
    [(2.0, 1.0, 1.0), (0.0, 1.0, 0.0), (0.0, 1.0, float("nan"))],
)
def test_openyam_xacro_limits_reject_bad_values(
    monkeypatch: pytest.MonkeyPatch, lower: float, upper: float, velocity: float
) -> None:
    joints = [
        JointDescription(
            f"yam_joint{i}",
            "revolute",
            lower if i == 1 else -1.0,
            upper if i == 1 else 1.0,
            velocity if i == 1 else 1.0,
        )
        for i in range(1, 7)
    ]
    monkeypatch.setattr(
        adapter_module, "parse_model", lambda *args, **kwargs: ModelDescription(joints=joints)
    )

    with pytest.raises(ValueError):
        adapter_module._active_arm_limits()
