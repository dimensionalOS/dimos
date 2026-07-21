# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Focused OpenYAM adapter tests.

The shared Damiao runtime is an optional hardware dependency in this checkout;
the tests become active when that runtime is installed (as they are in the
hardware test environment).
"""

from pathlib import Path
from unittest.mock import Mock

import pytest

pytest.importorskip("dimos.hardware.damiao")

import dimos.hardware.manipulators.openyam_damiao.adapter as adapter_module
from dimos.hardware.manipulators.openyam_damiao.adapter import (
    ARM_MOTOR_SPECS,
    GRIPPER_MOTOR_SPECS,
    OPENING_METRES,
    OpenYamDamiaoAdapter,
    aperture_to_opening,
    opening_to_aperture,
)
from dimos.robot.model_parser import JointDescription, ModelDescription
from dimos.utils.data import LfsPath

GRAVITY_MODEL_PATH = Path(LfsPath("yam_description")) / "urdf/yam_gripper_gravity.urdf"


def test_gripper_aperture_conversion_is_linear() -> None:
    assert aperture_to_opening(0.0) == 0.0
    assert aperture_to_opening(OPENING_METRES / 2) == pytest.approx(0.5)
    assert aperture_to_opening(OPENING_METRES) == 1.0
    assert opening_to_aperture(0.5) == pytest.approx(OPENING_METRES / 2)


def test_openyam_motor_topology() -> None:
    assert [motor.name for motor in ARM_MOTOR_SPECS] == [f"yam_joint{i}" for i in range(1, 7)]
    assert [motor.send_id for motor in ARM_MOTOR_SPECS] == list(range(1, 7))
    assert [motor.type for motor in ARM_MOTOR_SPECS] == ["DM4340"] * 3 + ["DM4310"] * 3
    assert GRIPPER_MOTOR_SPECS[0].send_id == 7
    assert GRIPPER_MOTOR_SPECS[0].type == "DM4310"


def test_openyam_physical_gripper_is_disabled_without_calibrated_readback() -> None:
    adapter = OpenYamDamiaoAdapter(gravity_model_path=GRAVITY_MODEL_PATH, use_mock_bus=True)

    assert adapter.read_gripper_position() is None
    assert not adapter.write_gripper_position(0.01)


def test_openyam_limits_are_loaded_from_active_model() -> None:
    adapter = OpenYamDamiaoAdapter(gravity_model_path=GRAVITY_MODEL_PATH, use_mock_bus=True)

    limits = adapter.get_limits()
    assert limits.position_lower == pytest.approx([-3.92699, 0.0, 0.0, -1.65806, -1.5708, -2.35619])
    assert limits.position_upper == pytest.approx(
        [1.5708, 3.66519, 4.01426, 1.65806, 1.5708, 1.8326]
    )
    assert limits.velocity_max == pytest.approx([3.0, 10.0, 3.0, 10.0, 3.0, 10.0])


def test_openyam_requires_gravity_comp_and_model() -> None:
    with pytest.raises(ValueError, match="gravity model"):
        OpenYamDamiaoAdapter(use_mock_bus=True)
    with pytest.raises(ValueError, match="gravity compensation"):
        OpenYamDamiaoAdapter(
            gravity_model_path=GRAVITY_MODEL_PATH,
            gravity_comp=False,
            use_mock_bus=True,
        )


def test_openyam_normal_enable_and_error_recovery() -> None:
    adapter = OpenYamDamiaoAdapter(
        gravity_model_path=GRAVITY_MODEL_PATH,
        use_mock_bus=True,
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


@pytest.mark.parametrize("value", [-1e-6, OPENING_METRES + 1e-6])
def test_gripper_aperture_rejects_out_of_range(value: float) -> None:
    with pytest.raises(ValueError):
        aperture_to_opening(value)
