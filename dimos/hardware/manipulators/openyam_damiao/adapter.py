# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""OpenYAM's six-axis Damiao adapter."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, cast

from dimos.hardware.damiao import (
    DamiaoArmAdapter,
    DamiaoBusSpec,
    DamiaoJointGroupSpec,
    DamiaoMotorSpec,
    DamiaoRobotSpec,
)
from dimos.robot.model_parser import parse_model
from dimos.utils.data import LfsPath

OPENING_METRES = 0.096
_BUS_NAME = "openyam_can"
_ARM_GROUP = "arm"
_OPENYAM_MODEL_PATH = Path(LfsPath("yam_description")) / "urdf/yam_gripper.urdf.xacro"
_OPENYAM_PACKAGE_PATHS = {"yam_description": Path(LfsPath("yam_description"))}

ARM_MOTOR_SPECS = tuple(
    DamiaoMotorSpec(
        name=f"yam_joint{index}",
        type="DM4340" if index <= 3 else "DM4310",
        send_id=index,
    )
    for index in range(1, 7)
)
GRIPPER_MOTOR_SPECS = (DamiaoMotorSpec("yam_gripper", "DM4310", 7),)


def aperture_to_opening(aperture: float) -> float:
    """Convert a metre aperture to the driver's normalized opening."""
    if not 0.0 <= aperture <= OPENING_METRES:
        raise ValueError(f"gripper aperture must be in [0, {OPENING_METRES}] m")
    return aperture / OPENING_METRES


def opening_to_aperture(opening: float) -> float:
    """Convert a calibrated normalized opening to a metre aperture."""
    if not 0.0 <= opening <= 1.0:
        raise ValueError("gripper opening must be in [0, 1]")
    return opening * OPENING_METRES


def _group_spec(
    *,
    bus_name: str,
    motors: tuple[DamiaoMotorSpec, ...],
    lower: tuple[float, ...],
    upper: tuple[float, ...],
    velocity: tuple[float, ...],
    kp: tuple[float, ...],
    kd: tuple[float, ...],
    gravity_model_path: str | Path | None = None,
) -> DamiaoJointGroupSpec:
    return DamiaoJointGroupSpec(
        bus_name=bus_name,
        motors=motors,
        position_lower=lower,
        position_upper=upper,
        velocity_max=velocity,
        kp=kp,
        kd=kd,
        gravity_model_path=gravity_model_path,
    )


def _active_arm_limits() -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    """Read arm limits from the active planning Xacro, failing closed."""
    model = parse_model(_OPENYAM_MODEL_PATH, package_paths=_OPENYAM_PACKAGE_PATHS)
    names = [joint.name for joint in model.joints]
    if len(names) != len(set(names)):
        raise ValueError("active OpenYAM Xacro contains duplicate joint names")
    joints = [model.get_joint(f"yam_joint{index}") for index in range(1, 7)]
    resolved = [joint for joint in joints if joint is not None]
    if len(resolved) != 6:
        raise ValueError("active OpenYAM Xacro does not define all six arm joints")
    for joint in resolved:
        lower = joint.lower_limit
        upper = joint.upper_limit
        velocity = joint.velocity_limit
        if lower is None or upper is None or velocity is None:
            raise ValueError("active OpenYAM Xacro has incomplete or nonfinite arm limits")
        lower = cast("float", lower)
        upper = cast("float", upper)
        velocity = cast("float", velocity)
        if not all(math.isfinite(value) for value in (lower, upper, velocity)):
            raise ValueError("active OpenYAM Xacro has incomplete or nonfinite arm limits")
        if lower > upper:
            raise ValueError(f"active OpenYAM Xacro has inverted limits for {joint.name}")
        if velocity <= 0:
            raise ValueError(f"active OpenYAM Xacro has nonpositive velocity for {joint.name}")
    return (
        tuple(cast("float", joint.lower_limit) for joint in resolved),
        tuple(cast("float", joint.upper_limit) for joint in resolved),
        tuple(cast("float", joint.velocity_limit) for joint in resolved),
    )


class OpenYamDamiaoAdapter(DamiaoArmAdapter):
    """Six-DOF OpenYAM arm; physical gripper IO is fail-closed."""

    def __init__(
        self,
        address: str = "can0",
        *,
        gravity_model_path: str | Path | None = None,
        gravity_comp: bool = True,
        operator_approved: bool = False,
        **kwargs: Any,
    ) -> None:
        if not gravity_comp:
            raise ValueError("OpenYAM requires gravity compensation")
        if gravity_model_path is None or not Path(gravity_model_path).is_file():
            raise ValueError("OpenYAM requires a valid gravity model path")
        self._operator_approved = operator_approved
        lower, upper, velocity = _active_arm_limits()
        arm = _group_spec(
            bus_name=_BUS_NAME,
            motors=ARM_MOTOR_SPECS,
            lower=lower,
            upper=upper,
            velocity=velocity,
            kp=(0.0,) * 6,
            kd=(0.0,) * 6,
            gravity_model_path=gravity_model_path,
        )
        robot_spec = DamiaoRobotSpec(
            name="openyam",
            vendor="Damiao",
            model="OpenYAM",
            buses={_BUS_NAME: DamiaoBusSpec(address=address)},
            # The upstream binding has no calibrated normalized gripper
            # readback API. Do not expose a guessed/raw or last-command state.
            groups={_ARM_GROUP: arm},
        )
        super().__init__(
            robot_spec=robot_spec,
            group_name=_ARM_GROUP,
            gravity_model_path=gravity_model_path,
            gravity_comp=True,
            **kwargs,
        )

    def write_enable(self, enable: bool) -> bool:
        """Gate every physical enable, including error-recovery enables."""
        if enable and not self._operator_approved:
            return False
        return super().write_enable(enable)

    def write_clear_errors(self) -> bool:
        """Gate error recovery before it disables or re-enables the runtime."""
        if not self._operator_approved:
            return False
        return super().write_clear_errors()

    def read_gripper_position(self) -> float | None:
        """Gripper feedback is disabled until the binding provides calibration."""
        return None

    def write_gripper_position(self, position: float) -> bool:
        """Reject physical gripper commands without calibrated feedback."""
        del position
        return False


OpenYAMDamiaoAdapter = OpenYamDamiaoAdapter
