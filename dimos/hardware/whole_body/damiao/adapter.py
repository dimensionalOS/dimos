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

"""Generic whole-body adapter for robots built with ``can-motor-control``."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping
from pathlib import Path
from typing import Any, NoReturn

import can_motor_control
import numpy as np
import pinocchio

from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.spec import IMUState, MotorCommand, MotorState
from dimos.robot.assets.model import RobotModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DamiaoWholeBodyAdapter(ABC):
    """Map DimOS whole-body IO onto one upstream robot lifecycle owner.

    Subclasses own immutable physical topology by implementing ``_build_robot``.
    The mappings declare how upstream arm and gripper groups appear in DimOS.
    """

    arm_joints: dict[str, tuple[str, ...]] = {}
    gripper_joints: dict[str, str] = {}
    bus_defaults: dict[str, str] = {}
    kinematic_joint_names: tuple[str, ...] = ()
    feedback_clamp_margin_rad = 0.05

    def __init__(
        self,
        address: str | Path | None = None,
        *,
        runtime_config: DamiaoRuntimeConfig | Mapping[str, Any] | None = None,
        dof: int | None = None,
        hardware_id: str = "whole_body",
        domain_id: int = 0,
    ) -> None:
        """Initialize runtime settings for a subclass-declared Damiao topology.

        ``address`` is accepted for the coordinator's common adapter factory
        convention, but one scalar cannot represent a multi-bus whole body.
        Configure physical CAN interfaces by logical bus name through
        ``runtime_config.bus_addresses`` instead.
        """
        del domain_id
        if address is not None:
            raise ValueError("configure Damiao CAN buses through runtime_config.bus_addresses")
        if runtime_config is None:
            config = DamiaoRuntimeConfig()
        elif isinstance(runtime_config, DamiaoRuntimeConfig):
            config = runtime_config
        else:
            config = DamiaoRuntimeConfig(**runtime_config)
        unknown_buses = config.bus_addresses.keys() - self.bus_defaults.keys()
        if unknown_buses:
            raise ValueError(f"unknown CAN bus overrides: {sorted(unknown_buses)}")

        joint_names = self.joint_names
        if len(joint_names) != len(set(joint_names)):
            raise ValueError("whole-body joint mappings contain duplicate names")
        if dof is not None and dof != len(joint_names):
            raise ValueError(f"expected {len(joint_names)} joints, got {dof}")

        arm_joint_count = sum(len(names) for names in self.arm_joints.values())
        if self.kinematic_joint_names and len(self.kinematic_joint_names) != arm_joint_count:
            raise ValueError("kinematic joint mapping must contain every angular arm joint")

        self._runtime_config = config
        self._hardware_id = hardware_id
        self._connected = False
        self._active = False
        self._has_state = False
        self._robot: can_motor_control.Robot
        self._arms: dict[str, can_motor_control.Arm] = {}
        self._grippers: dict[str, can_motor_control.Gripper] = {}
        self._pin_model: pinocchio.Model
        self._pin_data: pinocchio.Data
        self._arm_position_limits: tuple[tuple[float, float], ...] = ()
        self._feedback_fault: str | None = None
        self._warned_feedback_clamps: set[tuple[str, str]] = set()

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(
            joint for group_joints in self.arm_joints.values() for joint in group_joints
        ) + tuple(self.gripper_joints.values())

    def bus_address(self, name: str) -> str:
        """Resolve a subclass-declared bus name through runtime overrides."""
        try:
            return self._runtime_config.bus_addresses.get(name, self.bus_defaults[name])
        except KeyError as exc:
            raise ValueError(f"subclass did not declare CAN bus {name!r}") from exc

    @property
    def kinematic_model(self) -> RobotModel | None:
        """Return the subclass's arm model without resolving it at import time."""
        return None

    @abstractmethod
    def _build_robot(self) -> can_motor_control.Robot:
        """Construct the upstream robot from the subclass's physical topology."""

    def connect(self) -> bool:
        self._feedback_fault = None
        self._warned_feedback_clamps.clear()
        try:
            robot = self._build_robot()
        except Exception:
            logger.exception(
                "Damiao whole-body adapter failed to build",
                hardware_id=self._hardware_id,
            )
            return False

        try:
            robot.connect()
            arms = {name: self._require_arm(robot, name) for name in self.arm_joints}
            grippers = {name: self._require_gripper(robot, name) for name in self.gripper_joints}
            self._robot = robot
            self._arms = arms
            self._grippers = grippers
            self._load_kinematic_model()
            self._connected = True
            self._refresh()
            return True
        except Exception:
            logger.exception(
                "Damiao whole-body adapter failed to connect",
                hardware_id=self._hardware_id,
            )
            self._release_robot(robot)
            return False

    @staticmethod
    def _require_arm(
        robot: can_motor_control.Robot,
        name: str,
    ) -> can_motor_control.Arm:
        group = robot[name]
        if not isinstance(group, can_motor_control.Arm):
            raise TypeError(f"upstream group {name!r} is not an Arm")
        return group

    @staticmethod
    def _require_gripper(
        robot: can_motor_control.Robot,
        name: str,
    ) -> can_motor_control.Gripper:
        group = robot[name]
        if not isinstance(group, can_motor_control.Gripper):
            raise TypeError(f"upstream group {name!r} is not a Gripper")
        return group

    def disconnect(self) -> None:
        if not self._connected:
            return
        self._release_robot(self._robot)

    def _release_robot(self, robot: can_motor_control.Robot) -> None:
        """Disable hardware and drop every handle that owns the robot transport."""
        try:
            if robot.is_connected():
                robot.disable()
        except Exception:
            logger.warning(
                "Damiao whole-body adapter failed to disable while releasing robot",
                hardware_id=self._hardware_id,
                exc_info=True,
            )
        self._arms.clear()
        self._grippers.clear()
        if hasattr(self, "_robot") and self._robot is robot:
            del self._robot
        self._connected = False
        self._active = False
        self._has_state = False

    def is_connected(self) -> bool:
        return self._connected and self._robot.is_connected()

    def activate(self) -> bool:
        if not self._connected or self._feedback_fault is not None:
            if self._feedback_fault is not None:
                logger.error(
                    "Damiao activation rejected while feedback fault is latched",
                    hardware_id=self._hardware_id,
                    error=self._feedback_fault,
                )
            return False
        try:
            self._preflight_gravity()
            for arm in self._arms.values():
                arm.set_mode("mit")
            self._robot.enable()
            self._active = True
            self.read_motor_states()
            return True
        except Exception:
            logger.exception(
                "Damiao whole-body adapter failed to activate",
                hardware_id=self._hardware_id,
            )
            try:
                self._robot.disable()
            except Exception:
                logger.error(
                    "Damiao whole-body activation rollback failed",
                    hardware_id=self._hardware_id,
                    exc_info=True,
                )
            self._active = False
            return False

    def deactivate(self) -> bool:
        if not self._connected:
            return False
        try:
            self._robot.disable()
        except Exception:
            logger.exception(
                "Damiao whole-body adapter failed to deactivate",
                hardware_id=self._hardware_id,
            )
            return False
        self._active = False
        return True

    def has_motor_states(self) -> bool:
        if not self._connected or not self._has_state:
            return False
        return not self._grippers or self._active

    def read_motor_states(self) -> list[MotorState]:
        if not self._connected:
            raise RuntimeError("Damiao whole-body adapter is not connected")
        if self._feedback_fault is not None:
            raise RuntimeError(f"Damiao feedback fault is latched: {self._feedback_fault}")
        self._refresh()
        states: list[MotorState] = []
        for name, expected_joints in self.arm_joints.items():
            arm = self._arms[name]
            q = arm.positions().astype(np.float64).tolist()
            dq = arm.velocities().astype(np.float64).tolist()
            tau = arm.torques().astype(np.float64).tolist()
            if any(len(values) != len(expected_joints) for values in (q, dq, tau)):
                raise RuntimeError(f"upstream arm {name!r} returned the wrong state length")
            states.extend(
                MotorState(q=position, dq=velocity, tau=effort)
                for position, velocity, effort in zip(q, dq, tau, strict=True)
            )
        for name in self.gripper_joints:
            if not self._active:
                # Gripper opening calibrates during activation; report a
                # placeholder so read-only sessions still stream arm state.
                states.append(MotorState(q=0.0, dq=0.0, tau=0.0))
                continue
            opening = float(self._grippers[name].opening)
            if not np.isfinite(opening) or not 0.0 <= opening <= 1.0:
                raise RuntimeError(f"gripper {name!r} returned invalid opening {opening}")
            states.append(MotorState(q=opening, dq=0.0, tau=0.0))
        self._validate_finite_states(states)
        return self._normalize_arm_feedback(states)

    def read_imu(self) -> IMUState:
        return IMUState()

    def get_limits(self) -> JointLimits | None:
        """Return limits when the concrete robot declares them."""
        return None

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        if not self._connected or not self._active or len(commands) != len(self.joint_names):
            return False
        try:
            arm_count = sum(len(joints) for joints in self.arm_joints.values())
            arm_values = np.asarray(
                [
                    (command.q, command.dq, command.kp, command.kd, command.tau)
                    for command in commands[:arm_count]
                ],
                dtype=np.float64,
            )
            if not np.isfinite(arm_values).all():
                raise ValueError("arm command contains non-finite values")
            for name, command in zip(
                self.gripper_joints,
                commands[arm_count:],
                strict=True,
            ):
                if not np.isfinite(command.q) or not 0.0 <= command.q <= 1.0:
                    raise ValueError(f"gripper {name!r} opening must be in [0, 1]")

            gravity = self._gravity_torques()
            offset = 0
            gravity_offset = 0
            for name, joints in self.arm_joints.items():
                count = len(joints)
                group_commands = commands[offset : offset + count]
                rows = np.asarray(
                    [
                        [
                            command.kp,
                            command.kd,
                            command.q,
                            command.dq,
                            command.tau + gravity[gravity_offset + index],
                        ]
                        for index, command in enumerate(group_commands)
                    ],
                    dtype=np.float64,
                )
                self._arms[name].mit_control(rows)
                offset += count
                gravity_offset += count

            for name in self.gripper_joints:
                opening = commands[offset].q
                self._grippers[name].set_opening(opening)
                offset += 1

            self._robot.tick(self._runtime_config.tick_deadline_us)
            return True
        except Exception:
            logger.exception(
                "Damiao whole-body adapter rejected motor command",
                hardware_id=self._hardware_id,
            )
            return False

    def _refresh(self) -> None:
        self._robot.refresh()
        self._robot.tick(self._runtime_config.tick_deadline_us)
        self._has_state = True

    def _load_kinematic_model(self) -> None:
        if not self.kinematic_joint_names:
            return
        robot_model = self.kinematic_model
        if robot_model is None:
            raise ValueError("Damiao arm safety requires a kinematic model")
        model = pinocchio.buildModelFromXML(robot_model.load().xml)
        controlled_joints = set(self.kinematic_joint_names)
        locked_joint_ids = [
            joint_id
            for joint_id, name in enumerate(model.names)
            if joint_id != 0 and str(name) not in controlled_joints
        ]
        if locked_joint_ids:
            model = pinocchio.buildReducedModel(
                model,
                locked_joint_ids,
                pinocchio.neutral(model),
            )
        self._pin_model = model
        self._pin_data = self._pin_model.createData()
        arm_count = sum(len(joints) for joints in self.arm_joints.values())
        if self._pin_model.nq != arm_count or self._pin_model.nv != arm_count:
            raise ValueError(
                f"kinematic model dimensions ({self._pin_model.nq}, {self._pin_model.nv}) "
                f"do not match {arm_count} angular joints"
            )
        model_names = tuple(str(name) for name in self._pin_model.names[1:])
        if model_names != self.kinematic_joint_names:
            raise ValueError(
                f"kinematic model joint order {model_names!r} does not match "
                f"{self.kinematic_joint_names!r}"
            )
        lower = np.asarray(self._pin_model.lowerPositionLimit, dtype=np.float64)
        upper = np.asarray(self._pin_model.upperPositionLimit, dtype=np.float64)
        if (
            lower.shape != (arm_count,)
            or upper.shape != (arm_count,)
            or not np.isfinite(lower).all()
            or not np.isfinite(upper).all()
            or np.any(lower >= upper)
        ):
            raise ValueError("kinematic model contains invalid arm position limits")
        self._arm_position_limits = tuple(
            (float(lower_limit), float(upper_limit))
            for lower_limit, upper_limit in zip(lower, upper, strict=True)
        )

    def _preflight_gravity(self) -> None:
        if not self._runtime_config.gravity_comp:
            return
        q = self._arm_positions()
        gravity = self._gravity_torques()
        if len(gravity) != len(q) or not np.isfinite(gravity).all():
            raise ValueError("gravity compensation produced invalid torques")

    def _normalize_arm_feedback(self, states: list[MotorState]) -> list[MotorState]:
        normalized = list(states)
        arm_joint_names = self.joint_names[: len(self._arm_position_limits)]
        for index, (joint_name, limits) in enumerate(
            zip(arm_joint_names, self._arm_position_limits, strict=True)
        ):
            lower, upper = limits
            state = states[index]
            if lower <= state.q <= upper:
                continue

            boundary = "lower" if state.q < lower else "upper"
            limit = lower if state.q < lower else upper
            if abs(state.q - limit) > self.feedback_clamp_margin_rad + 1e-12:
                self._latch_feedback_fault(
                    joint_name=joint_name,
                    value=state.q,
                    lower=lower,
                    upper=upper,
                )

            warning_key = (joint_name, boundary)
            if warning_key not in self._warned_feedback_clamps:
                logger.warning(
                    "Clamping Damiao joint feedback at position limit",
                    hardware_id=self._hardware_id,
                    joint=joint_name,
                    value=state.q,
                    limit=limit,
                )
                self._warned_feedback_clamps.add(warning_key)
            normalized[index] = MotorState(q=limit, dq=state.dq, tau=state.tau)
        return normalized

    def _latch_feedback_fault(
        self,
        *,
        joint_name: str,
        value: float,
        lower: float,
        upper: float,
    ) -> NoReturn:
        fault = (
            f"{joint_name} reported {value} outside [{lower}, {upper}] by more than "
            f"{self.feedback_clamp_margin_rad} rad"
        )
        self._feedback_fault = fault
        disabled = self.deactivate()
        self._active = False
        if not disabled:
            fault = f"{fault}; motor disable failed"
            self._feedback_fault = fault
        raise RuntimeError(f"Damiao feedback fault: {fault}")

    def _arm_positions(self) -> np.ndarray:
        positions = np.concatenate(
            [arm.positions().astype(np.float64) for arm in self._arms.values()]
        )
        if not np.isfinite(positions).all():
            raise ValueError("arm feedback contains non-finite positions")
        return positions

    def _gravity_torques(self) -> np.ndarray:
        count = sum(len(joints) for joints in self.arm_joints.values())
        if not self._runtime_config.gravity_comp:
            return np.zeros(count, dtype=np.float64)
        return np.asarray(
            pinocchio.computeGeneralizedGravity(
                self._pin_model,
                self._pin_data,
                self._arm_positions(),
            ),
            dtype=np.float64,
        )

    @staticmethod
    def _validate_finite_states(states: list[MotorState]) -> None:
        values = np.asarray(
            [(state.q, state.dq, state.tau) for state in states],
            dtype=np.float64,
        )
        if not np.isfinite(values).all():
            raise RuntimeError("whole-body feedback contains non-finite values")
