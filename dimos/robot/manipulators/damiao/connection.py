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

"""The DimOS side of a Damiao robot: ports, limits, timing and faults.

The motor layer lives in :mod:`dimos.robot.manipulators.damiao.arm` and the
robot's data in :mod:`dimos.robot.manipulators.damiao.robot`. This module only
translates commands into targets, reads feedback on the coordinator's cycle,
and latches a fault when feedback breaks.

A Damiao connection is driven directly: the coordinator in its worker reads
``read_measured`` for the feedback it computes on and calls ``command`` for the
result. It has no reader thread and no command subscription, so the fast cycle
is the only path that touches the hardware.
"""

from __future__ import annotations

from abc import abstractmethod
import time
from typing import Any, ClassVar
import uuid

from pydantic import Field, model_validator
from typing_extensions import Self

from dimos.control.connection.connection_module import (
    ConnectionConfig,
    ConnectionModule,
    ConnectionNotReadyError,
    Values,
)
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import limits as limits_of, movable_joints
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.damiao.arm import ArmReading, DamiaoArm
from dimos.robot.manipulators.damiao.robot import DamiaoRobot
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def zero_stiffness_joints(robot: DamiaoRobot, *, teach: bool, source: str = "arm") -> list[str]:
    """Joints a teaching connection commands with zero stiffness, in joint order.

    Hand-guided tasks use this declaration to prove the deployment will not
    fight the operator: every arm joint plus the jaws that follow the hand.
    Names carry the connection's robot name, e.g. ``arm/yam_joint1``.
    """
    if not teach:
        return []
    names = [
        *robot.arm_joint_names,
        *(gripper.joint for gripper in robot.grippers if gripper.name in robot.passive_grippers),
    ]
    return [f"{source}/{name}" for name in names]


class DamiaoConnectionConfig(ConnectionConfig):
    joint_commands: tuple[Interface, ...] = (Interface.POSITION,)
    teach: bool = False
    state_rate_hz: float = Field(default=100.0, gt=0, allow_inf_nan=False)
    deadman_timeout_s: float = Field(default=0.1, gt=0, allow_inf_nan=False)
    #: Damiao connections are driven directly by a coordinator in the same
    #: worker; there is no reader-thread/stream fallback to opt into.
    hosted: bool = True

    @model_validator(mode="after")
    def _position_only(self) -> Self:
        if self.joint_commands != (Interface.POSITION,) or self.pd or self.base_range is not None:
            raise ValueError("Damiao connections accept position commands only")
        if not self.hosted:
            raise ValueError(
                "Damiao connections are driven directly by their coordinator; hosted must be True"
            )
        return self


class DamiaoConnection(ConnectionModule):
    """Own one Damiao robot, its measured states and position targets."""

    #: The robot this connection deploys: motors, gains, grippers and models.
    robot: ClassVar[DamiaoRobot]
    #: Can be driven directly by a coordinator in the same worker process.
    hosted_direct: ClassVar[bool] = True
    config: DamiaoConnectionConfig
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._arm: DamiaoArm | None = None
        self._fault: str | None = None
        self._state_frame = ""

    @property
    def joint_names(self) -> list[str]:
        return list(self.robot.joint_names)

    @property
    def arm_joints(self) -> dict[str, list[str]]:
        return self.robot.arm_joints

    @property
    def gripper_joints(self) -> dict[str, str]:
        return self.robot.gripper_joints

    @abstractmethod
    def can_ports(self) -> list[str]:
        """Return one explicitly configured CAN selector per robot bus, in order."""

    def connect(self) -> None:
        """Validate the deployed model and topology, then open the buses."""
        assert self.config.model is not None
        if {joint.name for joint in movable_joints(self.config.model)} != set(self.joint_names):
            raise ValueError("connection model joints must match the vendor topology")
        self._state_frame = self.frame(self.config.model.load().root_link)
        ports = self.can_ports()
        if len(ports) != len(self.robot.buses) or any(not port.strip() for port in ports):
            raise ValueError("CAN selectors must cover exactly the robot buses")
        if len(set(ports)) != len(ports):
            raise ValueError("CAN selectors must be distinct")
        arm = DamiaoArm(self.robot, ports, teach=self.config.teach)
        arm.connect()
        self._arm = arm
        self._read_state()

    @rpc
    def start(self) -> None:
        try:
            Module.start(self)
            self._session_id = uuid.uuid4().hex
            self._limits = limits_of(self.config)
            self.connect()
            with self._hardware_lock:
                arm = self._arm
                assert arm is not None
                logger.info(
                    "Enabling robot and calibrating grippers",
                    connection=type(self).__name__,
                    source=self.config.source,
                )
                arm.enable()
                self._read_state()
                self._start_telemetry()
                self._connected = True
                logger.info(
                    "Robot connection ready",
                    connection=type(self).__name__,
                    source=self.config.source,
                )
        except Exception:
            try:
                self.stop()
            except Exception as cleanup_error:
                self._report(
                    f"startup cleanup failed: {type(cleanup_error).__name__}: {cleanup_error}"
                )
            raise

    @rpc
    def stop(self) -> None:
        try:
            super().stop()
        finally:
            with self._hardware_lock:
                arm, self._arm = self._arm, None
                try:
                    if arm is not None:
                        arm.disable()
                finally:
                    self._fault = None

    def write(self, values: Values) -> None:
        if self._fault is not None:
            raise RuntimeError(self._fault)
        arm = self._arm
        if arm is None:
            raise RuntimeError("Damiao robot is inactive")
        prefix = f"{self.config.source}/"
        arm.command(
            {
                key.removeprefix(prefix).removesuffix("/position"): value
                for key, value in values.items()
            }
        )

    def halt(self) -> None:
        if self._fault is not None or self._arm is None:
            return
        self._arm.hold()

    def read_measured(self) -> JointState:
        """Read the arm's encoders now: the feedback a hosted cycle computes on."""
        with self._hardware_lock:
            if not self._connected:
                # The coordinator may tick while this module is still starting
                # in the same worker. That is not a hardware fault.
                raise ConnectionNotReadyError(f"{type(self).__name__} is not connected yet")
            if self._fault is not None:
                raise RuntimeError(self._fault)
            reading = self._read_state()
        return JointState(
            ts=time.time(),
            frame_id=self._state_frame,
            name=[f"{self.config.source}/{name}" for name in self.joint_names],
            position=reading.position,
            velocity=reading.velocity,
            effort=reading.effort,
        )

    def _read_state(self) -> ArmReading:
        try:
            arm = self._arm
            if arm is None:
                raise RuntimeError("Damiao robot is disconnected")
            return arm.read()
        except Exception as error:
            if self._fault is None:
                self._feedback_failed(error)
            raise

    def _feedback_failed(self, error: Exception) -> None:
        self._fault = f"feedback failed: {type(error).__name__}: {error}"
        self._report(self._fault)
        arm = self._arm
        if arm is not None:
            try:
                arm.disable()
            except Exception as disable_error:
                self._report(f"{self._fault}; disable failed: {disable_error}")
