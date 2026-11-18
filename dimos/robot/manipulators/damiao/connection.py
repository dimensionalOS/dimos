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
translates streams into targets, publishes readings at a fixed rate, and
latches a fault when feedback breaks.

At start it applies each arm's saved calibration file, if there is one
(``DamiaoConnectionConfig.calibration``): the file's gains and fitted link
masses. Without a file the arm runs exactly its URDF and bus defaults. The
``tuning_*`` RPCs (:class:`~dimos.robot.manipulators.calibration.hooks.
ArmTuningHooks`) let a calibration tool change gains live; they are for
calibration tools only.
"""

from __future__ import annotations

from abc import abstractmethod
from dataclasses import dataclass, fields
from pathlib import Path
import threading
import time
from typing import Any, ClassVar
import uuid

from pydantic import Field, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import (
    ConnectionConfig,
    ConnectionModule,
    ConnectionStatus,
    Values,
)
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import limits as limits_of, movable_joints
from dimos.control.contract.validate import Rejected, validate_command
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.calibration._inertial import LinkInertial
from dimos.robot.manipulators.calibration.hooks import (
    AppliedCalibration,
    JointGainsState,
    JointSample,
)
from dimos.robot.manipulators.calibration.schema import (
    ARMS_STATE_DIR,
    ArmCalibration,
    ArmCalibrationError,
    calibration_file,
    calibration_path,
    load_calibration,
    to_inertial_overrides,
)
from dimos.robot.manipulators.calibration.spec import model_sha256
from dimos.robot.manipulators.damiao.arm import ArmReading, DamiaoArm
from dimos.robot.manipulators.damiao.robot import DamiaoRobot
from dimos.robot.manipulators.damiao.status import DamiaoConnectionStatus
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

__all__ = [
    "CALIBRATION_ROOT",
    "DamiaoConnection",
    "DamiaoConnectionConfig",
    "DamiaoConnectionStatus",
    "zero_stiffness_joints",
]

#: Where "auto" looks for each arm's calibration, ``<root>/<arm_id>/calibration.json``.
#: Read at every load, so tests can point it at a scratch directory.
CALIBRATION_ROOT: Path = ARMS_STATE_DIR


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
    """Settings of a Damiao connection.

    Attributes:
        teach: Drive every arm joint with zero stiffness so an operator can
            guide it by hand.
        state_rate_hz: How often the joints are read and published.
        deadman_timeout_s: How long without a command before the arm holds.
        calibration: Which calibration to apply at start. "auto" applies each
            arm's saved file (``calibration_path(arm_id)``) when there is one;
            "off" applies none; anything else is the path of one calibration
            file, applied to the arm it names. An arm without a file, or whose
            file does not fit this robot, runs its URDF masses and bus gains.
    """

    teach: bool = False
    state_rate_hz: float = Field(default=100.0, gt=0, allow_inf_nan=False)
    deadman_timeout_s: float = Field(default=0.1, gt=0, allow_inf_nan=False)
    calibration: str = Field(default="auto", min_length=1)

    @model_validator(mode="after")
    def _position_only(self) -> Self:
        if any(interfaces != (Interface.POSITION,) for interfaces in self.joint_commands.values()):
            raise ValueError("Damiao connections accept position commands only")
        return self


@dataclass(frozen=True, slots=True)
class _ArmFile:
    """One arm's calibration file, read and checked against the robot."""

    path: Path
    calibration: ArmCalibration
    gains: dict[str, tuple[float, float]]
    links: dict[str, LinkInertial]


class DamiaoConnection(ConnectionModule):
    """Own one Damiao robot, its measured states and position targets.

    The ``tuning_*`` RPCs implement
    :class:`~dimos.robot.manipulators.calibration.hooks.ArmTuningHooks`. They
    change motor gains and torques of the energized arm live and are for
    calibration tools only.
    """

    #: The robot this connection deploys: motors, gains, grippers and models.
    robot: ClassVar[DamiaoRobot]
    config: DamiaoConnectionConfig
    position_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        kwargs.setdefault(
            "joint_commands", {name: (Interface.POSITION,) for name in self.robot.joint_names}
        )
        super().__init__(**kwargs)
        self._arm: DamiaoArm | None = None
        self._fault: str | None = None
        self._state_frame = ""
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None
        # Guarded by _lock: the calibration each arm runs, for status().
        self._calibration: dict[str, AppliedCalibration | None] = {}
        self._model_hash: str | None = None
        # The latest reading and when it was taken, for tuning_read().
        self._latest: tuple[float, ArmReading] | None = None

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
        # Before enable, so the first torque the motors get is the calibrated one.
        files = self._read_calibrations(arm, self.config.calibration, strict=False)
        self._apply_calibrations(arm, files, always=False)

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
                self.register_disposable(
                    Disposable(self.position_command.subscribe(self._on_position))
                )
                self._reader = threading.Thread(
                    target=self._read_loop, daemon=True, name=f"{self.config.source}-read"
                )
                self._reader.start()
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
        self._stop_reading.set()
        if (
            self._reader is not None
            and self._reader.is_alive()
            and self._reader is not threading.current_thread()
        ):
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        try:
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
        finally:
            if (
                self._reader is not None
                and self._reader.is_alive()
                and self._reader is not threading.current_thread()
            ):
                self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
                if self._reader.is_alive():
                    error = f"{self.config.source} reader did not stop before the shutdown timeout"
                    self._report(error)
                    raise RuntimeError(error)

    @rpc
    def status(self) -> DamiaoConnectionStatus:
        """A snapshot of the connection and the calibration each arm runs."""
        base = super().status()
        with self._lock:
            calibration = dict(self._calibration)
        return DamiaoConnectionStatus(
            **{item.name: getattr(base, item.name) for item in fields(ConnectionStatus)},
            calibration=calibration,
        )

    @rpc
    def tuning_arm_ids(self) -> list[str]:
        """The calibration ids of this robot's arms, in bus order. For calibration tools only."""
        return self.robot.arm_ids()

    @rpc
    def tuning_joint_names(self) -> list[str]:
        """The arm joints whose gains can be tuned, in motor order. For calibration tools only."""
        return self.robot.arm_joint_names

    @rpc
    def tuning_get_gains(self) -> dict[str, JointGainsState]:
        """How each arm joint is driven now. For calibration tools only."""
        with self._hardware_lock:
            arm = self._tuning_arm()
            gains, floating, feedforward = arm.get_gains(), arm.floating, arm.feedforward
        return {
            joint: JointGainsState(
                kp=kp,
                kd=kd,
                floating=joint in floating,
                float_kd=floating.get(joint),
                feedforward_nm=feedforward.get(joint, 0.0),
            )
            for joint, (kp, kd) in gains.items()
        }

    @rpc
    def tuning_set_gains(self, joint: str, kp: float, kd: float) -> None:
        """Hold ``joint`` with these gains now, clamped to kp 0..500 and kd 0..5.

        Changes the energized motor's gains at once; for calibration tools only.
        """
        with self._hardware_lock:
            self._tuning_arm().set_gains(joint, float(kp), float(kd))

    @rpc
    def tuning_reset_gains(self) -> None:
        """Back to the calibration's gains, else the bus defaults. For calibration tools only."""
        with self._hardware_lock:
            self._tuning_arm().reset_gains()

    @rpc
    def tuning_set_float(self, joint: str, on: bool, kd: float | None = None) -> None:
        """Let ``joint`` float with zero stiffness, or hold it again where it is.

        Gravity stays compensated. Changes the energized motor's gains at
        once; for calibration tools only.
        """
        with self._hardware_lock:
            self._tuning_arm().set_float(joint, bool(on), None if kd is None else float(kd))

    @rpc
    def tuning_set_feedforward(self, joint: str, tau_nm: float) -> None:
        """Add ``tau_nm`` to ``joint``'s gravity torque, clamped to the motor's peak.

        Changes the energized motor's torque at once; for calibration tools only.
        """
        with self._hardware_lock:
            self._tuning_arm().set_feedforward(joint, float(tau_nm))

    @rpc
    def tuning_read(self) -> dict[str, JointSample]:
        """The latest reading of every joint, as published. For calibration tools only."""
        with self._hardware_lock:
            self._tuning_arm()
            latest = self._latest
        if latest is None:
            raise RuntimeError("the Damiao robot has not been read yet")
        ts, reading = latest
        return {
            name: JointSample(position=position, velocity=velocity, effort=effort, ts=ts)
            for name, position, velocity, effort in zip(
                self.joint_names, reading.position, reading.velocity, reading.effort, strict=True
            )
        }

    @rpc
    def tuning_command(self, targets: dict[str, float]) -> None:
        """Send position targets by joint name, exactly as a ``position_command`` would.

        The targets are checked against the joint limits and restart the
        deadman; stop sending and the arm holds. A refused command raises
        ``ValueError``. Nothing limits how far one command jumps, so step the
        targets at the rate the arm may move. For calibration tools only.
        """
        unknown = sorted(set(targets) - set(self.joint_names))
        if unknown:
            raise ValueError(f"not joints of this Damiao robot: {unknown}")
        values = {
            f"{self.config.source}/{name}/{Interface.POSITION.value}": float(value)
            for name, value in targets.items()
        }
        result = validate_command(values, self.limits)
        if isinstance(result, Rejected):
            raise ValueError(f"{result.reason}: {result.detail}")
        with self._hardware_lock:
            self._tuning_arm()
            if not self._connected:
                raise RuntimeError("the Damiao connection is not running")
        self.command(values)

    @rpc
    def tuning_apply_calibration(
        self, path: str | None = None
    ) -> dict[str, AppliedCalibration | None]:
        """Apply a calibration now and return what each arm runs, by arm id.

        ``None`` applies each arm's saved file, as "auto" does at start; "off"
        returns every arm to its URDF masses and bus gains; any other value is
        one file, applied to the arm it names, and a file that does not fit
        raises ``ArmCalibrationError`` with nothing changed. Changes the
        energized motors' gains and gravity torques at once; for calibration
        tools only.
        """
        source = "auto" if path is None else path
        with self._hardware_lock:
            arm = self._tuning_arm()
            files = self._read_calibrations(arm, source, strict=source not in ("auto", "off"))
            return self._apply_calibrations(arm, files, always=True)

    def _tuning_arm(self) -> DamiaoArm:
        """The connected arm, for a tuning call made under the hardware lock."""
        if self._fault is not None:
            raise RuntimeError(self._fault)
        if self._arm is None:
            raise RuntimeError("Damiao robot is disconnected")
        return self._arm

    def _arm_joints_by_id(self) -> dict[str, list[str]]:
        ids = self.robot.arm_ids()
        if not ids:
            return {}
        return {
            arm_id: [motor.name for motor in bus.arm]
            for arm_id, bus in zip(ids, self.robot.buses, strict=True)
        }

    def _robot_model_sha256(self) -> str:
        if self._model_hash is None:
            self._model_hash = model_sha256(self.robot.gravity_model)
        return self._model_hash

    def _read_calibrations(
        self, arm: DamiaoArm, source: str, *, strict: bool
    ) -> dict[str, _ArmFile | None]:
        """Each arm's calibration from ``source`` ("auto", "off" or a file), checked.

        A file that does not fit is logged as a warning and that arm gets its
        defaults, or, when ``strict``, raised.
        """
        arms = self._arm_joints_by_id()
        files: dict[str, _ArmFile | None] = dict.fromkeys(arms)
        if source == "off":
            logger.info(f"{self._label}: calibration is off, using URDF defaults for {list(arms)}")
            return files
        if source == "auto":
            if not arms:
                logger.debug(f"{self._label}: the robot has no name, so no calibration is applied")
            for arm_id, joints in arms.items():
                path = calibration_path(arm_id, CALIBRATION_ROOT)
                if not path.exists():
                    logger.info(f"No calibration for {arm_id}, using URDF defaults")
                    continue
                try:
                    calibration = load_calibration(
                        path,
                        expected_model_sha256=self._robot_model_sha256(),
                        expected_arm_id=arm_id,
                    )
                    files[arm_id] = self._check_calibration(arm, path, calibration, joints)
                except (ArmCalibrationError, ValueError) as error:
                    logger.warning(
                        f"Ignoring the calibration for {arm_id}, using URDF defaults: {error}"
                    )
            return files
        path = calibration_file(Path(source).expanduser())
        try:
            calibration = load_calibration(path, expected_model_sha256=self._robot_model_sha256())
            if calibration.arm_id not in arms:
                raise ArmCalibrationError(
                    f"The arm calibration at {path} is for arm {calibration.arm_id!r}; "
                    f"this robot's arms are {list(arms)}."
                )
            files[calibration.arm_id] = self._check_calibration(
                arm, path, calibration, arms[calibration.arm_id]
            )
        except (ArmCalibrationError, ValueError) as error:
            if strict:
                raise
            logger.warning(f"Ignoring the calibration {path}, using URDF defaults: {error}")
        for arm_id, found in files.items():
            if found is None:
                logger.info(f"No calibration for {arm_id}, using URDF defaults")
        return files

    def _check_calibration(
        self, arm: DamiaoArm, path: Path, calibration: ArmCalibration, joints: list[str]
    ) -> _ArmFile:
        """The file's gains and masses, once the arm would accept them for ``joints``' arm."""
        gains = {name: (item.kp, item.kd) for name, item in (calibration.gains or {}).items()}
        foreign = sorted(set(gains) - set(joints))
        if foreign:
            raise ArmCalibrationError(
                f"The arm calibration at {path} gives gains to joints of no arm "
                f"{calibration.arm_id!r} drives: {foreign}."
            )
        links = to_inertial_overrides(calibration)
        arm.check_calibration(gains, links)
        # TODO(calibration): calibration.friction_nm is read but not applied;
        # apply it through DamiaoArm.set_feedforward once friction compensation
        # (direction from velocity, a dead band near zero) is designed.
        return _ArmFile(path=path, calibration=calibration, gains=gains, links=links)

    def _apply_calibrations(
        self, arm: DamiaoArm, files: dict[str, _ArmFile | None], *, always: bool
    ) -> dict[str, AppliedCalibration | None]:
        """Put the files' gains and masses in place, log one line per arm, and record them.

        Without ``always`` and without a file, the arm is left untouched.
        """
        found = {arm_id: item for arm_id, item in files.items() if item is not None}
        if found or always:
            gains: dict[str, tuple[float, float]] = {}
            links: dict[str, LinkInertial] = {}
            for item in found.values():
                gains.update(item.gains)
                links.update(item.links)
            arm.apply_calibration(gains, links)
        applied: dict[str, AppliedCalibration | None] = dict.fromkeys(files)
        for arm_id, item in found.items():
            created = item.calibration.created
            applied[arm_id] = AppliedCalibration(
                path=str(item.path),
                created=created.isoformat(),
                gains=sorted(item.gains),
                links=sorted(item.links),
            )
            notes = "" if self.robot.gravity else " (gravity compensation is off, masses unused)"
            if item.calibration.friction_nm:
                notes += " (its friction is not applied yet)"
            logger.info(
                f"Calibration for {arm_id} from {item.path}, created "
                f"{created:%Y-%m-%d %H:%M} UTC: gains for {sorted(item.gains) or 'no joints'}, "
                f"masses for {sorted(item.links) or 'no links'}{notes}"
            )
        with self._lock:
            self._calibration = applied
        return applied

    def _on_position(self, msg: JointState) -> None:
        try:
            values = joint_state_to_values(msg, (Interface.POSITION,))
        except ValueError as error:
            self._report(str(error), rejection=True)
            return
        self.command(values)

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

    def _read_state(self) -> ArmReading:
        try:
            arm = self._arm
            if arm is None:
                raise RuntimeError("Damiao robot is disconnected")
            reading = arm.read()
            self._latest = (time.time(), reading)
            return reading
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

    def _read_loop(self) -> None:
        period = 1.0 / self.config.state_rate_hz
        while not self._stop_reading.is_set():
            started = time.monotonic()
            try:
                with self._hardware_lock:
                    if self._arm is None or not self._connected:
                        return
                    reading = self._read_state()
                if self._stop_reading.is_set():
                    return
                self.joint_state.publish(
                    JointState(
                        ts=time.time(),
                        frame_id=self._state_frame,
                        name=[f"{self.config.source}/{name}" for name in self.joint_names],
                        position=reading.position,
                        velocity=reading.velocity,
                        effort=reading.effort,
                    )
                )
            except Exception as error:
                with self._hardware_lock:
                    if self._fault is None:
                        self._feedback_failed(error)
                return
            self._stop_reading.wait(max(0.0, period - (time.monotonic() - started)))
