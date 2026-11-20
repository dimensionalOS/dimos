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

At connect it applies each arm's saved calibration file
(``DamiaoConnectionConfig.calibration``): the file's gains and fitted link
masses. A file that does not load or fit the robot makes connect raise.
``get_config`` reports how each arm is driven; with ``config_writes`` on
(calibration blueprints only) ``set_config`` changes gains live or saves them,
with fitted masses, to each arm's file. Every halt (the deadman, a failed
write, stop) undoes the live changes and holds the arm where it is.
"""

from __future__ import annotations

from abc import abstractmethod
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
from pathlib import Path
import threading
import time
from typing import Any, ClassVar
import uuid

from pydantic import Field, ValidationError, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.calibration._inertial import LinkInertial
from dimos.control.calibration.schema import (
    ARMS_STATE_DIR,
    AppliedFile,
    ArmCalibration,
    ArmCalibrationError,
    ArmConfigPatch,
    ArmConfigState,
    JointGains,
    RobotModelRef,
    calibration_file,
    calibration_path,
    calibration_text,
    check_arm_fits,
    load_calibration,
    merge_patch,
    save_calibration,
    to_inertial_overrides,
)
from dimos.control.calibration.spec import model_name, model_sha256
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule, Values
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import limits as limits_of, movable_joints
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.damiao.arm import (
    MIT_KD_MAX,
    MIT_KP_MAX,
    ArmReading,
    DamiaoArm,
    check_mit_gains,
)
from dimos.robot.manipulators.damiao.robot import DamiaoRobot
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

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
        calibration: Which calibration to apply at connect. "auto" applies each
            arm's saved file (``calibration_path(arm_id)``) when there is one;
            "off" applies none; anything else is the path of one calibration
            file, applied to the arm it names. A file that does not load or
            fit makes connect raise.
        config_writes: Serve ``set_config``; calibration blueprints only.
    """

    teach: bool = False
    state_rate_hz: float = Field(default=100.0, gt=0, allow_inf_nan=False)
    deadman_timeout_s: float = Field(default=0.1, gt=0, allow_inf_nan=False)
    calibration: str = Field(default="auto", min_length=1)
    config_writes: bool = False

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
    sha256: str


class DamiaoConnection(ConnectionModule):
    """Own one Damiao robot, its measured states and position targets.

    ``get_config`` and ``set_config`` read and change how each arm is held;
    see the module docstring.
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
        # Guarded by _hardware_lock: the calibration file each arm runs, None for none.
        self._applied: dict[str, _ArmFile | None] = {}
        self._model_sha256 = ""

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
        """Validate the deployed model and topology, open the buses and read the calibrations."""
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
        self._model_sha256 = model_sha256(self.robot.gravity_model)
        files: dict[str, _ArmFile | None] = dict.fromkeys(self._arm_joints())
        source = self.config.calibration
        expected: dict[Path, str | None] = {}
        if source == "auto":
            expected = {calibration_path(arm_id, CALIBRATION_ROOT): arm_id for arm_id in files}
            expected = {path: arm_id for path, arm_id in expected.items() if path.exists()}
        elif source != "off":
            expected = {calibration_file(Path(source).expanduser()): None}
        for path, arm_id in expected.items():
            try:
                calibration = load_calibration(
                    path, expected_model_sha256=self._model_sha256, expected_arm_id=arm_id
                )
                item = self._read_file(arm, calibration, path)
            except (ArmCalibrationError, ValueError) as error:
                raise ArmCalibrationError(
                    f"{error} Set calibration: off to run without it."
                ) from error
            files[item.calibration.arm_id] = item
        logger.info(
            f"{self._label}: calibration {source}; URDF defaults for "
            f"{[arm_id for arm_id, item in files.items() if item is None]}"
        )
        self._apply(arm, files)

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
    def get_config(self) -> dict[str, Any]:
        """How each arm is driven now, by arm id: ``ArmConfigState`` as a dict."""
        with self._hardware_lock:
            arm = self._live_arm()
            gains, floating = arm.get_gains(), arm.floating
            states = {}
            for arm_id, joints in self._arm_joints().items():
                item = self._applied.get(arm_id)
                gravity = None if item is None else item.calibration.gravity
                states[arm_id] = ArmConfigState(
                    gains={
                        joint: JointGains(kp=0.0, kd=floating[joint])
                        if joint in floating
                        else JointGains(kp=gains[joint][0], kd=gains[joint][1])
                        for joint in joints
                    },
                    floating=[joint for joint in joints if joint in floating],
                    links={} if gravity is None else dict(gravity.links),
                    file=None
                    if item is None
                    else AppliedFile(
                        path=str(item.path), sha256=item.sha256, created=item.calibration.created
                    ),
                    can_float=self.robot.gravity,
                ).model_dump(mode="json")
            return states

    @rpc
    def set_config(self, patch: dict[str, Any], save: bool = False) -> dict[str, Any]:
        """Change how arms are held, all or nothing; returns ``get_config()``.

        ``patch`` maps arm ids to ``ArmConfigPatch`` dicts. Live (``save``
        False) it takes gains only: kp 0 floats the joint with kd damping
        (needs gravity compensation), kp above 0 holds it with those gains,
        until the next halt. Saved, its gains (kp above 0), links, fit and
        notes are merged into each arm's calibration file, which is then
        applied, dropping every live change. Any bad entry raises
        ``ValueError`` naming every bad entry, and nothing changes.
        """
        if not self.config.config_writes:
            raise RuntimeError(
                f"{self._label} does not take set_config: config_writes is off; "
                "the calibration blueprints (<robot>-calibrate) turn it on"
            )
        with self._hardware_lock:
            arm = self._live_arm()
            arms = self._arm_joints()
            errors: list[str] = []
            patches: dict[str, ArmConfigPatch] = {}
            for arm_id, raw in patch.items():
                if arm_id not in arms:
                    errors.append(f"{arm_id}: not an arm of this robot; its arms are {list(arms)}")
                    continue
                try:
                    patches[arm_id] = ArmConfigPatch.model_validate(raw, strict=True)
                except ValidationError as error:
                    errors += [
                        f"{arm_id}.{'.'.join(map(str, item['loc']))}: {item['msg']}"
                        for item in error.errors()
                    ]
                    continue
                for joint, gains in (patches[arm_id].gains or {}).items():
                    name = f"{arm_id}.gains.{joint}"
                    if joint not in arms[arm_id]:
                        errors.append(f"{name}: not a joint of {arm_id}; it has {arms[arm_id]}")
                    elif gains.kp == 0.0 and save:
                        errors.append(f"{name}: kp 0 floats a joint, which is live only")
                    elif gains.kp == 0.0 and not self.robot.gravity:
                        errors.append(f"{name}: kp 0 needs gravity compensation, which is off")
                    else:
                        try:
                            check_mit_gains(joint, gains.kp, gains.kd)
                        except ValueError as error:
                            errors.append(f"{name}: {error}")
                if not save:
                    errors += [
                        f"{arm_id}.{key}: saved only (save=True); live takes gains"
                        for key in patches[arm_id].model_dump(exclude_none=True)
                        if key != "gains"
                    ]
            files = dict(self._applied)
            now = datetime.now(timezone.utc)
            for arm_id, arm_patch in patches.items() if save and not errors else ():
                path = calibration_path(arm_id, CALIBRATION_ROOT)
                try:
                    stored = (
                        load_calibration(
                            path,
                            expected_model_sha256=self._model_sha256,
                            expected_arm_id=arm_id,
                        )
                        if path.exists()
                        else ArmCalibration(
                            arm_id=arm_id,
                            robot_model=RobotModelRef(
                                name=model_name(self.robot.gravity_model),
                                sha256=self._model_sha256,
                            ),
                            created=now,
                        )
                    )
                    files[arm_id] = self._read_file(
                        arm, merge_patch(stored, arm_patch, created=now), path
                    )
                except (ArmCalibrationError, ValueError) as error:
                    errors.append(f"{arm_id}: {error}")
            if errors:
                raise ValueError("set_config changed nothing: " + "; ".join(errors))
            if save:
                for arm_id in patches:
                    item = files[arm_id]
                    assert item is not None
                    save_calibration(item.path, item.calibration)
                self._apply(arm, files)
            else:
                for arm_patch in patches.values():
                    for joint, gains in (arm_patch.gains or {}).items():
                        if gains.kp == 0.0:
                            arm.set_float(joint, gains.kd)
                        else:
                            arm.set_gains(joint, gains.kp, gains.kd)
                # Live changes last only while commands stream: the deadman undoes them.
                with self._lock:
                    self._command_seq += 1
                    seq = self._command_seq
                self._on_loop(lambda: self._arm_deadman(seq))
        return self.get_config()

    def _live_arm(self) -> DamiaoArm:
        """The connected arm; call under the hardware lock."""
        if self._fault is not None:
            raise RuntimeError(self._fault)
        if self._arm is None or not self._connected:
            raise RuntimeError("Damiao robot is disconnected")
        return self._arm

    def _arm_joints(self) -> dict[str, list[str]]:
        """Each arm id's joints; none for an unnamed robot."""
        return {
            arm_id: [motor.name for motor in bus.arm]
            for arm_id, bus in zip(self.robot.arm_ids(), self.robot.buses, strict=False)
        }

    def _read_file(self, arm: DamiaoArm, calibration: ArmCalibration, path: Path) -> _ArmFile:
        """``calibration``, once it fits this robot's model, one of its arms and its motors.

        Every gain must be for one of the arm's joints, inside the MIT range
        with kp above 0, and every mass for a link they move; the masses must
        give finite torques, none above a motor's rated torque at the measured
        pose or at home.
        """
        arms = self._arm_joints()
        joints = arms.get(calibration.arm_id)
        if joints is None:
            raise ArmCalibrationError(
                f"The arm calibration at {path} is for arm {calibration.arm_id!r}; "
                f"this robot's arms are {list(arms)}."
            )
        moved = {
            joint.child_link
            for joint in self.robot.gravity_model.load().joints
            if joint.name in joints
        }
        check_arm_fits(calibration, path, joints, moved, kp_max=MIT_KP_MAX, kd_max=MIT_KD_MAX)
        gains = {name: (item.kp, item.kd) for name, item in (calibration.gains or {}).items()}
        links = to_inertial_overrides(calibration)
        arm.check_calibration(gains, links)
        rated = arm.rated_torques
        home = self.robot.home_joints
        for label, pose in [("the measured pose", None), *([("home", home)] if home else [])]:
            torques = arm.gravity(links, pose)
            for joint in joints:
                if abs(torques[joint]) > rated[joint]:
                    raise ArmCalibrationError(
                        f"The arm calibration at {path} needs {torques[joint]:+.2f} N m of "
                        f"gravity torque on {joint} at {label}, more than its motor's rated "
                        f"{rated[joint]:g} N m; its masses cannot be right."
                    )
        sha256 = hashlib.sha256(calibration_text(calibration).encode("utf-8")).hexdigest()
        return _ArmFile(path=path, calibration=calibration, gains=gains, links=links, sha256=sha256)

    def _apply(self, arm: DamiaoArm, files: Mapping[str, _ArmFile | None]) -> None:
        """Run every file's gains and masses (arms share no joints or links) and record them."""
        gains: dict[str, tuple[float, float]] = {}
        links: dict[str, LinkInertial] = {}
        for item in files.values():
            if item is not None:
                gains.update(item.gains)
                links.update(item.links)
                logger.info(
                    f"Calibration for {item.calibration.arm_id} from {item.path}, created "
                    f"{item.calibration.created:%Y-%m-%d %H:%M} UTC: gains for "
                    f"{sorted(item.gains) or 'no joints'}, masses for {sorted(item.links) or 'no links'}"
                )
        arm.apply_calibration(gains, links)
        self._applied = dict(files)

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
        """Undo the live changes and hold every joint where it is (``DamiaoArm.hold``)."""
        if self._fault is not None or self._arm is None:
            return
        self._arm.hold()

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
