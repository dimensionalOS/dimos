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
masses. Without a file the arm runs exactly its URDF and bus defaults.

Every connection serves the arm config contract of
:mod:`dimos.control.connection.arm_config`. One deployed with
``config_sessions=True`` (calibration blueprints only) lets one calibration
tool own the arm in a config session and change its gains, float its joints
(kp 0) and add torque offsets live, and save calibrations to each arm's file
(scope "persist"). Gains are those of a Damiao MIT frame: kp 0 to 500 N m/rad
and kd 0 to 5 N m s/rad, the range of the frame's fields, so a larger gain is
clamped to them. Stopping such a connection first gives the calibration tool,
stopping at the same time, up to ``CONFIG_PARK_WAIT_S`` to park the arm and
end its session (:meth:`DamiaoConnection.stop`): the modules of a run stop in
parallel, so nothing else orders the two.
"""

from __future__ import annotations

from abc import abstractmethod
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
from pathlib import Path
import threading
import time
from typing import Any, ClassVar
import uuid

from pydantic import Field, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.arm_config import (
    AppliedFile,
    ArmConfigState,
    ConfigApplied,
    ConfigKeys,
    ConfigPatch,
    ConfigScope,
    JointGains,
)
from dimos.control.connection.connection_module import (
    ConnectionConfig,
    ConnectionModule,
    Values,
)
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import limits as limits_of, movable_joints
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.calibration._inertial import LinkInertial
from dimos.robot.manipulators.calibration.schema import (
    ARMS_STATE_DIR,
    ArmCalibration,
    ArmCalibrationError,
    RobotModelRef,
    calibration_file,
    calibration_path,
    calibration_text,
    load_calibration,
    merge_patch,
    save_calibration,
    to_inertial_overrides,
)
from dimos.robot.manipulators.calibration.spec import model_name, model_sha256
from dimos.robot.manipulators.damiao.arm import (
    MIT_KD_MAX,
    MIT_KP_MAX,
    ArmReading,
    DamiaoArm,
    TuningReset,
    _mit_gains,
)
from dimos.robot.manipulators.damiao.robot import DamiaoRobot
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

__all__ = [
    "CALIBRATION_ROOT",
    "MAX_GRAVITY_STEP_NM",
    "DamiaoConnection",
    "DamiaoConnectionConfig",
    "zero_stiffness_joints",
]

#: Where "auto" looks for each arm's calibration, ``<root>/<arm_id>/calibration.json``.
#: Read at every load, so tests can point it at a scratch directory.
CALIBRATION_ROOT: Path = ARMS_STATE_DIR
#: The most any joint's gravity torque at the measured pose may change when a
#: saved calibration is applied live without ``force``, N m.
MAX_GRAVITY_STEP_NM = 2.0
#: Why a session does not take what only a calibration file holds.
_FILE_ONLY = "saved with scope 'persist' only; a session changes gains, floating and torque offsets"


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
            A calibration saved through ``config_set`` always goes to
            ``calibration_path(arm_id)``, whatever this says.
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
    sha256: str

    def applied_file(self) -> AppliedFile:
        return AppliedFile(
            path=str(self.path), created=self.calibration.created.isoformat(), sha256=self.sha256
        )


def _merged(
    files: Mapping[str, _ArmFile | None],
) -> tuple[dict[str, tuple[float, float]], dict[str, LinkInertial]]:
    """Every file's gains and link masses together; arms never share joints or links."""
    gains: dict[str, tuple[float, float]] = {}
    links: dict[str, LinkInertial] = {}
    for item in files.values():
        if item is not None:
            gains.update(item.gains)
            links.update(item.links)
    return gains, links


def _undone(reset: TuningReset) -> str:
    parts = []
    if reset.released:
        parts.append(f"held {reset.released} where they are")
    if reset.feedforward:
        parts.append(f"dropped the feed-forward of {reset.feedforward}")
    if reset.gains:
        parts.append(f"gains of {reset.gains} back to the calibration")
    return f"{'; '.join(parts) or 'nothing to undo'}; every joint holds where it is"


class DamiaoConnection(ConnectionModule):
    """Own one Damiao robot, its measured states and position targets.

    It serves the arm config contract in full (see ``ConnectionModule``): a
    config session changes gains (kp 0 floats a joint, gravity still held)
    and torque offsets of the energized arm live, and a save merges into each
    arm's calibration file and applies it. Calibration tools only, and only
    with ``config.config_sessions``.
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
        # Guarded by _lock: the calibration file each arm runs, None for none.
        self._applied: dict[str, _ArmFile | None] = {}
        self._model_hash: str | None = None

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
        # Before the reader stops: a calibration tool may still be parking the arm.
        self._await_config_park()
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
                    self._end_config_session("the connection stopped")
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

    def _config_keys(self) -> ConfigKeys:
        """Each arm's joints, and what a session and a save change; "float" needs gravity."""
        arms = self._arm_joints_by_id()
        gravity = self.robot.gravity
        return ConfigKeys(
            source=self.config.source,
            sessions=self.config.config_sessions,
            arms=arms,
            readable=["gains", "links", "torque_offset_nm"],
            session=["gains", *(["float"] if gravity else []), "torque_offset_nm"],
            persist=["gains", "links", "fit", "friction_nm", "checks", "notes"],
            persist_paths={
                arm_id: str(calibration_path(arm_id, CALIBRATION_ROOT)) for arm_id in arms
            },
            gravity_compensation=gravity,
        )

    def _config_state(self) -> dict[str, ArmConfigState]:
        """How each arm is driven now: its gains, floating joints, offsets and file."""
        arm = self._live_arm()
        gains, floating, offsets = arm.get_gains(), arm.floating, arm.feedforward
        with self._lock:
            applied = dict(self._applied)
        states: dict[str, ArmConfigState] = {}
        for arm_id, joints in self._arm_joints_by_id().items():
            item = applied.get(arm_id)
            gravity = None if item is None else item.calibration.gravity
            states[arm_id] = ArmConfigState(
                arm_id=arm_id,
                gains={
                    joint: JointGains(kp=gains[joint][0], kd=gains[joint][1]) for joint in joints
                },
                floating={joint: floating[joint] for joint in joints if joint in floating},
                torque_offset_nm={joint: offsets[joint] for joint in joints if offsets.get(joint)},
                links={} if gravity is None else dict(gravity.links),
                file=None if item is None else item.applied_file(),
            )
        return states

    def _config_apply(
        self, patch: ConfigPatch, scope: ConfigScope, force: bool, apply_now: bool
    ) -> ConfigApplied:
        """Change one arm live (session) or save its calibration file and apply it (persist).

        Live, gains are clamped to the MIT frame's range; kp 0 floats the
        joint with kd damping (``DamiaoArm.set_float``), refused without
        gravity compensation; kp above 0 holds a floating joint again where it
        is, then with those gains. A torque offset is clamped to a fifth of
        the motor's rated torque. Links, fit, friction, checks and notes are
        for a save only. See ``_save`` for a save.
        """
        arm = self._live_arm()
        arms = self._arm_joints_by_id()
        joints = arms.get(patch.arm_id)
        if joints is None:
            raise ValueError(
                f"{patch.arm_id!r} is not an arm of this Damiao robot; its arms are {list(arms)}"
            )
        named = set(patch.gains or {}) | set(patch.torque_offset_nm or {})
        strangers = sorted(named - set(joints))
        if strangers:
            raise ValueError(
                f"not arm joints of {patch.arm_id}: {strangers}; its joints are {joints}"
            )
        if scope == "persist":
            return self._save(arm, patch, joints, force=force, apply_now=apply_now)
        unsupported: dict[str, str] = {}
        file_only = patch.model_copy(update={"gains": None, "torque_offset_nm": None})
        unsupported.update(dict.fromkeys(file_only.keys(), _FILE_ONLY))
        gains: dict[str, JointGains] = {}
        clamped: list[str] = []
        for joint, asked in (patch.gains or {}).items():
            keys = [f"gains.{joint}.kp", f"gains.{joint}.kd"]
            if asked.kp == 0.0:
                if not self.robot.gravity:
                    unsupported.update(
                        dict.fromkeys(
                            keys,
                            "this robot does not compensate gravity, so a joint at kp 0 would fall",
                        )
                    )
                    continue
                arm.set_float(joint, True, asked.kd)
                gains[joint] = JointGains(kp=0.0, kd=arm.floating[joint])
            else:
                if joint in arm.floating:
                    arm.set_float(joint, False)
                kp, kd = arm.set_gains(joint, asked.kp, asked.kd)
                gains[joint] = JointGains(kp=kp, kd=kd)
            clamped += [
                key
                for key, want, got in zip(
                    keys, (asked.kp, asked.kd), (gains[joint].kp, gains[joint].kd), strict=True
                )
                if want != got
            ]
        offsets: dict[str, float] = {}
        for joint, tau in (patch.torque_offset_nm or {}).items():
            offsets[joint] = arm.set_feedforward(joint, tau)
            if offsets[joint] != tau:
                clamped.append(f"torque_offset_nm.{joint}")
        return ConfigApplied(
            arm_id=patch.arm_id,
            scope="session",
            applied=ConfigPatch(
                arm_id=patch.arm_id, gains=gains or None, torque_offset_nm=offsets or None
            ),
            clamped=clamped,
            unsupported=list(unsupported),
            why=unsupported,
        )

    def _save(
        self,
        arm: DamiaoArm,
        patch: ConfigPatch,
        joints: list[str],
        *,
        force: bool,
        apply_now: bool,
    ) -> ConfigApplied:
        """Merge ``patch`` into the arm's calibration file, check it, write it and apply it.

        The file is always ``calibration_path(arm_id, CALIBRATION_ROOT)``.
        Torque offsets and kp 0 are live changes only and are left out. Gains
        are clamped to the MIT frame's range before they are merged, so the
        file holds, and ``applied`` reports, what the arm runs. The merged
        calibration is checked as a file read at start is; applying
        it must not step any joint's gravity torque at the measured pose by
        more than ``MAX_GRAVITY_STEP_NM`` unless ``force``, else nothing is
        written. ``apply_now`` False only writes the file. A save without
        gains or links changes nothing the arm runs, so it is not applied; one
        with nothing a file holds writes nothing.
        """
        arm_id = patch.arm_id
        unsupported: dict[str, str] = dict.fromkeys(
            [key for key in patch.keys() if key.startswith("torque_offset_nm")],
            "a torque offset is a live change only (scope 'session'); a calibration file "
            "does not hold it",
        )
        floats = [joint for joint, gains in (patch.gains or {}).items() if gains.kp == 0.0]
        unsupported.update(
            dict.fromkeys(
                [f"gains.{joint}.{gain}" for joint in floats for gain in ("kp", "kd")],
                "kp 0 floats a joint, which only a session does; a calibration holds every joint",
            )
        )
        kept: dict[str, JointGains] = {}
        clamped: list[str] = []
        for joint, asked in (patch.gains or {}).items():
            if asked.kp == 0.0:
                continue
            (kp, kd), _ = _mit_gains(joint, asked.kp, asked.kd)
            kept[joint] = JointGains(kp=kp, kd=kd)
            clamped += [f"gains.{joint}.kp"] if kp != asked.kp else []
            clamped += [f"gains.{joint}.kd"] if kd != asked.kd else []
        saved = patch.model_copy(update={"gains": kept or None, "torque_offset_nm": None})
        if not saved.keys():
            # Nothing a file holds: the file and its history stay as they are.
            return ConfigApplied(
                arm_id=arm_id,
                scope="persist",
                applied=ConfigPatch(arm_id=arm_id),
                unsupported=list(unsupported),
                why=unsupported,
                applied_now=False,
            )
        path = calibration_path(arm_id, CALIBRATION_ROOT)
        now = datetime.now(timezone.utc)
        merged = merge_patch(self._stored(path, arm_id, now), saved, created=now)
        sha256 = hashlib.sha256(calibration_text(merged).encode("utf-8")).hexdigest()
        item = self._check_calibration(arm, path, merged, joints, sha256)
        with self._lock:
            running = dict(self._applied)
        files = {**running, arm_id: item}
        live = apply_now and (saved.gains is not None or saved.links is not None)
        step: dict[str, float] = {}
        if live:
            _, links = _merged(files)
            before, after = arm.gravity(), arm.gravity(links)
            step = {joint: after[joint] - before[joint] for joint in before}
            large = {
                joint: round(value, 2)
                for joint, value in step.items()
                if abs(value) > MAX_GRAVITY_STEP_NM
            }
            if large and not force:
                return ConfigApplied(
                    arm_id=arm_id,
                    scope="persist",
                    applied=ConfigPatch(arm_id=arm_id),
                    unsupported=list(unsupported),
                    why=unsupported,
                    applied_now=False,
                    refused=(
                        f"applying it would change the gravity torque at this pose by more than "
                        f"{MAX_GRAVITY_STEP_NM:g} N m: {large} (N m), so nothing was written; "
                        "save calibrations near home, or pass force=True"
                    ),
                    gravity_step_nm=step,
                )
        target = save_calibration(path, merged)
        if live:
            self._apply_calibrations(arm, files, always=True)
        else:
            current = running.get(arm_id)
            if (
                current is not None
                and current.path == item.path
                and (current.gains, current.links) == (item.gains, item.links)
            ):
                # The arm runs what the file now holds; only its record is new.
                with self._lock:
                    self._applied[arm_id] = item
        return ConfigApplied(
            arm_id=arm_id,
            scope="persist",
            applied=saved,
            clamped=clamped,
            unsupported=list(unsupported),
            why=unsupported,
            path=str(target),
            applied_now=live,
            gravity_step_nm=step,
        )

    def _stored(self, path: Path, arm_id: str, now: datetime) -> ArmCalibration:
        """The arm's saved calibration, or an empty one for this model if none fits."""
        if path.exists():
            try:
                return load_calibration(
                    path, expected_model_sha256=self._robot_model_sha256(), expected_arm_id=arm_id
                )
            except ArmCalibrationError as error:
                logger.warning(
                    f"{self._label}: saving {arm_id}'s calibration afresh; the saved one "
                    f"goes to history/: {error}"
                )
        return ArmCalibration(
            arm_id=arm_id,
            robot_model=RobotModelRef(
                name=model_name(self.robot.gravity_model), sha256=self._robot_model_sha256()
            ),
            created=now,
        )

    def _config_undo(self) -> str:
        """Drop offsets, gain overrides and floating, and hold every joint where it is."""
        arm = self._arm
        if arm is None:
            return "nothing undone"
        return _undone(arm.reset_tuning())

    def _config_parkable(self) -> bool:
        return self._connected and self._fault is None and self._arm is not None

    def _live_arm(self) -> DamiaoArm:
        """The connected arm, for a config hook called under the hardware lock."""
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
        self,
        arm: DamiaoArm,
        source: str,
        *,
        strict: bool,
        keep: Mapping[str, _ArmFile | None] | None = None,
    ) -> dict[str, _ArmFile | None]:
        """Each arm's calibration from ``source`` ("auto", "off" or a file), checked.

        A file that does not fit is logged as a warning and that arm gets its
        defaults, or, when ``strict``, raised. With one file, the arms it does
        not name get what ``keep`` gives them, else their defaults.
        """
        arms = self._arm_joints_by_id()
        files: dict[str, _ArmFile | None] = dict.fromkeys(arms)
        if source == "off":
            logger.info(f"{self._label}: calibration is off, using URDF defaults for {list(arms)}")
            return files
        if source == "auto":
            if not arms:
                logger.debug(f"{self._label}: the robot has no name, so no calibration is applied")
            for arm_id in arms:
                path = calibration_path(arm_id, CALIBRATION_ROOT)
                if not path.exists():
                    logger.info(f"No calibration for {arm_id}, using URDF defaults")
                    continue
                try:
                    files[arm_id] = self._read_file(arm, path, arm_id)
                except (ArmCalibrationError, ValueError) as error:
                    logger.warning(
                        f"Ignoring the calibration for {arm_id}, using URDF defaults: {error}"
                    )
            return files
        files.update({arm_id: item for arm_id, item in (keep or {}).items() if arm_id in files})
        path = calibration_file(Path(source).expanduser())
        try:
            item = self._read_file(arm, path)
            files[item.calibration.arm_id] = item
        except (ArmCalibrationError, ValueError) as error:
            if strict:
                raise
            logger.warning(f"Ignoring the calibration {path}, using URDF defaults: {error}")
        for arm_id, found in files.items():
            if found is None:
                logger.info(f"No calibration for {arm_id}, using URDF defaults")
        return files

    def _read_file(self, arm: DamiaoArm, path: Path, arm_id: str | None = None) -> _ArmFile:
        """One calibration file, refused unless made for this model and one of its arms."""
        calibration = load_calibration(
            path, expected_model_sha256=self._robot_model_sha256(), expected_arm_id=arm_id
        )
        sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        arms = self._arm_joints_by_id()
        if calibration.arm_id not in arms:
            raise ArmCalibrationError(
                f"The arm calibration at {path} is for arm {calibration.arm_id!r}; "
                f"this robot's arms are {list(arms)}."
            )
        return self._check_calibration(arm, path, calibration, arms[calibration.arm_id], sha256)

    def _check_calibration(
        self,
        arm: DamiaoArm,
        path: Path,
        calibration: ArmCalibration,
        joints: list[str],
        sha256: str,
    ) -> _ArmFile:
        """The file's gains and masses, once they are safe for the arm that ``joints`` move.

        Every gain must be for one of ``joints``, inside the MIT frame's
        range with kp above 0 (a file holds every joint; kp 0 would float
        it), and every mass for a link they move; the masses must give
        finite torques, and none above a motor's rated torque at the
        measured pose or at home.
        """
        gains = {name: (item.kp, item.kd) for name, item in (calibration.gains or {}).items()}
        foreign = sorted(set(gains) - set(joints))
        if foreign:
            raise ArmCalibrationError(
                f"The arm calibration at {path} gives gains to joints of no arm "
                f"{calibration.arm_id!r} drives: {foreign}."
            )
        outside = {
            joint: {"kp": kp, "kd": kd}
            for joint, (kp, kd) in gains.items()
            if not (0.0 < kp <= MIT_KP_MAX and 0.0 <= kd <= MIT_KD_MAX)
        }
        if outside:
            raise ArmCalibrationError(
                f"The arm calibration at {path} gives gains outside 0 < kp <= {MIT_KP_MAX:g} "
                f"and 0 <= kd <= {MIT_KD_MAX:g}: {outside}."
            )
        links = to_inertial_overrides(calibration)
        moved = self._links_moved_by(joints)
        strangers = sorted(set(links) - moved)
        if strangers:
            raise ArmCalibrationError(
                f"The arm calibration at {path} gives masses to links arm "
                f"{calibration.arm_id!r} does not move: {strangers}; it moves {sorted(moved)}."
            )
        arm.check_calibration(gains, links)
        self._check_rated_torques(arm, path, links, joints)
        # TODO(calibration): calibration.friction_nm is read but not applied;
        # apply it through DamiaoArm.set_feedforward once friction compensation
        # (direction from velocity, a dead band near zero) is designed.
        return _ArmFile(path=path, calibration=calibration, gains=gains, links=links, sha256=sha256)

    def _links_moved_by(self, joints: Sequence[str]) -> set[str]:
        """The links ``joints`` move directly: each one's child link in the gravity model."""
        names = set(joints)
        return {
            joint.child_link
            for joint in self.robot.gravity_model.load().joints
            if joint.name in names
        }

    def _check_rated_torques(
        self, arm: DamiaoArm, path: Path, links: Mapping[str, LinkInertial], joints: list[str]
    ) -> None:
        """Refuse masses that need more than a motor's rated torque, measured or at home."""
        rated = arm.rated_torques
        poses: list[tuple[str, Sequence[float] | None]] = [("the measured pose", None)]
        if self.robot.home_joints is not None:
            poses.append(("home", self.robot.home_joints))
        for label, pose in poses:
            torques = arm.gravity(links, pose)
            for joint in joints:
                if abs(torques[joint]) > rated[joint]:
                    raise ArmCalibrationError(
                        f"The arm calibration at {path} needs {torques[joint]:+.2f} N m of "
                        f"gravity torque on {joint} at {label}, more than its motor's rated "
                        f"{rated[joint]:g} N m; its masses cannot be right."
                    )

    def _apply_calibrations(
        self, arm: DamiaoArm, files: dict[str, _ArmFile | None], *, always: bool
    ) -> None:
        """Put the files' gains and masses in place, log one line per arm, and record them.

        Without ``always`` and without a file, the arm is left untouched.
        """
        found = {arm_id: item for arm_id, item in files.items() if item is not None}
        if found or always:
            arm.apply_calibration(*_merged(files))
        for arm_id, item in found.items():
            created = item.calibration.created
            notes = "" if self.robot.gravity else " (gravity compensation is off, masses unused)"
            if item.calibration.friction_nm:
                notes += " (its friction is not applied yet)"
            logger.info(
                f"Calibration for {arm_id} from {item.path}, created "
                f"{created:%Y-%m-%d %H:%M} UTC: gains for {sorted(item.gains) or 'no joints'}, "
                f"masses for {sorted(item.links) or 'no links'}{notes}"
            )
        with self._lock:
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
        # The config session already ended (ConnectionModule._call_halt).
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
        # Disabled first, so ending it sends nothing to the failed buses.
        self._end_config_session(self._fault)

    def _read_loop(self) -> None:
        period = 1.0 / self.config.state_rate_hz
        while not self._stop_reading.is_set():
            started = time.monotonic()
            try:
                with self._hardware_lock:
                    if self._arm is None or not self._connected:
                        return
                    # Besides the lease timer, which needs the module's loop.
                    self._end_config_session("no config call within its lease", lapsed=True)
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
