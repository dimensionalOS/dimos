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

"""One calibration file per arm: fitted gravity, gains, friction and checks.

The file lives at ``calibration_path(arm_id)``, i.e.
``STATE_DIR/arms/<arm_id>/calibration.json``. ``save_calibration`` replaces
it atomically and keeps the file it replaces in ``history/``. It records the
fingerprint of the robot model it was made against (``ArmCalibrationSpec.
model_sha256``), and ``load_calibration`` refuses a file made for a different
model, since the fitted masses are corrections to that model.

``ArmConfigPatch`` and ``ArmConfigState`` are one arm's entry in a Damiao
connection's ``set_config`` and ``get_config``. Joints are named as in the
robot's model, without the connection's source prefix, e.g. "left_joint2";
kp is in N m/rad and kd in N m s/rad.

A fitted link's mass and centre of mass are those of the link's pinocchio body:
the link plus the links merged into it (``merged_links``: fixed children and
locked fingers), centre of mass in the link frame. ``bake_into_urdf`` writes
them back into a URDF by adjusting only the named link, so the merged links
keep their own URDF values and the sum matches the fit. It bakes only into the
URDF the calibration was made against.
"""

from __future__ import annotations

from collections.abc import Collection, Mapping, Sequence
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Annotated, Any, Literal
import xml.etree.ElementTree as ET

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

import numpy as np
from pydantic import BeforeValidator, Field, ValidationError, field_validator

from dimos.constants import STATE_DIR
from dimos.control.calibration._inertial import LinkInertial
from dimos.control.calibration.spec import (
    ArmCalibrationSpec,
    merged_links,
    urdf_sha256,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.robot.assets.model import RobotModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

SCHEMA_VERSION: Literal[1] = 1
CALIBRATION_FILENAME = "calibration.json"
HISTORY_DIR_NAME = "history"
ARMS_STATE_DIR = STATE_DIR / "arms"

PositiveFloat = Annotated[float, Field(gt=0.0, allow_inf_nan=False)]
NonNegativeFloat = Annotated[float, Field(ge=0.0, allow_inf_nan=False)]
FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]


class JointGains(BaseConfig):
    """How stiffly one joint is held: kp in N m/rad, kd in N m s/rad."""

    kp: NonNegativeFloat
    kd: NonNegativeFloat


def _tuple_of(value: Any) -> Any:
    """A JSON array as a tuple, so strict validation takes it; anything else as given."""
    return tuple(value) if isinstance(value, list) else value


class LinkGravity(BaseConfig):
    """Fitted mass and centre of mass of one link's body, centre of mass in the link frame.

    Attributes:
        mass_kg: The body's mass, including ``merged_links``.
        com_m: The body's centre of mass, link frame, metres.
        merged_links: Links whose URDF inertia the body includes.
    """

    mass_kg: PositiveFloat
    com_m: Annotated[tuple[FiniteFloat, FiniteFloat, FiniteFloat], BeforeValidator(_tuple_of)]
    merged_links: list[str] = Field(default_factory=list)


class ArmConfigPatch(BaseConfig):
    """One arm's entry in ``set_config``; only what is given changes.

    Attributes:
        gains: By joint, merged per joint. Live, kp 0 floats the joint with kd
            damping; saved, kp must be above 0.
        links: Fitted link masses; replace the stored set as a whole (save only).
        fit: The fit's statistics, stored with the links (save only).
        friction_nm: Coulomb friction by joint, N m; replaces the stored set
            (save only).
        checks: Validation results by name, e.g. "float", "tracking"; merged
            by name (save only).
        notes: Free text for the operator (save only).
    """

    gains: dict[str, JointGains] | None = None
    links: dict[str, LinkGravity] | None = None
    fit: dict[str, Any] | None = None
    friction_nm: dict[str, NonNegativeFloat] | None = None
    checks: dict[str, Any] | None = None
    notes: str | None = None


class AppliedFile(BaseConfig):
    """The calibration file an arm runs, as it was when applied."""

    path: str
    sha256: str
    created: datetime


class ArmConfigState(BaseConfig):
    """One arm's entry in ``get_config``: how it is driven now.

    Attributes:
        gains: Every arm joint's gains as sent: kp 0 and the damping for a
            floating joint, else a live override, else the file's, else the
            robot's defaults.
        floating: The joints at kp 0.
        links: The fitted link masses the arm runs; empty for the URDF's.
        file: The calibration file it runs, or ``None`` for its defaults.
        can_float: Whether the robot compensates gravity, which floating needs.
        deadman_timeout_s: How long the connection waits for a command before
            it holds the arm and undoes the live changes.
    """

    gains: dict[str, JointGains]
    floating: list[str]
    links: dict[str, LinkGravity]
    file: AppliedFile | None
    can_float: bool
    deadman_timeout_s: float


class ArmCalibrationError(RuntimeError):
    """A calibration file is missing, unreadable, or made for something else."""


class RobotModelRef(BaseConfig):
    """The robot model a calibration was made against.

    Attributes:
        name: The URDF's robot name.
        sha256: ``ArmCalibrationSpec.model_sha256()`` of that model.
    """

    name: str
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")


class GravityCalibration(BaseConfig):
    """The fitted links and the fit's statistics (see ``gravity_fit``)."""

    links: dict[str, LinkGravity]
    fit: dict[str, Any] = Field(default_factory=dict)


class ArmCalibration(BaseConfig):
    """One arm's calibration file.

    Attributes:
        schema_version: The file format; this build reads version 1.
        arm_id: The arm, e.g. "dual_openyam/left".
        robot_model: The model the calibration was made against.
        created: When the file was written, UTC.
        gravity: Fitted link masses and centres of mass, if identified.
        gains: Position and velocity gains per joint, if tuned.
        friction_nm: Coulomb friction per joint, N m, if measured.
        checks: Results of the validation steps by name (float drift, tracking).
        notes: Free text for the operator.
    """

    schema_version: Literal[1] = SCHEMA_VERSION
    arm_id: str
    robot_model: RobotModelRef
    created: datetime
    gravity: GravityCalibration | None = None
    gains: dict[str, JointGains] | None = None
    friction_nm: dict[str, NonNegativeFloat] | None = None
    checks: dict[str, Any] | None = None
    notes: str | None = None

    @field_validator("created")
    @classmethod
    def _utc(cls, value: datetime) -> datetime:
        if value.tzinfo is None:
            raise ValueError("created must carry a time zone")
        return value.astimezone(timezone.utc)

    @classmethod
    def for_spec(
        cls, spec: ArmCalibrationSpec, *, created: datetime | None = None, **fields: Any
    ) -> Self:
        """An empty calibration for ``spec``'s arm and model, created now unless given."""
        return cls(
            arm_id=spec.arm_id,
            robot_model=RobotModelRef(name=spec.model_name(), sha256=spec.model_sha256()),
            created=created or datetime.now(timezone.utc),
            **fields,
        )


def calibration_path(arm_id: str, root: Path = ARMS_STATE_DIR) -> Path:
    """``STATE_DIR/arms/<arm_id>/calibration.json``."""
    if not arm_id or arm_id.startswith("/") or ".." in arm_id.split("/"):
        raise ValueError(f"arm_id must be a relative path of plain names, got {arm_id!r}")
    return root / arm_id / CALIBRATION_FILENAME


def calibration_file(path: Path) -> Path:
    """A calibration file, or the directory holding ``calibration.json``."""
    return path if path.suffix == ".json" else path / CALIBRATION_FILENAME


def calibration_text(calibration: ArmCalibration) -> str:
    """The exact text ``save_calibration`` writes, UTF-8, so its sha256 is the file's."""
    return calibration.model_dump_json(indent=2) + "\n"


def save_calibration(path: Path, calibration: ArmCalibration) -> Path:
    """Write ``calibration`` atomically, keeping the file it replaces in ``history/``.

    The file it replaces must load as one for the same arm and model, else
    ``ArmCalibrationError`` and nothing is written. It is copied to
    ``history/<its created time>.json`` before the new file is moved into
    place, so a crash leaves either the old or the new file, never half of one.
    """
    target = calibration_file(path)
    if target.exists():
        try:
            old = load_calibration(
                target,
                expected_model_sha256=calibration.robot_model.sha256,
                expected_arm_id=calibration.arm_id,
            )
        except ArmCalibrationError as error:
            raise ArmCalibrationError(
                f"{error} Not saving over it; move it away to start {calibration.arm_id} afresh."
            ) from error
        history = target.parent / HISTORY_DIR_NAME
        history.mkdir(exist_ok=True)
        stem = old.created.strftime("%Y-%m-%dT%H-%M-%SZ")
        candidate, index = history / f"{stem}.json", 1
        while candidate.exists():
            candidate, index = history / f"{stem}-{index}.json", index + 1
        shutil.copy2(target, candidate)
    target.parent.mkdir(parents=True, exist_ok=True)
    handle = tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        dir=target.parent,
        prefix=f".{target.name}.",
        suffix=".tmp",
        delete=False,
    )
    try:
        with handle:
            handle.write(calibration_text(calibration))
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(handle.name, target)
    except BaseException:
        Path(handle.name).unlink(missing_ok=True)
        raise
    return target


def load_calibration(
    path: Path,
    *,
    expected_model_sha256: str | None = None,
    expected_arm_id: str | None = None,
) -> ArmCalibration:
    """Read a calibration file, raising ``ArmCalibrationError`` in plain words on any problem.

    Args:
        path: The file, or the directory holding ``calibration.json``.
        expected_model_sha256: The model the caller drives; a file made for
            another model is refused.
        expected_arm_id: The arm the caller drives; a file for another arm is
            refused.
    """
    target = calibration_file(path)
    if not target.exists():
        raise ArmCalibrationError(f"No arm calibration at {target}.")
    try:
        raw = json.loads(target.read_text())
    except (OSError, ValueError) as error:  # ValueError: bad JSON or not UTF-8
        raise ArmCalibrationError(f"The arm calibration at {target} is not valid JSON: {error}")
    if not isinstance(raw, dict):
        raise ArmCalibrationError(f"The arm calibration at {target} is not a JSON object.")
    version = raw.get("schema_version")
    if version != SCHEMA_VERSION:
        raise ArmCalibrationError(
            f"The arm calibration at {target} has schema_version {version!r}; this version of "
            f"dimos reads schema_version {SCHEMA_VERSION}. Re-run the calibration or use a "
            "matching dimos version."
        )
    try:
        calibration = ArmCalibration.model_validate(raw)
    except ValidationError as error:
        problems = "; ".join(
            f"{'.'.join(str(p) for p in item['loc']) or 'file'}: {item['msg']}"
            for item in error.errors()
        )
        raise ArmCalibrationError(f"The arm calibration at {target} is invalid: {problems}")
    if expected_arm_id is not None and calibration.arm_id != expected_arm_id:
        raise ArmCalibrationError(
            f"The arm calibration at {target} is for arm {calibration.arm_id!r}, "
            f"not {expected_arm_id!r}."
        )
    if (
        expected_model_sha256 is not None
        and calibration.robot_model.sha256 != expected_model_sha256
    ):
        raise ArmCalibrationError(
            f"The arm calibration at {target} was made for a different robot model: it was "
            f"fitted against {calibration.robot_model.name!r} with fingerprint "
            f"{calibration.robot_model.sha256[:12]}, but this arm's model has fingerprint "
            f"{expected_model_sha256[:12]}. The URDF changed since the calibration, so its "
            "corrections no longer apply; calibrate the arm again."
        )
    return calibration


def load_for_spec(spec: ArmCalibrationSpec, path: Path | None = None) -> ArmCalibration:
    """The calibration of ``spec``'s arm, refused unless made for that arm and its model.

    Args:
        spec: The arm about to be driven.
        path: The file or its directory; defaults to ``calibration_path(spec.arm_id)``.
    """
    return load_calibration(
        calibration_path(spec.arm_id) if path is None else path,
        expected_model_sha256=spec.model_sha256(),
        expected_arm_id=spec.arm_id,
    )


def check_arm_fits(
    calibration: ArmCalibration,
    path: Path,
    joints: Sequence[str],
    moved_links: Collection[str],
    *,
    kp_max: float,
    kd_max: float,
) -> None:
    """Refuse a calibration that names other joints or links, or gains a held joint cannot take.

    Every gain must be for one of ``joints``, with 0 < kp <= ``kp_max`` (a
    file holds every joint; kp 0 would float it) and 0 <= kd <= ``kd_max``,
    and every fitted link one that ``joints`` move (``moved_links``).

    Raises:
        ArmCalibrationError: Naming what does not fit, and ``path``.
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
        if not (0.0 < kp <= kp_max and 0.0 <= kd <= kd_max)
    }
    if outside:
        raise ArmCalibrationError(
            f"The arm calibration at {path} gives gains outside 0 < kp <= {kp_max:g} "
            f"and 0 <= kd <= {kd_max:g}: {outside}."
        )
    links = set(calibration.gravity.links) if calibration.gravity is not None else set()
    strangers = sorted(links - set(moved_links))
    if strangers:
        raise ArmCalibrationError(
            f"The arm calibration at {path} gives masses to links arm "
            f"{calibration.arm_id!r} does not move: {strangers}; it moves {sorted(moved_links)}."
        )


def merge_patch(
    stored: ArmCalibration, patch: ArmConfigPatch, *, created: datetime
) -> ArmCalibration:
    """``stored`` with what ``patch`` gives changed, written at ``created``.

    Gains merge by joint. Links replace the stored set as a whole, with the
    patch's fit, else the stored fit if the links are unchanged, else none;
    the stored checks ran with the old links, so they stay only when the links
    are unchanged. Fit alone replaces the stored fit; friction replaces the
    stored friction; checks merge by name; notes replace the stored notes.

    Raises:
        ValueError: A fit without links, stored or given, to go with it.
    """
    gains = stored.gains if patch.gains is None else {**(stored.gains or {}), **patch.gains}
    gravity, checks = stored.gravity, stored.checks
    if patch.links is not None:
        same = gravity is not None and gravity.links == patch.links
        fit = patch.fit if patch.fit is not None else (gravity.fit if same and gravity else {})
        gravity = GravityCalibration(links=dict(patch.links), fit=dict(fit))
        checks = checks if same else None
    elif patch.fit is not None:
        if gravity is None:
            raise ValueError(f"{stored.arm_id}: a fit needs its fitted links; give links with it")
        gravity = GravityCalibration(links=dict(gravity.links), fit=dict(patch.fit))
    if patch.checks is not None:
        checks = {**(checks or {}), **patch.checks}
    friction = stored.friction_nm if patch.friction_nm is None else dict(patch.friction_nm)
    notes = stored.notes if patch.notes is None else patch.notes
    return ArmCalibration(
        **{
            **dict(stored),
            "created": created,
            "gravity": gravity,
            "gains": gains,
            "friction_nm": friction,
            "checks": checks,
            "notes": notes,
        }
    )


def to_inertial_overrides(calibration: ArmCalibration) -> dict[str, LinkInertial]:
    """The fitted links as ``_inertial`` overrides for a pinocchio model (body semantics)."""
    if calibration.gravity is None:
        return {}
    return {
        name: LinkInertial(mass_kg=link.mass_kg, com_m=link.com_m)
        for name, link in calibration.gravity.links.items()
    }


def gravity_links_from_inertial(
    spec: ArmCalibrationSpec, links: Mapping[str, LinkInertial]
) -> dict[str, LinkGravity]:
    """Fitted bodies of ``spec``'s gravity links, with the links merged into each."""
    joints = list(spec.model.load().joints)
    return {
        name: LinkGravity(
            mass_kg=float(link.mass_kg),
            com_m=link.com_m,
            merged_links=merged_links(joints, spec.joint_names, name),
        )
        for name, link in links.items()
    }


def bake_into_urdf(
    calibration: ArmCalibration | Sequence[ArmCalibration],
    urdf_xml: str,
    *,
    skip_model_check: str | None = None,
) -> str:
    """``urdf_xml`` with the fitted links' ``<inertial>`` mass and origin rewritten.

    Each fitted body is the link plus its ``merged_links``; those keep their URDF
    inertia and the link gets the rest, so a model built from the result has the
    fitted body mass and centre of mass. Rotational inertia and every other link
    are left untouched.

    Args:
        calibration: One arm's calibration, or several (each arm of a rig) to
            bake into the same URDF in turn.
        urdf_xml: The materialized URDF every calibration was made against
            (``RobotModel.load().xml``); its fingerprint must match theirs.
        skip_model_check: Bake into another URDF anyway, for this reason
            (logged), e.g. "the URDF only gained a camera link".

    Raises:
        ArmCalibrationError: The URDF is not the one a calibration was made
            against, a link is missing, or its merged links already weigh
            more than the fit.
    """
    calibrations = [calibration] if isinstance(calibration, ArmCalibration) else list(calibration)
    if skip_model_check is None:
        fingerprint = urdf_sha256(urdf_xml)
        for item in calibrations:
            if item.robot_model.sha256 != fingerprint:
                raise ArmCalibrationError(
                    f"The calibration of {item.arm_id} was made for the model "
                    f"{item.robot_model.name!r} with fingerprint {item.robot_model.sha256[:12]}, "
                    f"but the URDF to bake has fingerprint {fingerprint[:12]}. Bake into the "
                    "URDF it was fitted against (RobotModel.load().xml), or pass "
                    "skip_model_check with the reason."
                )
    elif not skip_model_check.strip():
        raise ValueError("skip_model_check needs a reason")
    else:
        logger.warning(
            f"Baking {[item.arm_id for item in calibrations]} without checking the model "
            f"fingerprint: {skip_model_check}"
        )
    for item in calibrations:
        urdf_xml = _bake_one(item, urdf_xml)
    return urdf_xml


def _bake_one(calibration: ArmCalibration, urdf_xml: str) -> str:
    if calibration.gravity is None:
        return urdf_xml
    root = ET.fromstring(urdf_xml, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    links = {element.get("name"): element for element in root.findall("link")}
    transforms = _link_transforms(urdf_xml)
    for name, fitted in calibration.gravity.links.items():
        element = links.get(name)
        if element is None:
            raise ArmCalibrationError(f"the URDF has no link {name!r} to bake")
        mass = fitted.mass_kg
        moment = mass * np.asarray(fitted.com_m, dtype=np.float64)
        for child in fitted.merged_links:
            child_mass, child_com = _inertial(links.get(child), child)
            if child_mass == 0.0:
                continue
            if child not in transforms or name not in transforms:
                raise ArmCalibrationError(f"{child!r} is not connected to {name!r} in the URDF")
            link_from_child = np.linalg.inv(transforms[name]) @ transforms[child]
            mass -= child_mass
            moment -= child_mass * (link_from_child[:3, :3] @ child_com + link_from_child[:3, 3])
        if mass <= 0.0:
            raise ArmCalibrationError(
                f"{name}: the fitted body ({fitted.mass_kg:.3f} kg) is lighter than its merged "
                f"links {fitted.merged_links} in the URDF"
            )
        _set_inertial(element, mass, moment / mass)
    return ET.tostring(root, encoding="unicode")


def _link_transforms(urdf_xml: str) -> dict[str, np.ndarray]:
    """Root -> link transform of every link with all joints at zero."""
    joints = RobotModel.from_urdf_string(urdf_xml).load().joints
    by_child = {joint.child_link: joint for joint in joints}
    cache: dict[str, np.ndarray] = {}

    def transform(link: str) -> np.ndarray:
        if link not in cache:
            joint = by_child.get(link)
            if joint is None:
                cache[link] = np.eye(4)
            elif joint.type in ("fixed", "revolute", "continuous", "prismatic"):
                # Locked joints sit at zero, as in the calibration's reduced model.
                cache[link] = transform(joint.parent_link) @ joint.parent_from_child(0.0)
            else:
                cache[link] = transform(joint.parent_link) @ joint.origin()
        return cache[link]

    for joint in joints:
        transform(joint.child_link)
    return cache


def _inertial(element: ET.Element | None, name: str) -> tuple[float, np.ndarray]:
    if element is None:
        raise ArmCalibrationError(f"the URDF has no merged link {name!r}")
    inertial = element.find("inertial")
    if inertial is None:
        return 0.0, np.zeros(3)
    mass_element = inertial.find("mass")
    mass = float(mass_element.get("value", "0")) if mass_element is not None else 0.0
    origin = inertial.find("origin")
    xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
    return mass, np.asarray([float(v) for v in xyz.split()], dtype=np.float64)


def _set_inertial(link: ET.Element, mass: float, com: np.ndarray) -> None:
    inertial = link.find("inertial")
    if inertial is None:
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "inertia", ixx="0", ixy="0", ixz="0", iyy="0", iyz="0", izz="0")
    origin = inertial.find("origin")
    if origin is None:
        origin = ET.Element("origin", rpy="0 0 0")
        inertial.insert(0, origin)
    origin.set("xyz", " ".join(repr(float(v)) for v in com))
    mass_element = inertial.find("mass")
    if mass_element is None:
        mass_element = ET.SubElement(inertial, "mass")
    mass_element.set("value", repr(float(mass)))
