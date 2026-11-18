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
model, since the fitted masses are corrections to that model;
``load_for_spec`` always checks both the model and the arm.

A fitted link's mass and centre of mass are those of the link's pinocchio body:
the link plus the links merged into it (``merged_links``: fixed children and
locked fingers), centre of mass in the link frame. ``bake_into_urdf`` writes
them back into a URDF by adjusting only the named link, so the merged links
keep their own URDF values and the sum matches the fit. It bakes only into the
URDF the calibration was made against.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
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
from pydantic import Field, ValidationError, field_validator

from dimos.constants import STATE_DIR
from dimos.protocol.service.spec import BaseConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.calibration._inertial import LinkInertial
from dimos.robot.manipulators.calibration.spec import (
    ArmCalibrationSpec,
    merged_links,
    urdf_sha256,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

SCHEMA_VERSION: Literal[1] = 1
CALIBRATION_FILENAME = "calibration.json"
HISTORY_DIR_NAME = "history"
ARMS_STATE_DIR = STATE_DIR / "arms"

PositiveFloat = Annotated[float, Field(gt=0.0, allow_inf_nan=False)]
NonNegativeFloat = Annotated[float, Field(ge=0.0, allow_inf_nan=False)]
FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]


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


class LinkGravity(BaseConfig):
    """Fitted mass and centre of mass of one link's body, centre of mass in the link frame.

    Attributes:
        mass_kg: The body's mass, including ``merged_links``.
        com_m: The body's centre of mass, link frame, metres.
        merged_links: Links whose URDF inertia the body includes.
    """

    mass_kg: PositiveFloat
    com_m: tuple[FiniteFloat, FiniteFloat, FiniteFloat]
    merged_links: list[str] = Field(default_factory=list)


class GravityCalibration(BaseConfig):
    """The fitted links and the fit's statistics (see ``gravity_fit``)."""

    links: dict[str, LinkGravity]
    fit: dict[str, Any] = Field(default_factory=dict)


class JointGains(BaseConfig):
    kp: NonNegativeFloat
    kd: NonNegativeFloat


class ArmCalibration(BaseConfig):
    """One arm's calibration file.

    Attributes:
        schema_version: The file format; this build reads version 1.
        arm_id: The arm, e.g. "dual_openyam/left".
        robot_model: The model the calibration was made against.
        created: When the file was written, UTC.
        gravity: Fitted link masses and centres of mass, if identified.
        gains: Position and velocity gains per joint, if tuned.
        friction_nm: Coulomb friction per joint, if measured; never negative.
        checks: Results of validation runs (float drift and the like).
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


def save_calibration(path: Path, calibration: ArmCalibration) -> Path:
    """Write ``calibration`` atomically, keeping the file it replaces in ``history/``.

    The previous file is copied to ``history/<its created time>.json`` next to
    it before the new file is moved into place, so a crash leaves either the old
    or the new file, never half of one.
    """
    target = calibration_file(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        history = target.parent / HISTORY_DIR_NAME
        history.mkdir(exist_ok=True)
        shutil.copy2(target, _history_name(history, target))
    text = calibration.model_dump_json(indent=2) + "\n"
    handle = tempfile.NamedTemporaryFile(
        "w", dir=target.parent, prefix=f".{target.name}.", suffix=".tmp", delete=False
    )
    try:
        with handle:
            handle.write(text)
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


LEGACY_NOTES = "legacy fit; model hash supplied by caller"


def calibration_from_legacy_gravity(
    source: Path | Mapping[str, Any], spec: ArmCalibrationSpec, *, model_sha256: str
) -> ArmCalibration:
    """Convert a pre-spec ``gravity_fit`` file (``{"links": ...}``) into one arm's calibration.

    Only the links among ``spec.gravity_links`` are taken, so one file holding
    both arms of a rig converts into each arm's calibration in turn.

    Args:
        source: The legacy file, or its parsed content.
        spec: The arm the links belong to.
        model_sha256: The fingerprint (``model_sha256``) of the model the
            legacy fit was made against. The file recorded none, so the
            caller vouches for it: stamping the current model's would let a
            fit made against an older URDF load as if it were current.
    """
    data: Mapping[str, Any]
    if isinstance(source, Path):
        data = json.loads(source.read_text())
    else:
        data = source
    raw_links = data.get("links")
    if not isinstance(raw_links, Mapping):
        raise ArmCalibrationError("a legacy gravity file needs a 'links' mapping")
    links = {
        name: LinkInertial(mass_kg=value["mass_kg"], com_m=tuple(value["com_m"]))  # type: ignore[arg-type]
        for name, value in raw_links.items()
        if name in spec.gravity_links
    }
    if not links:
        raise ArmCalibrationError(f"the legacy gravity file has none of {spec.gravity_links}")
    created = datetime.fromisoformat(str(data["created"])) if "created" in data else None
    if created is not None and created.tzinfo is None:
        created = created.replace(tzinfo=timezone.utc)
    try:
        model = RobotModelRef(name=spec.model_name(), sha256=model_sha256)
    except ValidationError as error:
        raise ArmCalibrationError(f"{model_sha256!r} is not a model fingerprint: {error}")
    return ArmCalibration(
        arm_id=spec.arm_id,
        robot_model=model,
        created=created or datetime.now(timezone.utc),
        gravity=GravityCalibration(
            links=gravity_links_from_inertial(spec, links),
            fit={
                "source": "converted from a legacy gravity_fit file",
                "legacy_source": data.get("source"),
                "merged_from": data.get("merged_from"),
            },
        ),
        notes=LEGACY_NOTES,
    )


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


def _history_name(history: Path, current: Path) -> Path:
    try:
        created = datetime.fromisoformat(json.loads(current.read_text())["created"])
        stem = created.astimezone(timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    except (OSError, ValueError, KeyError, TypeError):
        stem = datetime.fromtimestamp(current.stat().st_mtime, timezone.utc).strftime(
            "unreadable-%Y-%m-%dT%H-%M-%SZ"
        )
    candidate = history / f"{stem}.json"
    index = 1
    while candidate.exists():
        candidate = history / f"{stem}-{index}.json"
        index += 1
    return candidate


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
        ET.SubElement(
            inertial, "inertia", ixx="1e-6", ixy="0", ixz="0", iyy="1e-6", iyz="0", izz="1e-6"
        )
    origin = inertial.find("origin")
    if origin is None:
        origin = ET.Element("origin", rpy="0 0 0")
        inertial.insert(0, origin)
    origin.set("xyz", " ".join(repr(float(v)) for v in com))
    mass_element = inertial.find("mass")
    if mass_element is None:
        mass_element = ET.SubElement(inertial, "mass")
    mass_element.set("value", repr(float(mass)))
