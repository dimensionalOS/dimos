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

"""Portable robot models loaded from URDF or Xacro sources."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, replace
from functools import cached_property
import math
import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from dimos.robot.assets.xacro import expand_xacro
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class JointDescription:
    """A joint parsed from a materialized URDF description."""

    name: str
    type: str
    parent_link: str = ""
    child_link: str = ""
    origin_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class LoadedRobotModel:
    """Materialized model description and its resolved filesystem context."""

    xml: str
    source_path: Path
    package_paths: Mapping[str, Path]

    @cached_property
    def _topology(self) -> tuple[tuple[JointDescription, ...], str]:
        return _parse_topology(self.xml)

    @property
    def joints(self) -> tuple[JointDescription, ...]:
        """Return joints in source-document order."""
        return self._topology[0]

    @property
    def root_link(self) -> str:
        """Return the unique URDF root link, or an empty string if none exists."""
        return self._topology[1]

    def get_joint(self, name: str) -> JointDescription | None:
        """Return the named joint when it exists."""
        return next((joint for joint in self.joints if joint.name == name), None)


@dataclass(frozen=True)
class _FixedFrame:
    name: str
    parent: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]

    @property
    def joint_name(self) -> str:
        return f"{self.name}_joint"


@dataclass(frozen=True)
class _JointPositionLimits:
    name: str
    lower: float
    upper: float


@dataclass(frozen=True)
class RobotModel:
    """Lazy, immutable robot model shared by planning backends."""

    _source_path: Path | str | os.PathLike[str]
    _package_paths: tuple[tuple[str, Path | str | os.PathLike[str]], ...] = ()
    _xacro_args: tuple[tuple[str, str], ...] = ()
    _fixed_frames: tuple[_FixedFrame, ...] = ()
    _fixed_joints: tuple[str, ...] = ()
    _joint_position_limits: tuple[_JointPositionLimits, ...] = ()

    @classmethod
    def from_file(
        cls,
        source_path: Path | str | os.PathLike[str],
        *,
        package_paths: Mapping[str, Path | str | os.PathLike[str]] | None = None,
        xacro_args: Mapping[str, str] | None = None,
    ) -> RobotModel:
        """Create a lazy model from a URDF or Xacro source."""
        return cls(
            _source_path=source_path,
            _package_paths=tuple((package_paths or {}).items()),
            _xacro_args=tuple((xacro_args or {}).items()),
        )

    @property
    def source_path(self) -> Path | str | os.PathLike[str]:
        """Return the source handle without forcing lazy asset checkout."""
        return self._source_path

    def with_fixed_frame(
        self,
        name: str,
        parent: str,
        *,
        xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> RobotModel:
        """Return a model with an additional fixed frame."""
        return replace(
            self,
            _fixed_frames=(*self._fixed_frames, _FixedFrame(name, parent, xyz, rpy)),
        )

    def with_fixed_joints(self, *names: str) -> RobotModel:
        """Return a model with movable joints fixed at their URDF zero pose."""
        if not names:
            raise ValueError("At least one joint must be fixed")
        return replace(self, _fixed_joints=(*self._fixed_joints, *names))

    def with_joint_position_limits(
        self,
        name: str,
        *,
        lower: float,
        upper: float,
    ) -> RobotModel:
        """Return a model with replacement position limits for one joint."""
        if not math.isfinite(lower) or not math.isfinite(upper):
            raise ValueError("Joint position limits must be finite")
        if lower > upper:
            raise ValueError(f"Joint position limits are inverted: {lower} > {upper}")
        return replace(
            self,
            _joint_position_limits=(
                *self._joint_position_limits,
                _JointPositionLimits(name, lower, upper),
            ),
        )

    def load(self) -> LoadedRobotModel:
        """Materialize and memoize the model for a backend adapter."""
        return self._loaded

    @cached_property
    def _loaded(self) -> LoadedRobotModel:
        source_path = Path(os.fspath(self._source_path)).resolve()
        if source_path.suffix.lower() not in {".urdf", ".xacro"}:
            raise ValueError(
                f"RobotModel source must reference a .urdf or .xacro file, got {source_path.name!r}"
            )
        package_paths = _normalize_package_paths(dict(self._package_paths))
        if not source_path.exists():
            raise FileNotFoundError(f"Robot model not found: {source_path}")

        if source_path.suffix.lower() == ".xacro":
            xml = expand_xacro(source_path, package_paths, dict(self._xacro_args))
        else:
            xml = source_path.read_text()
        xml = _resolve_package_uris(xml, package_paths)
        if self._fixed_joints:
            xml = _set_joints_fixed(xml, self._fixed_joints)
        if self._joint_position_limits:
            xml = _set_joint_position_limits(xml, self._joint_position_limits)
        if self._fixed_frames:
            xml = _add_fixed_frames(xml, self._fixed_frames)

        return LoadedRobotModel(xml, source_path, package_paths)

    def __getstate__(self) -> dict[str, object]:
        """Exclude materialized XML when configurations cross worker processes."""
        state = dict(self.__dict__)
        state.pop("_loaded", None)
        return state

    def __setstate__(self, state: dict[str, object]) -> None:
        for name, value in state.items():
            object.__setattr__(self, name, value)


def _add_fixed_frames(xml: str, frames: tuple[_FixedFrame, ...]) -> str:
    root = ET.fromstring(xml)
    link_names = {link.get("name") for link in root.findall("link")}
    joint_names = {joint.get("name") for joint in root.findall("joint")}

    for frame in frames:
        if frame.name in link_names:
            raise ValueError(f"Fixed frame link already exists: {frame.name}")
        if frame.joint_name in joint_names:
            raise ValueError(f"Fixed frame joint already exists: {frame.joint_name}")
        if frame.parent not in link_names:
            raise ValueError(
                f"Fixed frame {frame.name} references unknown parent link: {frame.parent}"
            )

        ET.SubElement(root, "link", {"name": frame.name})
        joint = ET.SubElement(root, "joint", {"name": frame.joint_name, "type": "fixed"})
        ET.SubElement(
            joint,
            "origin",
            {"xyz": _vector_text(frame.xyz), "rpy": _vector_text(frame.rpy)},
        )
        ET.SubElement(joint, "parent", {"link": frame.parent})
        ET.SubElement(joint, "child", {"link": frame.name})
        link_names.add(frame.name)
        joint_names.add(frame.joint_name)

    return ET.tostring(root, encoding="unicode")


def _set_joints_fixed(xml: str, names: tuple[str, ...]) -> str:
    root = ET.fromstring(xml)
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    seen: set[str] = set()
    movable_elements = {
        "axis",
        "calibration",
        "dynamics",
        "limit",
        "mimic",
        "safety_controller",
    }

    for name in names:
        if name in seen:
            raise ValueError(f"Joint already requested as fixed: {name}")
        seen.add(name)
        joint = joints.get(name)
        if joint is None:
            raise ValueError(f"Joint not found: {name}")
        if joint.get("type") == "fixed":
            raise ValueError(f"Joint is already fixed: {name}")

        joint.set("type", "fixed")
        for element in list(joint):
            if element.tag in movable_elements:
                joint.remove(element)

    return ET.tostring(root, encoding="unicode")


def _set_joint_position_limits(
    xml: str,
    replacements: tuple[_JointPositionLimits, ...],
) -> str:
    root = ET.fromstring(xml)
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    seen: set[str] = set()
    for replacement in replacements:
        if replacement.name in seen:
            raise ValueError(f"Joint position limits already replaced: {replacement.name}")
        seen.add(replacement.name)
        joint = joints.get(replacement.name)
        if joint is None:
            raise ValueError(f"Joint not found: {replacement.name}")
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"Joint has no position limits: {replacement.name}")
        limit.set("lower", str(replacement.lower))
        limit.set("upper", str(replacement.upper))
    return ET.tostring(root, encoding="unicode")


def _parse_topology(xml: str) -> tuple[tuple[JointDescription, ...], str]:
    root = ET.fromstring(xml)
    links = [name for link in root.findall("link") if (name := link.get("name")) is not None]
    child_links: set[str] = set()
    joints: list[JointDescription] = []

    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        parent_link = parent.get("link", "") if parent is not None else ""
        child_link = child.get("link", "") if child is not None else ""
        if child_link:
            child_links.add(child_link)

        origin = joint.find("origin")
        joints.append(
            JointDescription(
                name=joint.get("name", ""),
                type=joint.get("type", "fixed"),
                parent_link=parent_link,
                child_link=child_link,
                origin_xyz=_triple(origin.get("xyz") if origin is not None else None),
                origin_rpy=_triple(origin.get("rpy") if origin is not None else None),
            )
        )

    root_candidates = [link for link in links if link not in child_links]
    if len(root_candidates) > 1:
        logger.warning(f"Multiple root candidates: {root_candidates}; using {root_candidates[0]}")
    root_link = root_candidates[0] if root_candidates else ""
    return tuple(joints), root_link


def _triple(value: str | None) -> tuple[float, float, float]:
    if value is None:
        return (0.0, 0.0, 0.0)
    parts = tuple(float(part) for part in value.split())
    if len(parts) != 3:
        raise ValueError(f"Expected 3 values, got {value!r}")
    return (parts[0], parts[1], parts[2])


def _vector_text(vector: tuple[float, float, float]) -> str:
    return " ".join(str(value) for value in vector)


def _resolve_package_uris(xml: str, package_paths: Mapping[str, Path]) -> str:
    pattern = r"""package://([^/]+)/(.+?)(["'<>\s])"""

    def replace_uri(match: re.Match[str]) -> str:
        package_name = match.group(1)
        relative_path = match.group(2)
        suffix = match.group(3)
        if package_name in package_paths:
            full_path = package_paths[package_name] / relative_path
            if full_path.exists():
                return f"{full_path}{suffix}"
            logger.warning(f"File not found: {full_path}")
        return match.group(0)

    return re.sub(pattern, replace_uri, xml)


def _normalize_package_paths(
    package_paths: Mapping[str, Path | str | os.PathLike[str]],
) -> dict[str, Path]:
    return {
        package_name: Path(os.fspath(package_path)).resolve()
        for package_name, package_path in package_paths.items()
    }
