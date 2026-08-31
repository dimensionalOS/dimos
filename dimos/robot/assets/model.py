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
from typing import Annotated
import xml.etree.ElementTree as ET

from pydantic import ConfigDict, Field, model_validator
from pydantic.dataclasses import dataclass as pydantic_dataclass
from typing_extensions import Self

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


_PLANAR_BASE_CONFIG = ConfigDict(extra="forbid", validate_default=True)
_NonEmptyString = Annotated[str, Field(min_length=1)]
_PositiveFiniteFloat = Annotated[float, Field(gt=0.0, allow_inf_nan=False)]
_PositiveVector3 = tuple[_PositiveFiniteFloat, _PositiveFiniteFloat, _PositiveFiniteFloat]


@pydantic_dataclass(frozen=True, config=_PLANAR_BASE_CONFIG)
class PlanarBaseDefinition:
    """Synthetic floor-constrained base coordinates.

    The three coordinates are ordered ``x``, ``y``, then ``yaw``. Linear
    coordinates use meters and angular coordinates use radians. Translation is
    unbounded and yaw is periodic.
    """

    velocity_limits: _PositiveVector3
    acceleration_limits: _PositiveVector3
    root_link: _NonEmptyString = "planar_base_root"
    joint_names: tuple[_NonEmptyString, _NonEmptyString, _NonEmptyString] = (
        "base/x",
        "base/y",
        "base/yaw",
    )

    @model_validator(mode="after")
    def _validate_cross_field_invariants(self) -> Self:
        if len(set(self.joint_names)) != 3:
            raise ValueError("Planar base joint names must be unique")
        return self


@dataclass(frozen=True)
class RobotModel:
    """Lazy, immutable robot model shared by planning backends."""

    _source_path: Path | str | os.PathLike[str]
    _package_paths: tuple[tuple[str, Path | str | os.PathLike[str]], ...] = ()
    _xacro_args: tuple[tuple[str, str], ...] = ()
    _fixed_frames: tuple[_FixedFrame, ...] = ()
    _fixed_joints: tuple[str, ...] = ()
    _renamed_joints: tuple[tuple[str, str], ...] = ()
    _joint_position_limits: tuple[_JointPositionLimits, ...] = ()
    _subtree_root_link: str | None = None
    _removed_joint_subtrees: tuple[str, ...] = ()
    _planar_base: PlanarBaseDefinition | None = None

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

    @property
    def planar_base(self) -> PlanarBaseDefinition | None:
        """Return the configured synthetic planar base, if any."""
        return self._planar_base

    def with_planar_base(self, definition: PlanarBaseDefinition) -> RobotModel:
        """Return a model whose original root moves through ``x``, ``y``, and ``yaw``."""
        if self._planar_base is not None:
            raise ValueError("Robot model already has a planar base")
        return replace(self, _planar_base=definition)

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

    def with_subtree_rooted_at(self, root_link: str) -> RobotModel:
        """Return a view containing an existing link and its descendants.

        This selects an existing structural subtree. It does not reverse joints
        or recompute transforms as a kinematic rerooting operation would.
        """
        if not root_link:
            raise ValueError("Subtree root link must not be empty")
        if self._subtree_root_link is not None:
            raise ValueError(f"Subtree root link is already selected: {self._subtree_root_link}")
        return replace(self, _subtree_root_link=root_link)

    def without_joint_subtrees(self, *joint_names: str) -> RobotModel:
        """Return a view without the named joints and their descendant branches."""
        if not joint_names:
            raise ValueError("At least one joint subtree must be removed")
        if any(not name for name in joint_names):
            raise ValueError("Joint subtree names must not be empty")
        requested = (*self._removed_joint_subtrees, *joint_names)
        if len(set(requested)) != len(requested):
            duplicate = next(name for name in requested if requested.count(name) > 1)
            raise ValueError(f"Joint subtree already requested for removal: {duplicate}")
        return replace(self, _removed_joint_subtrees=requested)

    def with_fixed_joints(self, *names: str) -> RobotModel:
        """Return a model with movable joints fixed at their URDF zero pose."""
        if not names:
            raise ValueError("At least one joint must be fixed")
        return replace(self, _fixed_joints=(*self._fixed_joints, *names))

    def with_renamed_joints(self, names: Mapping[str, str]) -> RobotModel:
        """Return a model view whose joints use the supplied canonical names."""
        if not names:
            raise ValueError("At least one joint rename is required")
        if any(not source or not target for source, target in names.items()):
            raise ValueError("Joint rename names must not be empty")
        requested = (*self._renamed_joints, *names.items())
        sources = [source for source, _target in requested]
        if len(sources) != len(set(sources)):
            duplicate = next(name for name in sources if sources.count(name) > 1)
            raise ValueError(f"Joint is already renamed: {duplicate}")
        return replace(self, _renamed_joints=requested)

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
        if self._subtree_root_link is not None or self._removed_joint_subtrees:
            xml = _select_structural_subtree(
                xml,
                root_link=self._subtree_root_link,
                removed_joint_subtrees=self._removed_joint_subtrees,
            )
        xml = _resolve_relative_asset_paths(
            xml,
            search_directories=(source_path.parent, *package_paths.values()),
        )
        if self._planar_base is not None:
            xml = _add_planar_base(xml, self._planar_base)
        if self._fixed_joints:
            xml = _set_joints_fixed(xml, self._fixed_joints)
        if self._joint_position_limits:
            xml = _set_joint_position_limits(xml, self._joint_position_limits)
        if self._renamed_joints:
            xml = _rename_joints(xml, dict(self._renamed_joints))
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


def _add_planar_base(xml: str, definition: PlanarBaseDefinition) -> str:
    root = ET.fromstring(xml)
    link_names = {link.get("name") for link in root.findall("link")}
    joint_names = {joint.get("name") for joint in root.findall("joint")}
    child_links = {
        child.get("link")
        for joint in root.findall("joint")
        if (child := joint.find("child")) is not None
    }
    root_links = sorted(name for name in link_names - child_links if name is not None)
    if len(root_links) != 1:
        raise ValueError(f"Planar base requires one URDF root link, found {root_links}")

    x_link = f"{definition.root_link}_x"
    xy_link = f"{definition.root_link}_xy"
    generated_links = {definition.root_link, x_link, xy_link}
    duplicate_links = sorted(generated_links & link_names)
    if duplicate_links:
        raise ValueError(f"Planar base link names already exist: {duplicate_links}")
    duplicate_joints = sorted(set(definition.joint_names) & joint_names)
    if duplicate_joints:
        raise ValueError(f"Planar base joint names already exist: {duplicate_joints}")

    for name in (definition.root_link, x_link, xy_link):
        ET.SubElement(root, "link", {"name": name})
    links = (definition.root_link, x_link, xy_link, root_links[0])
    axes = ("1 0 0", "0 1 0", "0 0 1")
    types = ("prismatic", "prismatic", "continuous")
    for index, (name, axis, joint_type) in enumerate(
        zip(definition.joint_names, axes, types, strict=True)
    ):
        joint = ET.SubElement(root, "joint", {"name": name, "type": joint_type})
        ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(joint, "parent", {"link": links[index]})
        ET.SubElement(joint, "child", {"link": links[index + 1]})
        ET.SubElement(joint, "axis", {"xyz": axis})
        ET.SubElement(
            joint,
            "limit",
            {
                "effort": "1",
                "velocity": str(definition.velocity_limits[index]),
                "acceleration": str(definition.acceleration_limits[index]),
            },
        )
    return ET.tostring(root, encoding="unicode")


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


def _select_structural_subtree(
    xml: str,
    *,
    root_link: str | None,
    removed_joint_subtrees: tuple[str, ...],
) -> str:
    root = ET.fromstring(xml)
    links = {link.get("name"): link for link in root.findall("link")}
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    if None in links or len(links) != len(root.findall("link")):
        raise ValueError("Robot model links must have unique non-empty names")
    if None in joints or len(joints) != len(root.findall("joint")):
        raise ValueError("Robot model joints must have unique non-empty names")

    topology = _parse_topology(xml)
    selected_root = root_link or topology[1]
    if selected_root not in links:
        raise ValueError(f"Subtree root link not found: {selected_root}")

    children_by_link: dict[str, list[tuple[str, str]]] = {}
    joint_children: dict[str, str] = {}
    for joint in topology[0]:
        if not joint.parent_link or not joint.child_link:
            raise ValueError(f"Joint has incomplete topology: {joint.name}")
        children_by_link.setdefault(joint.parent_link, []).append((joint.name, joint.child_link))
        joint_children[joint.name] = joint.child_link

    selected_links, selected_joints = _descendant_closure(selected_root, children_by_link)
    for joint_name in removed_joint_subtrees:
        if joint_name not in joints:
            raise ValueError(f"Joint subtree not found: {joint_name}")
        if joint_name not in selected_joints:
            raise ValueError(
                f"Joint subtree is outside selected root '{selected_root}': {joint_name}"
            )

    removed_links: set[str] = set()
    removed_joints: set[str] = set()
    for joint_name in removed_joint_subtrees:
        child_link = joint_children[joint_name]
        branch_links, branch_joints = _descendant_closure(child_link, children_by_link)
        removed_links.update(branch_links)
        removed_joints.add(joint_name)
        removed_joints.update(branch_joints)

    kept_links = selected_links - removed_links
    kept_joints = selected_joints - removed_joints
    for link in list(root.findall("link")):
        if link.get("name") not in kept_links:
            root.remove(link)
    for joint_element in list(root.findall("joint")):
        if joint_element.get("name") not in kept_joints:
            root.remove(joint_element)
    return ET.tostring(root, encoding="unicode")


def _descendant_closure(
    root_link: str,
    children_by_link: Mapping[str, list[tuple[str, str]]],
) -> tuple[set[str], set[str]]:
    links: set[str] = set()
    joints: set[str] = set()
    pending = [root_link]
    while pending:
        link_name = pending.pop()
        if link_name in links:
            raise ValueError(f"Robot model topology contains a cycle at link: {link_name}")
        links.add(link_name)
        for joint_name, child_link in children_by_link.get(link_name, []):
            joints.add(joint_name)
            pending.append(child_link)
    return links, joints


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


def _rename_joints(xml: str, names: Mapping[str, str]) -> str:
    root = ET.fromstring(xml)
    model_joints = {joint.get("name") for joint in root.findall("joint")}
    missing = sorted(set(names) - model_joints)
    if missing:
        raise ValueError(f"Joint not found: {missing[0]}")

    unchanged = model_joints - set(names)
    targets = list(names.values())
    if len(targets) != len(set(targets)) or unchanged & set(targets):
        raise ValueError("Renamed joint names must be unique in the model")

    for joint in root.findall("joint"):
        if (name := joint.get("name")) is not None and name in names:
            joint.set("name", names[name])
    for mimic in root.findall(".//mimic"):
        if (name := mimic.get("joint")) is not None and name in names:
            mimic.set("joint", names[name])
    for transmission_joint in root.findall(".//transmission/joint"):
        if (name := transmission_joint.get("name")) is not None and name in names:
            transmission_joint.set("name", names[name])
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


def _resolve_relative_asset_paths(
    xml: str,
    *,
    search_directories: tuple[Path, ...],
) -> str:
    """Resolve URDF assets against the source directory and package roots."""
    root = ET.fromstring(xml)
    changed = False
    for element in (*root.findall(".//mesh"), *root.findall(".//texture")):
        filename = element.get("filename")
        if not filename or Path(filename).is_absolute() or "://" in filename:
            continue
        candidates = list(
            dict.fromkeys((directory / filename).resolve() for directory in search_directories)
        )
        matches = [candidate for candidate in candidates if candidate.exists()]
        if len(matches) == 1:
            element.set("filename", str(matches[0]))
            changed = True
        elif len(matches) > 1:
            raise ValueError(f"Ambiguous relative asset path {filename!r}: {matches}")
        else:
            logger.warning(f"Relative asset not found in {search_directories}: {filename}")
    return ET.tostring(root, encoding="unicode") if changed else xml


def _normalize_package_paths(
    package_paths: Mapping[str, Path | str | os.PathLike[str]],
) -> dict[str, Path]:
    return {
        package_name: Path(os.fspath(package_path)).resolve()
        for package_name, package_path in package_paths.items()
    }
