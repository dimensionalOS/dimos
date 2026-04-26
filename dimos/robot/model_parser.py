# Copyright 2025-2026 Dimensional Inc.
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

"""Parse URDF, xacro, and MJCF files into a ModelDescription."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import xml.etree.ElementTree as ET

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class JointDescription:
    """A joint parsed from a robot description file (URDF/MJCF)."""

    name: str
    type: str
    lower_limit: float | None = None
    upper_limit: float | None = None
    velocity_limit: float | None = None
    effort_limit: float | None = None
    parent_link: str = ""
    child_link: str = ""


@dataclass
class ModelDescription:
    """Structured representation of a robot description file (URDF/MJCF)."""

    joints: list[JointDescription] = field(default_factory=list)
    root_link: str = ""
    links: list[str] = field(default_factory=list)

    @property
    def actuated_joint_names(self) -> list[str]:
        return [j.name for j in self.joints if j.type != "fixed"]

    @property
    def actuated_joints(self) -> list[JointDescription]:
        return [j for j in self.joints if j.type != "fixed"]

    def get_joint(self, name: str) -> JointDescription | None:
        for j in self.joints:
            if j.name == name:
                return j
        return None


def parse_model(
    path: Path | str,
    package_paths: dict[str, Path] | None = None,
    xacro_args: dict[str, str] | None = None,
) -> ModelDescription:
    """Parse a robot description file (.urdf, .xacro, .xml/MJCF)."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Robot model file not found: {path}")

    suffix = path.suffix.lower()
    name = path.name.lower()

    if suffix == ".xacro" or name.endswith(".urdf.xacro"):
        xml_string = _expand_xacro(path, package_paths or {}, xacro_args or {})
        return _parse_urdf_string(xml_string)
    elif suffix == ".urdf":
        return _parse_urdf_string(path.read_text())
    elif suffix == ".xml":
        return _parse_mjcf_file(path)
    else:
        raise ValueError(
            f"Unrecognized model file format: {path.suffix}. "
            "Expected .urdf, .xacro, .urdf.xacro, or .xml (MJCF)."
        )


def _expand_xacro(
    path: Path,
    package_paths: dict[str, Path],
    xacro_args: dict[str, str],
) -> str:
    """Expand a xacro file to URDF XML string."""
    try:
        from dimos.utils.ament_prefix import process_xacro
    except ImportError:
        raise ImportError(
            "xacro is required for processing .xacro files. "
            "Install the manipulation extra: pip install dimos[manipulation]"
        )

    return process_xacro(path, package_paths, xacro_args)


def _parse_urdf_string(xml_string: str) -> ModelDescription:
    """Parse a URDF XML string."""
    root = ET.fromstring(xml_string)

    links: list[str] = []
    for link_elem in root.findall("link"):
        name = link_elem.get("name")
        if name:
            links.append(name)

    joints: list[JointDescription] = []
    child_links: set[str] = set()

    for joint_elem in root.findall("joint"):
        name = joint_elem.get("name", "")
        joint_type = joint_elem.get("type", "fixed")

        parent_elem = joint_elem.find("parent")
        child_elem = joint_elem.find("child")
        parent_link = parent_elem.get("link", "") if parent_elem is not None else ""
        child_link = child_elem.get("link", "") if child_elem is not None else ""

        if child_link:
            child_links.add(child_link)

        lower = upper = velocity = effort = None
        limit_elem = joint_elem.find("limit")
        if limit_elem is not None:
            lower = _float_or_none(limit_elem.get("lower"))
            upper = _float_or_none(limit_elem.get("upper"))
            velocity = _float_or_none(limit_elem.get("velocity"))
            effort = _float_or_none(limit_elem.get("effort"))

        joints.append(
            JointDescription(
                name=name,
                type=joint_type,
                lower_limit=lower,
                upper_limit=upper,
                velocity_limit=velocity,
                effort_limit=effort,
                parent_link=parent_link,
                child_link=child_link,
            )
        )

    # Root link = parent that is never a child
    non_child_links = [l for l in links if l not in child_links]
    if len(non_child_links) == 1:
        root_link = non_child_links[0]
    elif len(non_child_links) > 1:
        logger.warning(
            "Multiple root candidates: %s; using %s", non_child_links, non_child_links[0]
        )
        root_link = non_child_links[0]
    else:
        root_link = ""

    return ModelDescription(joints=joints, root_link=root_link, links=links)


def _parse_mjcf_file(path: Path) -> ModelDescription:
    """Parse a MuJoCo MJCF XML file. Joints are nested inside <body> elements."""
    tree = ET.parse(path)
    root = tree.getroot()

    joints: list[JointDescription] = []
    links: list[str] = []

    root_link = ""
    worldbody = root.find("worldbody")
    if worldbody is not None:
        _walk_mjcf_bodies(worldbody, joints, links, parent_body="")
        first_body = worldbody.find("body")
        if first_body is not None:
            root_link = first_body.get("name", "")

    return ModelDescription(joints=joints, root_link=root_link, links=links)


def _walk_mjcf_bodies(
    element: ET.Element,
    joints: list[JointDescription],
    links: list[str],
    parent_body: str,
) -> None:
    """Recursively walk MJCF <body> elements to extract joints and links."""
    for body in element.findall("body"):
        body_name = body.get("name", "")
        if body_name:
            links.append(body_name)

        for joint_elem in body.findall("joint"):
            joint_name = joint_elem.get("name", "")
            joint_type = joint_elem.get("type", "hinge")

            # Map MJCF types to URDF-compatible names
            type_map = {"hinge": "revolute", "slide": "prismatic", "free": "free"}
            mapped_type = type_map.get(joint_type, joint_type)

            lower = upper = None
            range_str = joint_elem.get("range")
            if range_str:
                parts = range_str.split()
                if len(parts) == 2:
                    lower = _float_or_none(parts[0])
                    upper = _float_or_none(parts[1])

            joints.append(
                JointDescription(
                    name=joint_name,
                    type=mapped_type,
                    lower_limit=lower,
                    upper_limit=upper,
                    velocity_limit=None,
                    effort_limit=None,
                    parent_link=parent_body,
                    child_link=body_name,
                )
            )

        _walk_mjcf_bodies(body, joints, links, parent_body=body_name)


def _float_or_none(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except ValueError:
        return None


__all__ = ["JointDescription", "ModelDescription", "parse_model"]
