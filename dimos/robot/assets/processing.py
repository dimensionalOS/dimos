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

"""In-memory robot description loading and transformation."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, replace
import os
from pathlib import Path
import re
from typing import Literal
import xml.etree.ElementTree as ET

from dimos.robot.assets.xacro import expand_xacro
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

PackageUriMode = Literal["preserve", "absolute"]


@dataclass(frozen=True)
class LoadedUrdf:
    """In-memory URDF plus the filesystem context needed by consumers."""

    urdf_xml: str
    source_path: Path
    package_paths: Mapping[str, Path]


UrdfProcessor = Callable[[LoadedUrdf], LoadedUrdf]


@dataclass(frozen=True)
class AddFixedFrame:
    """URDF processor that adds an empty link connected by a fixed joint."""

    name: str
    parent: str
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def joint_name(self) -> str:
        return f"{self.name}_joint"

    def __call__(self, description: LoadedUrdf) -> LoadedUrdf:
        """Return the description with this frame appended."""
        root = ET.fromstring(description.urdf_xml)
        link_names = {link.get("name") for link in root.findall("link")}
        joint_names = {joint.get("name") for joint in root.findall("joint")}

        if self.name in link_names:
            raise ValueError(f"Fixed frame link already exists: {self.name}")
        if self.joint_name in joint_names:
            raise ValueError(f"Fixed frame joint already exists: {self.joint_name}")
        if self.parent not in link_names:
            raise ValueError(
                f"Fixed frame {self.name} references unknown parent link: {self.parent}"
            )

        ET.SubElement(root, "link", {"name": self.name})
        joint = ET.SubElement(root, "joint", {"name": self.joint_name, "type": "fixed"})
        ET.SubElement(
            joint, "origin", {"xyz": _vector_text(self.xyz), "rpy": _vector_text(self.rpy)}
        )
        ET.SubElement(joint, "parent", {"link": self.parent})
        ET.SubElement(joint, "child", {"link": self.name})
        return replace(description, urdf_xml=ET.tostring(root, encoding="unicode"))


def load_urdf(
    urdf_path: Path | str | os.PathLike[str],
    package_paths: Mapping[str, Path | str | os.PathLike[str]] | None = None,
    xacro_args: Mapping[str, str] | None = None,
    *,
    package_uri_mode: PackageUriMode = "preserve",
    processors: Sequence[UrdfProcessor] = (),
) -> LoadedUrdf:
    """Load a URDF or Xacro artifact entirely in memory.

    Xacro expansion and package URI rewriting happen without writing an
    intermediate URDF. Processors run in declaration order after loading.
    """
    if package_uri_mode not in ("preserve", "absolute"):
        raise ValueError(f"Unsupported package URI mode: {package_uri_mode!r}")

    source_path = Path(os.fspath(urdf_path)).resolve()
    resolved_package_paths = normalize_package_paths(package_paths or {})
    resolved_xacro_args = dict(xacro_args or {})
    if source_path.suffix.lower() not in {".urdf", ".xacro"}:
        raise ValueError(
            f"Expected a URDF or Xacro model, got {source_path.name!r}; "
            "MJCF .xml files must be loaded by the backend"
        )

    if source_path.suffix.lower() == ".xacro":
        urdf_content = expand_xacro(source_path, resolved_package_paths, resolved_xacro_args)
    else:
        urdf_content = source_path.read_text()

    if package_uri_mode == "absolute":
        urdf_content = resolve_package_uris(urdf_content, resolved_package_paths)

    description = LoadedUrdf(
        urdf_xml=urdf_content,
        source_path=source_path,
        package_paths=resolved_package_paths,
    )
    for processor in processors:
        description = processor(description)
    return description


def _vector_text(vector: tuple[float, float, float]) -> str:
    return " ".join(str(value) for value in vector)


def resolve_package_uris(
    urdf_content: str,
    package_paths: Mapping[str, Path | str | os.PathLike[str]],
) -> str:
    """Rewrite ``package://`` URIs in URDF XML to absolute filesystem paths."""
    resolved_package_paths = normalize_package_paths(package_paths)
    pattern = r"""package://([^/]+)/(.+?)(["'<>\s])"""

    def replace_uri(match: re.Match[str]) -> str:
        pkg_name = match.group(1)
        rel_path = match.group(2)
        suffix = match.group(3)

        if pkg_name in resolved_package_paths:
            full_path = resolved_package_paths[pkg_name] / rel_path
            if full_path.exists():
                return f"{full_path}{suffix}"
            logger.warning(f"File not found: {full_path}")

        return match.group(0)

    return re.sub(pattern, replace_uri, urdf_content)


def normalize_package_paths(
    package_paths: Mapping[str, Path | str | os.PathLike[str]],
) -> dict[str, Path]:
    return {
        package_name: Path(os.fspath(package_path)).resolve()
        for package_name, package_path in package_paths.items()
    }
