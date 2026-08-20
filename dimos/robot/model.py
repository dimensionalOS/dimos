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
import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from dimos.robot.assets.xacro import expand_xacro
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class LoadedRobotModel:
    """Materialized model description and its resolved filesystem context."""

    xml: str
    source_path: Path
    package_paths: Mapping[str, Path]


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
class RobotModel:
    """Lazy, immutable robot model shared by planning backends."""

    _source_path: Path | str | os.PathLike[str]
    _package_paths: tuple[tuple[str, Path | str | os.PathLike[str]], ...] = ()
    _xacro_args: tuple[tuple[str, str], ...] = ()
    _fixed_frames: tuple[_FixedFrame, ...] = ()

    @classmethod
    def from_file(
        cls,
        source_path: Path | str | os.PathLike[str],
        *,
        package_paths: Mapping[str, Path | str | os.PathLike[str]] | None = None,
        xacro_args: Mapping[str, str] | None = None,
    ) -> RobotModel:
        """Create a lazy model from a URDF or Xacro source."""
        suffix = source_path.suffix if isinstance(source_path, Path) else Path(source_path).suffix
        if suffix.lower() not in {".urdf", ".xacro"}:
            raise ValueError(
                f"RobotModel source must reference a .urdf or .xacro file, "
                f"got {Path(source_path).name!r}"
            )
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

    def load(self) -> LoadedRobotModel:
        """Materialize and memoize the model for a backend adapter."""
        return self._loaded

    @cached_property
    def _loaded(self) -> LoadedRobotModel:
        source_path = Path(os.fspath(self._source_path)).resolve()
        package_paths = _normalize_package_paths(dict(self._package_paths))
        if not source_path.exists():
            raise FileNotFoundError(f"Robot model not found: {source_path}")

        if source_path.suffix.lower() == ".xacro":
            xml = expand_xacro(source_path, package_paths, dict(self._xacro_args))
        else:
            xml = source_path.read_text()
        xml = _resolve_package_uris(xml, package_paths)
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
