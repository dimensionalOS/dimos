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

"""Validate and stage suite-selected robot-only context for coding agents."""

import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil

_REQUIRED = {"README.md", "robot.urdf", "gripper.urdf", "robot_info.json"}
ROBOT_CONTEXT_DIR_ENV = "DIMOS_ROBOT_CONTEXT_DIR"


def local_robot_context(name: str) -> Path | None:
    """``$DIMOS_ROBOT_CONTEXT_DIR/<name>``; None (no robot context) when the variable is unset."""
    root = os.environ.get(ROBOT_CONTEXT_DIR_ENV)
    return Path(root).expanduser() / name if root else None


def robot_context_files(source: Path) -> dict[str, Path]:
    """Validate the disclosure manifest; unrelated source-directory files stay private."""
    root = source.resolve()
    files = json.loads((root / "manifest.json").read_text())["files"]
    if not isinstance(files, dict) or not _REQUIRED.issubset(files):
        raise ValueError(
            "Robot context needs README.md, robot.urdf, gripper.urdf, and robot_info.json"
        )
    resolved: dict[str, Path] = {}
    for name, expected_digest in files.items():
        relative = PurePosixPath(name)
        if relative.is_absolute() or ".." in relative.parts or not relative.parts:
            raise ValueError(f"Invalid robot context path: {name}")
        if name not in _REQUIRED and relative.parts[0] not in {"meshes", "LICENSES"}:
            raise ValueError(f"Not a robot-description asset: {name}")
        path = (root / name).resolve(strict=True)
        if not path.is_relative_to(root):
            raise ValueError(f"Robot context asset escapes its bundle: {name}")
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected_digest:
            raise ValueError(f"Robot context checksum mismatch: {name}")
        resolved[name] = path
    return resolved


def stage_robot_context(source: Path, destination: Path) -> dict[str, Path]:
    """Copy only manifest-listed files into the agent's own, keyword-guard-safe workspace."""
    files = robot_context_files(source)
    for relative, path in files.items():
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
    shutil.copy2(source / "manifest.json", destination / "manifest.json")
    return {
        "robot_context": destination / "README.md",
        "robot_urdf": destination / "robot.urdf",
        "gripper_urdf": destination / "gripper.urdf",
        "robot_info": destination / "robot_info.json",
    }
