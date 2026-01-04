#!/usr/bin/env python3
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

from __future__ import annotations

from contextlib import contextmanager
import os
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Iterator


def _remove_from_path(path_to_remove: Path) -> None:
    path_to_remove = path_to_remove.resolve()
    parts = [Path(p) for p in os.environ.get("PATH", "").split(os.pathsep) if p]
    filtered = [str(p) for p in parts if p.resolve() != path_to_remove]
    os.environ["PATH"] = os.pathsep.join(filtered)


def deactivate_external() -> None:
    old_virtual_path = os.environ.pop("_OLD_VIRTUAL_PATH", "")
    if old_virtual_path:
        env_path = os.environ.get("VIRTUAL_ENV")
        if env_path:
            _remove_from_path(Path(env_path) / "bin")
        else:
            os.environ["PATH"] = old_virtual_path
    os.environ.pop("_OLD_VIRTUAL_PYTHONHOME", None)
    os.environ.pop("_OLD_VIRTUAL_PS1", None)
    os.environ.pop("VIRTUAL_ENV", None)
    os.environ.pop("VIRTUAL_ENV_PROMPT", None)


def purge_broken_external_venv() -> None:
    """
    Note: this is useful for docker inheriting a venv from the host
    """
    env_path = os.environ.get("VIRTUAL_ENV")
    if env_path:
        if not Path(env_path).exists():
            _remove_from_path(Path(env_path) / "bin")
            os.environ.pop("_OLD_VIRTUAL_PYTHONHOME", None)
            os.environ.pop("_OLD_VIRTUAL_PS1", None)
            os.environ.pop("VIRTUAL_ENV", None)
            os.environ.pop("VIRTUAL_ENV_PROMPT", None)


def activate_venv(project_directory: str | Path):
    project_directory = Path(project_directory).resolve()
    bin_dir = project_directory / ("Scripts" if os.name == "nt" else "bin")

    old_env = dict(os.environ)
    deactivate_external()

    os.environ["VIRTUAL_ENV"] = str(project_directory)
    os.environ["_OLD_VIRTUAL_PATH"] = old_env.get("PATH", "")
    os.environ["PATH"] = os.pathsep.join([str(bin_dir), old_env.get("PATH", "")])
    if "PYTHONHOME" in os.environ:
        os.environ["_OLD_VIRTUAL_PYTHONHOME"] = os.environ["PYTHONHOME"]
        os.environ.pop("PYTHONHOME", None)

    def deactivate():
        os.environ.clear()
        os.environ.update(old_env)

    return deactivate


def get_venv_dirs_at(path: str | Path) -> list[str]:
    path = Path(path)
    valid_activate_paths: list[str] = []
    for activate_file in path.glob("*/bin/activate"):
        python_cmd = activate_file.parent / "python"
        if python_cmd.is_file():
            valid_activate_paths.append(str(activate_file.parent.parent))
    return valid_activate_paths


@contextmanager
def activated_venv(project_directory: str | Path) -> Iterator[None]:
    deactivate = activate_venv(project_directory)
    try:
        yield
    finally:
        deactivate()


__all__ = ["activate_venv", "activated_venv", "deactivate_external", "get_venv_dirs_at"]
