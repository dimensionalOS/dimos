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

"""Locate mujoco_playground's mujoco_menagerie checkout without importing the package."""

import importlib.util
from pathlib import Path

SIM_INSTALL_HINT = "Simulation dependencies are not installed. Run `uv sync --extra sim --inexact` to install them."


def menagerie_path() -> Path:
    # Same location as mujoco_playground._src.mjx_env.MENAGERIE_PATH, found without
    # importing mujoco_playground (a 4 s import: torch, jax, mjx).
    spec = importlib.util.find_spec("mujoco_playground")
    if spec is None or not spec.submodule_search_locations:
        raise ImportError(SIM_INSTALL_HINT)
    package_dir = Path(next(iter(spec.submodule_search_locations)))
    return package_dir / "external_deps" / "mujoco_menagerie"


def ensure_menagerie() -> None:
    """Clone the menagerie on first use; a no-op once it is on disk."""
    if menagerie_path().exists():
        return
    # mujoco_playground: 4 s import (torch, jax, mjx), only needed to clone once.
    from mujoco_playground._src import mjx_env

    mjx_env.ensure_menagerie_exists()
