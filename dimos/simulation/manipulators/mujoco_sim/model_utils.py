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

"""Utilities for loading MuJoCo models for manipulator simulation."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import mujoco

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def find_model_path(robot_name: str, num_joints: int | None = None) -> Path:
    """
    Find the path to a MuJoCo model XML file.

    Args:
        robot_name: Name of the robot (e.g., "piper", "xarm")
        num_joints: Optional number of joints (for robots with multiple variants like xarm6, xarm7)

    Returns:
        Path to the scene.xml file

    Raises:
        FileNotFoundError: If the model file is not found
    """
    # Path: dimos/simulation/manipulators/mujoco_sim/ -> dimos/simulation/manipulators/data/
    base_path = Path(__file__).parent.parent / "data"

    # Try DOF-based path first (for robots like xarm6, xarm7)
    if num_joints is not None:
        model_folder = f"{robot_name}{num_joints}"
        model_path = base_path / model_folder / "scene.xml"
        if model_path.exists():
            return model_path

    # Fall back to simple robot name (for robots like piper)
    model_folder = robot_name
    model_path = base_path / model_folder / "scene.xml"

    if not model_path.exists():
        available = []
        if base_path.exists():
            available = [d.name for d in base_path.iterdir() if d.is_dir()]
        raise FileNotFoundError(
            f"MuJoCo model not found: {model_path}\n"
            f"Robot: {robot_name}, DOF: {num_joints}\n"
            f"Tried paths: {base_path / f'{robot_name}{num_joints}' / 'scene.xml' if num_joints else 'N/A'}, {model_path}\n"
            f"Available models: {', '.join(available) if available else 'none'}"
        )

    return model_path


def load_manipulator_model(
    robot_name: str,
    num_joints: int | None = None,
    model_path: Path | str | None = None,
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """
    Load a MuJoCo model and data for a manipulator.

    Args:
        robot_name: Name of the robot (e.g., "piper", "xarm")
        num_joints: Optional number of joints (for robots with multiple variants)
        model_path: Optional explicit path to model. If None, will be found automatically.

    Returns:
        Tuple of (MjModel, MjData)

    Raises:
        FileNotFoundError: If the model file is not found
    """
    if model_path is None:
        model_path = find_model_path(robot_name, num_joints)
    elif isinstance(model_path, str):
        model_path = Path(model_path)

    if not model_path.exists():
        raise FileNotFoundError(f"MuJoCo model not found: {model_path}")

    logger.info(f"Loading MuJoCo model: {model_path}")

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    return model, data
