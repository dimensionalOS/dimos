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

"""OpenArm v10 robot configurations."""

from __future__ import annotations

from typing import Any

from dimos.robot.config import RobotConfig
from dimos.utils.data import LfsPath

# Collision exclusion pairs — structural mesh overlaps in the OpenArm URDF.
# link5 and link7 collision meshes overlap by ~3mm at zero pose (and every
# other pose) — same pattern as R1 Pro's non-adjacent link overlap.
OPENARM_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("openarm_left_link5", "openarm_left_link7"),
    ("openarm_right_link5", "openarm_right_link7"),
]

# LFS-backed: data/.lfs/openarm_description.tar.gz extracts to data/openarm_description/
_OPENARM_PKG = LfsPath("openarm_description")
_OPENARM_MODEL_PATH = _OPENARM_PKG / "urdf/robot/openarm_v10_bimanual.urdf"
# Per-side URDFs: extracted from bimanual expansion, only one arm + torso each.
# Avoids phantom-arm collisions when Drake loads both sides into one world.
_OPENARM_LEFT_MODEL = _OPENARM_PKG / "urdf/robot/openarm_v10_left.urdf"
_OPENARM_RIGHT_MODEL = _OPENARM_PKG / "urdf/robot/openarm_v10_right.urdf"

# Pre-expanded single-arm URDF for Pinocchio FK (keyboard teleop, IK, etc.)
OPENARM_V10_FK_MODEL = _OPENARM_PKG / "urdf/robot/openarm_v10_single.urdf"


def openarm_arm(
    side: str = "left",
    name: str | None = None,
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    **overrides: Any,
) -> RobotConfig:
    """OpenArm v10 config for one side. Uses per-side URDF (arm + torso only)."""
    if side not in ("left", "right"):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")

    resolved_name = name or f"{side}_arm"
    # Pre-expanded bimanual URDF uses openarm_{side}_* naming.
    joint_names = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
    ee_link = f"openarm_{side}_link7"

    defaults: dict[str, Any] = {
        "name": resolved_name,
        "model_path": _OPENARM_LEFT_MODEL if side == "left" else _OPENARM_RIGHT_MODEL,
        "end_effector_link": ee_link,
        "adapter_type": adapter_type,
        "address": address,
        "joint_names": joint_names,
        # URDF already prefixes joints with "left_"/"right_" in bimanual mode,
        # so suppress RobotConfig's automatic "{name}_" prefix.
        "joint_prefix": "",
        "base_link": "openarm_body_link0",
        "home_joints": [0.0] * 7,
        "base_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "package_paths": {"openarm_description": _OPENARM_PKG},
        "collision_exclusion_pairs": OPENARM_COLLISION_EXCLUSIONS,
        "auto_convert_meshes": True,
        "max_velocity": 0.5,
        "max_acceleration": 1.0,
        "adapter_kwargs": {"side": side},
    }
    # Merge adapter_kwargs rather than replace, so callers can add keys
    # (e.g. auto_set_mit_mode) without clobbering the catalog's "side".
    if "adapter_kwargs" in overrides:
        defaults["adapter_kwargs"] = {
            **defaults["adapter_kwargs"],
            **overrides.pop("adapter_kwargs"),
        }
    defaults.update(overrides)
    return RobotConfig(**defaults)


def openarm_single(
    name: str = "arm",
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    **overrides: Any,
) -> RobotConfig:
    """Single-arm config (keyboard teleop, cartesian IK). Use openarm_arm() for bimanual."""
    defaults: dict[str, Any] = {
        "name": name,
        "model_path": OPENARM_V10_FK_MODEL,
        "end_effector_link": "openarm_left_link7",
        "adapter_type": adapter_type,
        "address": address,
        "joint_names": [f"openarm_left_joint{i}" for i in range(1, 8)],
        "joint_prefix": "",
        "base_link": "openarm_body_link0",
        "home_joints": [0.0] * 7,
        "package_paths": {"openarm_description": _OPENARM_PKG},
        "auto_convert_meshes": True,
        "max_velocity": 0.5,
        "max_acceleration": 1.0,
        "adapter_kwargs": {"side": "left"},
    }
    defaults.update(overrides)
    return RobotConfig(**defaults)


__all__ = ["OPENARM_V10_FK_MODEL", "openarm_arm", "openarm_single"]
