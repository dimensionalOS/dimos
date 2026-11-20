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

"""The shared controller config (m20_controller.toml) and where the vendor policies live."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import numpy as np
import tomllib

CONFIG_PATH = (
    Path(__file__).resolve().parents[2]
    / "robot/deeprobotics/m20/controller/rust/m20_controller.toml"
)
# extract_policies.py writes here by default; M20_POLICY_DIR overrides
DEFAULT_POLICY_DIR = Path("~/.cache/dimos/m20_policies").expanduser()

JOINTS = 16
LEGS_BY_TYPE = np.array([0, 4, 8, 12, 1, 5, 9, 13, 2, 6, 10, 14])
LEGS_PER_LEG = np.array([0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14])
WHEELS = np.array([3, 7, 11, 15])
ALL_PER_LEG = np.arange(JOINTS)
JOINT_SETS = {
    "legs_by_type": LEGS_BY_TYPE,
    "legs_per_leg": LEGS_PER_LEG,
    "wheels": WHEELS,
    "all_per_leg": ALL_PER_LEG,
}
IS_WHEEL = np.arange(JOINTS) % 4 == 3
IS_HIPX = np.arange(JOINTS) % 4 == 0


def policy_dir() -> Path:
    return Path(os.environ.get("M20_POLICY_DIR", DEFAULT_POLICY_DIR)).expanduser()


def load_config(path: Path = CONFIG_PATH) -> dict[str, Any]:
    with open(path, "rb") as f:
        return tomllib.load(f)


def tile(values: Any, n: int) -> np.ndarray:
    """A 1-element array broadcast, or a shorter array repeated, to length n."""
    arr = np.asarray(values, np.float64).ravel()
    if n % len(arr):
        raise ValueError(f"cannot tile {len(arr)} values to {n}")
    return np.tile(arr, n // len(arr))


def runner_for_gait(cfg: dict[str, Any], gait: int) -> str | None:
    for name, runner in cfg["runners"].items():
        if gait in runner["gaits"]:
            return name
    return None
