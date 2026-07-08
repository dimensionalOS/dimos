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

"""Configuration helpers for OpenArm Mini leader teleoperation."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

from dimos.constants import STATE_DIR

OPENARM_MINI_TELEOP_EXTRA = "openarm-mini-teleop"
OPENARM_MINI_STATE_DIR = STATE_DIR / "teleop" / "openarm_mini"
OpenArmMiniSide = Literal["left", "right"]
OPENARM_MINI_SIDES: tuple[OpenArmMiniSide, OpenArmMiniSide] = ("left", "right")
OPENARM_MINI_UNCONFIGURED_PORT = ""
OPENARM_MINI_DEFAULT_BAUDRATE = 1_000_000


def default_calibration_path(side: OpenArmMiniSide) -> Path:
    """Return the default persistent calibration directory for an OpenArm Mini side."""
    return OPENARM_MINI_STATE_DIR / side


class OpenArmMiniDependencyError(ImportError):
    """Raised when the optional Feetech SDK dependency is unavailable."""


class OpenArmMiniCalibrationError(RuntimeError):
    """Raised when OpenArm Mini calibration is missing or invalid."""


def missing_dependency_error() -> OpenArmMiniDependencyError:
    """Build the localized missing dependency error for OpenArm Mini teleop."""
    return OpenArmMiniDependencyError(
        "OpenArm Mini teleop requires the Feetech SDK. Install it with "
        "`uv sync --extra openarm`, "
        f"`uv sync --extra {OPENARM_MINI_TELEOP_EXTRA}`, or "
        f"`pip install 'dimos[{OPENARM_MINI_TELEOP_EXTRA}]'`."
    )
