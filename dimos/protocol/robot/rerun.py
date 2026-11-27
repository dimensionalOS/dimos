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

"""Robots in rerun. Owns what has to be logged once (meshes), so the values stay plain.

SKETCH: signatures only.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

from dimos.protocol.robot.model import RobotModel
from dimos.protocol.robot.state import RobotState


class RobotLogger:
    def __init__(
        self, model: RobotModel, packages: Mapping[str, Path] | None = None, entity: str = ""
    ) -> None:
        """Meshes resolve on this machine through `packages`; ones that don't resolve
        are skipped (kinematics still render as frames)."""
        raise NotImplementedError

    def log(self, state: RobotState) -> None:
        """Link poses relative to the root frame; meshes on first call or model change.
        Rerun places the root from tf like any other frame."""
        raise NotImplementedError
