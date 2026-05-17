#!/usr/bin/env python3
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

"""G1 in the pimsim Babylon viewer — no nav stack, just the viewer + manual drive.

Use ``uv run dimos run unitree-g1-pimsim`` for a quick visual + cmd_vel
sandbox in the dimos_office scene.
"""

from __future__ import annotations

from pathlib import Path

from mujoco_playground._src import mjx_env

from dimos.core.coordination.blueprints import autoconnect
from dimos.experimental.pimsim.module import BabylonSceneViewerModule
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.g1.config import G1_PELVIS_HEIGHT
from dimos.simulation.scenes import get_dimos_office


def _resolve_g1_mjcf() -> Path:
    mjx_env.ensure_menagerie_exists()
    return Path(mjx_env.MENAGERIE_PATH) / "unitree_g1" / "scene.xml"


_OFFICE = get_dimos_office()

unitree_g1_pimsim = autoconnect(
    BabylonSceneViewerModule.blueprint(
        mjcf_path=str(_resolve_g1_mjcf()),
        scene=_OFFICE,
        vehicle_height=G1_PELVIS_HEIGHT,
        init_x=-2.0,
        init_z=_OFFICE.floor_z,
        lock_z=True,
        enable_sim=True,
    ),
    MovementManager.blueprint(),
).global_config(n_workers=4, robot_model="unitree_g1", simulation=True)
