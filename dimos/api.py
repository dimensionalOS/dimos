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

"""DimOS Python API — programmatic entry point for robot control.

Usage::

    import dimos

    robot = dimos.connect("unitree-go2")
    robot.module("UnitreeSkillContainer").relative_move(forward=1.0)
    robot.stop()
"""

from __future__ import annotations

from typing import Any

from dimos.core.blueprints import autoconnect
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.robot.get_all_blueprints import get_by_name


def connect(
    *blueprint_names: str,
    simulation: bool = False,
    **config: Any,
) -> ModuleCoordinator:
    """Launch a blueprint and return the coordinator.

    Args:
        blueprint_names: One or more blueprint names to compose.
        simulation: If True, sets ``simulation=True`` in global config.
        **config: Additional GlobalConfig overrides.

    Returns:
        A running ``ModuleCoordinator`` with all modules deployed and started.

    Example::

        robot = dimos.connect("unitree-go2")
        robot.module("UnitreeSkillContainer").execute_sport_command("StandUp")
        robot.stop()
    """
    blueprints = [get_by_name(name) for name in blueprint_names]
    merged = autoconnect(*blueprints)

    overrides = dict(config)
    if simulation:
        overrides["simulation"] = True

    return merged.build(cli_config_overrides=overrides)


def list_blueprints() -> list[str]:
    """List all available blueprint and module names."""
    from dimos.robot.all_blueprints import all_blueprints, all_modules

    return sorted(set(all_blueprints.keys()) | set(all_modules.keys()))
