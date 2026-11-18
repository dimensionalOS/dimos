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

"""The Damiao robots the calibration tools know by name, e.g. ``--robot dual-openyam --arm left``.

``robot_specs`` applies a robot's own calibration setup where it has one (the
Dual OpenYAM bench guard, pose boxes and grid); every other robot gets the
defaults derived from its model, whose implicit workspace guard lets its poses
be fitted but not generated. Arm ids are the robot's ``arm_ids()``.
"""

from __future__ import annotations

from collections.abc import Callable

from dimos.robot.manipulators.calibration.spec import (
    ArmCalibrationSpec,
    arm_name,
    specs_from_damiao_robot,
)
from dimos.robot.manipulators.damiao.robot import DamiaoRobot
from dimos.robot.manipulators.dual_makermods_metal.config import DUAL_METAL
from dimos.robot.manipulators.dual_openyam.calibration_workspace import (
    dual_openyam_calibration_specs,
)
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM
from dimos.robot.manipulators.makermods_metal.config import METAL
from dimos.robot.manipulators.openarm.config import OPENARM
from dimos.robot.manipulators.openyam.config import OPENYAM
from dimos.robot.manipulators.seeed_b601_dm.config import B601_DM

KNOWN_ROBOTS: dict[str, DamiaoRobot] = {
    "openyam": OPENYAM,
    "dual-openyam": DUAL_OPENYAM,
    "metal": METAL,
    "dual-metal": DUAL_METAL,
    "b601-dm": B601_DM,
    "openarm": OPENARM,
}

# Robots whose calibration setup is more than the model-derived defaults.
_CUSTOM_SPECS: dict[str, Callable[[], list[ArmCalibrationSpec]]] = {
    "dual-openyam": dual_openyam_calibration_specs,
}


def robot_id(robot: str) -> str:
    """The robot part of its arm ids, its ``DamiaoRobot.name``: ``dual-openyam`` -> ``dual_openyam``."""
    return robot.replace("-", "_")


def robot_specs(robot: str) -> list[ArmCalibrationSpec]:
    """Every arm of a known robot, in bus order."""
    if robot not in KNOWN_ROBOTS:
        raise ValueError(f"unknown robot {robot!r}; known: {', '.join(sorted(KNOWN_ROBOTS))}")
    custom = _CUSTOM_SPECS.get(robot)
    if custom is not None:
        return custom()
    return specs_from_damiao_robot(KNOWN_ROBOTS[robot])


def arm_spec(robot: str, arm: str | None = None) -> ArmCalibrationSpec:
    """One arm of a known robot.

    Args:
        robot: A ``KNOWN_ROBOTS`` key, e.g. "dual-openyam".
        arm: The arm ("left"), its bus ("left_arm") or its arm id
            ("dual_openyam/left"); may be omitted for a single-arm robot.
    """
    specs = robot_specs(robot)
    if arm is None:
        if len(specs) != 1:
            names = [spec.arm_id.rsplit("/", 1)[-1] for spec in specs]
            raise ValueError(f"{robot} has several arms; choose one of {names}")
        return specs[0]
    buses = KNOWN_ROBOTS[robot].buses
    for spec, bus in zip(specs, buses, strict=True):
        if arm in (spec.arm_id, spec.arm_id.rsplit("/", 1)[-1], bus.name, arm_name(bus.name)):
            return spec
    names = [spec.arm_id.rsplit("/", 1)[-1] for spec in specs]
    raise ValueError(f"{robot} has no arm {arm!r}; choose one of {names}")
