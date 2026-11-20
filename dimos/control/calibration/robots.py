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

"""The Damiao robots the calibration tools know by name, e.g. ``--robot dual-openyam --arm left``."""

from __future__ import annotations

from dimos.control.calibration.spec import ArmCalibrationSpec, arm_name, specs_from_damiao_robot
from dimos.robot.manipulators.damiao.robot import DamiaoRobot
from dimos.robot.manipulators.dual_makermods_metal.config import DUAL_METAL
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


def robot_specs(robot: str) -> list[ArmCalibrationSpec]:
    """Every arm of a known robot, in bus order."""
    if robot not in KNOWN_ROBOTS:
        raise ValueError(f"unknown robot {robot!r}; known: {', '.join(sorted(KNOWN_ROBOTS))}")
    return specs_from_damiao_robot(KNOWN_ROBOTS[robot])


def arm_spec(robot: str, arm: str | None = None) -> ArmCalibrationSpec:
    """One arm of a known robot, by arm ("left"), bus ("left_arm") or arm id; optional for one arm."""
    specs = robot_specs(robot)
    names = [spec.arm_id.rsplit("/", 1)[-1] for spec in specs]
    if arm is None:
        if len(specs) != 1:
            raise ValueError(f"{robot} has several arms; choose one of {names}")
        return specs[0]
    for spec, bus in zip(specs, KNOWN_ROBOTS[robot].buses, strict=True):
        if arm in (spec.arm_id, spec.arm_id.rsplit("/", 1)[-1], bus.name, arm_name(bus.name)):
            return spec
    raise ValueError(f"{robot} has no arm {arm!r}; choose one of {names}")
