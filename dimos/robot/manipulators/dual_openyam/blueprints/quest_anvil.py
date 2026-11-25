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

"""Quest teleoperation and collection for the dual OpenYAM with Anvil's tuning.

Left X sends the left arm home and right A the right arm, each by a planned
move. Right B starts a take, ends it, then saves it; left Y discards it. The
operator console page stops the robot, saves a home per arm and moves an arm
home or to rest without the planner.
"""

from dimos.core.coordination.blueprints import MetaBlueprint, autoconnect
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import collect
from dimos.manipulation.arm_home import ArmHomeModule
from dimos.robot.manipulators.dual_openyam.anvil import (
    ANVIL_OPENYAM_HOME,
    ANVIL_OPENYAM_HOME_VIA,
    ANVIL_OPENYAM_MAX_JOINT_SPEED,
    ANVIL_OPENYAM_REST,
    anvil_dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.blueprints.leader_collection import (
    DUAL_OPENYAM_CAMERAS,
    dual_openyam_collection_camera,
    dual_openyam_collection_profile,
)
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import build_dual_openyam_webxr
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM_SIDES
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamAnvilConnection
from dimos.teleop.console.module import OperatorConsoleModule

# The Quest button that sends each arm home, and the grip that teleoperates it.
_HOME_BUTTONS = {"left": ("X", "LG"), "right": ("A", "RG")}


def _teleop() -> MetaBlueprint:
    return build_dual_openyam_webxr(
        connection=DualOpenYamAnvilConnection,
        model=anvil_dual_openyam_model_config(robot="arm"),
        # The model carries Anvil's speed limit per joint; this only lifts the task's own cap.
        teleop_params={"max_joint_velocity_rad_s": max(ANVIL_OPENYAM_MAX_JOINT_SPEED)},
        # B belongs to the episode monitor here; X and A send the arms home.
        zero_button=None,
    )


def _arm_home() -> MetaBlueprint:
    return autoconnect(
        ArmHomeModule.blueprint(
            robot="dual_openyam_anvil",
            arms={
                f"{side}_manipulator": {
                    "home": ANVIL_OPENYAM_HOME,
                    "rest": ANVIL_OPENYAM_REST,
                    "via": ANVIL_OPENYAM_HOME_VIA,
                    "max_joint_speeds": list(ANVIL_OPENYAM_MAX_JOINT_SPEED),
                    "button": _HOME_BUTTONS[side][0],
                    "grip": _HOME_BUTTONS[side][1],
                }
                for side in DUAL_OPENYAM_SIDES
            },
        ),
        OperatorConsoleModule.blueprint(),
    )


teleop_webxr_dual_openyam_anvil = autoconnect(_teleop(), _arm_home())

learning_collect_webxr_dual_openyam_anvil = autoconnect(
    collect(profile=dual_openyam_collection_profile("dual-openyam-quest-anvil")),
    EpisodeMonitorModule.blueprint(instance_name="episodes", confirm_save=True),
    _teleop(),
    _arm_home(),
    *[dual_openyam_collection_camera(name) for name in DUAL_OPENYAM_CAMERAS],
)
