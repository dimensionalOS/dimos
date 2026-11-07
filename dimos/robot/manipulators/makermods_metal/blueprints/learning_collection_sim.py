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

"""Quest-guided Metal collection rehearsed with the Metal in MuJoCo instead of the arm.

The same recorder, episode monitor, Quest teleop and planner as
``metal_quest_collection``; the sim's wrist camera stands in for the webcam.
"""

from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import collect
from dimos.imitation.dataprep.schema import DEFAULT_FPS
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.common.blueprints import coordinator
from dimos.robot.manipulators.makermods_metal.blueprints.learning_collection import (
    METAL_QUEST_PROFILE,
)
from dimos.robot.manipulators.makermods_metal.blueprints.teleop import (
    METAL_QUEST_KINEMATICS,
    METAL_QUEST_MODEL,
    metal_quest_tasks,
)
from dimos.robot.manipulators.makermods_metal.config import make_metal_sim_hardware
from dimos.robot.manipulators.makermods_metal.mjcf import make_metal_sim_module_kwargs
from dimos.robot.urdf_manager import UrdfManager
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.teleop.webxr.extensions import ArmTeleopModule

# The sim camera renders at the size the profile records and at the dataset
# rate: export resamples to DEFAULT_FPS and drops an episode whose camera ran
# below 95% of it.
_HEIGHT, _WIDTH = METAL_QUEST_PROFILE.observations["observation.images.wrist"].shape[:2]

# The recorder comes first so it stops after the producers it records.
metal_quest_collection_sim = autoconnect(
    collect(profile=METAL_QUEST_PROFILE),
    EpisodeMonitorModule.blueprint(instance_name="episodes"),
    ArmTeleopModule.blueprint(zero_button="X"),
    # The simulated arm is attached through shared memory, so the coordinator
    # needs only the Quest inputs.
    coordinator(
        hardware=[make_metal_sim_hardware()],
        tasks=metal_quest_tasks(),
        cls=TeleopControlCoordinator,
        instance_name="ControlCoordinator",
    ),
    MujocoSimModule.blueprint(
        **make_metal_sim_module_kwargs(), width=_WIDTH, height=_HEIGHT, fps=int(DEFAULT_FPS)
    ),
    ManipulationModule.blueprint(
        model=METAL_QUEST_MODEL,
        gripper_tasks={"arm": "metal_gripper"},
        kinematics=METAL_QUEST_KINEMATICS,
        visualization={"backend": "viser"},
        default_speed_scale=0.2,
    ),
    # The planner already puts the arm's links on tf.
    UrdfManager.blueprint(model=METAL_QUEST_MODEL.model, publish_joint_tf=False),
).remappings(
    [
        (ArmTeleopModule, "right_controller_output", "right_cartesian_command"),
        (MujocoSimModule, "color_image", "wrist_image"),
    ]
)
