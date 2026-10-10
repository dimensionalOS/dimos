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

"""``panda-libero-sim``: the ``xarm-perception-sim`` stack on LIBERO's own Panda.

Same planner, skills, perception and coordinator as the xArm sim, with the Panda's
planning model; ``LiberoSim`` takes ``MujocoSimModule``'s place and the arm is driven
over topics instead of shared memory. Pick the task with ``LIBEROSIM__BDDL=<file.bddl>``.
"""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport_factory import make_transport
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.panda.config import make_panda_hardware, make_panda_model_config
from dimos.simulation.libero.module import LiberoSim
from dimos.visualization.rerun.bridge import RerunBridgeModule

_hardware = make_panda_hardware("arm")

panda_libero_sim = autoconnect(
    ManipulationModule.blueprint(
        # LiberoSim publishes poses with the Panda's link0 at the world origin.
        model=make_panda_model_config(gripper_hardware_id="arm", tf_extra_links=["link7"]),
        planning_timeout=10.0,
        visualization={"backend": "viser"},
    ),
    ManipulationSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world"),
    HeuristicGraspModule.blueprint(),
    LiberoSim.blueprint(),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        detector_backend="moondream",
        segmentation_backend="edgetam",
        detect_on_request=True,
    ),
    coordinator(
        hardware=[_hardware],
        tasks=[
            trajectory_task(_hardware),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
            ),
        ],
    ),
    RerunBridgeModule.blueprint(),
).transports(
    {
        # The sim_transport adapter's topics for hardware "arm".
        ("sim_state", JointState): make_transport("/arm/sim_state", JointState),
        ("sim_command", JointState): make_transport("/arm/sim_command", JointState),
    }
)
