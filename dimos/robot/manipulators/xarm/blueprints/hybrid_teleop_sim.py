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

"""Simulated xArm7 pick-and-place stack that an operator can hold.

The same modules as xarm-perception-sim, on a coordinator that also runs the
operator hold and takes hold commands and sends hold status as streams.
"""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.task import ControlTask
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OperatorHoldCommand,
    OperatorHoldStatus,
    OperatorHoldTask,
    operator_hold_task,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule


class OperatorHoldCoordinator(ControlCoordinator):
    """Add the ports the operator_hold task card reads and reports on.

    ``operator_hold_command`` takes an OperatorHoldCommand that requests or
    acknowledges the hold. ``operator_hold_status`` carries every
    OperatorHoldStatus the hold task sends. Combine it with another coordinator
    subclass by inheriting from both, e.g.
    ``class MyCoordinator(OperatorHoldCoordinator, TeleopControlCoordinator)``.
    """

    operator_hold_command: In[OperatorHoldCommand]
    operator_hold_status: Out[OperatorHoldStatus]

    @rpc
    def add_task(
        self,
        task: ControlTask,
        task_type: str | None = None,
        stream_bind: dict[str, str] | None = None,
    ) -> bool:
        """Register a task; a hold task's status also goes out on operator_hold_status."""
        added = super().add_task(task, task_type=task_type, stream_bind=stream_bind)
        if added and isinstance(task, OperatorHoldTask):
            task.set_status_publisher(self.operator_hold_status.publish)
        return added


_xarm7_sim_hw = make_xarm7_sim_hardware()

hybrid_teleop_sim = autoconnect(
    ManipulationModule.blueprint(
        model=make_xarm7_sim_robot_config(),
        planning_timeout=10.0,
        visualization={"backend": "viser"},
    ),
    ManipulationSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world"),
    HeuristicGraspModule.blueprint(),
    MujocoSimModule.blueprint(**make_xarm7_sim_module_kwargs(XARM7_SIM_PATH)),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        detector_backend="moondream",
        segmentation_backend="edgetam",
        detect_on_request=True,
    ),
    coordinator(
        hardware=[_xarm7_sim_hw],
        tasks=[
            trajectory_task(_xarm7_sim_hw),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
            ),
            operator_hold_task(),
        ],
        cls=OperatorHoldCoordinator,
        instance_name="ControlCoordinator",
    ),
    RerunBridgeModule.blueprint(),
)
