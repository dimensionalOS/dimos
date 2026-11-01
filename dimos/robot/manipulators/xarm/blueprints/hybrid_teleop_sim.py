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

"""Simulated xArm7 pick and place that an operator can hold and e-stop.

The same modules as xarm-perception-sim, plus the operator hold task, on/off
streams for the hold and the e-stop, a point cloud of the scene on
``scene_pointcloud`` and the arm's URDF, so a Desktop page can show the arm and
the scene and stop or hold it.
"""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.task import ControlTask
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OperatorHoldTask,
    operator_hold_task,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.std_msgs.Bool import Bool
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    XARM_HEURISTIC_AIM,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.robot.urdf_manager import UrdfManager
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.bridge import RerunBridgeModule

logger = setup_logger()


class OperatorHoldCoordinator(ControlCoordinator):
    """A ControlCoordinator that only adds on/off streams for the hold and the e-stop.

    The e-stop is ControlCoordinator's own ``set_estop``. The hold is the "operator_hold" task, reached over ``task_invoke``
    ("request", "acknowledge", "get_status"); an e-stop turns it on and it
    stays on until acknowledged.

    ``operator_hold_status`` is True while the hold is on, sent each time the
    hold task sends its status. ``operator_estop_status`` is True while the
    e-stop is latched, sent at start and on every ``set_estop``.

    Combine it with another coordinator subclass by inheriting from both, e.g.
    ``class MyCoordinator(OperatorHoldCoordinator, TeleopControlCoordinator)``.
    """

    operator_hold_status: Out[Bool]
    operator_estop_status: Out[Bool]

    @rpc
    def add_task(
        self,
        task: ControlTask,
        task_type: str | None = None,
        stream_bind: dict[str, str] | None = None,
    ) -> bool:
        """Register a task; a hold task also sends its on/off on operator_hold_status."""
        added = super().add_task(task, task_type=task_type, stream_bind=stream_bind)
        if added and isinstance(task, OperatorHoldTask):
            task.set_status_publisher(
                lambda status: self.operator_hold_status.publish(Bool(data=status.on))
            )
        return added

    @rpc
    def start(self) -> None:
        """Start the coordinator and send the current e-stop state."""
        super().start()
        self._publish_estop_status()

    @rpc
    def set_estop(self, estopped: bool) -> bool:
        """Stop (True) or clear (False) like ControlCoordinator, then send the state.

        Args:
            estopped: True to stop, False to clear.
        """
        try:
            return super().set_estop(estopped)
        finally:
            self._publish_estop_status()

    def _publish_estop_status(self) -> None:
        try:
            self.operator_estop_status.publish(Bool(data=self._estopped))
        except Exception:
            logger.exception("Publishing e-stop status failed")


# The planner publishes these links on tf at the arm's real mount; the Desktop
# draws each URDF link at its tf frame.
_ARM_LINKS = ["link_base", *(f"link{i}" for i in range(1, 8))]
_xarm7_sim_model = make_xarm7_sim_robot_config(tf_extra_links=_ARM_LINKS)
_xarm7_sim_hw = make_xarm7_sim_hardware()

hybrid_teleop_sim = autoconnect(
    ManipulationModule.blueprint(
        model=_xarm7_sim_model,
        planning_timeout=10.0,
        visualization={"backend": "viser"},
    ),
    ManipulationSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world", robot=_xarm7_sim_model),
    HeuristicGraspModule.blueprint(**XARM_HEURISTIC_AIM),
    # Rays from the wrist camera (80 x 60, about 1 cm apart on the table) give
    # world-frame points at 5 Hz. Renamed so they do not mix with the object
    # clouds ObjectSceneRegistrationModule sends on "pointcloud".
    MujocoSimModule.blueprint(
        **make_xarm7_sim_module_kwargs(XARM7_SIM_PATH),
        enable_pointcloud=True,
        enable_mujoco_lidar=True,
        mujoco_lidar_raycast_width=80,
        mujoco_lidar_raycast_height=60,
        mujoco_lidar_voxel_size=0.01,
    ).remappings([(MujocoSimModule, "pointcloud", "scene_pointcloud")]),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        detector_backend="moondream",
        segmentation_backend="edgetam",
        detect_on_request=True,
        # Detections closer than this (metres) count as one object. The objects
        # on the table are 10 to 17 cm apart; the 0.2 m default merges them.
        distance_threshold=0.05,
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
    # Serves the URDF and its meshes; the planner already puts the links on tf.
    UrdfManager.blueprint(model=_xarm7_sim_model.model, publish_joint_tf=False),
    RerunBridgeModule.blueprint(),
)
