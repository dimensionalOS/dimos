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

The same modules as xarm-perception-sim, plus the operator hold task, the
request_operator skill an agent calls to ask for the hold, on/off streams for
the hold and the e-stop, the operator event log (OperatorLog), a point cloud of
the scene on ``scene_pointcloud`` and the arm's URDF, so a Desktop page can
show the arm and the scene and stop or hold it. Picks try GraspGenX's grasps
first and the heuristic grasp module's after them.

``hybrid_teleop_sim_collect`` is hybrid-teleop-sim plus a collection session
(wrist camera and joint positions, recorded from launch to
``RECORDINGS_DIR/session_xarm7_sim_<date>_<time>``) and episode marking through
EpisodeMonitorModule's ``command`` RPC. ``--recorder.recording <path>`` names
the session folder at launch, and the operator can rename it afterwards from
Desktop.
"""

from __future__ import annotations

from dataclasses import asdict
from datetime import datetime
import json

from dimos.agents.skills.teleop_skills import TeleopSkills
from dimos.constants import RECORDINGS_DIR
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.operator_log import OperatorLog
from dimos.control.task import ControlTask
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OperatorHoldStatus,
    OperatorHoldTask,
    operator_hold_task,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recorder import collect
from dimos.imitation.dataprep.schema import FeatureSpec
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.String import String
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.blueprints.grasp import (
    XARM_GRASP_FRAME_TO_TCP,
    XARM_GRIPPER_SWEEP_VOLUME,
)
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

    ``operator_hold_status`` is True while the hold is on, and
    ``operator_hold`` is the hold task's whole OperatorHoldStatus as JSON; both
    are sent each time the hold task sends its status. ``operator_estop_status``
    is True while the e-stop is latched, sent at start and on every ``set_estop``.

    Combine it with another coordinator subclass by inheriting from both, e.g.
    ``class MyCoordinator(OperatorHoldCoordinator, TeleopControlCoordinator)``.
    """

    operator_hold_status: Out[Bool]
    operator_hold: Out[String]
    operator_estop_status: Out[Bool]

    @rpc
    def add_task(
        self,
        task: ControlTask,
        task_type: str | None = None,
        stream_bind: dict[str, str] | None = None,
    ) -> bool:
        """Register a task; a hold task also sends its status on the two hold streams."""
        added = super().add_task(task, task_type=task_type, stream_bind=stream_bind)
        if added and isinstance(task, OperatorHoldTask):
            task.set_status_publisher(self._publish_hold_status)
        return added

    def _publish_hold_status(self, status: OperatorHoldStatus) -> None:
        self.operator_hold_status.publish(Bool(data=status.on))
        self.operator_hold.publish(String(json.dumps(asdict(status))))

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

# The wrist camera's image size in pixels. The recording's profile states it,
# so both take it from here.
_CAMERA_WIDTH, _CAMERA_HEIGHT = 640, 480
_arm_state = FeatureSpec(
    stream="coordinator_joint_state",
    message_type=JointState,
    field="position",
    dtype="float32",
    shape=(len(_xarm7_sim_hw.joints),),
    names=list(_xarm7_sim_hw.joints),
)
# The xarm7 profile of dimos/imitation/collection/blueprint.py, with the sim
# camera's size instead of the RealSense's 848 x 480.
_collection_profile = CollectionProfile(
    metadata={"name": "xarm7-sim", "robot_type": "xarm7"},
    observations={
        "camera": FeatureSpec(
            stream="color_image",
            message_type=Image,
            field="data",
            dtype="video",
            shape=(_CAMERA_HEIGHT, _CAMERA_WIDTH, 3),
            names=["height", "width", "channels"],
        ),
        "state": _arm_state,
    },
    actions={"action": _arm_state.model_copy(deep=True)},
)

hybrid_teleop_sim = autoconnect(
    ManipulationModule.blueprint(
        model=_xarm7_sim_model,
        planning_timeout=10.0,
        visualization={"backend": "viser"},
    ),
    ManipulationSkills.blueprint(),
    TeleopSkills.blueprint(),
    OperatorLog.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world", robot=_xarm7_sim_model),
    GraspGenXModule.blueprint(
        gripper=XARM_GRIPPER_SWEEP_VOLUME,
        grasp_frame_to_tcp=XARM_GRASP_FRAME_TO_TCP,
        # Return every sample: the pick's contact check chooses among them.
        max_candidates=500,
    ),
    HeuristicGraspModule.blueprint(**XARM_HEURISTIC_AIM),
    # Rays from the wrist camera (80 x 60, about 1 cm apart on the table) give
    # world-frame points at 5 Hz. Renamed so they do not mix with the object
    # clouds ObjectSceneRegistrationModule sends on "pointcloud".
    MujocoSimModule.blueprint(
        **make_xarm7_sim_module_kwargs(XARM7_SIM_PATH),
        width=_CAMERA_WIDTH,
        height=_CAMERA_HEIGHT,
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
).remappings(
    [
        (PickAndPlaceModule, "_grasp_generator", GraspGenXModule),
        (PickAndPlaceModule, "_fallback_grasp_generator", HeuristicGraspModule),
    ]
)

hybrid_teleop_sim_collect = autoconnect(
    # First, so the recorder and the episode monitor stop after what they record.
    collect(
        profile=_collection_profile,
        recording=RECORDINGS_DIR / f"session_xarm7_sim_{datetime.now():%Y%m%d_%H%M%S}",
        format="sqlite",
    ),
    EpisodeMonitorModule.blueprint(task="Teleoperate the xArm (sim)"),
    hybrid_teleop_sim,
)
