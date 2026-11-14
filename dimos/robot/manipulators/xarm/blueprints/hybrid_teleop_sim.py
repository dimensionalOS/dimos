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
request_operator skill an agent calls to ask for the hold, an end-effector jog
on ``ee_twist_command``, on/off streams for the hold and the e-stop, a point
cloud of the scene on ``scene_pointcloud`` and the arm's URDF, so a Desktop page
can show the arm and the scene, stop or hold it, and drive it by hand after
taking over. An agent can ask either grasp generator, GraspGenX or the
heuristic one, for grasps with propose_grasps; a pick given no grasp
generates its own with GraspGenX.

``hybrid_teleop_sim_collect`` is hybrid-teleop-sim plus a collection session
(wrist camera and joint positions, recorded from launch to
``RECORDINGS_DIR/session_xarm7_sim_<date>_<time>``) and episode marking through
EpisodeMonitorModule's ``command`` RPC. ``--recorder.recording <path>`` names
the session folder at launch, and the operator can rename it afterwards from
Desktop.
"""

from __future__ import annotations

from datetime import datetime

from dimos.agents.skills.teleop_skills import TeleopSkills
from dimos.constants import RECORDINGS_DIR
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.task import ControlTask
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OperatorHoldTask,
    operator_hold_task,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recorder import collect
from dimos.imitation.dataprep.schema import DEFAULT_FPS, FeatureSpec
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_localize import ManipulationLocalize
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Bool import Bool
from dimos.robot.manipulators.common.blueprints import (
    coordinator,
    eef_twist_task,
    trajectory_task,
)
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
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


class HybridTeleopCoordinator(OperatorHoldCoordinator, ArmTwistCoordinator):
    """OperatorHoldCoordinator plus the ``ee_twist_command`` input the jog task reads.

    A TwistStamped on ``ee_twist_command`` moves the gripper tip at that
    velocity (m/s, rad/s, world frame); a zero twist, or none for 0.3 s, stops it.
    """


# The planner publishes these links on tf at the arm's real mount; the Desktop
# draws each URDF link at its tf frame.
_ARM_LINKS = ["link_base", *(f"link{i}" for i in range(1, 8))]
_xarm7_sim_model = make_xarm7_sim_robot_config(tf_extra_links=_ARM_LINKS)
_xarm7_sim_hw = make_xarm7_sim_hardware()

# The wrist camera's image size in pixels. The recording's profile states it,
# so both take it from here.
_CAMERA_WIDTH, _CAMERA_HEIGHT = 640, 480
# The wrist camera's frame rate (frames per second). Dataset export resamples a
# recording at DEFAULT_FPS and rejects an episode whose camera ran below 95% of
# it, so the camera runs at that rate.
_CAMERA_FPS = int(DEFAULT_FPS)
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
        gripper_tasks={"manipulator": "arm_gripper"},
        model=_xarm7_sim_model,
        planning_timeout=10.0,
        visualization={"backend": "viser"},
    ),
    ManipulationSkills.blueprint(),
    TeleopSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world", robot=_xarm7_sim_model),
    GraspGenXModule.blueprint(
        gripper=XARM_GRIPPER_SWEEP_VOLUME,
        grasp_frame_to_tcp=XARM_GRASP_FRAME_TO_TCP,
        # Return every sample: the pick's contact check chooses among them.
        max_candidates=500,
    ),
    HeuristicGraspModule.blueprint(**XARM_HEURISTIC_AIM),
    # Rays from the wrist camera (80 x 60, about 1 cm apart on the table) give
    # world-frame points at 5 Hz, renamed for the Desktop page's scene view.
    MujocoSimModule.blueprint(
        **make_xarm7_sim_module_kwargs(XARM7_SIM_PATH),
        width=_CAMERA_WIDTH,
        height=_CAMERA_HEIGHT,
        fps=_CAMERA_FPS,
        enable_pointcloud=True,
        enable_mujoco_lidar=True,
        mujoco_lidar_raycast_width=80,
        mujoco_lidar_raycast_height=60,
        mujoco_lidar_voxel_size=0.01,
    ).remappings([(MujocoSimModule, "pointcloud", "scene_pointcloud")]),
    ManipulationLocalize.blueprint(world_frame="world"),
    coordinator(
        hardware=[_xarm7_sim_hw],
        tasks=[
            trajectory_task(_xarm7_sim_hw),
            eef_twist_task(
                _xarm7_sim_hw,
                robot_model=_xarm7_sim_model,
                target_frame="link_tcp",
                priority=20,
            ),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
            ),
            operator_hold_task(),
        ],
        cls=HybridTeleopCoordinator,
        instance_name="ControlCoordinator",
    ),
    # Serves the URDF and its meshes; the planner already puts the links on tf.
    UrdfManager.blueprint(model=_xarm7_sim_model.model, publish_joint_tf=False),
    RerunBridgeModule.blueprint(),
).remappings([(PickAndPlaceModule, "_grasp_generator", GraspGenXModule)])

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
