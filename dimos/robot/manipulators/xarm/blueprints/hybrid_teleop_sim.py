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
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Bool import Bool
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import (
    coordinator,
    eef_twist_task,
    trajectory_task,
)
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    XARM_HEURISTIC_AIM,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.robot.usd_manager import UsdManager
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.bridge import RerunBridgeModule

logger = setup_logger()


class OperatorHoldCoordinator(ControlCoordinator):
    operator_hold_status: Out[Bool]
    operator_estop_status: Out[Bool]

    @rpc
    def add_task(
        self,
        task: ControlTask,
        task_type: str | None = None,
        stream_bind: dict[str, str] | None = None,
    ) -> bool:
        added = super().add_task(task, task_type=task_type, stream_bind=stream_bind)
        if added and isinstance(task, OperatorHoldTask):
            task.set_status_publisher(
                lambda status: self.operator_hold_status.publish(Bool(data=status.on))
            )
        return added

    @rpc
    def start(self) -> None:
        super().start()
        self._publish_estop_status()

    @rpc
    def set_estop(self, estopped: bool) -> bool:
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
    pass


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
        model=_xarm7_sim_model,
        planning_timeout=10.0,
        visualization={"backend": "viser"},
    ),
    ManipulationSkills.blueprint(),
    TeleopSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world", robot=_xarm7_sim_model),
    HeuristicGraspModule.blueprint(**XARM_HEURISTIC_AIM),
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
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        detector_backend="moondream",
        segmentation_backend="edgetam",
        detect_on_request=True,
        distance_threshold=0.05,
    ),
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
    UsdManager.blueprint(model=_xarm7_sim_model.model, publish_joint_tf=False),
    RerunBridgeModule.blueprint(),
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
