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
operator hold and takes hold and e-stop commands and sends their status as
streams. OperatorCommandsModule runs scan, click-to-target and pick commands.
The camera, a point cloud of the scene, every status and every command also go
out on the cockpit channels for an operator page.
"""

from __future__ import annotations

from dimos_lcm.std_msgs import Bool

from dimos.control.coordinator import TaskConfig
from dimos.control.operator_hold_coordinator import OperatorHoldCoordinator
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OperatorEstopStatus,
    OperatorHoldCommand,
    OperatorHoldStatus,
    operator_hold_task,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.operator_assist.operator_commands_module import (
    OperatorCommand,
    OperatorCommandsModule,
    OperatorState,
)
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.protocol.rpc.spec import DEFAULT_RPC_TIMEOUTS
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.web.cockpit import Channel, cockpit

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
    # A pick blocks until the object is held or every grasp failed: minutes.
    OperatorCommandsModule.blueprint(rpc_timeouts={**DEFAULT_RPC_TIMEOUTS, "pick_object": 600.0}),
    # The page's 3D view: rays from the wrist camera (80 x 60, about 1 cm
    # apart on the table) give world-frame points at 5 Hz. Renamed so they do
    # not mix with the object clouds ObjectSceneRegistrationModule sends on
    # "pointcloud".
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
    RerunBridgeModule.blueprint(),
    cockpit(
        channels=[
            Channel(
                "color_image",
                Image,
                encoding="jpeg.v1",
                delivery="latest",
                max_hz=15.0,
                params={"quality": 75},
            ),
            # frame_id is for the page only; the encoder reads res (metres).
            Channel(
                "scene_pointcloud",
                PointCloud2,
                encoding="voxels.zlib.v1",
                delivery="latest",
                max_hz=5.0,
                params={"res": 0.01, "frame_id": "world"},
            ),
            Channel(
                "operator_hold_status",
                OperatorHoldStatus,
                max_hz=5.0,
                paced=True,
                resend_on_subscribe=True,
            ),
            Channel(
                "operator_hold_command",
                OperatorHoldCommand,
                dir="tx",
                publish="shared",
                encoding="operator_hold_command.json.v1",
                max_hz=5.0,
            ),
            Channel(
                "operator_estop",
                Bool,
                dir="tx",
                publish="shared",
                encoding="bool.json.v1",
                max_hz=5.0,
            ),
            Channel(
                "operator_estop_status",
                OperatorEstopStatus,
                max_hz=5.0,
                paced=True,
                resend_on_subscribe=True,
            ),
            Channel(
                "operator_command",
                OperatorCommand,
                dir="tx",
                publish="shared",
                encoding="operator_command.json.v1",
                max_hz=5.0,
            ),
            # Paced: a state that follows another within 0.2 s still reaches the page.
            Channel(
                "operator_state",
                OperatorState,
                max_hz=5.0,
                paced=True,
                resend_on_subscribe=True,
            ),
        ]
    ),
)
