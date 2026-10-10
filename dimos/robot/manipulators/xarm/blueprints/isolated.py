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

"""Normal DimOS control/skills with a dedicated simulator I/O connection."""

from pathlib import Path

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.mujoco_robot_io import MujocoRobotIO
from dimos.robot.manipulators.xarm.config import make_xarm7_sim_robot_config, make_xarm_hardware

_hardware = make_xarm_hardware(
    "arm",
    7,
    gripper=True,
    adapter_type="mujoco_eval",
    address=global_config.mujoco_eval_endpoint,
    adapter_kwargs={
        "run": global_config.mujoco_eval_run,
        "episode": global_config.mujoco_eval_episode,
    },
)

_model = make_xarm7_sim_robot_config(
    base_pose=PoseStamped(frame_id="world", position=Vector3(z=0.912))
)
_public_description = Path("/opt/models/xarm_ros2/xarm_description")
_model.model = RobotModel.from_file(
    _public_description / "urdf/xarm_device.urdf.xacro",
    package_paths={"xarm_description": _public_description},
    xacro_args={
        "dof": "7",
        "robot_type": "xarm",
        "prefix": "",
        "limited": "true",
        "attach_xyz": "0 0 0",
        "attach_rpy": "0 0 0",
        "add_gripper": "true",
    },
).with_default_joint_acceleration_limit(2.0)

xarm_eval = autoconnect(
    ManipulationModule.blueprint(
        model=_model,
        planning_timeout=10.0,
        visualization={"backend": "none"},
    ),
    ManipulationSkills.blueprint(),
    MujocoRobotIO.blueprint(
        endpoint=global_config.mujoco_eval_endpoint,
        run=global_config.mujoco_eval_run,
        episode=global_config.mujoco_eval_episode,
    ),
    coordinator(
        hardware=[_hardware],
        tasks=[
            trajectory_task(_hardware),
            TaskConfig(
                name="arm_gripper", type="gripper", joint_names=["arm/gripper"], priority=20
            ),
        ],
    ),
)
