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

"""Two FR3s clamped to a tabletop, planned and driven together, with three cameras.

    dimos run fr3-tabletop-bimanual

Not Franka's bimanual product or a fixed dual-arm mount: two independent FR3s,
each with its own control box, clamped to one table, their relative placement
measured (see ``franka.tabletop_bimanual``). The "right" arm is arm 1 and the
"left" arm is arm 2, seen from behind the arms.

- One planner holds both arms in one collision world, so every plan, for one arm
  or both, is checked against the other arm. Planning groups "right", "left" and
  "both".
- One coordinator drives both arms (joints "right/fr3_joint1" ... and
  "left/fr3_joint1" ...) and both Franka Hands, through a FrankaConnection each.
- A SeparationGuard between the coordinator and the connections drops any joint
  target that would bring the grippers within 5 mm of each other, or anything
  else on the arms (wrists and wrist cameras included) within 5 cm of the other
  arm, wherever the command came from. The grippers can meet for a handover.
- A RealSense D435i on each wrist and one on a tripod over the table, colour and
  depth at 640x480, each on its own named streams ("right_wrist_color_image",
  "scene_color_image", ...), placed on tf by their calibrations.

The FCI addresses and camera serials default to Dimensional's setup; override
them after the blueprint name, e.g. ``--right_arm.ip 172.16.0.2`` or
``--scene_camera.serial_number ...``.
"""

from __future__ import annotations

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import movable_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import MetaBlueprint, autoconnect
from dimos.core.stream import In, Out
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.franka.config import fr3_model
from dimos.robot.manipulators.franka.connection import FrankaConnection
from dimos.robot.manipulators.franka.separation_guard import SeparationGuard
from dimos.robot.manipulators.franka.tabletop_bimanual import (
    SIDES,
    arm_joint_names,
    finger_joint,
    link,
    make_tabletop_bimanual_model_config,
    scene_camera_transform,
    wrist_camera_transform,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule

#: Dimensional's setup: each arm's FCI address and the RealSense serials.
DEFAULT_IPS = {"right": "10.42.0.230", "left": "10.43.0.53"}
DEFAULT_CAMERA_SERIALS = {
    "right_wrist": "261922077925",
    "left_wrist": "261922072855",
    "scene": "261922070328",
}
# Every camera output but tf, which all cameras share.
_CAMERA_OUTPUTS = (
    "color_image",
    "depth_image",
    "infrared_left",
    "infrared_right",
    "imu",
    "imu_info",
    "pointcloud",
    "camera_info",
    "depth_camera_info",
    "infrared_left_camera_info",
    "infrared_right_camera_info",
)


class FrankaTabletopCoordinator(ControlCoordinator):
    """Joint targets out to the "right" and "left" FR3s, and their joints in."""

    right_position_command: Out[JointState]
    right_joint_state: In[JointState]
    left_position_command: Out[JointState]
    left_joint_state: In[JointState]


def _connection_instance(side: str) -> str:
    return f"{side}_arm"


def _coordinator() -> MetaBlueprint:
    tasks = [
        joint_trajectory_task([name for side in SIDES for name in arm_joint_names(side)]),
        *(
            # fr3_finger_joint1 is open at 0.04 m.
            TaskConfig(
                name=f"{side}_gripper",
                type="gripper",
                joint_names=[finger_joint(side)],
                priority=20,
                params={"open_at": "upper"},
            )
            for side in SIDES
        ),
    ]
    connections = []
    for side in SIDES:
        model = fr3_model(robot=side)
        connections.append(
            ConnectionConfig(
                source=side,
                model=model,
                joint_commands={j.name: (Interface.POSITION,) for j in movable_joints(model)},
            )
        )
    return autoconnect(
        FrankaTabletopCoordinator.blueprint(
            instance_name="ControlCoordinator", connections=connections, tasks=tasks
        ),
        *(
            FrankaConnection.blueprint(
                instance_name=_connection_instance(side), ip=DEFAULT_IPS[side], source=side
            )
            for side in SIDES
        ),
        SeparationGuard.blueprint(),
    )


def _remaps() -> list[tuple[str | type, str, str]]:
    remaps: list[tuple[str | type, str, str]] = []
    for side in SIDES:
        arm = _connection_instance(side)
        remaps += [
            # coordinator -> guard -> connection
            (SeparationGuard, f"{side}_request", f"{side}_position_command"),
            (SeparationGuard, f"{side}_command", f"{side}_guarded_command"),
            (arm, "position_command", f"{side}_guarded_command"),
            # connection -> coordinator and guard
            (arm, "joint_state", f"{side}_joint_state"),
        ]
    for camera in DEFAULT_CAMERA_SERIALS:
        remaps += [(camera, name, f"{camera}_{name}") for name in _CAMERA_OUTPUTS]
    return remaps


def _cameras() -> MetaBlueprint:
    return autoconnect(
        *(
            RealSenseCamera.blueprint(
                instance_name=camera,
                serial_number=serial,
                frame_id=f"{camera}_camera_link",
                width=640,
                height=480,
            )
            for camera, serial in DEFAULT_CAMERA_SERIALS.items()
        )
    )


_model = make_tabletop_bimanual_model_config(
    # The wrist cameras' transforms hang off each Hand.
    tf_extra_links=[link(side, "hand") for side in SIDES],
).model_copy(update={"base_pose": PoseStamped(frame_id="world")})

fr3_tabletop_bimanual = autoconnect(
    ManipulationModule.blueprint(
        model=_model,
        gripper_tasks={side: f"{side}_gripper" for side in SIDES},
        static_transforms=[
            *(wrist_camera_transform(side) for side in SIDES),
            scene_camera_transform(),
        ],
        planning_timeout=10.0,
        visualization={"backend": "viser"},
        world_frame="world",
    ),
    ManipulationSkills.blueprint(),
    _coordinator(),
    _cameras(),
    RerunBridgeModule.blueprint(),
).remappings(_remaps())
