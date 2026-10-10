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

"""Franka Emika Panda planning model and hardware helpers.

No real-arm adapter yet; the Panda is driven in simulation (LIBERO) over the
``sim_transport`` adapter.
"""

from __future__ import annotations

from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators._modeling import joint_names

FRANKA_DESCRIPTION_REPO = "https://github.com/frankaemika/franka_description"
FRANKA_DESCRIPTION_REF = "7aeeddc449edf8d62b594f9e36a81da53e7796f9"
_FRANKA_REPO = RobotDescriptionSource(url=FRANKA_DESCRIPTION_REPO, ref=FRANKA_DESCRIPTION_REF)
# "fer" (Franka Emika Robot) is the Panda.
PANDA_MODEL_PATH = _FRANKA_REPO / "robots" / "fer" / "fer.urdf.xacro"
PANDA_PACKAGE_PATHS: dict[str, Path] = {"franka_description": _FRANKA_REPO.path()}

# Each finger is 0..0.04 m; the gripper position is the opening between them.
PANDA_GRIPPER_RANGE = (0.0, 0.08)
PANDA_FINGER_JOINTS = ("finger_joint1", "finger_joint2")
# Robosuite's (and LIBERO's) Panda start pose.
PANDA_HOME = [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.785398163]
PANDA_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("leftfinger", "rightfinger"),
    ("link7", "hand"),
    ("link7", "leftfinger"),
    ("link7", "rightfinger"),
]


def make_panda_model_config(
    *,
    gripper_hardware_id: str | None = None,
    base_pose: PoseStamped | None = None,
    tf_extra_links: list[str] | None = None,
    home_joints: list[float] | None = None,
    pre_grasp_offset: float = 0.10,
) -> RobotModelConfig:
    """Panda with the Franka Hand; joints ``joint1..7``, tip ``hand_tcp``.

    The fingers are fixed (closed) for planning: the planner moves the arm and the
    gripper is commanded separately, as for the other arms.
    """
    model_joint_names = joint_names(7)
    return RobotModelConfig(
        model=RobotModel.from_file(
            PANDA_MODEL_PATH,
            package_paths=PANDA_PACKAGE_PATHS,
            # no_prefix names the links link0..7 / hand and the joints joint1..7.
            xacro_args={"hand": "true", "no_prefix": "true", "with_sc": "false"},
        )
        # Drop the xacro's empty "base" link above link0: the planner roots at base_link.
        .with_subtree_rooted_at("link0")
        .with_fixed_joints(*PANDA_FINGER_JOINTS)
        .with_default_joint_acceleration_limit(2.0),
        base_pose=base_pose if base_pose is not None else PoseStamped(),
        joint_names=model_joint_names,
        base_link="link0",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(model_joint_names),
                base_link="link0",
                tip_link="hand_tcp",
            )
        ],
        auto_convert_meshes=True,
        collision_exclusion_pairs=PANDA_GRIPPER_COLLISION_EXCLUSIONS,
        gripper_hardware_id=gripper_hardware_id,
        tf_extra_links=tf_extra_links or [],
        home_joints=home_joints or PANDA_HOME,
        pre_grasp_offset=pre_grasp_offset,
    )


def make_panda_hardware(
    hw_id: str = "arm",
    *,
    adapter_type: str = "sim_transport",
    gripper: bool = True,
) -> HardwareComponent:
    gripper_joints = [f"{hw_id}/gripper"] if gripper else []
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=[*joint_names(7), *gripper_joints],
        adapter_type=adapter_type,
        auto_enable=True,
        adapter_kwargs={"gripper_range": PANDA_GRIPPER_RANGE},
    )
