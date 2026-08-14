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

"""OpenYAM hardware and planning model configuration helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators._modeling import (
    base_pose,
    coordinator_joint_mapping,
    joint_names,
)
from dimos.utils.data import LfsPath

OPENYAM_DOF = 6
OPENYAM_PACKAGE = LfsPath("yam_description")
OPENYAM_MODEL_PATH = OPENYAM_PACKAGE / "urdf/yam_gripper.urdf.xacro"
OPENYAM_PACKAGE_PATHS: dict[str, Path] = {"yam_description": OPENYAM_PACKAGE}

# Default CAN address for a single real arm. On Linux this is a SocketCAN
# interface name; on macOS the openyam adapter transparently falls back to a
# gs_usb (candlelight-firmware) dongle — see hardware/manipulators/openyam.
YAM_CAN = "can0"

# The CANable 2.0 dongle is Full-Speed USB (~1 CAN frame per ~1 ms host
# round-trip). 100 Hz x 7 motors saturates it and motor state goes stale;
# 50 Hz stays well inside the budget. Trajectory execution is
# rate-independent (time-based sampling), so this only affects control
# granularity.
OPENYAM_TICK_RATE_HZ = 50.0

# Gripper motor endpoints in adapter-native units (motor rad) for
# normalized-command mapping. Opening DECREASES motor position (per the
# canonical OpenYamDamiaoAdapter in PR #3129). Calibrated on hardware
# (Aug 2026): jaws pressed closed read +0.02 (motor zeroed at closed),
# fully open read -2.94.
OPENYAM_GRIPPER_OPEN_RAD = -2.94
OPENYAM_GRIPPER_CLOSED_RAD = 0.0


def make_openyam_hardware(
    hw_id: str = "arm",
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    adapter_kwargs: dict[str, Any] | None = None,
    auto_enable: bool = True,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    """Create OpenYAM hardware, defaulting to the generic mock adapter.

    Pass ``adapter_type="openyam"`` and ``address=YAM_CAN`` for a real arm.
    """
    kwargs: dict[str, object] = dict(adapter_kwargs or {})
    if home_joints is not None and adapter_type == "mock":
        kwargs["initial_positions"] = home_joints
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hw_id, OPENYAM_DOF),
        adapter_type=adapter_type,
        address=address,
        auto_enable=auto_enable,
        gripper_joints=[f"{hw_id}/gripper"],
        adapter_kwargs=kwargs,
        gripper_open_position=OPENYAM_GRIPPER_OPEN_RAD,
        gripper_closed_position=OPENYAM_GRIPPER_CLOSED_RAD,
    )


def openyam_hardware(
    hw_id: str = "arm",
    *,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    """Create mock OpenYAM hardware for simulation and configuration checks."""
    return make_openyam_hardware(hw_id, home_joints=home_joints)


# Wrist link that anchors a wrist-mounted camera in the TF tree — the
# ManipulationModule publishes world->tf_extra_links transforms, and a
# camera blueprint hangs its hand-eye base_transform off this frame.
OPENYAM_WRIST_LINK = "yam_link_6"


def make_openyam_model_config(
    name: str = "arm",
    *,
    joint_prefix: str | None = None,
    home_joints: list[float] | None = None,
    tf_extra_links: list[str] | None = None,
) -> RobotModelConfig:
    """Build a planning config for the gripper-equipped OpenYAM.

    Pass ``tf_extra_links=[OPENYAM_WRIST_LINK]`` for perception blueprints
    that mount a camera on the wrist.
    """
    local_joint_names = joint_names(OPENYAM_DOF, prefix="yam_joint")
    return RobotModelConfig(
        name=name,
        model_path=OPENYAM_MODEL_PATH,
        base_pose=base_pose(),
        joint_names=local_joint_names,
        base_link="yam_base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(local_joint_names),
                base_link="yam_base_link",
                tip_link="yam_hand_tcp",
            )
        ],
        package_paths=OPENYAM_PACKAGE_PATHS,
        auto_convert_meshes=True,
        # Adjacent-link pairs whose meshes meet at their shared joint —
        # verified colliding at the neutral pose with raw all-pairs
        # checking, which otherwise marks every start configuration
        # infeasible.
        collision_exclusion_pairs=[
            ("yam_base_link", "yam_link_1"),
            ("yam_link_1", "yam_link_2"),
            ("yam_link_2", "yam_link_3"),
            ("yam_link_3", "yam_link_4"),
            ("yam_link_4", "yam_link_5"),
            ("yam_link_5", "yam_link_6"),
            ("yam_link_6", "yam_hand"),
            ("yam_hand", "yam_right_finger"),
            ("yam_hand", "yam_left_finger"),
            ("yam_right_finger", "yam_left_finger"),
        ],
        joint_name_mapping=coordinator_joint_mapping(
            name,
            OPENYAM_DOF,
            joint_prefix=joint_prefix,
            urdf_joint_prefix="yam_",
        ),
        gripper_hardware_id=name,
        tf_extra_links=tf_extra_links or [],
        home_joints=home_joints or [0.0] * OPENYAM_DOF,
    )
