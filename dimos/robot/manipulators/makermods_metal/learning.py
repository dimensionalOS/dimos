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

"""Metal learning composition shared by single- and dual-arm deployments."""

from collections.abc import Mapping

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.eef_trajectory_task.eef_trajectory_task import eef_trajectory_task
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recorder import collect
from dimos.imitation.dataprep.schema import FeatureSpec
from dimos.imitation.policy.dreamlabs.camera import DreamlabsCameraModule
from dimos.imitation.policy.dreamlabs.config import (
    ArmSide,
    CameraRole,
    DreamlabsArmConfig,
    DreamlabsBackendConfig,
)
from dimos.imitation.policy.dreamlabs.requirements import check_planning_dependencies
from dimos.imitation.policy.module import policy_module
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common.blueprints import teleop_ik_task
from dimos.robot.manipulators.dual_makermods_metal.config import ARM_JOINTS, dual_metal_hardware
from dimos.robot.manipulators.dual_makermods_metal.coordinator import DualMetalCoordinator
from dimos.robot.manipulators.dual_makermods_metal.model import dual_metal_model_config
from dimos.robot.manipulators.makermods_metal.config import (
    METAL_ARM_JOINTS,
    METAL_GRIPPER_JOINT,
    metal_hardware,
)
from dimos.robot.manipulators.makermods_metal.coordinator import MetalCoordinator
from dimos.robot.manipulators.makermods_metal.model import make_metal_model_config
from dimos.teleop.webxr.extensions import ArmTeleopModule

ToolTransform = tuple[tuple[float, float, float], tuple[float, float, float, float]]
_CAMERA_NAMES = {
    "overhead": "OverheadCamera",
    "wrist_l": "LeftWristCamera",
    "wrist_r": "RightWristCamera",
}


class MetalLearningCoordinator(MetalCoordinator, TeleopControlCoordinator):
    """Keep Metal CAN selection and add the existing teleop input ports."""


class DualMetalLearningCoordinator(DualMetalCoordinator, TeleopControlCoordinator):
    """Keep dual-CAN selection and add the existing teleop input ports."""


def build_metal_learning(
    *,
    dual: bool = False,
    collection: bool = True,
    camera_roles: tuple[CameraRole, ...] = ("overhead",),
    model: RobotModelConfig | None = None,
    tcp_transforms: Mapping[ArmSide, ToolTransform] | None = None,
    checkpoint: str = "dreamlabs/dream-1-beta",
) -> Blueprint:
    """Compose one station; real deployment supplies measured model mounts and TCPs.

    Single Metal uses the right-hand protocol slot and right Quest controller.
    Missing TCP calibration is allowed for collection, but rollout preflight fails.
    The dual model's default mount positions describe only a mock example station.
    """
    model = model or (dual_metal_model_config() if dual else make_metal_model_config())
    sides: tuple[ArmSide, ...] = ("left", "right") if dual else ("right",)
    hardware = dual_metal_hardware() if dual else [metal_hardware()]
    arm_names = {side: list(ARM_JOINTS[side] if dual else METAL_ARM_JOINTS) for side in sides}
    grippers = {side: f"{side}_arm/gripper" if dual else METAL_GRIPPER_JOINT for side in sides}
    targets = {side: f"{side}_gripper_base" if dual else "gripper_base" for side in sides}
    joint_names = [name for side in sides for name in arm_names[side]] + list(grippers.values())
    arms = {}
    for side in sides:
        transform = (tcp_transforms or {}).get(side)
        arms[side] = DreamlabsArmConfig(
            joint_names=arm_names[side],
            base_frame=f"{side}_base_link" if dual else "base_link",
            target_frame=targets[side],
            gripper_joint=grippers[side],
            target_to_sdk_xyz=transform[0] if transform else None,
            target_to_sdk_xyzw=transform[1] if transform else None,
        )
    backend = DreamlabsBackendConfig(
        mode="eef", model=model.model, arms=arms, camera_roles=camera_roles
    )
    tasks = [joint_trajectory_task(joint_names, priority=20)]
    for side, component in zip(sides, hardware, strict=True):
        tasks.extend(
            [
                teleop_ik_task(
                    component,
                    robot_model=model,
                    name=f"teleop_metal_{side}",
                    joint_names=arm_names[side],
                    priority=30,
                    bindings=[{"hand": side, "target_frame": targets[side]}],
                    params={"max_joint_velocity_rad_s": 0.5, "feedback_limit_tolerance": 0.06},
                ),
                TaskConfig(
                    name=f"{side}_arm_gripper" if dual else "metal_gripper",
                    type="gripper",
                    joint_names=[grippers[side]],
                    priority=30,
                    stream_bind={"gripper_command": f"{side}_gripper_command"},
                ),
            ]
        )
    if not collection:
        tasks.append(
            eef_trajectory_task(
                [name for side in sides for name in arm_names[side]],
                robot_model=model,
                target_frames=list(targets.values()),
                gripper_joint_names=list(grippers.values()),
                max_joint_velocity_rad_s=0.5,
                feedback_limit_tolerance=0.06,
            )
        )
    coordinator = DualMetalLearningCoordinator if dual else MetalLearningCoordinator
    parts = [
        coordinator.blueprint(instance_name="ControlCoordinator", tasks=tasks),
        ArmTeleopModule.blueprint(zero_button="B"),
        ManipulationModule.blueprint(
            model=model, visualization={"backend": "viser"}, default_speed_scale=0.2
        ),
        *[
            DreamlabsCameraModule.blueprint(instance_name=_CAMERA_NAMES[role], transform=None)
            for role in camera_roles
        ],
    ]
    robot_type = "dual_makermods_metal" if dual else "makermods_metal"
    image_mapping: dict[str, str] = {f"{role}_image": role for role in camera_roles}
    if collection:
        state = FeatureSpec(
            stream="coordinator_joint_state",
            message_type=JointState,
            field="position",
            dtype="float32",
            shape=(len(joint_names),),
            names=joint_names,
        )
        profile = CollectionProfile(
            observations={
                "observation.state": state,
                **{
                    f"observation.images.{role}": FeatureSpec(
                        stream=port,
                        message_type=Image,
                        field="data",
                        dtype="video",
                        shape=(480, 640, 3),
                        names=["height", "width", "channels"],
                    )
                    for port, role in image_mapping.items()
                },
            },
            actions={
                "action": state.model_copy(
                    update={
                        "stream": "coordinator_joint_command",
                        "source_kind": "joint_position_updates",
                    },
                    deep=True,
                )
            },
            metadata={"name": f"{robot_type}-{len(camera_roles)}cam", "robot_type": robot_type},
        )
        parts.extend(
            [collect(profile=profile), EpisodeMonitorModule.blueprint(instance_name="episodes")]
        )
    else:
        parts.append(
            policy_module(
                backend="dreamlabs",
                instance_name="policy",
                policy_path=checkpoint,
                task="pick up the object",
                robot_type=robot_type,
                joint_names=joint_names,
                dreamlabs=backend,
                image_mapping=image_mapping,
            )
        )
    return (
        autoconnect(*parts)
        .remappings(
            [
                *[(_CAMERA_NAMES[role], "color_image", f"{role}_image") for role in camera_roles],
                *[
                    (ArmTeleopModule, f"{side}_controller_output", f"{side}_cartesian_command")
                    for side in sides
                ],
            ]
        )
        .requirements(check_planning_dependencies)
    )
