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

"""R1 Pro hosted teleoperation over the dimensional-teleop broker.

One operator session covers both arms (controller poses through the shared
teleoperation IK task), the holonomic chassis (right stick), and the torso
height (left stick Y).

``r1pro-hosted-teleop-quest`` and ``r1pro-hosted-teleop-pico`` are the same
stack: both headsets deliver WebXR poses and Joy over the same datachannel, so
nothing robot-side distinguishes them. They exist as separate names so the
demo has one entry point per headset and a place to pin a device-specific
deadzone or stick mapping once each is measured on hardware.

Name the robot in the operator console with ``TRANSPORTS__BROKER__ROBOT_NAME``
rather than in the blueprint: broker settings are part of the provider's
singleton key, so differing them per stream would open a second session.

Usage:
    TRANSPORTS__BROKER__API_KEY=dtk_live_... \\
    TRANSPORTS__BROKER__ROBOT_NAME=r1pro \\
    dimos run r1pro-hosted-teleop-quest
"""

from __future__ import annotations

from dimos.control.components import make_twist_base_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.stream import In
from dimos.core.transport import CloudflareTransport, CloudflareVideoTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import (
    r1pro_control,
    r1pro_gripper_tasks,
    r1pro_whole_body_hardware,
)
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_UPPER_BODY_PLANNING_JOINTS,
    make_r1pro_model_config,
)
from dimos.robot.galaxea.r1pro.joints import (
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    coordinator_name,
)
from dimos.robot.galaxea.r1pro.ready_pose import READY_POSE, R1ProReadyPoseModule
from dimos.robot.galaxea.r1pro.teleop_ik import (
    R1PRO_TELEOP_PINK,
    R1ProPinkPoseTargetSolver,
)
from dimos.robot.galaxea.r1pro.torso import (
    TORSO_FOLD_DROPS,
    TORSO_FOLD_JOINTS,
)
from dimos.robot.manipulators.common.blueprints import teleop_ik_task
from dimos.teleop.hosted.camera_mux import CameraMuxModule
from dimos.teleop.hosted.hosted_stats import HostedStatsModule
from dimos.teleop.hosted.image_decode import ImageDecodeModule
from dimos.teleop.hosted.mjpeg_preview import MjpegPreviewModule
from dimos.teleop.hosted.mobile_arm_command import MobileArmCommandModule
from dimos.teleop.hosted.robot_type import RobotType

R1PRO_HOSTED_TASK_NAME = "teleop_r1pro"


class R1ProTeleopCoordinator(TeleopControlCoordinator):
    """Teleoperation coordinator plus the R1 Pro's torso height stream.

    The torso task is bound to its own port rather than the shared
    joint_command, which is what keeps the arm solver and the torso on
    separate streams. No other robot has this joint group, so the port lives
    here instead of on the shared coordinator.
    """

    torso_command: In[JointState]


# Both arms without the torso, for locking the waist during arm teleoperation.
R1PRO_ARM_ONLY_JOINTS = tuple(
    coordinator_name(joint) for joint in (*LEFT_ARM_JOINTS, *RIGHT_ARM_JOINTS)
)


# Distinct classes only because blueprints can't yet run two instances of one
# module (same reason the hosted xArm blueprints declare Front/WristCamera).
class HeadCameraDecode(ImageDecodeModule):
    pass


class WristCameraDecode(ImageDecodeModule):
    pass


def r1pro_teleop_tasks(*, torso: bool = True) -> list[TaskConfig]:
    """Arms, torso and chassis, each on joints no other task owns.

    The teleoperation IK gets the two arms only. The torso is four joints the
    solver never sees, driven instead as a plain joint goal off a height table
    (see ``r1pro.torso``). That split is deliberate: with the torso in the
    solver, a hand target arriving late or jumping swung the waist, which on a
    real robot is how something gets hit. It also gives the operator a straight
    vertical axis instead of whatever the redundancy resolver decides.

    Nothing here reads the operator's headset pose.
    """
    return [
        teleop_ik_task(
            r1pro_whole_body_hardware(),
            name=R1PRO_HOSTED_TASK_NAME,
            robot_model=make_r1pro_model_config(),
            joint_names=R1PRO_ARM_ONLY_JOINTS,
            bindings=[
                {"hand": "left", "target_frame": "left_gripper_link"},
                {"hand": "right", "target_frame": "right_gripper_link"},
            ],
            solver_type=R1ProPinkPoseTargetSolver,
            params={"pink": R1PRO_TELEOP_PINK},
        ),
        TaskConfig(
            name="vel_chassis",
            type="velocity",
            joint_names=make_twist_base_joints("chassis"),
            priority=10,
        ),
        # Owns the torso as well as the arms. Arbitration is per joint, so the
        # torso jog commanding only the four torso joints leaves the arms with
        # the teleoperation task; a whole-body goal (hold-to-recover, a planned
        # move) claims everything and clears the engagement, as it should.
        joint_trajectory_task(list(R1PRO_UPPER_BODY_PLANNING_JOINTS), priority=20),
        # MobileArmCommandModule.gripper_command feeds these through the
        # coordinator's broadcast gripper_command port.
        # Per-hand: each controller's analog trigger drives its own gripper.
        # Bound broadcast, the trigger stream is dropped at the coordinator.
        *r1pro_gripper_tasks(per_hand=True),
    ]


def r1pro_hosted_teleop() -> Blueprint:
    """Broker-facing modules plus the real R1 Pro control stack.

    Head-left and right-wrist colour are decoded to raw frames for the mux;
    the R1 Pro driver only publishes them compressed.
    """
    return (
        autoconnect(
            # Hold A to walk both arms back to the tray pose; the same pose
            # the ready-pose module boots into and the posture task aims at.
            MobileArmCommandModule.blueprint(
                recover_pose=dict(READY_POSE),
                torso_fold_drops=list(TORSO_FOLD_DROPS),
                torso_fold_joints={k: list(v) for k, v in TORSO_FOLD_JOINTS.items()},
            ),
            # Same tray pose the local blueprint starts from, so an operator
            # sees the arms in a known posture before engaging either way.
            R1ProReadyPoseModule.blueprint(),
            HostedStatsModule.blueprint(),
            # Uncapped, the mux hands the software H.264 encoder two
            # full-size 30 fps frames side by side. On this board the encoder
            # falls behind, the sender queue grows, and the operator sees
            # latency climb until the track stalls. Cap it; raise with
            # --cameramuxmodule.video-max-width / --video-max-fps once the
            # uplink and encoder headroom are known.
            # Mirror of exactly what the operator is sent, on the LAN, for
            # anyone standing next to the robot without a headset.
            MjpegPreviewModule.blueprint(),
            CameraMuxModule.blueprint(
                cameras=["cam1", "cam2"],
                video_max_width=960,
                video_max_fps=15.0,
            ),
            HeadCameraDecode.blueprint(),
            WristCameraDecode.blueprint(),
            r1pro_control(
                tasks=r1pro_teleop_tasks(),
                coordinator_cls=R1ProTeleopCoordinator,
            ),
        )
        .remappings(
            [
                (HeadCameraDecode, "compressed_in", "head_left_color"),
                (HeadCameraDecode, "image_out", "cam1"),
                (WristCameraDecode, "compressed_in", "wrist_right_color"),
                (WristCameraDecode, "image_out", "cam2"),
                (MjpegPreviewModule, "image_in", "mux_image"),
                (MobileArmCommandModule, "left_controller_output", "left_cartesian_command"),
                (MobileArmCommandModule, "right_controller_output", "right_cartesian_command"),
            ]
        )
        .transports(
            {
                # Inbound operator planes. cmd_raw carries the WebXR controller
                # poses and Joy; state_json carries gripper / E-STOP / scale.
                ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),
                ("state_json", bytes): CloudflareTransport.spec(
                    "state_reliable", robot_type=RobotType.ARM
                ),
                ("camera_select", bytes): CloudflareTransport.spec("state_reliable"),
                # Outbound operator planes.
                ("mux_image", Image): CloudflareVideoTransport.spec(),
                ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
                ("cmd_ack", bytes): CloudflareTransport.spec("state_reliable_back"),
            }
        )
        # n_workers=1 is load-bearing, not a performance choice. The broker
        # provider is a per-PROCESS singleton, so every worker holding a
        # Cloudflare transport dials its own session and the robot shows up in
        # the operator console once per worker. All seven modules share one
        # process so there is exactly one session, as the hosted xArm
        # blueprints do. The R1 Pro sensor bus stays Zenoh; only the operator
        # planes above ride Cloudflare, and autoconnect merges global config
        # last-wins, so both are pinned here.
        .global_config(transport="zenoh", viewer="none", n_workers=1)
    )


# Wrapped in autoconnect so the blueprint-registry AST scan sees them; a bare
# helper call at module scope registers nothing.
r1pro_hosted_teleop_quest = autoconnect(r1pro_hosted_teleop())
r1pro_hosted_teleop_pico = autoconnect(r1pro_hosted_teleop())
