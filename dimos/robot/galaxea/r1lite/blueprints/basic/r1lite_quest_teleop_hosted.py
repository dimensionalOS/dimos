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

"""Hosted R1 Lite quest teleop: the local teleop, operated over the broker.

Same arms/grippers/chassis control as r1lite-quest-teleop — identical
tasks, identical mapping values — but the operator's headset connects
through the hosted (Cloudflare) broker instead of the robot-local HTTPS
server, and the head camera streams back as the operator's view. Adds
the remote-operation safety plane: E-STOP latch (arms + chassis +
coordinator), operator-loss disengage, and stale-command drops.

    dimos run r1lite-quest-teleop-hosted        # robot
    dimos run r1lite-quest-teleop-hosted-sim    # mock hardware + viser

Broker credentials come from the standard hosted env
(``TRANSPORTS__BROKER__...``); motion still requires arming through the
preflight tool exactly like local teleop.
"""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import CloudflareTransport, CloudflareVideoTransport
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_coordinator import (
    r1lite_control_base,
    r1lite_standard_tasks,
)
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_quest_teleop import (
    _TASK_NAMES,
    _sim_arm_model,
    _sim_hardware,
    _sim_trajectory_tasks,
    _teleop_tasks,
)
from dimos.robot.galaxea.r1lite.hosted_module import R1LiteHostedTeleopModule
from dimos.teleop.hosted.camera_mux import CameraMuxModule
from dimos.teleop.hosted.hosted_stats import HostedStatsModule
from dimos.teleop.hosted.robot_type import RobotType

# Mapping values mirror the validated local blueprint exactly: the remote
# operator gets the same feel as the local one.
_MODULE_KWARGS = {
    "task_names": _TASK_NAMES,
    "motion_gain": 1.3,
    "local_rotation": False,
    "position_deadband_m": 0.02,
}

r1lite_quest_teleop_hosted = (
    autoconnect(
        R1LiteHostedTeleopModule.blueprint(**_MODULE_KWARGS),
        HostedStatsModule.blueprint(),
        # Width/fps caps keep the head-camera stream from saturating the
        # robot's uplink: uncapped video queued the WebRTC data channel
        # until pose lag blew past the freshness guard and froze arms
        # and sticks (hardware session 2026-08-11). 640@15 is ample to
        # drive by and leaves the control plane its bandwidth.
        CameraMuxModule.blueprint(cameras=["cam1"], video_max_width=640, video_max_fps=15.0),
        r1lite_control_base(
            extra_tasks=_teleop_tasks(),
            # Head camera on for the operator's view; watch TELEM for
            # control-loop pressure (the local blueprint keeps cameras
            # off for exactly that reason).
            connection_kwargs={"tracking_speed": 3.5, "enable_cameras": True},
        ),
    )
    .remappings(
        [
            (R1LiteHostedTeleopModule, "left_controller_output", "coordinator_cartesian_command"),
            (R1LiteHostedTeleopModule, "right_controller_output", "coordinator_cartesian_command"),
            (CameraMuxModule, "cam1", "head_left_color"),
        ]
    )
    .transports(
        {
            # inbound operator planes
            ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),
            ("state_json", bytes): CloudflareTransport.spec(
                "state_reliable", robot_type=RobotType.ARM
            ),
            ("camera_select", bytes): CloudflareTransport.spec("state_reliable"),
            # outbound operator planes
            ("mux_image", Image): CloudflareVideoTransport.spec(),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            ("cmd_ack", bytes): CloudflareTransport.spec("state_reliable_back"),
        }
    )
    # One process: every Cloudflare-bound module must share one broker
    # session (the hosted xarm blueprints run coordinator + cameras the
    # same way).
    .global_config(viewer="none", n_workers=1)
)


# Sim variant: mock hardware and viser in place of the robot and camera —
# the full hosted command plane (broker in, E-STOP, acks) against the sim
# arms, for end-to-end validation without hardware.
r1lite_quest_teleop_hosted_sim = (
    autoconnect(
        R1LiteHostedTeleopModule.blueprint(**_MODULE_KWARGS),
        HostedStatsModule.blueprint(),
        ControlCoordinator.blueprint(
            hardware=_sim_hardware(),
            tasks=[*r1lite_standard_tasks(), *_teleop_tasks(), *_sim_trajectory_tasks()],
        ),
        ManipulationModule.blueprint(
            robots=[_sim_arm_model("left", 0.25), _sim_arm_model("right", -0.25)],
            visualization={"backend": "viser"},
        ),
    )
    .remappings(
        [
            (R1LiteHostedTeleopModule, "left_controller_output", "coordinator_cartesian_command"),
            (R1LiteHostedTeleopModule, "right_controller_output", "coordinator_cartesian_command"),
            (R1LiteHostedTeleopModule, "cmd_vel", "twist_command"),
        ]
    )
    .transports(
        {
            ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),
            ("state_json", bytes): CloudflareTransport.spec(
                "state_reliable", robot_type=RobotType.ARM
            ),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            ("cmd_ack", bytes): CloudflareTransport.spec("state_reliable_back"),
        }
    )
    .global_config(viewer="none", n_workers=1)
)
