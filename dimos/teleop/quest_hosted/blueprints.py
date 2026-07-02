# Copyright 2025-2026 Dimensional Inc.
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

"""Hosted teleop blueprints (WebRTC transport)."""

from dimos.control.blueprints.teleop import coordinator_teleop_xarm7
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import (
    CloudflareTransport,
    CloudflareVideoTransport,
    LCMTransport,
    LiveKitTransport,
    LiveKitVideoTransport,
)
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.hosted_connection import Go2HostedConnection
from dimos.teleop.quest.quest_types import Buttons
from dimos.teleop.quest_hosted.hosted_extensions import (
    HostedArmTeleopModule,
    HostedTwistTeleopModule,
)

# Single XArm7 hosted teleop. Pass `--simulation` to run in MuJoCo.
teleop_hosted_xarm7 = (
    autoconnect(
        HostedArmTeleopModule.blueprint(task_names={"right": "teleop_xarm"}),
        coordinator_teleop_xarm7,
    )
    .transports(
        {
            ("right_controller_output", PoseStamped): LCMTransport.spec(
                "/coordinator/cartesian_command", PoseStamped
            ),
            ("buttons", Buttons): LCMTransport.spec("/teleop/buttons", Buttons),
        }
    )
    .global_config(rerun_open="none")
)


# Hosted teleop via the legacy module wrapper (the transport swap is preferred).
teleop_hosted_go2 = autoconnect(
    HostedTwistTeleopModule.blueprint(),
    unitree_go2_basic,
).global_config(n_workers=8, viewer="none")

# Hosted teleop over CF Realtime. Run with -o transports.broker.api_key=dtk_live_...
teleop_hosted_go2_transport = (
    autoconnect(
        unitree_go2_basic.disabled_modules(GO2Connection),
        Go2HostedConnection.blueprint(),
    )
    .transports(
        {
            ("cmd_vel", Twist): CloudflareTransport.spec("cmd_unreliable", TwistStamped),
            # mux_image (not color_image): the muxed out-frame carries the
            # latency stamp and works for 1 cam (mux returns cam1 alone) or more.
            ("mux_image", Image): CloudflareVideoTransport.spec(),
            ("state_json", bytes): CloudflareTransport.spec("state_reliable"),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),  # stats tap
            ("cmd_vel_stamped", TwistStamped): LCMTransport.spec("cmd_vel_stamped", TwistStamped),
        }
    )
    .global_config(viewer="none")
)


# LiveKit twin of teleop_hosted_go2_transport — same channels, LiveKit SFU.
# Run with -o transports.broker.api_key=dtk_live_...
teleop_hosted_go2_livekit = (
    autoconnect(
        unitree_go2_basic.disabled_modules(GO2Connection),
        Go2HostedConnection.blueprint(),
    )
    .transports(
        {
            ("cmd_vel", Twist): LiveKitTransport.spec("cmd_unreliable", TwistStamped),
            ("color_image", Image): LiveKitVideoTransport.spec(),
            ("state_json", bytes): LiveKitTransport.spec("state_reliable"),
            ("telemetry_out", bytes): LiveKitTransport.spec("state_reliable_back"),
            ("cmd_raw", bytes): LiveKitTransport.spec("cmd_unreliable"),  # stats tap
            ("cmd_vel_stamped", TwistStamped): LiveKitTransport.spec(
                "cmd_unreliable", TwistStamped
            ),
        }
    )
    .global_config(viewer="none")
)


# Adds a RealSense as cam2 (mux'd into the video track by Go2HostedConnection).
# Needs the RealSense wired in; use teleop-hosted-go2-transport otherwise.
teleop_hosted_go2_multicam = (
    autoconnect(
        unitree_go2_basic.disabled_modules(GO2Connection),
        Go2HostedConnection.blueprint(),
        RealSenseCamera.blueprint(enable_depth=False, enable_pointcloud=False),
    )
    .remappings([(RealSenseCamera, "color_image", "cam2_in")])
    .transports(
        {
            ("cmd_vel", Twist): CloudflareTransport.spec("cmd_unreliable", TwistStamped),
            ("mux_image", Image): CloudflareVideoTransport.spec(),
            ("cam2_in", Image): LCMTransport.spec("cam2_in", Image),
            ("state_json", bytes): CloudflareTransport.spec("state_reliable"),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),
            ("cmd_vel_stamped", TwistStamped): CloudflareTransport.spec(
                "cmd_unreliable", TwistStamped
            ),
        }
    )
    .global_config(viewer="none")
)


__all__ = [
    "teleop_hosted_go2",
    "teleop_hosted_go2_livekit",
    "teleop_hosted_go2_multicam",
    "teleop_hosted_go2_transport",
    "teleop_hosted_xarm7",
]
