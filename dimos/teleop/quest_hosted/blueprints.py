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

from pathlib import Path

from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import CloudflareTransport, CloudflareVideoTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.robot.manipulators.xarm.blueprints.teleop import coordinator_teleop_xarm7
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.teleop.quest_hosted.hosted_extensions import (
    HostedArmTeleopModule,
    HostedTwistTeleopModule,
)
from dimos.teleop.utils.recorder import TeleopRecorder, TeleopRecorderConfig

# Single XArm7 teleop via the hosted (WebRTC) client. Pass `--simulation` to
# run the coordinator inside MuJoCo, omit it for real hardware.
teleop_hosted_xarm7 = (
    autoconnect(
        HostedArmTeleopModule.blueprint(task_names={"right": "teleop_xarm"}),
        coordinator_teleop_xarm7,
    )
    .remappings(
        [(HostedArmTeleopModule, "right_controller_output", "coordinator_cartesian_command")]
    )
    .global_config(rerun_open="none")
)


# viewer="none" drops the rerun window (operator gets video over WebRTC, so the
# robot-side rerun view is unwanted here).
teleop_hosted_go2 = autoconnect(
    HostedTwistTeleopModule.blueprint(),
    unitree_go2_basic,
).global_config(n_workers=8, viewer="none")


# Hosted teleop as a pure transport swap — no teleop module wrapper. The
# browser's keyboard/VR view sends LCM TwistStamped on cmd_unreliable; the
# transport decodes it straight onto the go2 cmd_vel stream (commands arrive
# as sent: normalized [-1, 1], no speed rescaling). The camera stream feeds
# the session's WebRTC video track via CloudflareVideoTransport (same
# provider/PeerConnection), and robot → operator telemetry can ride
# CloudflareTransport.spec("state_reliable_back", ...) the same way.
#
# Run:  dimos run teleop-hosted-go2-transport -o transports.broker.api_key=dtk_live_...
#       (or TRANSPORTS__BROKER__API_KEY=dtk_live_... in env; robot identity is
#       derived from the key, override with transports.broker.robot_id if needed)
# then connect from https://teleop.dimensionalos.com (keyboard view).
teleop_hosted_go2_transport = unitree_go2_basic.transports(
    {
        ("cmd_vel", Twist): CloudflareTransport.spec("cmd_unreliable", TwistStamped),
        ("color_image", CompressedImage): CloudflareVideoTransport.spec(),
    }
).global_config(viewer="none")


HOSTED_RECORDINGS_DIR = STATE_DIR / "hosted_teleop" / "recordings"


class HostedTeleopRecorderConfig(TeleopRecorderConfig):
    # Same generic recorder, just defaulting recordings into the hosted dir.
    db_path: str | Path = HOSTED_RECORDINGS_DIR / "recording_hosted.db"


class HostedTeleopRecorder(TeleopRecorder):
    """Generic ``TeleopRecorder`` defaulting to the hosted recordings dir.

    Ports + per-run timestamping are inherited; this only changes the default
    output path. Compose at the CLI::

        dimos run teleop-hosted-xarm7 hosted-teleop-recorder
        dimos run teleop-hosted-go2   hosted-teleop-recorder
    """

    config: HostedTeleopRecorderConfig
