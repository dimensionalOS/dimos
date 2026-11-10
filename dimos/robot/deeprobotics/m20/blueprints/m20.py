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

"""Robot-local M20 integration for the current DimOS 3D navigation stack."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos_messages.foxglove_msgs.msg import CompressedVideo
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic as ZenohTopic
from dimos.robot.deeprobotics.m20.blueprints._nav import m20_nav_stack
from dimos.robot.deeprobotics.m20.camera import M20CameraRelay
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.model import M20_MODEL, M20StandingJointState
from dimos.robot.deeprobotics.m20.rerun import m20_rerun_config
from dimos.robot.urdf_manager import UrdfManager
from dimos.visualization.vis_module import vis_module

m20 = (
    autoconnect(
        vis_module(viewer_backend=global_config.viewer, rerun_config=m20_rerun_config()),
        M20CameraRelay.blueprint(instance_name="M20CameraRelay"),
        M20Connection.blueprint(auto_stand=True).remappings(
            [(M20Connection, "odometry", "slam_odom")]
        ),
        UrdfManager.blueprint(model=M20_MODEL),
        M20StandingJointState.blueprint(),
        m20_nav_stack().remappings([(MovementManager, "cmd_vel", "requested_cmd_vel")]),
    )
    .global_config(
        n_workers=4,
        transport="zenoh",
    )
    .transports(
        {
            ("front_camera", CompressedVideo): ZenohTransport.spec(
                ZenohTopic("dimos/front_camera", CompressedVideo, qos=QOS_LATEST_WINS)
            ),
            ("rear_camera", CompressedVideo): ZenohTransport.spec(
                ZenohTopic("dimos/rear_camera", CompressedVideo, qos=QOS_LATEST_WINS)
            ),
        }
    )
)
