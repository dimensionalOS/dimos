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

"""Camera-nav perception stack wired onto the FlowBase keyboard-teleop blueprint.

ZED Mini variant lives in blueprint_zed.py to keep pyzed imports lazy.
"""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.drive_trains.flowbase.odom_tf import FlowBaseOdomModule
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.camera_nav.recorder import CameraNavRecorder
from dimos.navigation.camera_nav.viz import cloud_points, pinhole_setup
from dimos.perception.depth.accumulator import DepthAccumulatorModule
from dimos.perception.depth.monocular_depth_module import MonocularDepthModule
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.rerun.bridge import RerunBridgeModule


camera_nav_static_trial = autoconnect(
    CameraModule.blueprint(),
    MonocularDepthModule.blueprint(),
    DepthAccumulatorModule.blueprint(),
    RerunBridgeModule.blueprint(
        pubsubs=[LCM()],
        rerun_open="web",
        visual_override={
            "world/global_map": cloud_points,
            "world/frame_cloud": cloud_points,
            "world/camera_info": pinhole_setup,
        },
    ),
)

# Camera mount: x=forward, y=left, z=up (ROS convention).
# Tune translation and pitch to match physical mount.
_CAMERA_MOUNT = Transform(
    translation=Vector3(0.0, 0.0, 0.5),
    rotation=Quaternion.from_euler(Vector3(0.0, -0.26, 0.0)),  # ≈15° downward tilt
    frame_id="base_link",
    child_frame_id="camera_link",
)


def _make_flowbase_coordinator(address: str | None = None):
    _joints = make_twist_base_joints("base")
    hw = HardwareComponent(
        hardware_id="base",
        hardware_type=HardwareType.BASE,
        joints=_joints,
        adapter_type="flowbase",
        address=address,
    )
    return autoconnect(
        ControlCoordinator.blueprint(
            hardware=[hw],
            tasks=[TaskConfig(name="vel_base", type="velocity", joint_names=_joints, priority=10)],
        ),
        KeyboardTeleop.blueprint(),
    ).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])


camera_nav_flowbase_teleop = autoconnect(
    _make_flowbase_coordinator(),
    FlowBaseOdomModule.blueprint(),
    CameraModule.blueprint(transform=_CAMERA_MOUNT),
    MonocularDepthModule.blueprint(),
    DepthAccumulatorModule.blueprint(),
    CameraNavRecorder.blueprint(db_path="traversal.db"),
    RerunBridgeModule.blueprint(
        pubsubs=[LCM()],
        rerun_open="web",
        visual_override={
            "world/global_map": cloud_points,
            "world/frame_cloud": cloud_points,
            "world/camera_info": pinhole_setup,
        },
    ),
)
