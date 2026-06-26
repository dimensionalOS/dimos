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

"""Camera-nav blueprint for ZED Mini stereo camera (requires ZED SDK / pyzed.sl).

Kept in a separate file so importing blueprint_flowbase does not pull in pyzed.
ZED VIO provides world←base_link TF directly, so FlowBaseOdomModule is not needed.
"""

from __future__ import annotations

from dimos.control.blueprints.mobile import coordinator_flowbase_keyboard_teleop
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.zed.camera import ZEDCamera
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.camera_nav.recorder import CameraNavRecorder
from dimos.perception.depth.accumulator import DepthAccumulatorModule
from dimos.perception.depth.hardware_depth_module import HardwareDepthModule
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.rerun.bridge import RerunBridgeModule


def _cloud_points(cloud):
    import numpy as np
    import rerun as rr
    pts, cols = cloud.as_numpy()
    if len(pts) == 0:
        return rr.Points3D([])
    if cols is not None and len(cols) == len(pts):
        return rr.Points3D(positions=pts, colors=(cols * 255).astype(np.uint8))
    return rr.Points3D(positions=pts)


# Tune translation and pitch to match physical ZED Mini mount.
_ZED_MOUNT = Transform(
    translation=Vector3(0.0, 0.0, 0.5),
    rotation=Quaternion.from_euler(Vector3(0.0, -0.26, 0.0)),  # ≈15° downward tilt
    frame_id="base_link",
    child_frame_id="camera_link",
)

_RERUN_VIZ = RerunBridgeModule.blueprint(
    pubsubs=[LCM()],
    rerun_open="web",
    visual_override={
        "world/global_map": _cloud_points,
        "world/frame_cloud": _cloud_points,
    },
)

# Standalone: ZED only, no robot. ZED VIO publishes world←camera_link TF,
# so the map builds from camera motion alone.
camera_nav_zed_standalone = autoconnect(
    ZEDCamera.blueprint(
        resolution="VGA",
        enable_depth=True,
        depth_mode="ULTRA",
        enable_fill_mode=True,
        enable_pointcloud=False,
        enable_tracking=True,
        enable_imu_fusion=True,
        set_floor_as_origin=True,
    ),
    HardwareDepthModule.blueprint(
        camera_frame="camera_color_optical_frame",
        stride=1,
        max_depth=12.0,
        max_freq=10.0,
    ),
    DepthAccumulatorModule.blueprint(
        voxel_size=0.03,
        icp_window=5,
        merge_interval=10,
        publish_freq=5.0,
    ),
    _RERUN_VIZ,
)

def _make_zed_teleop(address: str | None = None):
    """ZED + FlowBase keyboard teleop with configurable robot IP."""
    from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
    from dimos.control.coordinator import ControlCoordinator, TaskConfig
    from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

    _joints = make_twist_base_joints("base")
    hw = HardwareComponent(
        hardware_id="base",
        hardware_type=HardwareType.BASE,
        joints=_joints,
        adapter_type="flowbase",
        address=address,
    )
    coordinator = autoconnect(
        ControlCoordinator.blueprint(
            hardware=[hw],
            tasks=[TaskConfig(name="vel_base", type="velocity", joint_names=_joints, priority=10)],
        ),
        KeyboardTeleop.blueprint(),
    ).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])

    return autoconnect(
        coordinator,
        ZEDCamera.blueprint(
            base_transform=_ZED_MOUNT,
            resolution="VGA",
            enable_depth=True,
            depth_mode="ULTRA",
            enable_fill_mode=True,
            enable_pointcloud=False,
            enable_tracking=True,
            enable_imu_fusion=True,
            set_floor_as_origin=True,
        ),
        HardwareDepthModule.blueprint(
            camera_frame="camera_color_optical_frame",
            stride=1,
            max_depth=12.0,
            max_freq=10.0,
        ),
        DepthAccumulatorModule.blueprint(
            voxel_size=0.03,
            icp_window=5,
            merge_interval=10,
            publish_freq=5.0,
        ),
        CameraNavRecorder.blueprint(db_path="traversal.db"),
        _RERUN_VIZ,
    )


# Full: ZED + FlowBase keyboard teleop + traversal recording (default IP).
camera_nav_zed_teleop = autoconnect(
    coordinator_flowbase_keyboard_teleop,
    ZEDCamera.blueprint(
        base_transform=_ZED_MOUNT,
        resolution="VGA",
        enable_depth=True,
        depth_mode="ULTRA",
        enable_fill_mode=True,
        enable_pointcloud=False,
        enable_tracking=True,
        enable_imu_fusion=True,
        set_floor_as_origin=True,
    ),
    HardwareDepthModule.blueprint(
        camera_frame="camera_color_optical_frame",
        stride=1,
        max_depth=12.0,
        max_freq=10.0,
    ),
    DepthAccumulatorModule.blueprint(
        voxel_size=0.03,
        icp_window=5,
        merge_interval=10,
        publish_freq=5.0,
    ),
    CameraNavRecorder.blueprint(db_path="traversal.db"),
    _RERUN_VIZ,
)
