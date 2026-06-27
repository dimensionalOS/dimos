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

"""Camera-nav blueprints for ZED Mini stereo camera (requires ZED SDK / pyzed.sl).

Kept in a separate file so importing blueprint_flowbase does not pull in pyzed.
ZED VIO provides world←camera_link TF directly, so FlowBaseOdomModule is not needed.
"""

from __future__ import annotations

from dimos.control.blueprints.mobile import coordinator_flowbase_keyboard_teleop
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.zed.camera import ZEDCamera
from dimos.mapping.costmapper import CostMapper
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.camera_nav.recorder import CameraNavRecorder
from dimos.navigation.camera_nav.viz import cloud_points, costmap_viz, pinhole_setup
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.perception.depth.accumulator import DepthAccumulatorModule
from dimos.perception.depth.hardware_depth_module import HardwareDepthModule
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.rerun.bridge import RerunBridgeModule


# ---------------------------------------------------------------------------
# Shared config blocks — avoids repeating the same kwargs everywhere
# ---------------------------------------------------------------------------

_ZED_MOUNT = Transform(
    translation=Vector3(0.0, 0.0, 0.5),
    rotation=Quaternion.from_euler(Vector3(0.0, -0.26, 0.0)),  # ≈15° downward tilt
    frame_id="base_link",
    child_frame_id="camera_link",
)

# ZED camera blueprint — VGA + NEURAL depth: SDK 4.x replacement for deprecated ULTRA.
_ZED_CAMERA = ZEDCamera.blueprint(
    resolution="VGA",
    enable_depth=True,
    depth_mode="NEURAL",
    enable_fill_mode=True,
    enable_pointcloud=False,
    enable_tracking=True,
    enable_imu_fusion=True,
    set_floor_as_origin=True,
)

_ZED_DEPTH = HardwareDepthModule.blueprint(
    camera_frame="camera_color_optical_frame",
    stride=1,
    max_depth=12.0,
    max_freq=10.0,
)

_ZED_ACCUM = DepthAccumulatorModule.blueprint(
    voxel_size=0.03,
    icp_window=5,
    merge_interval=10,
    publish_freq=5.0,
)

_RERUN_VIZ = RerunBridgeModule.blueprint(
    pubsubs=[LCM()],
    rerun_open="web",
    visual_override={
        "world/global_map": cloud_points,
        "world/frame_cloud": cloud_points,
        "world/global_costmap": costmap_viz,
        "world/camera_info": pinhole_setup,
    },
)

_RERUN_VIZ_COMPARE = RerunBridgeModule.blueprint(
    pubsubs=[LCM()],
    rerun_open="web",
    visual_override={
        "world/global_map": cloud_points,
        "world/frame_cloud": cloud_points,
        "world/pointcloud": cloud_points,
        "world/global_costmap": costmap_viz,
        "world/camera_info": pinhole_setup,
    },
)


# ---------------------------------------------------------------------------
# Blueprints
# ---------------------------------------------------------------------------

# Full nav stack on ZED: stereo depth → 3D map → costmap → frontier → A* → velocity.
# Compose on top of a robot blueprint that exposes odometry TF and cmd_vel.
camera_nav_zed_stack = autoconnect(
    _ZED_CAMERA,
    _ZED_DEPTH,
    _ZED_ACCUM,
    CostMapper.blueprint(algo="height_cost"),
    WavefrontFrontierExplorer.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    MovementManager.blueprint(),
)

# Standalone: ZED only, no robot body — builds map and costmap, visualises in Rerun.
camera_nav_zed_standalone = autoconnect(
    _ZED_CAMERA,
    _ZED_DEPTH,
    _ZED_ACCUM,
    CostMapper.blueprint(algo="height_cost"),
    _RERUN_VIZ,
)

# Compare: side-by-side ZED SDK native cloud vs our pipeline (enable_pointcloud=True).
camera_nav_zed_compare = autoconnect(
    ZEDCamera.blueprint(
        resolution="VGA",
        enable_depth=True,
        depth_mode="NEURAL",
        enable_fill_mode=True,
        enable_pointcloud=True,
        enable_tracking=True,
        enable_imu_fusion=True,
        set_floor_as_origin=True,
    ),
    _ZED_DEPTH,
    _ZED_ACCUM,
    CostMapper.blueprint(algo="height_cost"),
    _RERUN_VIZ_COMPARE,
)

# ZED + FlowBase keyboard teleop + full autonomous nav + traversal recording.
camera_nav_zed_teleop = autoconnect(
    coordinator_flowbase_keyboard_teleop,
    ZEDCamera.blueprint(
        base_transform=_ZED_MOUNT,
        resolution="VGA",
        enable_depth=True,
        depth_mode="NEURAL",
        enable_fill_mode=True,
        enable_pointcloud=False,
        enable_tracking=True,
        enable_imu_fusion=True,
        set_floor_as_origin=True,
    ),
    _ZED_DEPTH,
    _ZED_ACCUM,
    CostMapper.blueprint(algo="height_cost"),
    WavefrontFrontierExplorer.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    MovementManager.blueprint(),
    CameraNavRecorder.blueprint(db_path="traversal.db"),
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
            depth_mode="NEURAL",
            enable_fill_mode=True,
            enable_pointcloud=False,
            enable_tracking=True,
            enable_imu_fusion=True,
            set_floor_as_origin=True,
        ),
        _ZED_DEPTH,
        _ZED_ACCUM,
        CostMapper.blueprint(algo="height_cost"),
        WavefrontFrontierExplorer.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        MovementManager.blueprint(),
        CameraNavRecorder.blueprint(db_path="traversal.db"),
        _RERUN_VIZ,
    )
