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

"""Unitree G1 watering smoke test: GR00T + Point-LIO + camera perception.

Hardware only. The GR00T WBC coordinator (policy at priority 50, arm
trajectories at 30 over the servo hold at 10), the manipulation module with
Viser off, and the head RealSense publishing JPEG color for the Rerun
stream. Comes up unarmed + dry-run with the base blueprint's 10 s
activation ramp.

The Mid360 feeds Point-LIO directly (no mapper, costmap, planner, or nav
controller). A G1 adapter combines its physical-sensor pose with live waist
joints and publishes ``world -> pelvis``. The head camera then feeds AprilTag
detection in that same world frame; tags 0/1/2 mark pot plants.

The combined watering sequence remains disabled on hardware. Separate
``start_approach()`` and ``start_pour()`` gates keep base and arm motion as two
explicit operator-approved steps. Pouring refuses to execute unless the live
stopped-base pose is inside the offline-verified reach region.

Marker detection stays silent until this robot's camera intrinsics are
captured — see ``tool_dump_camera_info``.

Operator and approach commands enter the watering task on separate topics.
The task forwards operator commands while idle and treats any operator command
as an immediate override while approaching. One GR00T policy remains the only
locomotion task in the ControlCoordinator.

There is still no navigation stack: Point-LIO supplies observer-only odometry
and a live sensor-frame point cloud. The robot mesh is rooted under the
corrected ``world/base_pose`` and the cloud under the raw moving lidar pose.

The onboard computer (measured 2026-08-12): NVIDIA Orin NX Developer Kit,
8 cores, 15 GiB RAM + 7 GiB swap, kernel 5.10.104-tegra. The worker count
and camera caps below are sized against it.

Usage (on the G1; ``--rerun-open none`` because it has no display):
    dimos --rerun-open none --rerun-host 0.0.0.0 run unitree-g1-water-demo
Terminal teleop (in a second SSH terminal on the G1):
    dimos teleop --topic g1/tele_cmd_vel
Laptop viewer:
    dimos-viewer --connect rerun+http://<robot>:9877/proxy --ws-url ws://<robot>:3030/ws
"""

from __future__ import annotations

from typing import Any, cast

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.mobile.pose_target_observation_module import (
    PoseTargetObservationModule,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.perception.fiducial.marker_detection_stream_module import MarkerDetectionStreamModule
from dimos.perception.fiducial.marker_latch_module import MarkerLatchModule
from dimos.perception.fiducial.marker_tf_module import MarkerTfModule
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import (
    _backend,
    _G1GrootCoordinator,
    _rerun_config,
    g1_groot_coordinator,
    g1_groot_task_config,
)
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc_manip import (
    _ARM_TRAJECTORY_TASK,
    g1_manipulation,
)
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.head_camera import (
    CAMERA_STREAM_CONFIG,
    HEAD_CAMERA_MOUNT_FRAME,
    HEAD_CAMERA_NAME,
    head_camera_info,
)
from dimos.robot.unitree.g1.head_camera_tf import G1HeadCameraTf
from dimos.robot.unitree.g1.lio_base_pose import G1LioBasePose
from dimos.robot.unitree.g1.watering_task import WateringTaskModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

if global_config.simulation:
    raise ValueError(
        "unitree-g1-water-demo is hardware-only; use unitree-g1-groot-wbc-manip for sim"
    )

# Operator input and task output must stay on different topics: the task owns
# the handoff and forwards one command stream to the single GR00T policy.
_demo_remappings = [
    (_G1GrootCoordinator, "twist_command", "cmd_vel"),
    (RerunWebSocketServer, "tele_cmd_vel", "tele_cmd_vel"),
    (WebsocketVisModule, "tele_cmd_vel", "tele_cmd_vel"),
    (ManipulationModule, "odom", "base_pose"),
    (WateringTaskModule, "operator_command", "tele_cmd_vel"),
    (WateringTaskModule, "base_command", "cmd_vel"),
    (WateringTaskModule, "approach_path", "path"),
    (WateringTaskModule, "approach_goal", "goal_request"),
]

_HARDWARE_GROOT_TASK = g1_groot_task_config(timeout=0.25)

_CAMERA_ENTITY = "world/color_compressed"

# All three tags mark pot plants at different spots; the first one the robot
# sees consistently becomes the target. Physical black-border edge, printed at
# 100% from `dimos apriltag --ids 0,1,2 --size-mm 150`.
_PLANT_MARKER_IDS = [0, 1, 2]
_MARKER_LENGTH_M = 0.15
_WORLD_FRAME = "world"
_BASE_FRAME = "pelvis"


def _plant_perception() -> Any:
    """Head camera -> AprilTag detections -> one latched pot pose."""
    return autoconnect(
        G1HeadCameraTf.blueprint(base_frame=_BASE_FRAME, camera_frame=HEAD_CAMERA_MOUNT_FRAME),
        MarkerDetectionStreamModule.blueprint(
            marker_length_m=_MARKER_LENGTH_M,
            # Detection resolves poses in this frame, so it is what the
            # latched pose ends up expressed in.
            world_frame=_WORLD_FRAME,
            camera_info=head_camera_info(),
        ),
        MarkerTfModule.blueprint(world_frame=_WORLD_FRAME),
        MarkerLatchModule.blueprint(marker_ids=_PLANT_MARKER_IDS, frame_id=_WORLD_FRAME),
    )


def _water_demo_rerun_blueprint() -> Any:
    """3D scene beside the head camera; the base layout is 3D-only."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                origin="world",
                name="G1 water demo",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.0)),
            ),
            rrb.Spatial2DView(origin=_CAMERA_ENTITY, name="Head camera"),
            column_shares=[2, 1],
        ),
        rrb.TimePanel(state="collapsed"),
    )


def _pointcloud_to_rerun(cloud: Any) -> Any:
    """Render a point cloud from a worker-picklable callable."""
    return cloud.to_rerun()


def _object_pose_to_rerun(pose: Any) -> Any:
    """Render the detected object pose from a worker-picklable callable."""
    return pose.to_rerun_arrow(length=0.35)


def _goal_pose_to_rerun(pose: Any) -> Any:
    """Render the approach goal from a worker-picklable callable."""
    return pose.to_rerun_arrow(length=0.45)


# Camera caps are non-negotiable on the Jetson: uncapped image streams have
# eaten 20 GB of Rerun RAM before. coordinator_joint_state is the coordinator's
# full 29-joint aggregate at the 100 Hz tick — it saturates the viewer's gRPC
# channel on its own, and the per-robot /g1/joints stream already drives the mesh.
_demo_rerun_config: dict[str, Any] = {
    **_rerun_config,
    "blueprint": _water_demo_rerun_blueprint,
    "max_hz": {
        **_rerun_config["max_hz"],
        _CAMERA_ENTITY: 5.0,
        "world/lidar": 10.0,
        "world/path": 0,
    },
    "visual_override": {
        **_rerun_config["visual_override"],
        "world/coordinator_joint_state": None,
        # PointLIO stamps the cloud as mid360_link and publishes the matching
        # world -> mid360_link TF; Rerun attaches the cloud to that frame.
        "world/lidar": _pointcloud_to_rerun,
        "world/object_pose": _object_pose_to_rerun,
        "world/goal_request": _goal_pose_to_rerun,
    },
    "memory_limit": "5%",
}

unitree_g1_water_demo = (
    autoconnect(
        _backend,
        g1_groot_coordinator(
            extra_tasks=(_ARM_TRAJECTORY_TASK,),
            locomotion_task=_HARDWARE_GROOT_TASK,
        ),
        PointLio.blueprint(
            frame_id="world",
            host_ip=G1.lidar_host_ip,
            lidar_ip=G1.lidar_ip,
        ),
        G1LioBasePose.blueprint(
            world_frame="world",
            sensor_frame="mid360_link",
            base_frame="pelvis",
        ),
        # Raw color feeds marker detection on-robot; the JPEG copy is what
        # crosses the network, logged to Rerun as an EncodedImage so the robot
        # never decodes and raw color never costs ~18 MB/s of egress.
        RealSenseCamera.blueprint(
            camera_name=HEAD_CAMERA_NAME,
            base_frame_id=HEAD_CAMERA_MOUNT_FRAME,
            enable_pointcloud=False,
            compress_color=True,
            **CAMERA_STREAM_CONFIG,
        ),
        _plant_perception(),
        PoseTargetObservationModule.blueprint(
            object_id="plant_pot_1",
            label="plant pot",
            source="perception",
        ),
        g1_manipulation(visualization=ViserVisualizationConfig(host="0.0.0.0")),
        WateringTaskModule.blueprint(
            target_id="plant_pot_1",
            motion_enabled=False,
            approach_motion_enabled=True,
            pour_motion_enabled=True,
            approach_holonomic=True,
            approach_max_linear=0.15,
            approach_max_angular=0.25,
        ),
        vis_module(viewer_backend=global_config.viewer, rerun_config=_demo_rerun_config),
    )
    .remappings(cast("Any", _demo_remappings))
    .transports(
        {
            ("tele_cmd_vel", Twist): LCMTransport("/g1/tele_cmd_vel", Twist),
        }
    )
    # Sized for the heavy modules only — the connection pump, the 100 Hz tick,
    # the camera capture loop, marker detection, planning and the Rerun bridge.
    # The rest share: every worker is a process that imports the whole stack,
    # and one-per-module exhausted the Jetson's RAM (workers died with EOF).
    .global_config(robot_model="unitree_g1", n_workers=7)
)
