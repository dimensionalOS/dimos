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

"""PX4 hardware and Gazebo Harmonic blueprints."""

from typing import Any

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport, pSHMTransport
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.drone.px4.flight_control import FlightController
from dimos.robot.drone.px4.gstreamer_tee_camera import GsTeeCamera, GstInputFormat
from dimos.robot.drone.px4.mid360_mount_tf import Mid360MountStaticTf
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module

RERUN_VIDEO_ENTITY = "drone/video"


def _video_h264_to_rerun(video: CompressedVideo) -> Any:
    return [(RERUN_VIDEO_ENTITY, video.to_rerun())]


def _rerun_layout() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin=RERUN_VIDEO_ENTITY, name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


def _gazebo_rerun_layout() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial2DView(origin=RERUN_VIDEO_ENTITY, name="Camera"),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


def _static_drone_body(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[0.2, 0.2, 0.1],
            colors=[(255, 100, 0)],
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


rerun_config: dict[str, Any] = {
    "blueprint": _rerun_layout,
    "visual_override": {
        "world/color_image": None,
        "world/video_h264": _video_h264_to_rerun,
        "world/region_bounds": None,
    },
    "max_hz": {
        "world/global_map": 0,
        "world/local_map": 0,
        "world/lidar": 5.0,
    },
    "static": {"world/drone/body": _static_drone_body},
}

gazebo_rerun_config: dict[str, Any] = {
    "blueprint": _gazebo_rerun_layout,
    "visual_override": {
        "world/color_image": None,
        "world/video_h264": _video_h264_to_rerun,
    },
}


px4_basic = (
    autoconnect(
        PointLio.blueprint(sensor_frame_id="mid360_link"),
        Mid360MountStaticTf.blueprint(),
        FlightController.blueprint(),
        GsTeeCamera.blueprint(),
        RayTracingVoxelMap.blueprint(
            voxel_size=0.1,
            max_range=5.0,
            shadow_depth=0.1,
            min_health=0,
            max_health=5,
            emit_every=2,
            ray_subsample=1,
            global_emit_every=10,
        ),
        vis_module(
            global_config.viewer,
            rerun_config=rerun_config,
        ),
    )
    .transports(
        {
            ("color_image", Image): pSHMTransport.spec(
                "/color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
            ),
            ("video_h264", CompressedVideo): LCMTransport.spec("/video_h264", CompressedVideo),
        }
    )
    .remappings([(RerunWebSocketServer, "tele_cmd_vel", "cmd_vel")])
    .global_config(n_workers=8, robot_model="px4_basic")
)

px4_gazebo_harmonic = (
    autoconnect(
        FlightController.blueprint(connection_url="udpin://0.0.0.0:14540"),
        GsTeeCamera.blueprint(
            input_pipeline=(
                "udpsrc port=5600 caps=application/x-rtp,media=video,"
                "encoding-name=H264,payload=96 ! rtph264depay ! h264parse config-interval=-1"
            ),
            input_format=GstInputFormat.H264,
        ),
        vis_module(global_config.viewer, rerun_config=gazebo_rerun_config),
    )
    .transports(
        {
            ("color_image", Image): pSHMTransport.spec(
                "/color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
            ),
            ("video_h264", CompressedVideo): LCMTransport.spec("/video_h264", CompressedVideo),
        }
    )
    .remappings([(RerunWebSocketServer, "tele_cmd_vel", "cmd_vel")])
    .global_config(n_workers=4, robot_model="px4_gazebo_harmonic")
)
