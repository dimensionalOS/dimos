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

import threading
import time
from typing import Any

import av

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.nav_3d.mls_planner.viz import planner_visual_override
from dimos.robot.deeprobotics.m20.bridge.module import M20ROSBridge
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.constants import (
    BASE_LINK_HEIGHT_M,
    BODY_LENGTH_M,
    BODY_WIDTH_M,
    FRONT_CAMERA_RTSP_URL,
    MAX_ANGULAR_Z_RAD_S,
    MAX_LINEAR_X_M_S,
    MAX_LINEAR_Y_M_S,
    PLANNING_HEIGHT_M,
    REAR_CAMERA_RTSP_URL,
    ROTATION_DIAMETER_M,
)
from dimos.robot.deeprobotics.m20.pointlio.module import M20PointLio
from dimos.utils.logging_config import setup_logger
from dimos.visualization.vis_module import vis_module

VOXEL_SIZE_M = 0.1
PLANNER_VIZ_HZ = 0.0
logger = setup_logger()


class _M20CameraRelay(Module):
    """Relay the vendor RTSP cameras as compressed H.265 DimOS streams."""

    front_camera: Out[CompressedVideo]
    rear_camera: Out[CompressedVideo]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._threads: list[threading.Thread] = []

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        streams = (
            (FRONT_CAMERA_RTSP_URL, self.front_camera, "m20_front_camera"),
            (REAR_CAMERA_RTSP_URL, self.rear_camera, "m20_rear_camera"),
        )
        self._threads = [
            threading.Thread(
                target=self._relay,
                args=stream,
                name=f"{stream[2]}-rtsp",
                daemon=True,
            )
            for stream in streams
        ]
        for thread in self._threads:
            thread.start()

    def _relay(self, url: str, output: Out[CompressedVideo], camera_name: str) -> None:
        while not self._stop_event.is_set():
            try:
                with av.open(
                    url,
                    options={"rtsp_transport": "tcp", "fflags": "nobuffer"},
                    timeout=(1.0, 1.0),
                ) as container:
                    video = container.streams.video[0]
                    if video.codec_context.name != "hevc":
                        raise ValueError(f"expected H.265, got {video.codec_context.name}")
                    annex_b = av.BitStreamFilterContext("hevc_mp4toannexb", video)
                    logger.info("M20 camera stream connected", camera=camera_name)
                    for packet in container.demux(video):
                        if self._stop_event.is_set():
                            return
                        for filtered in annex_b.filter(packet):
                            if filtered.size:
                                output.publish(
                                    CompressedVideo(
                                        bytes(filtered),
                                        format="h265",
                                        frame_id="",
                                        ts=time.time(),
                                    )
                                )
            except (av.FFmpegError, IndexError, ValueError) as exc:
                logger.warning(
                    "M20 camera stream unavailable",
                    camera=camera_name,
                    error=str(exc),
                )
                self._stop_event.wait(2.0)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        for thread in self._threads:
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._threads.clear()
        super().stop()


def _render_global_map(msg: Any) -> Any:
    return msg.to_rerun()


def _render_path(msg: Any) -> Any:
    if len(msg.poses) == 0:
        return None
    return msg


def _render_camera_packet(msg: CompressedVideo) -> Any:
    import rerun as rr

    return rr.VideoStream.from_fields(sample=msg.data.tobytes())


def _static_h265_video(rr: Any) -> Any:
    return rr.VideoStream(codec=rr.VideoCodec.H265)


def _static_robot_body(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[
                BODY_LENGTH_M * 0.5,
                BODY_WIDTH_M * 0.5,
                PLANNING_HEIGHT_M * 0.5,
            ],
            centers=[0.0, 0.0, PLANNING_HEIGHT_M * 0.5 - BASE_LINK_HEIGHT_M],
            colors=[(0, 255, 127)],
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _m20_rerun_blueprint() -> Any:
    """Go2-style navigation layout with both M20 camera streams."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/front_camera", name="Front Camera"),
                rrb.Spatial2DView(origin="world/rear_camera", name="Rear Camera"),
                row_shares=[1, 1],
            ),
            rrb.Spatial3DView(
                origin="world",
                name="M20 KronkNav",
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


_rerun_config = {
    "blueprint": _m20_rerun_blueprint,
    # Match the Go2 navigation replay budget so a newly attached viewer catches
    # up quickly instead of replaying a large sensor backlog.
    "memory_limit": "64MB",
    "tf_axes": 0.35,
    "max_hz": {
        "world/local_map": 0.5,
        # RayTracingVoxelMap already limits this at the source.
        "world/global_map": 0,
    },
    "visual_override": {
        # The navigation view shows maps rather than the registered lidar.
        "world/lidar": None,
        "world/global_map": _render_global_map,
        "world/planner_path": None,
        "world/path": _render_path,
        "world/front_camera": _render_camera_packet,
        "world/rear_camera": _render_camera_packet,
        **planner_visual_override(PLANNER_VIZ_HZ),
    },
    "static": {
        "world/robot_body": _static_robot_body,
        "world/front_camera": _static_h265_video,
        "world/rear_camera": _static_h265_video,
    },
}


deeprobotics_m20_kronknav_control = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config),
    _M20CameraRelay.blueprint(instance_name="M20CameraRelay"),
    M20ROSBridge.blueprint(
        max_linear_x=MAX_LINEAR_X_M_S,
        max_linear_y=MAX_LINEAR_Y_M_S,
        max_angular_z=MAX_ANGULAR_Z_RAD_S,
    ),
    M20PointLio.blueprint(),
    M20Connection.blueprint(),
    RayTracingVoxelMap.blueprint(
        voxel_size=VOXEL_SIZE_M,
        max_range=25.0,
        emit_every=1,
        global_emit_every=50,
        support_min=4,
        world_frame="odom",
        worker_threads=3,
        cpu_affinity=frozenset({5}),
    ),
    MLSPlannerNative.blueprint(
        world_frame="odom",
        base_frame="base_link",
        voxel_size=VOXEL_SIZE_M,
        robot_height=PLANNING_HEIGHT_M,
        start_z_offset_m=BASE_LINK_HEIGHT_M,
        wall_clearance_m=0.3,
        wall_buffer_m=0.85,
        wall_buffer_weight=100.0,
        step_threshold_m=0.25,
        step_penalty_weight=4.0,
        viz_publish_hz=PLANNER_VIZ_HZ,
        worker_threads=2,
        cpu_affinity=frozenset({0, 1}),
    ).remappings(
        [
            (MLSPlannerNative, "global_map", "global_map_unused"),
            (MLSPlannerNative, "path", "planner_path"),
        ]
    ),
    DanLocalPlanner.blueprint(
        lock_replan=0.4,
        # Preserve MLS's 3D waypoints; the 2D resampler replaces every Z with zero.
        resample_spacing_m=0.0,
    ),
    DanHolonomicTC.blueprint(
        run_profile="walk",
        control_frequency=10.0,
    ),
    MovementManager.blueprint(),
).global_config(
    n_workers=4,
    obstacle_avoidance=False,
    robot_width=BODY_WIDTH_M,
    robot_rotation_diameter=ROTATION_DIAMETER_M,
    transport="lcm",
)
