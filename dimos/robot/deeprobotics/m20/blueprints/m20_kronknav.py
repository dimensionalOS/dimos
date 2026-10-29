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

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic as ZenohTopic
from dimos.robot.deeprobotics.m20.blueprints._nav import (
    _m20_nav,
    m20_nav_overrides,
    m20_nav_static,
)
from dimos.robot.deeprobotics.m20.camera import M20CameraRelay
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.visualization.vis_module import vis_module


def _render_h265(msg: CompressedVideo) -> Any:
    import rerun as rr

    return rr.VideoStream(codec=rr.VideoCodec.H265, sample=msg.data.tobytes())


def _render_front_camera_info(msg: CameraInfo) -> Any:
    return msg.to_rerun(
        image_topic="world/front_camera",
        optical_frame="front_camera_optical",
    )


def _render_rear_camera_info(msg: CameraInfo) -> Any:
    return msg.to_rerun(
        image_topic="world/rear_camera",
        optical_frame="rear_camera_optical",
    )


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
    "tf_axes": 0.35,
    "max_hz": {
        "world/local_map": 0.5,
    },
    "visual_override": {
        **m20_nav_overrides(),
        "world/front_camera": _render_h265,
        "world/rear_camera": _render_h265,
        "world/front_camera_info": _render_front_camera_info,
        "world/rear_camera_info": _render_rear_camera_info,
    },
    "static": m20_nav_static(),
}


deeprobotics_m20_kronknav_control = (
    autoconnect(
        vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config),
        M20CameraRelay.blueprint(instance_name="M20CameraRelay"),
        M20Connection.blueprint().remappings([(M20Connection, "odometry", "slam_odom")]),
        _m20_nav,
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
