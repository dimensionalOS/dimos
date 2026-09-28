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

"""The PX4 blueprints: the connection alone, the aircraft, its simulator twin, and teleop.

``px4-basic``
    Px4DroneConnection plus the viewer, against mavlink-router endpoint 14556.
``px4-drone``
    Everything on the aircraft: the connection, the A8 camera and gimbal.
``px4-sitl``
    The same modules against PX4 SITL (``make px4_sitl gz_x500``). The camera replays a
    generated clip; the simulator has no A8, so the gimbal module gets no attitude and
    publishes no tf. ``tool_sitl_gate.py`` runs this blueprint end to end.
``px4-teleop``, ``px4-sitl-teleop``
    The aircraft and its twin with the viewer's keyboard on ``cmd_vel``, the way
    ``r1pro-teleop`` wires it.

The streams that need a pinned topic or QoS are declared once in :func:`px4_transports`; a
key for a port no module in the blueprint has is simply unused.
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import Blueprint, TransportSpec, autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import Transport
from dimos.core.transport import ZenohTransport
from dimos.hardware.gimbal.siyi.gimbal import SiyiA8Gimbal
from dimos.hardware.sensors.camera.rtsp.camera import SYNTHETIC_URL, RtspCamera
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.px4_msgs.VehicleStatus import VehicleStatus
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic as ZenohTopic, Zenoh
from dimos.robot.px4.config import A8_RTSP_URL, GIMBAL_MOUNT_XYZ_UNMEASURED, SITL_MAV_URL
from dimos.robot.px4.connection import Px4DroneConnection
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

# Streams


def _zenoh_transport(topic: str, msg_type: type, *, latest_wins: bool = False) -> TransportSpec:
    """One zenoh topic ``dimos/<topic>``; latest-wins for streams where stale beats late."""
    return ZenohTransport.spec(
        ZenohTopic(f"dimos/{topic}", msg_type, qos=QOS_LATEST_WINS if latest_wins else None)
    )


def px4_transports() -> dict[tuple[str, type], TransportSpec | Transport[Any]]:
    """Every stream of the package, keyed by (port name, type), the way modules bind.

    The heavy vehicle streams are latest-wins: a stale odometry sample is worse than a
    dropped one. ``color_image`` is left on the factory default (zenoh, latest-wins): 2.8 MB
    a frame, nothing off the aircraft should subscribe to it.
    """
    return {
        # Into the connection. cmd_vel is the public Twist bus any teleop module drives.
        ("cmd_vel", Twist): _zenoh_transport("cmd_vel", Twist),
        # Out of the connection: the vehicle.
        ("odometry", Odometry): _zenoh_transport("odometry", Odometry, latest_wins=True),
        ("odom", PoseStamped): _zenoh_transport("odom", PoseStamped, latest_wins=True),
        ("tf", TFMessage): _zenoh_transport("tf", TFMessage, latest_wins=True),
        ("imu", Imu): _zenoh_transport("imu", Imu, latest_wins=True),
        ("gps", NavSatFix): _zenoh_transport("gps", NavSatFix),
        ("battery", BatteryState): _zenoh_transport("battery", BatteryState),
        ("gimbal_attitude", JointState): _zenoh_transport(
            "gimbal_attitude", JointState, latest_wins=True
        ),
        ("vehicle_status", VehicleStatus): _zenoh_transport("vehicle_status", VehicleStatus),
        ("statustext", String): _zenoh_transport("statustext", String),
        # Out of the connection: the supervisor.
        ("supervisor_state", String): _zenoh_transport("supervisor_state", String),
        # Camera and gimbal.
        ("video", CompressedVideo): _zenoh_transport("video", CompressedVideo, latest_wins=True),
        ("color_jpeg", CompressedImage): _zenoh_transport(
            "color_jpeg", CompressedImage, latest_wins=True
        ),
        ("camera_info", CameraInfo): _zenoh_transport("camera_info", CameraInfo),
    }


# Viewer


def _px4_rerun_blueprint() -> Any:
    """3D world over the video, the battery trace and the status log.

    Entity paths assume the bridge's default ``entity_prefix="world"``.
    """
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial3DView(
                    origin="world",
                    name="3D",
                    background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                    line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(1.0)),
                ),
                rrb.Spatial2DView(origin="world/video", name="A8 H.265"),
                row_shares=[2, 1],
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(origin="world/battery", name="Battery"),
                rrb.TextLogView(origin="world", name="Status"),
            ),
            column_shares=[2, 1],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


# Per-entity rate caps for the rerun bridge (visualization only; the flight loop sees full
# rate). Sized for a companion computer viewed over Wi-Fi. `world/video` must never be capped:
# dropping H.265 access units breaks in-viewer decode; the camera sets that rate.
_RERUN_MAX_HZ = {
    "world/color_jpeg": 2.0,
    "world/odometry": 10.0,
    "world/odom": 10.0,
    "world/tf": 10.0,
    "world/imu": 5.0,
}
# Suppressed entirely: raw decoded frames are the heaviest payload on the viewer link.
_RERUN_VISUAL_OVERRIDE = {"world/color_image": None}

rerun_config = {
    "blueprint": _px4_rerun_blueprint,
    "pubsubs": [Zenoh()],
    "rerun_open": global_config.rerun_open,
    "rerun_web": global_config.rerun_web,
    # A live viewer needs only a small rolling buffer, and every viewer (re)connect
    # replays the whole buffer before going live.
    "memory_limit": "256MB",
    "max_hz": _RERUN_MAX_HZ,
    "visual_override": _RERUN_VISUAL_OVERRIDE,
}


def px4_visualization() -> Blueprint:
    if global_config.viewer == "rerun":
        return autoconnect(
            RerunBridgeModule.blueprint(**rerun_config),
            RerunWebSocketServer.blueprint(),
        )
    if global_config.viewer == "none":
        return Blueprint(blueprints=())
    raise ValueError(f"Unsupported viewer: {global_config.viewer}")


# Layers


def px4_control(**connection: Any) -> Blueprint:
    """Px4DroneConnection; kwargs are Px4DroneConnectionConfig fields (``mav_url``, ``sitl``...)."""
    return autoconnect(Px4DroneConnection.blueprint(**connection))


# n_workers keeps the viewer encode out of the interpreter that runs the gimbal; the
# connection and camera are dedicated workers, so the 20 Hz flight loop never shares a GIL
# with anything. (The registry scanner wants each blueprint as one top-level autoconnect
# chain, hence the repetition.)
px4_basic = (
    autoconnect(px4_visualization(), px4_control())
    .transports(px4_transports())
    .global_config(transport="zenoh", n_workers=2)
)

# The gimbal module owns the gimbal tf chain.
px4_drone = (
    autoconnect(
        px4_visualization(),
        px4_control(),
        RtspCamera.blueprint(url=A8_RTSP_URL, frame_id="a8_optical", capture_latency_s=0.08),
        SiyiA8Gimbal.blueprint(mount_xyz=GIMBAL_MOUNT_XYZ_UNMEASURED),
    )
    .transports(px4_transports())
    .global_config(transport="zenoh", n_workers=2)
)

# The camera keeps the aircraft's frame and latency pins so the twin stamps frames the same
# way.
px4_sitl = (
    autoconnect(
        px4_visualization(),
        px4_control(mav_url=SITL_MAV_URL, sitl=True),
        RtspCamera.blueprint(
            url=SYNTHETIC_URL,
            frame_id="a8_optical",
            capture_latency_s=0.08,
        ),
        SiyiA8Gimbal.blueprint(mount_xyz=GIMBAL_MOUNT_XYZ_UNMEASURED),
    )
    .transports(px4_transports())
    .global_config(transport="zenoh", n_workers=2)
)

# Viewer keyboard teleop, as in r1pro_teleop: the dimos-viewer's WASD twist drives cmd_vel.
# The connection honours it only in TELEOP, so the keys do nothing until the operator
# selects that mode.
_VIEWER_TELEOP = [(RerunWebSocketServer, "tele_cmd_vel", "cmd_vel")]

px4_teleop = px4_drone.remappings(_VIEWER_TELEOP)
px4_sitl_teleop = px4_sitl.remappings(_VIEWER_TELEOP)
