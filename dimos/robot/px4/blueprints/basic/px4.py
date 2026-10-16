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

"""Any PX4 vehicle: Px4DroneConnection, the viewer and its keyboard.

The connection reads mavlink-router endpoint 14556. The viewer's keyboard drives
``cmd_vel``, which the connection honours only in ``TELEOP``.

Usage:
    dimos run px4                                   # the router faces PX4's companion port
    dimos run px4 --px4droneconnection.link=radio   # the router faces a telemetry radio
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import Blueprint, TransportSpec, autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.px4_msgs.VehicleStatus import VehicleStatus
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic as ZenohTopic, Zenoh
from dimos.robot.px4.connection import Px4DroneConnection
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer


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
    """The viewer. Its keyboard (W/S, Q/E, A/D) drives ``cmd_vel``, as in ``r1pro-teleop``."""
    if global_config.viewer == "rerun":
        return autoconnect(
            RerunBridgeModule.blueprint(**rerun_config),
            RerunWebSocketServer.blueprint(),
        ).remappings([(RerunWebSocketServer, "tele_cmd_vel", "cmd_vel")])
    if global_config.viewer == "none":
        return Blueprint(blueprints=())
    raise ValueError(f"Unsupported viewer: {global_config.viewer}")


def zenoh_transport(topic: str, msg_type: type, *, latest_wins: bool = False) -> TransportSpec:
    """One zenoh topic ``dimos/<topic>``; latest-wins for streams where stale beats late."""
    return ZenohTransport.spec(
        ZenohTopic(f"dimos/{topic}", msg_type, qos=QOS_LATEST_WINS if latest_wins else None)
    )


def px4_control(**connection: Any) -> Blueprint:
    """Px4DroneConnection and its streams. Keyword args are ``Px4DroneConnectionConfig`` fields."""
    return (
        autoconnect(Px4DroneConnection.blueprint(**connection))
        .transports(
            {
                # The public Twist bus any teleop module drives.
                ("cmd_vel", Twist): zenoh_transport("cmd_vel", Twist),
                # The vehicle. A stale odometry sample is worse than a dropped one.
                ("odometry", Odometry): zenoh_transport("odometry", Odometry, latest_wins=True),
                ("odom", PoseStamped): zenoh_transport("odom", PoseStamped, latest_wins=True),
                ("tf", TFMessage): zenoh_transport("tf", TFMessage, latest_wins=True),
                ("imu", Imu): zenoh_transport("imu", Imu, latest_wins=True),
                ("gps", NavSatFix): zenoh_transport("gps", NavSatFix),
                ("battery", BatteryState): zenoh_transport("battery", BatteryState),
                ("gimbal_attitude", JointState): zenoh_transport(
                    "gimbal_attitude", JointState, latest_wins=True
                ),
                ("vehicle_status", VehicleStatus): zenoh_transport("vehicle_status", VehicleStatus),
                ("statustext", String): zenoh_transport("statustext", String),
                # The supervisor.
                ("supervisor_state", String): zenoh_transport("supervisor_state", String),
            }
        )
        .global_config(transport="zenoh")
    )


px4 = autoconnect(px4_visualization(), px4_control())
