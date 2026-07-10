#!/usr/bin/env python3
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

"""R1 Pro ControlCoordinator: R1ProConnection Module + transport_lcm bridges.

Mirrors ``unitree_g1_coordinator.py``. Whole-body upper-body (18-DOF) goes
through ``TransportWholeBodyAdapter``; the holonomic chassis (3-DOF) goes
through ``TransportTwistAdapter``. No R1Pro-specific adapter code — same
shape as the G1 wiring.

Usage:
    dimos run r1pro-coordinator                # no viewer
    dimos --viewer rerun run r1pro-coordinator # composes the rerun bridge
    dimos --viewer rerun-web run r1pro-coordinator
"""

from __future__ import annotations

from typing import Any

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import JpegLcmTransport, LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.galaxea.r1pro.connection import R1PRO_UPPER_BODY_JOINTS, R1ProConnection

_chassis_joints = make_twist_base_joints("chassis")


def _r1pro_rerun_blueprint() -> Any:
    """Two-tab viewer layout: main (head stereo + 3D) and all cameras + depth.

    Entity paths assume the bridge's default ``entity_prefix="world"`` — so
    LCM topic ``/r1pro/head_left_color`` lands at ``world/r1pro/head_left_color``.
    """
    import rerun as rr
    import rerun.blueprint as rrb

    main_tab = rrb.Horizontal(
        rrb.Vertical(
            rrb.Spatial2DView(origin="world/r1pro/head_left_color", name="Head left"),
            rrb.Spatial2DView(origin="world/r1pro/head_right_color", name="Head right"),
        ),
        rrb.Spatial3DView(
            origin="world",
            name="3D",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(
                plane=rr.components.Plane3D.XY.with_distance(0.5),
            ),
        ),
        column_shares=[1, 2],
        name="Head + 3D",
    )

    cameras_tab = rrb.Grid(
        # Row 1 — RGB.
        rrb.Spatial2DView(origin="world/r1pro/head_left_color", name="Head left"),
        rrb.Spatial2DView(origin="world/r1pro/head_right_color", name="Head right"),
        rrb.Spatial2DView(origin="world/r1pro/wrist_left_color", name="Wrist left"),
        rrb.Spatial2DView(origin="world/r1pro/wrist_right_color", name="Wrist right"),
        # Row 2 — depth.
        rrb.Spatial2DView(origin="world/r1pro/head_depth", name="Head depth"),
        rrb.Spatial2DView(origin="world/r1pro/wrist_left_depth", name="Wrist left depth"),
        rrb.Spatial2DView(origin="world/r1pro/wrist_right_depth", name="Wrist right depth"),
        grid_columns=4,
        name="All cameras",
    )

    return rrb.Blueprint(
        rrb.Tabs(main_tab, cameras_tab),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


# Per-entity rate caps for the rerun bridge. Camera/depth streams are
# visualization-only; capping them keeps the bridge off the hot path without
# affecting what the ControlCoordinator sees. Keys are rerun entity paths
# (entity_prefix "world" + the LCM topic).
#
# Sized for an ON-ROBOT deployment viewed over WiFi (2026-07-10): the Orin
# shares 8 cores with the vendor HDAS stack, and rerun's gRPC stream is a
# reliable ordered log — anything logged faster than the link carries shows
# up as unbounded viewer lag, not dropped frames. Raise these after the
# Tier-2 JPEG passthrough (NEWGEN_DROP_ELIMINATION.md) lands.
_RERUN_MAX_HZ = {
    "world/r1pro/head_left_color": 5.0,
    "world/r1pro/head_right_color": 5.0,
    "world/r1pro/wrist_left_color": 5.0,
    "world/r1pro/wrist_right_color": 5.0,
    # Raw float32 Points3D; uncapped it out-bytes every camera stream.
    "world/r1pro/lidar": 5.0,
}


def _compress_color_for_viewer(msg: Any) -> Any:
    """Re-encode a color frame to JPEG before it enters the rerun stream.

    The default path logs raw RGB (``rr.Image``) — ~2.7 MB/frame, which a
    WiFi-connected viewer can never keep up with (the robot was measured
    pushing >100 MB/s of raw pixels at a ~5 MB/s link; the viewer fell
    minutes behind). JPEG q75 is ~30-60x smaller. Costs one extra encode on
    the bridge worker — a clear net win over shipping raw. Remove once the
    Tier-2 JPEG passthrough carries original camera bytes end-to-end.
    """
    import rerun as rr

    try:
        data = getattr(msg, "data", None)
        if data is None:
            return msg  # not an Image — let the bridge's default conversion run
        image = rr.Image(data)
        compress = getattr(image, "compress", None)
        return compress(jpeg_quality=75) if compress is not None else image
    except Exception:
        # Never let a compression failure cost a frame — fall back to the
        # bridge's default (raw) conversion path.
        return msg


# Per-topic overrides for the rerun bridge (None = suppress entirely).
_RERUN_VISUAL_OVERRIDE = {
    "world/r1pro/head_left_color": _compress_color_for_viewer,
    "world/r1pro/head_right_color": _compress_color_for_viewer,
    "world/r1pro/wrist_left_color": _compress_color_for_viewer,
    "world/r1pro/wrist_right_color": _compress_color_for_viewer,
    # Raw 32FC1 depth frames are the largest per-message payloads on the
    # viewer link and depth is visualization-only here — suppressed until
    # Tier-2 lands. Head depth additionally has no producer on the new-gen
    # robot (no ZED).
    "world/r1pro/wrist_left_depth": None,
    "world/r1pro/wrist_right_depth": None,
    "world/r1pro/head_depth": None,
}


_rerun_config = {
    "blueprint": _r1pro_rerun_blueprint,
    "pubsubs": [LCM()],
    # Default is "25%" — ~4 GB of a 16 GB box reserved for rerun's recording
    # store. A live viewer needs only a small rolling buffer; cap it low.
    "memory_limit": "1GB",
    "max_hz": _RERUN_MAX_HZ,
    "visual_override": _RERUN_VISUAL_OVERRIDE,
}


_r1pro_base = (
    autoconnect(
        R1ProConnection.blueprint(),
        ControlCoordinator.blueprint(
            tick_rate=100,
            hardware=[
                HardwareComponent(
                    hardware_id="r1pro",
                    hardware_type=HardwareType.WHOLE_BODY,
                    joints=R1PRO_UPPER_BODY_JOINTS,
                    adapter_type="transport_lcm",
                ),
                HardwareComponent(
                    hardware_id="chassis",
                    hardware_type=HardwareType.BASE,
                    joints=_chassis_joints,
                    adapter_type="transport_lcm",
                ),
            ],
            tasks=[
                TaskConfig(
                    name="servo_r1pro",
                    type="servo",
                    joint_names=R1PRO_UPPER_BODY_JOINTS,
                    priority=10,
                ),
                TaskConfig(
                    name="vel_chassis",
                    type="velocity",
                    joint_names=_chassis_joints,
                    priority=10,
                ),
            ],
        ),
    )
    # Module's `cmd_vel`/`odom` collide with the chassis transport adapter's
    # /{hw}/cmd_vel + /{hw}/odom topics — rename so the adapter (hw_id="chassis")
    # owns the canonical names.
    .remappings(
        [
            (R1ProConnection, "cmd_vel", "chassis_cmd_vel"),
            (R1ProConnection, "odom", "chassis_odom"),
        ]
    )
    .transports(
        {
            # WholeBody bridge (hw_id="r1pro"). TransportWholeBodyAdapter
            # subscribes /{hw}/imu — only one IMU goes there.
            ("motor_states", JointState): LCMTransport("/r1pro/motor_states", JointState),
            ("imu_chassis", Imu): LCMTransport("/r1pro/imu", Imu),
            ("imu_torso", Imu): LCMTransport("/r1pro/imu_torso", Imu),
            ("motor_command", MotorCommandArray): LCMTransport(
                "/r1pro/motor_command", MotorCommandArray
            ),
            # Twist bridge (hw_id="chassis").
            ("chassis_cmd_vel", Twist): LCMTransport("/chassis/cmd_vel", Twist),
            ("chassis_odom", PoseStamped): LCMTransport("/chassis/odom", PoseStamped),
            # Public Twist bus on /cmd_vel — `cmd_vel` covers any module's Out
            # (KeyboardTeleop, phone teleop, etc.); `twist_command` is the
            # ControlCoordinator's matching In. Both pinned to the same LCM
            # topic so any Twist publisher drives the chassis.
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
            # Sensor pass-throughs — downstream consumers (rerun bridge,
            # perception modules, etc.) attach to these topics directly.
            # Chassis surround cams intentionally absent: disabled by default
            # in R1ProConnectionConfig (CSI capture meltdown — see
            # scripts/r1pro_test/STREAM_RELIABILITY_DIAGNOSTIC.md) and the new
            # chassis has none. To bring them back: enable_chassis_cameras=True
            # on R1ProConnection.blueprint() + re-pin their JpegLcmTransports.
            ("head_left_color", Image): JpegLcmTransport("/r1pro/head_left_color", Image),
            ("head_right_color", Image): JpegLcmTransport(
                "/r1pro/head_right_color", Image
            ),
            ("head_depth", Image): LCMTransport("/r1pro/head_depth", Image),
            ("lidar", PointCloud2): LCMTransport("/r1pro/lidar", PointCloud2),
            ("wrist_left_color", Image): JpegLcmTransport("/r1pro/wrist_left_color", Image),
            ("wrist_left_depth", Image): LCMTransport("/r1pro/wrist_left_depth", Image),
            ("wrist_right_color", Image): JpegLcmTransport(
                "/r1pro/wrist_right_color", Image
            ),
            ("wrist_right_depth", Image): LCMTransport("/r1pro/wrist_right_depth", Image),
            # ControlCoordinator outs.
            ("joint_state", JointState): LCMTransport(
                "/coordinator/joint_state", JointState
            ),
            ("joint_command", JointState): LCMTransport("/r1pro/joint_command", JointState),
        }
    )
    # One worker per module: the 100 Hz ControlCoordinator tick loop must not
    # share an interpreter with the connection's camera-decode threads.
    .global_config(n_workers=3)
)


# Compose the rerun bridge when the user asks for it via `dimos --viewer rerun`.
# Mirrors `unitree_go2_basic` / `uintree_g1_primitive_no_nav` — gate on
# global_config.viewer.startswith("rerun") so "rerun", "rerun-web", and
# "rerun-connect" all light up. Without the flag, no bridge is attached and
# nothing changes from the headless coordinator path.
if global_config.viewer.startswith("rerun"):
    from dimos.visualization.rerun.bridge import RerunBridgeModule

    r1pro_coordinator = autoconnect(
        _r1pro_base,
        RerunBridgeModule.blueprint(
            rerun_open=global_config.rerun_open,
            rerun_web=global_config.rerun_web,
            **_rerun_config,
        ),
    )
else:
    r1pro_coordinator = _r1pro_base


__all__ = ["r1pro_coordinator"]
