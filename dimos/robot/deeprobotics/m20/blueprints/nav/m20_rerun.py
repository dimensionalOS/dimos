"""Rerun visualization helpers for M20 navigation blueprints."""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.impl.lcmpubsub import LCM, Topic

REGISTERED_SCAN_THROTTLE = "registered_scan"
DEBUG_CLOUD_THROTTLE = "debug_cloud"


@dataclass(frozen=True)
class M20RerunTopicSpec:
    name: str
    msg_type: type[Any]
    throttle_group: str | None = None

    @property
    def topic(self) -> Topic:
        return Topic(self.name, self.msg_type)

    @property
    def channel(self) -> str:
        return str(self.topic)


M20_RERUN_TOPIC_SPECS = (
    M20RerunTopicSpec("/registered_scan", PointCloud2, REGISTERED_SCAN_THROTTLE),
    M20RerunTopicSpec("/terrain_map", PointCloud2, DEBUG_CLOUD_THROTTLE),
    M20RerunTopicSpec("/terrain_map_ext", PointCloud2, DEBUG_CLOUD_THROTTLE),
    M20RerunTopicSpec("/obstacle_cloud", PointCloud2, DEBUG_CLOUD_THROTTLE),
    M20RerunTopicSpec("/costmap_cloud", PointCloud2, DEBUG_CLOUD_THROTTLE),
    M20RerunTopicSpec("/global_map", PointCloud2, DEBUG_CLOUD_THROTTLE),
    M20RerunTopicSpec("/global_map_pgo", PointCloud2, DEBUG_CLOUD_THROTTLE),
    M20RerunTopicSpec("/odometry", Odometry),
    M20RerunTopicSpec("/corrected_odometry", Odometry),
    M20RerunTopicSpec("/path", Path),
    M20RerunTopicSpec("/goal_path", Path),
    M20RerunTopicSpec("/clicked_point", PointStamped),
    M20RerunTopicSpec("/goal_request", PoseStamped),
    M20RerunTopicSpec("/goal", PointStamped),
    M20RerunTopicSpec("/way_point", PointStamped),
)


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name, "").strip()
    if not value:
        return default
    return float(value)


def _env_int(name: str, default: int) -> int:
    value = os.environ.get(name, "").strip()
    if not value:
        return default
    return int(value)


def registered_scan_max_points() -> int:
    return _env_int("M20_RERUN_REGISTERED_SCAN_MAX_POINTS", 2_000)


def registered_scan_period_sec() -> float | None:
    hz = _env_float("M20_RERUN_REGISTERED_SCAN_HZ", 0.2)
    if hz <= 0.0:
        return None
    return 1.0 / hz


def debug_cloud_period_sec() -> float | None:
    hz = _env_float("M20_RERUN_DEBUG_CLOUD_HZ", 1.0)
    if hz <= 0.0:
        return None
    return 1.0 / hz


class M20RerunLCM(LCM):
    """LCM pubsub allowlist for the M20 Rerun bridge."""

    def subscribe_all(self, callback: Any) -> Any:
        periods = {
            REGISTERED_SCAN_THROTTLE: registered_scan_period_sec(),
            DEBUG_CLOUD_THROTTLE: debug_cloud_period_sec(),
        }
        last_by_channel: dict[str, float] = {}
        point_cloud_subs = []

        def subscribe_throttled(spec: M20RerunTopicSpec, period: float | None) -> None:
            if period is None:
                return

            def on_point_cloud(channel: str, data: bytes) -> None:
                now = time.monotonic()
                last = last_by_channel.get(spec.channel, 0.0)
                if now - last < period:
                    return
                last_by_channel[spec.channel] = now
                callback(
                    spec.msg_type.lcm_decode(data),
                    Topic.from_channel_str(channel, spec.msg_type),
                )

            sub = self.l.subscribe(  # type: ignore[union-attr]
                spec.channel,
                on_point_cloud,
            )
            sub.set_queue_capacity(2)
            point_cloud_subs.append(sub)

        unsubscribes = []
        for spec in M20_RERUN_TOPIC_SPECS:
            if spec.throttle_group is None:
                unsubscribes.append(self.subscribe(spec.topic, callback))
            else:
                subscribe_throttled(spec, periods[spec.throttle_group])

        def unsubscribe_all() -> None:
            for sub in point_cloud_subs:
                self.l.unsubscribe(sub)  # type: ignore[union-attr]
            for unsubscribe in unsubscribes:
                unsubscribe()

        return unsubscribe_all


def m20_rerun_blueprint() -> Any:
    """3D world view stacked over a 2D camera panel for the M20."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Spatial3DView(origin="world", name="3D"),
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            row_shares=[2, 1],
        ),
    )


def camera_info_override(camera_info: Any) -> Any:
    """Route camera intrinsics to the color_image topic for 3D projection."""
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def raw_points_override(cloud: Any) -> None:
    """Hide body-frame raw lidar; registered_scan is the useful live view."""
    return None


def registered_scan_override(cloud: Any) -> Any:
    """Render a bounded FAST-LIO registered-scan sample as points."""
    import rerun as rr

    points = cloud.points_f32()
    max_points = registered_scan_max_points()
    if max_points <= 0:
        return rr.Points3D([])
    if len(points) > max_points:
        step = math.ceil(len(points) / max_points)
        points = points[::step]
    if len(points) == 0:
        return rr.Points3D([])

    z = points[:, 2]
    class_ids = ((z - z.min()) / (z.max() - z.min() + 1e-8) * 255).astype(np.uint8)
    return rr.Points3D(positions=points, class_ids=class_ids)


def m20_odometry_tf_override(odom: Any) -> Any:
    """Publish FAST-LIO odometry as the vehicle TF frame for Rerun overlays."""
    import rerun as rr

    tf = rr.Transform3D(
        translation=[odom.x, odom.y, odom.z],
        rotation=rr.Quaternion(
            xyzw=[
                odom.orientation.x,
                odom.orientation.y,
                odom.orientation.z,
                odom.orientation.w,
            ]
        ),
        parent_frame="tf#/map",
        child_frame="tf#/sensor",
    )
    return [
        ("tf#/sensor", tf),
    ]


def static_robot(rr: Any) -> list[Any]:
    """Wireframe bounding box at M20 dimensions (820x430x570mm)."""
    return [
        rr.Boxes3D(
            half_sizes=[0.41, 0.215, 0.285],
            colors=[(255, 140, 0)],
            fill_mode="wireframe",
        ),
        rr.Transform3D(parent_frame="tf#/sensor"),
    ]
