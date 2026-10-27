# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
"""Model-facing content for generated sensor and geometry messages."""

import base64
import math
from typing import Any

from dimos.message_utils import image as image_utils
from dimos.message_utils.geometry import quaternion_euler
from dimos.messages.geometry_msgs.msg import Pose, PoseStamped
from dimos.messages.sensor_msgs.msg import Image, PointCloud2


def content_blocks(value: object) -> list[dict[str, Any]] | None:
    """Encode supported messages; leave custom result protocols to their owners."""
    if isinstance(value, Image):
        jpeg = base64.b64encode(image_utils.to_jpeg(value, quality=80)).decode("ascii")
        return [{"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{jpeg}"}}]
    if isinstance(value, (Pose, PoseStamped)):
        pose = value.pose if isinstance(value, PoseStamped) else value
        position = pose.position
        roll, pitch, yaw = quaternion_euler(pose.orientation)
        if isinstance(value, PoseStamped):
            summary = (
                f"PoseStamped(pos=[{position.x:.3f}, {position.y:.3f}, {position.z:.3f}], "
                f"euler=[{math.degrees(roll):.1f}, {math.degrees(pitch):.1f}, {math.degrees(yaw):.1f}])"
            )
        else:
            q = pose.orientation
            summary = (
                f"Pose(pos=[{position.x:.3f}, {position.y:.3f}, {position.z:.3f}], "
                f"euler=[{roll:.3f}, {pitch:.3f}, {yaw:.3f}]), "
                f"quaternion=[Quaternion({q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f})])"
            )
        return [{"type": "text", "text": summary}]
    if isinstance(value, PointCloud2):
        summary = (
            f"PointCloud2(frame_id='{value.header.frame_id}', "
            f"num_points={value.width * value.height})"
        )
        return [{"type": "text", "text": summary}]
    return None
