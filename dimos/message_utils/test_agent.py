# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
"""Check the bytes and summaries delivered to model clients."""

import base64

import cv2
import numpy as np

from dimos.message_utils.agent import content_blocks
from dimos.message_utils.image import from_array
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.std_msgs.msg import Header


def test_image_content_keeps_rgb_pixels_in_jpeg() -> None:
    pixels = np.full((8, 8, 3), [220, 30, 15], dtype=np.uint8)
    blocks = content_blocks(from_array(pixels, encoding="rgb8"))
    assert blocks is not None
    assert blocks[0]["type"] == "image_url"
    prefix, payload = blocks[0]["image_url"]["url"].split(",", 1)
    assert prefix == "data:image/jpeg;base64"
    bgr = cv2.imdecode(np.frombuffer(base64.b64decode(payload), dtype=np.uint8), cv2.IMREAD_COLOR)
    np.testing.assert_allclose(bgr[4, 4], [15, 30, 220], atol=3)


def test_pose_summary_retains_nonzero_position_and_angle() -> None:
    message = PoseStamped(
        header=Header(stamp=Time(sec=10, nanosec=250_000_000), frame_id="world"),
        pose=Pose(position=Point(1.25, -2.5, 0.75), orientation=Quaternion(0.0, 0.0, 1.0, 0.0)),
    )
    assert content_blocks(message) == [
        {
            "type": "text",
            "text": "PoseStamped(pos=[1.250, -2.500, 0.750], euler=[0.0, 0.0, 180.0])",
        }
    ]
    assert content_blocks(object()) is None
