"""Tests for public LIBERO observation conversion."""

import numpy as np

from dimos.benchmark.libero_pro.connection import _decode_image
from dimos.benchmark.libero_pro.proto import libero_pro_pb2 as pb2
from dimos.msgs.sensor_msgs.Image import ImageFormat


def test_camera_messages_keep_the_shared_simulator_timestamp() -> None:
    pixels = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    frame = pb2.ImageFrame(camera="agentview", width=2, height=2, rgb=pixels.tobytes())

    image = _decode_image(frame, 123.5)

    assert image.ts == 123.5
    assert image.frame_id == "agentview"
    assert image.format is ImageFormat.RGB
    assert np.array_equal(image.data, pixels)
