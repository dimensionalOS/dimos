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

import json
import struct

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.teleop.spatial.rgbd import RGBDEncoder


@pytest.fixture
def observations():
    color = Image(np.full((3, 4, 3), 120, dtype=np.uint8), ImageFormat.RGB, "optical", 100.0)
    depth = Image(np.ones((3, 4), dtype=np.float32), ImageFormat.DEPTH, "optical", 100.0)
    info = CameraInfo.from_intrinsics(100, 100, 1, 1, 4, 3, frame_id="optical")
    mount = Transform(translation=Vector3(1, 2, 3), frame_id="world", child_frame_id="optical")
    return color, depth, info, mount


def unpack(packet):
    length = struct.unpack_from("<I", packet)[0]
    header = json.loads(packet[4 : 4 + length])
    count = header["width"] * header["height"]
    depth = np.frombuffer(packet, dtype="<u2", count=count, offset=4 + length)
    return header, depth, packet[4 + length + count * 2 :]


def test_float_metres_and_physical_uint16_depth_produce_same_observations(observations):
    color, depth, info, mount = observations
    depth.data[0] = [0, float("nan"), float("inf"), 3]
    encoder = RGBDEncoder(depth_unit_m=0.0005)
    simulated = encoder.encode(color, depth, info, mount)
    physical = Image(
        np.array([[0, 0, 0, 6000], [2000] * 4, [2000] * 4], dtype=np.uint16),
        ImageFormat.DEPTH16,
        "optical",
        100.0,
    )
    measured = encoder.encode(color, physical, info, mount)
    assert simulated.payload == measured.payload
    np.testing.assert_allclose(simulated.points[0], [0.99, 2, 4])
    header, depth_mm, jpeg = unpack(measured.payload)
    assert header["ts"] == 100.0
    np.testing.assert_array_equal(depth_mm, [0, 0, 0, 0] + [1000] * 8)
    assert jpeg[:2] == b"\xff\xd8"


def test_payload_budget_preserves_grid_and_original_rgb_resolution(observations):
    packet = RGBDEncoder(max_pixels=4).encode(*observations)
    header, depth, jpeg = unpack(packet.payload)
    assert (header["width"], header["height"], header["step"]) == (2, 2, 2)
    assert (header["image_width"], header["image_height"]) == (4, 3)
    assert len(depth) == 4


def test_misaligned_or_late_frames_are_rejected(observations):
    color, depth, info, mount = observations
    depth.ts += 0.1
    with pytest.raises(ValueError, match="timestamps"):
        RGBDEncoder().encode(color, depth, info, mount)
    depth.ts = color.ts
    depth.frame_id = "different_imager"
    with pytest.raises(ValueError, match="registered optical"):
        RGBDEncoder().encode(color, depth, info, mount)


def test_rectification_keeps_depth_holes_and_rejects_unknown_distortion(observations):
    color, depth, info, mount = observations
    depth.data[1, 1] = 0
    info.D = [0.001, 0, 0, 0, 0]
    info.distortion_model = "plumb_bob"
    _, depth_mm, _ = unpack(RGBDEncoder().encode(color, depth, info, mount).payload)
    assert depth_mm.reshape(3, 4)[1, 1] == 0
    info.distortion_model = "unknown"
    with pytest.raises(ValueError, match="calibration"):
        RGBDEncoder().encode(color, depth, info, mount)


def test_wrist_depth_range_and_empty_frame_clear_previous_surface(observations):
    color, depth, info, mount = observations
    depth.data[:] = [0.06, 0.1, 0.5, 0.51]
    encoder = RGBDEncoder(min_depth_m=0.07, max_depth_m=0.5)
    _, depth_mm, _ = unpack(encoder.encode(color, depth, info, mount).payload)
    np.testing.assert_array_equal(depth_mm, [0, 100, 500, 0] * 3)
    depth.data[:] = 1
    empty = encoder.encode(color, depth, info, mount)
    _, depth_mm, jpeg = unpack(empty.payload)
    assert not depth_mm.any()
    assert len(empty.points) == 0
    assert jpeg[:2] == b"\xff\xd8"
