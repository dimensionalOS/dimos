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

from types import SimpleNamespace
from unittest.mock import patch

from dimos_generated.sensor_msgs.msg import CameraInfo, CompressedImage, Image
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest
import rerun as rr

from dimos.msgs.image import image_from_array, image_to_jpeg
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.message_helpers import image_archetype

pytestmark = pytest.mark.filterwarnings("error::rerun.error_utils.RerunWarning")


@pytest.mark.parametrize("camera_first", [True, False])
@pytest.mark.parametrize("compressed", [True, False])
def test_generated_calibration_pairs_with_image(camera_first: bool, compressed: bool) -> None:
    bridge = RerunBridgeModule()
    bridge._min_intervals = {}
    bridge._camera_infos = {}
    bridge._image_entities = set()
    header = Header(frame_id="optical")
    image = image_from_array(np.zeros((4, 6, 3), dtype=np.uint8), encoding="rgb8", header=header)
    message = (
        CompressedImage(header=header, format="jpeg", data=image_to_jpeg(image))
        if compressed
        else image
    )
    message = type(message).decode(message.encode())
    info = CameraInfo(header=header, width=6, height=4, k=[10, 0, 3, 0, 11, 2, 0, 0, 1])
    pairs = [(CameraInfo.decode(info.encode()), "/camera_info"), (message, "/image")]
    if not camera_first:
        pairs.reverse()
    try:
        with patch("rerun.log") as log:
            for value, topic in pairs:
                bridge._on_message(value, SimpleNamespace(name=topic))
            pinholes = [call for call in log.call_args_list if isinstance(call.args[1], rr.Pinhole)]
            assert len(pinholes) == 1
            assert pinholes[0].args[0] == "world/image"
            assert pinholes[0].args[1].parent_frame.as_arrow_array().to_pylist() == ["tf#/optical"]
            assert any(isinstance(call.args[1], rr.EncodedImage) for call in log.call_args_list)
    finally:
        bridge.stop()


@pytest.mark.parametrize(
    "encoding,dtype,meter", [("16UC1", np.uint16, 1000), ("32FC1", np.float32, 1)]
)
def test_generated_depth_units(encoding, dtype, meter) -> None:
    message = image_from_array(np.ones((2, 3), dtype=dtype), encoding=encoding)
    result = image_archetype(Image.decode(message.encode()))
    assert isinstance(result, rr.DepthImage)
    assert result.meter.as_arrow_array().to_pylist() == [meter]
