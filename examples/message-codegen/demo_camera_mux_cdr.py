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

"""Inspect generated camera compositing and JPEG with mocked module ports."""

from unittest.mock import MagicMock, patch

from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.core.module import Module
from dimos.msgs.image import image_from_array, image_to_jpeg, image_view
from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds
from dimos.teleop.hosted.camera_mux import CameraMuxConfig, CameraMuxModule


def main() -> None:
    with (
        patch.object(Module, "__init__", return_value=None),
        patch.object(
            CameraMuxModule, "config", CameraMuxConfig(cameras=["cam1", "cam2"]), create=True
        ),
    ):
        mux = CameraMuxModule()
    mux.config = CameraMuxConfig(cameras=["cam1", "cam2"], video_max_width=100)
    mux.mux_image = MagicMock()
    mux._cam_selected = ["cam1", "cam2"]
    stamp_ns = 1_700_000_000_123_456_789
    for index, camera in enumerate(("cam1", "cam2")):
        pixels = np.zeros((61, 81, 3), dtype=np.uint8)
        pixels[:, :, index] = 255
        image = image_from_array(
            pixels,
            encoding="rgb8",
            header=Header(stamp=time_from_nanoseconds(stamp_ns + index), frame_id=camera),
        )
        mux._on_cam(camera, Image.decode(image.encode()))
    output = Image.decode(mux.mux_image.publish.call_args.args[0].encode())
    assert output.width == 100 and output.height == 36
    assert to_nanoseconds(output.header.stamp) == stamp_ns + 1
    assert output.header.frame_id == "camera_mux"
    assert image_view(output)[0, 0].tolist() == [255, 0, 0]
    assert image_view(output)[0, -1].tolist() == [0, 255, 0]
    jpeg = image_to_jpeg(output)
    assert jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9")
    print(f"CDR camera mux: {output.width}x{output.height}, {len(jpeg)} JPEG bytes")
    print(f"PASS: RGB order, exact {stamp_ns + 1}ns source stamp; mocked ports only")


if __name__ == "__main__":
    main()
