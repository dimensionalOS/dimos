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

"""Record generated image and calibration messages through the Rerun bridge."""

from pathlib import Path
from types import SimpleNamespace

from dimos_generated.sensor_msgs.msg import CameraInfo, CompressedImage
from dimos_generated.std_msgs.msg import Header
import numpy as np
import rerun as rr

from dimos.msgs.image import image_from_array, image_to_jpeg
from dimos.visualization.rerun.bridge import RerunBridgeModule


def main() -> None:
    output = Path("build/message-codegen/demo/evidence/camera-bridge.rrd")
    output.parent.mkdir(parents=True, exist_ok=True)
    rr.init("generated-camera-bridge", spawn=False)
    rr.save(str(output))
    bridge = RerunBridgeModule()
    bridge._min_intervals = {}
    bridge._camera_infos = {}
    bridge._image_entities = set()
    header = Header(frame_id="camera_optical")
    pixels = np.zeros((120, 160, 3), dtype=np.uint8)
    pixels[:, :80, 0] = 255
    pixels[:, 80:, 1] = 255
    color = image_from_array(pixels, encoding="rgb8", header=header)
    info = CameraInfo(header=header, width=160, height=120, k=[100, 0, 80, 0, 100, 60, 0, 0, 1])
    messages = [
        ("color", color),
        ("camera_info", info),
        ("jpeg", CompressedImage(header=header, format="jpeg", data=image_to_jpeg(color))),
        (
            "depth_mm",
            image_from_array(
                np.full((120, 160), 1000, dtype=np.uint16), encoding="16UC1", header=header
            ),
        ),
        (
            "depth_m",
            image_from_array(
                np.ones((120, 160), dtype=np.float32), encoding="32FC1", header=header
            ),
        ),
    ]
    try:
        for name, value in messages:
            bridge._on_message(
                type(value).decode(value.encode()), SimpleNamespace(name="/camera/" + name)
            )
            print(f"CDR {type(value).__name__} → world/camera/{name}")
    finally:
        bridge.stop()
        rr.disconnect()
    assert output.stat().st_size > 0
    print(f"Recording: {output}; both depth images represent one metre")


if __name__ == "__main__":
    main()
