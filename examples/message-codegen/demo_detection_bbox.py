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

"""Render a generated-image detection and print its CDR metadata."""

from pathlib import Path

import cv2
from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import Detection2DArray
import numpy as np

from dimos.msgs.image import image_from_array, image_view
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


def main() -> None:
    pixels = np.full((240, 320, 3), 40, dtype=np.uint8)
    pixels[70:180, 100:220] = [180, 100, 40]
    source = image_from_array(
        pixels,
        encoding="rgb8",
        header=Header(frame_id="camera_optical", stamp=Time(sec=1700000000, nanosec=123456789)),
    )
    detection = Detection2DBBox(
        bbox=(100, 70, 220, 180),
        track_id=7,
        class_id=2,
        confidence=0.9,
        name="synthetic target",
        ts=1700000000,
        image=source,
    )
    collection = ImageDetections2D(image=source, detections=[detection])
    array = Detection2DArray.decode(collection.to_ros_detection2d_array().encode())
    wire = array.detections[0]
    assert array.header == source.header
    assert wire.header == source.header
    annotated = collection.annotated_image()
    output = Path("build/message-codegen/demo/evidence/detection-bbox.png")
    output.parent.mkdir(parents=True, exist_ok=True)
    assert cv2.imwrite(str(output), image_view(annotated))
    print(f"CDR detection: track={wire.id}, class={wire.results[0].hypothesis.class_id}")
    print(f"Image: {output}; source stamp=1700000000123456789 ns")
    print("PASS: generated image → annotated bbox and CDR Detection2DArray")


if __name__ == "__main__":
    main()
