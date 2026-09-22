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

"""Exercise generated image preparation with a fixed model response, offline."""

from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import Detection2DArray
import numpy as np

from dimos.models.vl.base import VlModel
from dimos.msgs.image import image_from_array


class FixedResponseModel(VlModel):
    def query(self, image: Image, query: str, **kwargs) -> str:
        return '[["synthetic target", 2, 4, 10, 12]]'

    def stop(self) -> None:
        pass


def main() -> None:
    image = image_from_array(
        np.zeros((40, 60, 3), dtype=np.uint8), encoding="rgb8", header=Header(frame_id="camera")
    )
    model = FixedResponseModel(auto_resize=(30, 20))
    resized, scale = model._prepare_image(image)
    assert (resized.width, resized.height, scale) == (30, 20, 0.5)
    detections = model.query_detections(image, "target")
    wire = Detection2DArray.decode(detections.to_ros_detection2d_array().encode())
    assert wire.header == image.header and len(wire.detections) == 1
    print("Generated image: 60x40 → 30x20, scale=0.5")
    print("Fixed model response → one generated CDR detection; source header preserved")
    print("PASS: offline image preparation and response conversion (no model inference)")


if __name__ == "__main__":
    main()
