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

import numpy as np

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.point import Detection2DPoint


def test_name_and_score_survive_the_wire() -> None:
    image = Image(data=np.zeros((48, 64, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=12.5)
    point = Detection2DPoint(x=20.0, y=30.0, name="door handle", ts=12.5, image=image)
    message = ImageDetections2D(image=image, detections=[point]).to_ros_detection2d_array()

    decoded = Detection2DArray.lcm_decode(message.lcm_encode())

    assert decoded.to_json() == [
        {"label": "door handle", "score": 1.0, "bbox": {"cx": 20.0, "cy": 30.0, "w": 0.0, "h": 0.0}}
    ]
