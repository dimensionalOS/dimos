# Copyright 2025 Dimensional Inc.
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

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection2d.types import Detection2D, better_detection_format
from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector
from dimos.utils.data import get_data


def test_detection2dtype():
    detector = Yolo2DDetector()

    image = Image.from_file(get_data("cafe.jpg"))
    raw_detections = detector.process_image(image.to_opencv())

    for det in better_detection_format(raw_detections):
        print(Detection2D.from_detection(det, image=image))
