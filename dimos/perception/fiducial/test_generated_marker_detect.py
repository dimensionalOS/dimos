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

import cv2
from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.sensor_msgs.msg import CameraInfo, Image
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import Detection3D
import numpy as np
import pytest

from dimos.msgs.image import image_from_array
from dimos.perception.fiducial.marker_detect import detect_markers_in_image


def test_generated_image_detects_real_aruco_and_composes_pose():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    pixels = np.full((400, 400), 255, dtype=np.uint8)
    pixels[100:300, 100:300] = cv2.aruco.generateImageMarker(dictionary, 7, 200)
    header = Header(frame_id="camera", stamp=Time(sec=1700000000, nanosec=123456789))
    image = image_from_array(pixels, encoding="mono8", header=header)
    image = Image.decode(image.encode())
    camera = CameraInfo(width=400, height=400, k=[400, 0, 200, 0, 400, 200, 0, 0, 1])
    tf = TransformStamped(
        header=Header(frame_id="world"),
        child_frame_id="camera",
        transform=Transform(translation=Vector3(x=2), rotation=Quaternion(w=1)),
    )
    found = detect_markers_in_image(
        image,
        camera_info=camera,
        world_T_optical=tf,
        marker_length_m=0.2,
        aruco_dictionary="DICT_4X4_50",
    )
    assert len(found) == 1
    wire = Detection3D.decode(found[0].to_detection3d_msg().encode())
    assert wire.id == "7"
    assert wire.results[0].hypothesis.class_id == "DICT_4X4_50:7"
    assert wire.header.stamp == header.stamp
    assert wire.header.frame_id == "world"
    assert wire.bbox.center.position.x == pytest.approx(2, abs=0.01)
    assert wire.bbox.center.position.z == pytest.approx(0.4, abs=0.01)
