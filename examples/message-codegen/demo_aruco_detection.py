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

from pathlib import Path

import cv2
from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.sensor_msgs.msg import CameraInfo, Image
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import Detection3D, Detection3DArray
import numpy as np

from dimos.memory.type.observation import Observation
from dimos.msgs.image import image_from_array, image_view
from dimos.perception.fiducial.marker_detect import detect_markers_in_image
from dimos.perception.fiducial.marker_transformer import DetectMarkers, MarkersPerFrame


def main() -> None:
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
    assert abs(wire.bbox.center.position.x - 2) < 0.01
    assert abs(wire.bbox.center.position.z - 0.4) < 0.01
    observation = Observation(
        id=1, ts=1700000000.1234567, data_type=Image, _data=image, pose=(2, 0, 0, 0, 0, 0, 1)
    )
    detector = DetectMarkers(
        camera_info=camera,
        marker_length_m=0.2,
        aruco_dictionary="DICT_4X4_50",
        emit_empty_frames=True,
    )
    arrays = list(MarkersPerFrame()(detector(iter([observation]))))
    decoded_array = Detection3DArray.decode(arrays[0].data.encode())
    assert len(decoded_array.detections) == 1
    assert decoded_array.header.stamp == image.header.stamp
    print("Memory stream: image → marker observation → generated CDR array")
    output = Path("build/message-codegen/demo/evidence/aruco-detection.png")
    output.parent.mkdir(parents=True, exist_ok=True)
    assert cv2.imwrite(str(output), image_view(found[0].annotated_image()))
    print(
        f"Detected {wire.results[0].hypothesis.class_id}: world x={wire.bbox.center.position.x:.3f}, z={wire.bbox.center.position.z:.3f} m"
    )
    print(f"Annotated image: {output}")
    print("PASS: generated image → ArUco pose → world transform → CDR marker")


if __name__ == "__main__":
    main()
