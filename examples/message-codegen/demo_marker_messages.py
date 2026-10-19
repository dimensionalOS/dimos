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

"""Show generated CDR marker identity and exact image-derived timestamps."""

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Vector3
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import Detection3DArray
import numpy as np

from dimos.msgs.image import image_from_array
from dimos.perception.detection.type.detection3d.imageDetections3D import ImageDetections3D
from dimos.perception.detection.type.detection3d.marker import Detection3DMarker


def main() -> None:
    image = image_from_array(
        np.zeros((80, 100, 3), dtype=np.uint8),
        encoding="bgr8",
        header=Header(frame_id="camera", stamp=Time(sec=1700000000, nanosec=123456789)),
    )
    markers = [
        Detection3DMarker(
            bbox=(10, 10, 30, 30),
            track_id=-1,
            class_id=identifier,
            confidence=1,
            name="",
            ts=0,
            image=image,
            center=Vector3(x=float(identifier), y=2, z=3),
            size=Vector3(x=0.16, y=0.16),
            frame_id="world",
            marker_id=identifier,
            dictionary="DICT_APRILTAG_36h11",
        )
        for identifier in (7, 42)
    ]
    message = ImageDetections3D(image, markers).to_ros_detection3d_array()
    decoded = Detection3DArray.decode(message.encode())
    assert decoded.header.stamp == image.header.stamp
    for marker in decoded.detections:
        assert marker.results[0].hypothesis.class_id == f"DICT_APRILTAG_36h11:{marker.id}"
        print(
            f"Marker {marker.id}: {marker.results[0].hypothesis.class_id}, world x={marker.bbox.center.position.x:.1f} m"
        )
    print("PASS: generated marker array retains identity and 1700000000123456789 ns stamp")


if __name__ == "__main__":
    main()
