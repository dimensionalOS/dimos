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

"""Print CDR velocity commands for synthetic camera detections."""

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Twist
from dimos_generated.sensor_msgs.msg import CameraInfo, RegionOfInterest
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np

from dimos.navigation.visual_servoing.visual_servoing_2d import VisualServoing2D


def main() -> None:
    camera = CameraInfo(
        width=640,
        height=480,
        k=np.array([500, 0, 320, 0, 500, 240, 0, 0, 1], dtype=np.float64),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        distortion_model="",
        d=np.array([], dtype=np.float64),
        r=np.zeros(9, dtype=np.float64),
        p=np.zeros(12, dtype=np.float64),
        binning_x=0,
        binning_y=0,
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
    )
    controller = VisualServoing2D(cdr_decode(cdr_encode(camera), CameraInfo))
    for label, bbox, expected in (
        ("far", (270, 0, 370, 100), (0.5, 0)),
        ("target distance", (245, 0, 395, 100), (0, 0)),
        ("too close", (120, 0, 520, 100), (-0.3, 0)),
        ("right of center", (370, 0, 470, 100), (0.36, -0.2)),
    ):
        command = controller.compute_twist(bbox, camera.width)
        decoded = cdr_decode(cdr_encode(command), Twist)
        assert abs(decoded.linear.x - expected[0]) < 1e-9
        assert abs(decoded.angular.z - expected[1]) < 1e-9
        print(f"{label}: linear={decoded.linear.x:.2f} m/s, angular={decoded.angular.z:.2f} rad/s")
    print("PASS: generated CameraInfo → visual servo → CDR Twist")


if __name__ == "__main__":
    main()
