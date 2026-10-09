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

"""Offline standalone Habitat encoder check; run with its installed message package."""

import importlib.util
import json
from pathlib import Path
import sys

from dimos_generated.geometry_msgs.msg import Twist, Vector3
from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.sensor_msgs.msg import CameraInfo, Image, PointCloud2
from dimos_generated.tf2_msgs.msg import TFMessage
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np

root = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "habitat_standalone", root / "dimos/simulation/habitat/server.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
output = root / "build/message-codegen/evidence/habitat-py39"
output.mkdir(parents=True, exist_ok=True)
payloads = {
    "image": (
        Image,
        module.image_msg(np.full((2, 3, 3), 7, dtype=np.uint8), "rgb8", "camera", 12.25),
    ),
    "camera_info": (
        CameraInfo,
        module.camera_info_msg(
            {"width": 3, "height": 2, "fx": 10, "fy": 11, "cx": 1, "cy": 1}, "camera", 12.25
        ),
    ),
    "cloud": (
        PointCloud2,
        module.cloud_msg(
            np.array([[1, 2, 3]], dtype=np.float32),
            np.array([[255, 128, 64]], dtype=np.uint8),
            "world",
            12.25,
        ),
    ),
    "odometry": (
        Odometry,
        module.odometry_msg(
            np.array([1, 2, 3]),
            np.array([0, 0, 0, 1]),
            (0.4, 0.2, 0.3),
            "world",
            "base_link",
            12.25,
        ),
    ),
    "tf": (
        TFMessage,
        module.tf_msg(
            [
                ("world", "base_link", (1, 2, 3), (0, 0, 0, 1)),
                ("base_link", "camera", (0, 0, 0.45), (0, 0, 0, 1)),
            ],
            12.25,
        ),
    ),
}
for name, (message_type, payload) in payloads.items():
    value = message_type.decode(payload)
    header = value.transforms[0].header if name == "tf" else value.header
    assert header.stamp.sec == 12 and header.stamp.nanosec == 250000000
    (output / (name + ".cdr")).write_bytes(payload)
assert cdr_decode(payloads["odometry"][1], Odometry).pose.pose.position.x == 1
assert cdr_decode(payloads["cloud"][1], PointCloud2).row_step == 16
assert cdr_decode(payloads["tf"][1], TFMessage).transforms[1].transform.translation.z == 0.45
assert (
    cdr_decode(
        cdr_encode(
            Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        ),
        Twist,
    ).linear.x
    == 0
)
print(
    json.dumps(
        {
            "python": sys.version.split()[0],
            "standalone_import": True,
            "cdr_payloads": sorted(payloads),
            "source_stamp": [12, 250000000],
        }
    )
)
