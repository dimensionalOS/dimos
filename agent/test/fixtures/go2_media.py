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

"""Standard Go2 replay + cockpit; the test codec preserves every XYZ float."""

import hashlib

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.web.cockpit import Channel, cockpit
from dimos.web.codecs import EncodedPayload, web_encoder
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


@web_encoder("dimcode-test.xyz.v1")
def encode_xyz(msg: PointCloud2) -> EncodedPayload:
    points = msg.points_f32().astype("<f4", copy=False)
    payload = points.tobytes()
    return EncodedPayload(
        payload,
        {
            "count": len(points),
            "frame": msg.frame_id,
            "capture_ts": msg.ts,
            "sha256": hashlib.sha256(payload).hexdigest(),
        },
    )


go2_media = autoconnect(
    unitree_go2.disabled_modules(WebsocketVisModule),
    cockpit(
        channels=[
            Channel("color_image", Image, encoding="jpeg.v1", delivery="latest", max_hz=10),
            Channel("lidar", PointCloud2, encoding="dimcode-test.xyz.v1", max_hz=5),
            Channel("odom", PoseStamped, encoding="pose.json.v1", max_hz=10),
        ]
    ),
)


def main() -> None:
    ModuleCoordinator.build(go2_media).loop()


if __name__ == "__main__":
    main()
