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

from threading import Event
from uuid import uuid4

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import PoseStamped
from dimos_generated.sensor_msgs.msg import CameraInfo
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    Point2D,
    Pose2D,
)
import pytest

from dimos.core.transport import LCMTransport
from dimos.navigation.bbox_navigation import BBoxNavigationModule


def test_generated_bbox_goal_preserves_header_and_projection():
    module = BBoxNavigationModule(goal_distance=2)
    prefix = uuid4().hex[:8]
    module.camera_info.transport = LCMTransport(f"/bc/{prefix}", CameraInfo)
    module.detection2d.transport = LCMTransport(f"/bd/{prefix}", Detection2DArray)
    goals = []
    ready = Event()
    received = Event()
    unsubscribe = module.goal_request.subscribe(
        lambda value: (goals.append(PoseStamped.decode(value.encode())), received.set())
    )
    message = Detection2DArray(
        header=Header(frame_id="base_link", stamp=Time(sec=1700000000, nanosec=123456789)),
        detections=[Detection2D(bbox=BoundingBox2D(center=Pose2D(position=Point2D(x=420, y=290))))],
    )
    try:
        module.start()
        module._on_detection(message)
        assert not goals
        camera = CameraInfo(k=[500, 0, 320, 0, 250, 240, 0, 0, 1])
        camera_unsubscribe = module.camera_info.subscribe(lambda _: ready.set())
        module.camera_info.transport.publish(CameraInfo.decode(camera.encode()))
        assert ready.wait(2)
        camera_unsubscribe()
        module.detection2d.transport.publish(Detection2DArray.decode(message.encode()))
        assert received.wait(2)
        assert len(goals) == 1
        goal = goals[0]
        assert goal.header == message.header
        assert (goal.pose.position.x, goal.pose.position.y, goal.pose.position.z) == pytest.approx(
            (2, -0.4, -0.4)
        )
        module._on_detection(Detection2DArray())
        assert len(goals) == 1
    finally:
        unsubscribe()
        module.stop()
        module.camera_info.transport.stop()
        module.detection2d.transport.stop()
