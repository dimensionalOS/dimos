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
from types import SimpleNamespace

from dimos_generated.sensor_msgs.msg import CameraInfo
from dimos_generated.tf2_msgs.msg import TFMessage

from dimos.robot.deeprobotics.m20.camera import M20CameraRelay


class _OneIteration(Event):
    def wait(self, timeout=None):
        self.set()
        return True


def test_camera_metadata_uses_generated_wire_values_without_connecting_streams():
    transforms, front, rear = [], [], []
    proxy = SimpleNamespace(
        _stop_event=_OneIteration(),
        tf=SimpleNamespace(publish=transforms.append),
        front_camera_info=SimpleNamespace(publish=front.append),
        rear_camera_info=SimpleNamespace(publish=rear.append),
    )
    M20CameraRelay._publish_camera_metadata(proxy)
    tf = TFMessage.decode(transforms[0].encode())
    assert len(tf.transforms) == 4
    assert [t.child_frame_id for t in tf.transforms] == [
        "front_camera_link",
        "front_camera_optical",
        "rear_camera_link",
        "rear_camera_optical",
    ]
    first, second = CameraInfo.decode(front[0].encode()), CameraInfo.decode(rear[0].encode())
    assert first.header.frame_id == "front_camera_optical"
    assert second.header.frame_id == "rear_camera_optical"
    assert first.header.stamp == second.header.stamp == tf.transforms[0].header.stamp
    assert first.width == second.width == 800
    assert first.height == second.height == 600
