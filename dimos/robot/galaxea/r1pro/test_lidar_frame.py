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


def test_the_lidar_stream_is_restamped_with_the_configured_frame():
    """The vendor stamps clouds `livox_frame`, which no tf edge reaches."""
    import queue
    import threading
    from unittest.mock import patch

    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.robot.galaxea.r1pro.connection import convert_loop

    stop = threading.Event()
    published: list[PointCloud2] = []

    class _Out:
        def publish(self, msg):
            published.append(msg)
            stop.set()

    incoming: queue.Queue = queue.Queue()
    incoming.put(object())

    cloud = PointCloud2(frame_id="livox_frame")
    with patch(
        "dimos.protocol.pubsub.impl.rospubsub_conversion.ros_to_dimos",
        return_value=cloud,
    ):
        convert_loop(
            stream="lidar",
            queue_in=incoming,
            dimos_type=PointCloud2,
            out=_Out(),
            stop=stop,
            record_decode=lambda *a, **k: None,
            frame_id="lidar_chassis_left_link",
        )

    assert len(published) == 1
    assert published[0].frame_id == "lidar_chassis_left_link"
