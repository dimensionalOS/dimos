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

"""Show changing generated images and poses crossing the optional ROS2 bridge."""

import threading
import uuid

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped
from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header

from dimos.msgs.protocol import DimosMsg
from dimos.protocol.pubsub.impl.rospubsub import DimosROS, ROSTopic


def main() -> None:
    prefix = f"/dimos_cdr_demo_{uuid.uuid4().hex[:8]}"
    topics = [ROSTopic(prefix + "/pose", PoseStamped), ROSTopic(prefix + "/image", Image)]
    publisher, subscriber = DimosROS(), DimosROS()
    done = threading.Event()
    received: dict[tuple[str, int], DimosMsg] = {}
    expected: dict[tuple[str, int], DimosMsg] = {}
    count = 0
    try:
        publisher.start()
        subscriber.start()

        def collect(message: DimosMsg, topic: ROSTopic) -> None:
            assert isinstance(message, (PoseStamped, Image))
            stamp = message.header.stamp
            key = (topic.topic, stamp.nanosec)
            received[key] = message
            print(f"ROS received {message.msg_name}: source={stamp.sec}.{stamp.nanosec:09d}")
            if len(received) == 6:
                done.set()

        for topic in topics:
            subscriber.subscribe(topic, collect)
        node = publisher._raw._node
        assert node is not None

        def publish_next() -> None:
            nonlocal count
            if count == 3 or any(node.count_subscribers(topic.topic) == 0 for topic in topics):
                return
            header = Header(stamp=Time(sec=1700000000, nanosec=123456789 + count), frame_id="map")
            messages: list[DimosMsg] = [
                PoseStamped(header=header, pose=Pose(position=Point(x=float(count)))),
                Image(
                    header=header,
                    height=1,
                    width=1,
                    encoding="rgb8",
                    step=3,
                    data=bytes([count, 128, 255 - count]),
                ),
            ]
            for topic, message in zip(topics, messages, strict=True):
                expected[(topic.topic, header.stamp.nanosec)] = message
                publisher.publish(topic, message)
            count += 1

        timer = node.create_timer(0.1, publish_next)
        try:
            assert done.wait(15), "ROS discovery or message delivery timed out"
            assert received == expected
            for index in range(3):
                pose = received[(topics[0].topic, 123456789 + index)]
                image = received[(topics[1].topic, 123456789 + index)]
                assert isinstance(pose, PoseStamped) and isinstance(image, Image)
                print(f"frame {index}: x={pose.pose.position.x}, RGB={list(image.data)}")
        finally:
            node.destroy_timer(timer)
    finally:
        subscriber.stop()
        publisher.stop()
    print("PASS: three changing image/pose pairs crossed ROS2 with exact fields and nanoseconds")


if __name__ == "__main__":
    main()
