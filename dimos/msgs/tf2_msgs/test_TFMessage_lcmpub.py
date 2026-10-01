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

from collections.abc import Iterator
import time

from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
import pytest

from dimos.msgs.time import time_from_seconds
from dimos.protocol.pubsub.impl.lcmpubsub import LCM, Topic
from dimos.utils.testing.collector import CallbackCollector


@pytest.fixture
def lcm(lcm_url: str) -> Iterator[LCM]:
    lcm = LCM(url=lcm_url)
    lcm.start()
    try:
        yield lcm
    finally:
        lcm.stop()


# Publishes a series of transforms representing a robot kinematic chain
# to actual LCM messages, rerun running in parallel should render this
def test_publish_transforms(lcm: LCM) -> None:
    topic = Topic(topic="/tf", msg_type=TFMessage)
    collector = CallbackCollector(2)
    lcm.subscribe(topic, collector)

    # Create a robot kinematic chain using our new types
    current_time = time.time()

    # 1. World to base_link transform (robot at position)
    world_to_base = TransformStamped(
        transform=Transform(
            translation=Vector3(x=4.0, y=3.0, z=0.0),
            rotation=Quaternion(z=0.382683, w=0.923880),  # 45 degrees around Z
        ),
        header=Header(frame_id="world", stamp=time_from_seconds(current_time)),
        child_frame_id="base_link",
    )

    # 2. Base to arm transform (arm lifted up)
    base_to_arm = TransformStamped(
        transform=Transform(
            translation=Vector3(x=0.2, y=0.0, z=1.5),
            rotation=Quaternion(y=0.258819, w=0.965926),  # 30 degrees around Y
        ),
        header=Header(frame_id="base_link", stamp=time_from_seconds(current_time)),
        child_frame_id="arm_link",
    )

    # 3. Arm to gripper transform (gripper extended)
    arm_to_gripper = TransformStamped(
        transform=Transform(
            translation=Vector3(x=0.5, y=0.0, z=0.0),
            rotation=Quaternion(w=1.0),  # No rotation
        ),
        header=Header(frame_id="arm_link", stamp=time_from_seconds(current_time)),
        child_frame_id="gripper_link",
    )

    lcm.publish(topic, TFMessage(transforms=[world_to_base, base_to_arm]))
    lcm.publish(topic, TFMessage(transforms=[world_to_base, arm_to_gripper]))
    collector.wait()

    assert len(collector.results) == 2

    first, _ = collector.results[0]
    assert isinstance(first, TFMessage)
    assert [t.child_frame_id for t in first.transforms] == ["base_link", "arm_link"]

    second, _ = collector.results[1]
    assert isinstance(second, TFMessage)
    assert [t.child_frame_id for t in second.transforms] == ["base_link", "gripper_link"]
