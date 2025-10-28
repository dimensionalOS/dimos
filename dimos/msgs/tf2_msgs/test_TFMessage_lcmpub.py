# Copyright 2025 Dimensional Inc.
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

import time

import pytest

from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.tf2_msgs import TFMessage
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic


# Publishes a series of transforms representing a robot kinematic chain
# to actual LCM messages, foxglove running in parallel should render this
@pytest.mark.skip
def test_publish_transforms() -> None:
    from dimos_lcm.tf2_msgs import TFMessage as LCMTFMessage

    lcm = LCM(autoconf=True)
    lcm.start()

    topic = Topic(topic="/tf", lcm_type=LCMTFMessage)

    # Create a robot kinematic chain using our new types
    current_time = time.time()

    # 1. World to base_link transform (robot at position)
    world_to_base = Transform(
        translation=Vector3(4.0, 3.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.382683, 0.923880),  # 45 degrees around Z
        frame_id="world",
        child_frame_id="base_link",
        ts=current_time,
    )

    # 2. Base to arm transform (arm lifted up)
    base_to_arm = Transform(
        translation=Vector3(0.2, 0.0, 1.5),
        rotation=Quaternion(0.0, 0.258819, 0.0, 0.965926),  # 30 degrees around Y
        frame_id="base_link",
        child_frame_id="arm_link",
        ts=current_time,
    )

    lcm.publish(topic, TFMessage(world_to_base, base_to_arm))

    time.sleep(0.05)
    # 3. Arm to gripper transform (gripper extended)
    arm_to_gripper = Transform(
        translation=Vector3(0.5, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),  # No rotation
        frame_id="arm_link",
        child_frame_id="gripper_link",
        ts=current_time,
    )

    lcm.publish(topic, TFMessage(world_to_base, arm_to_gripper))
