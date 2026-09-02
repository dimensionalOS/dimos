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

from unittest.mock import Mock

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_rust_replay import (
    Go2ReplaySupport,
    Go2RustReplayer,
    unitree_go2_rust_replay,
)


def test_go2_support_derives_the_existing_tf_chain() -> None:
    support = Go2ReplaySupport()
    support.tf.publish = Mock()  # type: ignore[method-assign]
    odom = PoseStamped(ts=12.5, frame_id="odom")

    support._publish_tf(odom)

    message = support.tf.publish.call_args.args[0]
    assert [transform.child_frame_id for transform in message.transforms] == [
        "base_link",
        "camera_link",
        "camera_optical",
    ]
    assert all(transform.ts == 12.5 for transform in message.transforms)
    support.stop()


def test_named_blueprint_contains_native_source_and_support() -> None:
    module_types = {atom.module for atom in unitree_go2_rust_replay.blueprints}

    assert Go2RustReplayer in module_types
    assert Go2ReplaySupport in module_types
