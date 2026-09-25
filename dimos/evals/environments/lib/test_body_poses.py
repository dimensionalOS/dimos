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

import pytest

from dimos.evals.environments.lib.body_poses import first_body_transform, last_body_transform
from dimos.memory.store.memory import MemoryStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


def _tf(ts: float, child: str, z: float) -> TFMessage:
    return TFMessage(
        Transform(translation=Vector3(0.4, 0.08, z), frame_id="world", child_frame_id=child, ts=ts)
    )


def test_body_transforms_come_from_the_newest_tf_naming_them():
    with MemoryStore() as store:
        with pytest.raises(LookupError):
            last_body_transform(store, "apple")
        tf = store.stream("tf", TFMessage)
        tf.append(_tf(1.0, "apple", 0.17))
        tf.append(_tf(2.0, "apple", 0.30))
        tf.append(_tf(3.0, "wrist_camera_link", 0.0))  # another publisher's message
        assert first_body_transform(store, "apple").translation.z == pytest.approx(0.17)
        assert last_body_transform(store, "apple").translation.z == pytest.approx(0.30)
        with pytest.raises(LookupError):
            last_body_transform(store, "orange")
