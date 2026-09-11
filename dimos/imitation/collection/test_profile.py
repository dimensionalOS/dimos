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


import numpy as np
import pytest

from dimos.imitation.collection.profile import CollectionFeature, CollectionProfile
from dimos.imitation.dataprep.core import DataPrepConfig, OutputConfig, SyncConfig, resolve_field
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState


@pytest.fixture
def profile():
    return CollectionProfile(
        name="custom",
        robot_type="test",
        observations={
            "camera": CollectionFeature(
                stream="camera",
                message_type=Image,
                field="data",
                dtype="video",
                shape=(8, 8, 3),
                names=["height", "width", "channels"],
            ),
            "positions": CollectionFeature(
                stream="measured",
                message_type=JointState,
                field="position",
                dtype="float32",
                shape=(2,),
                names=["b", "a"],
            ),
            "velocities": CollectionFeature(
                stream="measured",
                message_type=JointState,
                field="velocity",
                dtype="float32",
                shape=(1,),
                names=["a"],
            ),
        },
        actions={
            "target": CollectionFeature(
                stream="commanded",
                message_type=JointState,
                field="position",
                dtype="float32",
                shape=(1,),
                names=["b"],
            ),
        },
        sync=SyncConfig(anchor="camera", rate_hz=30, tolerance_ms=20),
    )


def test_profile_lowers_to_existing_json_protocol_and_preserves_projections(profile, tmp_path):
    config = profile.dataprep_config(output=OutputConfig(path=tmp_path))
    restored = DataPrepConfig.model_validate_json(config.model_dump_json())
    assert profile.input_types() == {
        "camera": Image,
        "measured": JointState,
        "commanded": JointState,
    }
    assert restored.observation["positions"].names == ["b", "a"]
    assert restored.action["target"].names == ["b"]
    message = JointState(name=["a", "b"], position=[1.0, 2.0], velocity=[3.0, 4.0])
    np.testing.assert_array_equal(resolve_field(message, restored.observation["positions"]), [2, 1])
    np.testing.assert_array_equal(resolve_field(message, restored.observation["velocities"]), [3])


def test_profile_rejects_conflicting_raw_types(profile):
    values = profile.model_dump()
    values["actions"]["target"]["stream"] = "camera"
    with pytest.raises(ValueError, match="conflicting message types"):
        CollectionProfile(**values)


@pytest.mark.parametrize("anchor", ["absent", "target"])
def test_sync_anchor_must_be_an_observation(profile, anchor):
    values = profile.model_dump()
    values["sync"]["anchor"] = anchor
    with pytest.raises(ValueError, match="sync anchor"):
        CollectionProfile(**values)
