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

"""A goal vector stays distinct from joint measurements throughout the profile."""

import numpy as np
import pytest

from dimos.imitation.dataprep.core import OutputConfig, resolve_field
from dimos.imitation.observation import VectorObservation
from dimos.imitation.policy.runtime import _read_source
from dimos.imitation.profile import PolicyIOProfile, VectorSource
from dimos.robot.galaxea.r1pro.learning import R1PRO_PACKING_IO


def test_packing_goal_roundtrips_and_matches_recorded_feature_extraction(tmp_path):
    profile = PolicyIOProfile.model_validate_json(R1PRO_PACKING_IO.model_dump_json())
    source = profile.observations["observation.environment_state"]
    assert isinstance(source, VectorSource)
    message = VectorObservation(12.0, (0.4, -0.3, 0.77, 0.3, -0.04, 0.785, 0.025, 0.07))
    actual, timestamp = _read_source(source, message)
    config = profile.dataprep_config(output=OutputConfig(path=tmp_path / "data"))
    recorded = resolve_field(message, config.observation["observation.environment_state"])
    np.testing.assert_allclose(actual, message.values)
    np.testing.assert_allclose(recorded, actual)
    assert timestamp == 12.0
    assert profile.action_state_key == "observation.state"


@pytest.mark.parametrize("values", [(1.0,), (1.0, float("nan")), (1.0, float("inf"))])
def test_goal_vectors_reject_missing_or_nonfinite_features(values):
    source = VectorSource(stream="goal", features=("x", "y"))
    with pytest.raises(ValueError, match="profile.*finite"):
        _read_source(source, VectorObservation(1.0, values))
