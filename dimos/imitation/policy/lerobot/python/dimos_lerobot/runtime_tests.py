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

from typing import Any

from dimos_lerobot.runtime import (
    LeRobotPolicyRuntime,
    _checkpoint_action_bounds,
    _validate_features,
)
import pytest
import torch

from dimos.experimental.isolated_python.bootstrap import validate_runtime
from dimos.imitation.policy.lerobot.module import OpenYamLeRobotPolicy
from dimos.robot.manipulators.openyam.learning import OPENYAM_QUEST_IO


class FakeFeature:
    def __init__(self, shape: tuple[int, ...]) -> None:
        self.shape = shape


class FakeConfig:
    temporal_ensemble_coeff = None
    input_features = {
        "observation.images.wrist": FakeFeature((3, 480, 640)),
        "observation.state": FakeFeature((7,)),
    }
    output_features = {"action": FakeFeature((7,))}


class FakeStats:
    def state_dict(self) -> dict[str, torch.Tensor]:
        return {
            "action.min": torch.zeros(7),
            "action.max": torch.ones(7),
        }


class FakePipeline:
    steps: list[Any] = [FakeStats()]


def test_generated_runtime_implements_the_host_contract() -> None:
    validate_runtime(OpenYamLeRobotPolicy, LeRobotPolicyRuntime)


def test_lerobot_feature_validation_uses_profile_keys_and_shapes() -> None:
    _validate_features(FakeConfig(), OPENYAM_QUEST_IO)  # type: ignore[arg-type]


def test_lerobot_feature_validation_rejects_missing_profile_key() -> None:
    config = FakeConfig()
    config.input_features = {"observation.state": FakeFeature((7,))}

    with pytest.raises(ValueError, match="observation.images.wrist"):
        _validate_features(config, OPENYAM_QUEST_IO)  # type: ignore[arg-type]


def test_lerobot_action_bounds_are_extracted_for_common_safety_loop() -> None:
    lower, upper = _checkpoint_action_bounds(FakePipeline(), 7)

    assert lower.tolist() == [0.0] * 7
    assert upper.tolist() == [1.0] * 7
