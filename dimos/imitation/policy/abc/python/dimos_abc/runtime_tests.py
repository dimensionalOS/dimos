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

from abc_minimal.config import DiTConfig
import numpy as np
import pytest

from dimos.experimental.isolated_python.bootstrap import validate_runtime
from dimos.imitation.policy.abc.module import DualOpenYamAbcPolicy
from dimos.robot.manipulators.dual_openyam.learning import DUAL_OPENYAM_ABC_IO
from dimos_abc.runtime import AbcPolicyRuntime, _validate_norm_stats, _validate_profile


def test_generated_runtime_implements_the_host_contract() -> None:
    validate_runtime(DualOpenYamAbcPolicy, AbcPolicyRuntime)


def test_released_abc_profile_matches_vendored_model_contract() -> None:
    _validate_profile(DUAL_OPENYAM_ABC_IO, DiTConfig())


def test_norm_stats_require_exact_released_dimensions() -> None:
    stats = {
        "state": {"mean": np.zeros(14), "std": np.ones(14)},
        "actions": {"mean": np.zeros(13), "std": np.ones(13)},
    }

    with pytest.raises(ValueError, match="actions mean shape"):
        _validate_norm_stats(stats, DiTConfig())
