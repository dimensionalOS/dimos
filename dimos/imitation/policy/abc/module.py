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

"""Dual OpenYAM binding for Amazon's released ABC-DiT checkpoint."""

from pathlib import Path

from pydantic import Field, field_validator

from dimos.imitation.policy.module import PolicyRolloutConfig, declare_policy_module
from dimos.robot.manipulators.dual_openyam.learning import DUAL_OPENYAM_ABC_IO


class AbcPolicyConfig(PolicyRolloutConfig):
    """Released ABC-DiT inference settings."""

    norm_stats_path: str | None = None
    diffusion_steps: int = Field(default=10, ge=1)
    fast_inference: bool = True

    @field_validator("norm_stats_path")
    @classmethod
    def resolve_norm_stats_path(cls, value: str | None) -> str | None:
        if value is None:
            return None
        path = Path(value).expanduser()
        return str(path.resolve()) if path.exists() else value


DualOpenYamAbcPolicy = declare_policy_module(
    "DualOpenYamAbcPolicy",
    __name__,
    DUAL_OPENYAM_ABC_IO,
    AbcPolicyConfig,
    "dimos_abc.runtime:AbcPolicyRuntime",
)
