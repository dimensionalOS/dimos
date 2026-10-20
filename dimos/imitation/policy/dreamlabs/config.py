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


"""Settings for the optional client-only Dreamlabs policy."""

from pydantic import Field

from dimos.core.module import ModuleConfig


class DreamlabsConfig(ModuleConfig):
    prompt: str = Field(min_length=1, default="fold the towel")
    checkpoint: str = "dreamlabs/dream-1-beta"
    allow_remote: bool = False
    enable_execution: bool = False
    # Operator confirmation of the physical station, not an automatic model guess.
    standard_yam_verified: bool = False
    max_observation_age_s: float = Field(default=0.5, gt=0, allow_inf_nan=False)
    max_skew_s: float = Field(default=0.1, gt=0, allow_inf_nan=False)
    max_response_age_s: float = Field(default=2.0, gt=0, le=5, allow_inf_nan=False)
    timeout_ms: int = Field(default=1500, gt=0, le=5000)
    max_joint_speed: float = Field(default=1.0, gt=0, le=2, allow_inf_nan=False)
    max_gripper_speed: float = Field(default=1.0, gt=0, le=2, allow_inf_nan=False)
    completion_timeout_s: float = Field(default=1.0, gt=0, allow_inf_nan=False)
