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


"""Host contract for Dream-1 on a standard, dual six-joint YAM station."""

from typing import Any, Protocol

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.experimental.isolated_python.module import (
    IsolatedPythonModule,
    IsolatedPythonModuleConfig,
)
from dimos.imitation.policy.lerobot.module import PolicyControlSpec
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryStatus
from dimos.spec.utils import Spec
from dimos.teleop.webxr.controller_types import Buttons


class DreamlabsConfig(IsolatedPythonModuleConfig):
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


class DreamlabsRolloutSpec(Spec, Protocol):
    def preflight_rollout(self) -> dict[str, Any]: ...
    def start_rollout(self) -> dict[str, Any]: ...
    def stop_rollout(self) -> dict[str, Any]: ...
    def rollout_status(self) -> dict[str, Any]: ...


class DreamlabsControlSpec(PolicyControlSpec, Protocol):
    def get_trajectory_status(self) -> TrajectoryStatus | None: ...


class DreamlabsModule(IsolatedPythonModule):
    """An idle-by-default remote policy, with explicit operator start and stop."""

    project_dir = "native/python/dreamlabs"
    implementation = "dimos_dreamlabs.runtime:DreamlabsRuntime"
    config: DreamlabsConfig
    overhead_image: In[Image]
    left_wrist_image: In[Image]
    right_wrist_image: In[Image]
    coordinator_joint_state: In[JointState]
    teleop_buttons: In[Buttons]
    _control: DreamlabsControlSpec

    @rpc
    def preflight_rollout(self) -> dict[str, Any]:
        """Check local gates and fresh inputs. Makes no API call and moves nothing."""
        raise NotImplementedError

    @rpc
    def start_rollout(self) -> dict[str, Any]:
        """Start an episode only after explicit remote and execution opt-in."""
        raise NotImplementedError

    @rpc
    def stop_rollout(self) -> dict[str, Any]:
        """Cancel immediately and reject any pending inference response."""
        raise NotImplementedError

    @rpc
    def rollout_status(self) -> dict[str, Any]:
        """Return lifecycle state and a sanitized failure category."""
        raise NotImplementedError
