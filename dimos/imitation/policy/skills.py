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

"""Agent controls for an explicitly configured policy."""

import json
from typing import Protocol

from dimos.agents.annotation import skill
from dimos.core.module import Module
from dimos.imitation.policy.module import RolloutStatus
from dimos.spec.utils import Spec


class PolicyRolloutSpec(Spec, Protocol):
    def preflight_rollout(self) -> RolloutStatus: ...
    def start_rollout(self) -> RolloutStatus: ...
    def stop_rollout(self) -> RolloutStatus: ...
    def rollout_status(self) -> RolloutStatus: ...


class PolicySkills(Module):
    _policy: PolicyRolloutSpec

    @skill
    def run_policy(self) -> str:
        """Preflight and start the configured learned task. Stop before classical motion."""
        status = self._policy.rollout_status()
        if not status["active"]:
            status = self._policy.preflight_rollout()
            if status["policy_ready"] and status["observations_ready"] and not status["last_error"]:
                status = self._policy.start_rollout()
        return json.dumps(status)

    @skill
    def stop_policy(self) -> str:
        """Stop policy motion and release its trajectory task before classical skills."""
        return json.dumps(self._policy.stop_rollout())

    @skill
    def policy_status(self) -> str:
        """Read configured task, readiness, activity and the last policy error."""
        return json.dumps(self._policy.rollout_status())
