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

"""Host-only RPC controls for the attached evaluation blueprint."""

from __future__ import annotations

from typing import Any

from dimos.agents.code_policy import CodePolicySessionReceipt
from dimos.navigation.base import NavigationState
from dimos.porcelain.dimos import Dimos


class AttachedDimosControl:
    """Use non-model-facing RPCs without taking ownership of the DimOS daemon."""

    def __init__(self, app: Dimos) -> None:
        self._app = app
        self._code_policy: Any = app.CodePolicyModule
        self._navigation: Any = app.ReplanningAStarPlanner

    @classmethod
    def connect(cls, timeout_s: float) -> AttachedDimosControl:
        return cls(Dimos.connect(timeout=timeout_s))

    def reset_session(self, timeout_s: float) -> CodePolicySessionReceipt:
        del timeout_s
        value = self._code_policy.reset_session()
        return CodePolicySessionReceipt.model_validate(value)

    def interrupt_active(self, timeout_s: float) -> bool:
        del timeout_s
        return bool(self._code_policy.interrupt_active())

    def motion_active(self, timeout_s: float) -> bool:
        del timeout_s
        return self._navigation.get_state() != NavigationState.IDLE

    def cancel_motion(self, timeout_s: float) -> None:
        del timeout_s
        self._navigation.cancel_goal()

    def close(self) -> None:
        self._app.stop()
