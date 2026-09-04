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

"""Small in-process contract implemented by isolated policy backends."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.imitation.profile import PolicyIOProfile


@dataclass(frozen=True)
class PolicyBackendInfo:
    """Execution information discovered while loading a policy artifact."""

    name: str
    chunk_length: int
    preferred_execution_steps: int
    action_lower: NDArray[np.float32] | None = None
    action_upper: NDArray[np.float32] | None = None


class PolicyBackend(Protocol):
    """Backend-specific loading and inference behind the common rollout loop."""

    def __init__(self, config: Any) -> None: ...

    def load(self, profile: PolicyIOProfile) -> PolicyBackendInfo: ...

    def reset(self) -> None: ...

    def predict(
        self,
        observations: Mapping[str, NDArray[Any]],
        task: str,
    ) -> NDArray[np.float32]: ...
