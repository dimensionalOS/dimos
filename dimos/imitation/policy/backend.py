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

"""Inference boundary shared by independently installed policy backends."""

from typing import Protocol

import numpy as np
from numpy.typing import NDArray

Images = dict[str, NDArray[np.uint8]]


def joint_permutations(hardware: list[str], policy: list[str]) -> tuple[list[int], list[int]]:
    """Translate vectors by joint identity in both directions."""
    if (
        len(policy) != len(hardware)
        or len(set(policy)) != len(policy)
        or set(policy) != set(hardware)
    ):
        raise ValueError("Policy joints must be unique and match the hardware joints")
    return [hardware.index(name) for name in policy], [policy.index(name) for name in hardware]


class PolicyBackend(Protocol):
    n_action_steps: int
    chunk_size: int | None
    fps: float
    action_lower: NDArray[np.float32] | None
    action_upper: NDArray[np.float32] | None

    def predict(
        self, images: Images, state: NDArray[np.float32], *, task: str
    ) -> NDArray[np.float32]: ...
    def reset(self) -> None: ...
    def close(self) -> None: ...
