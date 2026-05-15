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

"""Backend protocol for `PolicyModule`.

A `PolicyBackend` adapts a learned (or scripted) policy to the canonical
`PolicyObservation` / `PolicyCommand` boundary defined here. Concrete
backends live under `dimos.manipulation.policy.backends.*` and are looked
up by name via `dimos.manipulation.policy.registry`.

The protocol is structural — concrete backends do not need to inherit
from it.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from dimos.manipulation.policy.command import PolicyCommand
from dimos.manipulation.policy.observation import PolicyObservation


@runtime_checkable
class PolicyBackend(Protocol):
    """Narrow contract every policy backend MUST implement.

    Lifecycle:
        1. The factory in the registry returns an instance.
        2. `initialize()` is called once on `PolicyModule.start()`. Heavy
           one-time work (model loading, GPU warm-up) belongs here.
        3. `select_action()` is called per inference tick.
        4. `reset()` is called whenever the node hands control back to the
           backend after a teleop preemption window. It MUST drop any
           buffered action chunk and reinitialize recurrent state so the
           next `select_action()` recomputes from the next observation.
        5. `close()` is called once on `PolicyModule.stop()`.
    """

    def initialize(self) -> None:
        """One-time backend setup. Called from `PolicyModule.start()`."""

    def select_action(self, observation: PolicyObservation) -> PolicyCommand:
        """Translate an observation into a tagged `PolicyCommand`.

        Backends MUST keep framework-specific tensors and dictionaries
        inside this method's scope.
        """

    def reset(self) -> None:
        """Discard buffered action chunks and recurrent state.

        Called by the node on teleop preemption, when the backend is about
        to be re-engaged from a fresh observation. Implementations MUST NOT
        raise; the node's reset path is best-effort.
        """

    def close(self) -> None:
        """Release backend resources. Called from `PolicyModule.stop()`."""


__all__ = ["PolicyBackend"]
