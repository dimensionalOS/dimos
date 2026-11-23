# Copyright 2025-2026 Dimensional Inc.
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

"""Local, whole-group command contract (no transport or device semantics)."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class GroupCommandOutput:
    """One complete, immutable-by-contract payload addressed to a group."""

    interface: str
    payload: object


@dataclass(frozen=True, slots=True)
class GroupCommandLease:
    """Local authority for one payload. Not serializable or renewable.

    Compute outside the guard. Pass the final synchronous, bounded write to
    ``try_write``; revocation and writes share a lock. A false result means
    the write did not run. The write must not reenter the coordinator.
    """

    expires_at: float  # time.monotonic()
    try_write: Callable[[Callable[[], None]], bool]


GroupCommandHandler = Callable[[object, GroupCommandLease], None]
