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

"""The deliberately small executor boundary for scheduler-owned work."""

from __future__ import annotations

from collections.abc import Callable
import threading
from typing import Protocol

from .scheduler_models import (
    AttemptContext,
    ExecutorEvent,
    ExpandedCase,
    NamedCondition,
    TerminalOutcome,
)

EventSink = Callable[[ExecutorEvent], None]


class ExecutionInterrupted(Exception):  # noqa: N818 - public cooperative signal name is contractual
    """Cooperative signal used when an admitted execution is interrupted."""


class Executor(Protocol):
    """Execute one immutable case/condition attempt and return one outcome."""

    def run(
        self,
        case: ExpandedCase,
        condition: NamedCondition,
        context: AttemptContext,
        emit: EventSink,
        cancel_requested: threading.Event,
        publication_lock: threading.Lock,
    ) -> TerminalOutcome:
        """Emit safe progress/artifact events and return one terminal outcome."""
        ...
