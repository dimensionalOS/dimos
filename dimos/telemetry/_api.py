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

"""Public tracing helpers for `dimos.telemetry`.

Safe to call whether or not the `dimos[otel]` extra is installed and
whether or not tracing has been wired up. Short-circuits to a no-op
when the manager is in its default off state.
"""

from collections.abc import Iterator
from contextlib import contextmanager
from typing import Any

from dimos.telemetry._manager import _manager


@contextmanager
def span(name: str, **attributes: Any) -> Iterator[Any]:
    """Open a span around a block.

    Yields the active OTEL Span when tracing is configured, or None
    otherwise. In the no-op case the only cost is a single boolean
    check on the manager.
    """
    if not _manager._export_enabled:
        yield None
        return
    with _manager.tracer.start_as_current_span(name, attributes=attributes or None) as s:
        yield s
