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

"""Standard OTEL `BaseInstrumentor` integration for DimOS.

Host applications wire DimOS into their telemetry stack the same way
they wire every other instrumented library:

    from dimos.telemetry import DimosInstrumentor
    DimosInstrumentor().instrument(tracer_provider=my_provider)

This module is imported lazily by `dimos.telemetry.__getattr__` only
when `DimosInstrumentor` is accessed — that's the gate that keeps OTEL
imports out of `import dimos.telemetry`. Inside this module the OTEL
imports run at the top level, behind a try/except that falls back to a
stub class when `dimos[otel]` isn't installed.
"""

from typing import Any

try:
    from collections.abc import Collection

    from opentelemetry import trace as otel_trace
    from opentelemetry.instrumentation.instrumentor import (  # type: ignore[attr-defined]
        BaseInstrumentor,
    )

    class DimosInstrumentor(BaseInstrumentor):  # type: ignore[misc, valid-type]
        """OTEL instrumentor for DimOS.

        Calling `instrument(tracer_provider=...)` wires the supplied
        provider into `dimos.telemetry`. If `tracer_provider` is
        omitted, the global provider is used.
        """

        def instrumentation_dependencies(self) -> "Collection[str]":
            return []

        def _instrument(self, **kwargs: Any) -> None:
            from dimos.telemetry import configure_tracing

            tracer_provider = kwargs.get("tracer_provider") or otel_trace.get_tracer_provider()
            tracer_name = kwargs.get("tracer_name", "dimos")
            configure_tracing(tracer_provider, tracer_name)

        def _uninstrument(self, **kwargs: Any) -> None:
            from dimos.telemetry._manager import _manager

            _manager.reset()

except ImportError:

    class DimosInstrumentor:  # type: ignore[no-redef]
        """Stub: install `dimos[otel]` to use."""

        def __init__(self) -> None:
            raise RuntimeError(
                "DimosInstrumentor requires the `dimos[otel]` extra. "
                "Install with `pip install dimos[otel]` (or `uv sync --extra otel`)."
            )
