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

"""Opt-in OpenTelemetry tracing for DimOS.

Importing this package has no side effects and triggers no
opentelemetry imports, even when the `dimos[otel]` extra is installed.
The public `span` context manager is a silent no-op until tracing is
wired up.

Three ways to turn tracing on:

1. Env-driven setup. Install the extra and set OTEL_EXPORTER_OTLP_ENDPOINT
   (plus optional OTEL_EXPORTER_OTLP_HEADERS and OTEL_SERVICE_NAME).
   `enable()` runs automatically on first import of this package. The
   OTLP HTTP exporter is configured, and LangChain auto-instrumentation
   is applied when `openinference-instrumentation-langchain` is present.

2. Caller-owned provider. When the host app has its own TracerProvider:

       dimos.telemetry.configure_tracing(my_provider)

3. Standard OTEL `BaseInstrumentor`:

       DimosInstrumentor().instrument(tracer_provider=my_provider)

Any OTLP-compatible backend works (Langfuse, Arize Phoenix, LangSmith,
Opik, etc.). Vendor selection is by env var, not code.
"""

import os
from typing import Any

from dimos.telemetry._api import span
from dimos.telemetry._manager import _manager
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

__all__ = [
    "DimosInstrumentor",
    "configure_tracing",
    "enable",
    "session_attributes",
    "span",
]


def configure_tracing(tracer_provider: Any, tracer_name: str = "dimos") -> None:
    """Wire DimOS into a caller-owned TracerProvider.

    Raises RuntimeError when the `dimos[otel]` extra isn't installed.
    """
    try:
        import opentelemetry  # noqa: F401  (presence check)
    except ImportError:
        raise RuntimeError(
            "dimos.telemetry: opentelemetry is not installed. "
            "Install with `pip install dimos[otel]` (or `uv sync --extra otel`)."
        ) from None
    _manager.configure(tracer_provider.get_tracer(tracer_name))


def enable() -> bool:
    """Auto-configure tracing from the standard OTEL env vars.

    Returns True when tracing was wired up, False otherwise (no exporter
    endpoint set, or the `dimos[otel]` extra isn't installed).
    """
    if not os.environ.get("OTEL_EXPORTER_OTLP_ENDPOINT"):
        return False
    try:
        import atexit

        from opentelemetry import trace
        from opentelemetry.exporter.otlp.proto.http.trace_exporter import (
            OTLPSpanExporter,
        )
        from opentelemetry.sdk.resources import Resource
        from opentelemetry.sdk.trace import TracerProvider
        from opentelemetry.sdk.trace.export import BatchSpanProcessor
    except ImportError:
        logger.warning(
            "OTEL_EXPORTER_OTLP_ENDPOINT is set but `dimos[otel]` is not "
            "installed; tracing disabled."
        )
        return False

    provider = TracerProvider(
        resource=Resource.create({"service.name": os.environ.get("OTEL_SERVICE_NAME", "dimos")})
    )
    provider.add_span_processor(BatchSpanProcessor(OTLPSpanExporter()))
    trace.set_tracer_provider(provider)
    atexit.register(provider.shutdown)
    configure_tracing(provider)

    try:
        from openinference.instrumentation.langchain import LangChainInstrumentor

        LangChainInstrumentor().instrument(tracer_provider=provider)
    except ImportError:
        # Auto-instrumentation is best-effort and optional within the extra.
        pass

    return True


def session_attributes(session_id: str) -> dict[str, str]:
    """Span-attribute dict that groups traces under one session in
    common LLM-observability backends.

    Sets three keys to cover the four major OTLP-compatible backends:
        session.id                 — OpenInference convention.
                                     Used by Langfuse and Arize Phoenix / AX.
        langsmith.trace.session_id — LangSmith's own attribute namespace.
        thread_id                  — Opik's Threads attribute (added in
                                     comet-ml/opik#3441).
    """
    return {
        "session.id": session_id,
        "langsmith.trace.session_id": session_id,
        "thread_id": session_id,
    }


def __getattr__(name: str) -> Any:
    # Resolve DimosInstrumentor lazily so importing this package never
    # pulls in opentelemetry.instrumentation.
    if name == "DimosInstrumentor":
        from dimos.telemetry.instrumentor import DimosInstrumentor

        globals()[name] = DimosInstrumentor
        return DimosInstrumentor
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


# Boot-time auto-enable: treat OTEL_EXPORTER_OTLP_ENDPOINT being set as the
# user's explicit opt-in. When unset (the default), this is a single
# os.environ.get() check; no OTEL packages are imported.
if os.environ.get("OTEL_EXPORTER_OTLP_ENDPOINT"):
    enable()
