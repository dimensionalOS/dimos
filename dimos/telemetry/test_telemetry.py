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

"""Tests for `dimos.telemetry`.

Covers the strict-opt-in contract (no OTEL imports until the user opts
in), the no-op default state of the public helpers, and the shape of
the session-grouping attributes used by downstream backends.
"""

import importlib.util
import os
import subprocess
import sys
import textwrap

import pytest

import dimos.telemetry
from dimos.telemetry._manager import _manager


@pytest.fixture(autouse=True)
def _reset_telemetry_state(monkeypatch: pytest.MonkeyPatch) -> None:
    """Each test starts with the manager off and no auto-enable env vars."""
    monkeypatch.delenv("OTEL_EXPORTER_OTLP_ENDPOINT", raising=False)
    monkeypatch.delenv("OTEL_EXPORTER_OTLP_HEADERS", raising=False)
    _manager.reset()


def test_span_is_noop_by_default() -> None:
    with dimos.telemetry.span("test") as s:
        assert s is None


def test_span_accepts_dotted_attribute_keys() -> None:
    # OpenInference and LangSmith both use dotted keys (`session.id`,
    # `langsmith.trace.session_id`). The `**` unpack must accept them
    # without an `invalid kwarg` error.
    with dimos.telemetry.span("test", **{"session.id": "abc"}) as s:
        assert s is None


def test_session_attributes_sets_all_backend_keys() -> None:
    assert dimos.telemetry.session_attributes("session-123") == {
        "session.id": "session-123",
        "langsmith.trace.session_id": "session-123",
        "thread_id": "session-123",
    }


def test_enable_returns_false_without_endpoint_env_var() -> None:
    assert dimos.telemetry.enable() is False


def test_dimos_instrumentor_resolves_via_lazy_getattr() -> None:
    # The symbol is defined via module-level `__getattr__`, not as a
    # top-level name. Accessing it must succeed regardless of whether
    # the `dimos[otel]` extra is installed (stub class otherwise).
    cls = dimos.telemetry.DimosInstrumentor
    assert cls.__name__ == "DimosInstrumentor"


def test_importing_package_triggers_no_opentelemetry_imports() -> None:
    """Strict opt-in: `import dimos.telemetry` must not load any
    opentelemetry module, even when the `dimos[otel]` extra is
    installed. Runs in a fresh interpreter so previous in-process
    imports don't pollute the assertion.
    """
    if importlib.util.find_spec("opentelemetry") is None:
        pytest.skip("opentelemetry not installed; strict-opt-in is trivial here")

    code = textwrap.dedent(
        """
        import sys
        import dimos.telemetry  # noqa: F401
        otel = [
            m for m in sys.modules
            if m == "opentelemetry" or m.startswith("opentelemetry.")
        ]
        print(len(otel))
        """
    )
    env = {**os.environ}
    env.pop("OTEL_EXPORTER_OTLP_ENDPOINT", None)
    out = subprocess.check_output([sys.executable, "-c", code], env=env, text=True).strip()
    assert out == "0", f"expected 0 opentelemetry modules after import, got {out}"
