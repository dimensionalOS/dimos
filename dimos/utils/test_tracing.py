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

"""Contract tests for tracing.

Observability must not be able to break or slow a run. These check the three
ways it could: by raising when absent, by firing on the 15 Hz perception path,
and by carrying a secret off the machine.
"""

from __future__ import annotations

import pytest

from dimos.utils import tracing


@pytest.fixture(autouse=True)
def _reset_client(monkeypatch):
    """Each test decides for itself whether a client exists."""
    monkeypatch.setattr(tracing, "_CLIENT", None)
    monkeypatch.setattr(tracing, "_CLIENT_TRIED", True)
    monkeypatch.setattr(tracing._ACTIVE, "on", False, raising=False)


class TestAbsenceIsHarmless:
    def test_span_without_a_client_is_a_no_op(self):
        with tracing.span("anything", x=1) as handle:
            handle.set(y=2)  # must not raise

    def test_trace_without_a_client_still_runs_the_body(self):
        ran = []
        with tracing.agent_trace("task"):
            ran.append(True)

        assert ran == [True]

    def test_trace_never_swallows_a_body_exception(self, monkeypatch):
        monkeypatch.setattr(tracing, "_CLIENT", _FakeClient([]))

        with pytest.raises(RuntimeError, match="product failure"):
            with tracing.agent_trace("task"):
                raise RuntimeError("product failure")

    def test_span_never_swallows_a_body_exception(self, monkeypatch):
        monkeypatch.setattr(tracing, "_CLIENT", _FakeClient([]))

        with pytest.raises(RuntimeError, match="product failure"):
            with tracing.agent_trace("task"), tracing.span("skill:test"):
                raise RuntimeError("product failure")

    def test_enabled_is_false_without_a_client(self):
        assert not tracing.enabled()

    def test_an_import_failure_is_swallowed(self, monkeypatch):
        """A dev-only dependency must not take the robot down."""
        import builtins

        monkeypatch.setattr(tracing, "_CLIENT_TRIED", False)
        monkeypatch.setenv("LANGFUSE_PUBLIC_KEY", "pk")
        monkeypatch.setenv("LANGFUSE_SECRET_KEY", "sk")
        real_import = builtins.__import__

        def explode(name, *args, **kwargs):
            if name.startswith("langfuse"):
                raise ImportError("simulated: langfuse not installed")
            return real_import(name, *args, **kwargs)

        monkeypatch.setattr(builtins, "__import__", explode)

        assert tracing._client() is None  # swallowed, not raised

    def test_missing_credentials_skip_the_client_entirely(self, monkeypatch):
        """No keys means no client, and no attempt to build one."""
        monkeypatch.setattr(tracing, "_CLIENT_TRIED", False)
        monkeypatch.delenv("LANGFUSE_PUBLIC_KEY", raising=False)
        monkeypatch.delenv("LANGFUSE_SECRET_KEY", raising=False)

        assert tracing._client() is None


class TestNothingLeaksOutsideATrace:
    """Background perception runs the same wrapped code an agent does."""

    def test_a_skill_outside_a_trace_records_nothing(self, monkeypatch):
        calls = []
        monkeypatch.setattr(tracing, "_CLIENT", _FakeClient(calls))

        assert not tracing.enabled()
        with tracing.span("skill:perception_tick"):
            pass

        assert calls == []

    def test_inside_a_trace_it_records(self, monkeypatch):
        calls = []
        monkeypatch.setattr(tracing, "_CLIENT", _FakeClient(calls))

        with tracing.agent_trace("task"), tracing.span("skill:where_is_it"):
            pass

        assert "skill:where_is_it" in calls

    def test_the_gate_is_restored_after_the_trace(self, monkeypatch):
        monkeypatch.setattr(tracing, "_CLIENT", _FakeClient([]))

        with tracing.agent_trace("task"):
            assert tracing.enabled()

        assert not tracing.enabled()


class TestSecretsAndPixelsAreDropped:
    @pytest.mark.parametrize(
        "field", ["api_key", "OPENAI_API_KEY", "auth_token", "secret", "password", "authorization"]
    )
    def test_credential_shaped_fields_never_leave(self, field):
        assert tracing._clean({field: "sk-real-value", "safe": 1}) == {"safe": 1}

    def test_raw_bytes_are_dropped(self):
        """An image belongs in the store, not in a trace."""
        assert tracing._clean({"frame": b"\x89PNG...", "id": 7}) == {"id": 7}

    def test_empty_values_are_dropped(self):
        assert tracing._clean({"a": None, "b": "", "c": 0}) == {"c": 0}


class _FakeClient:
    def __init__(self, calls: list[str]) -> None:
        self._calls = calls

    def start_as_current_observation(self, *, name: str, as_type: str = "span"):
        import contextlib

        self._calls.append(name)

        @contextlib.contextmanager
        def _cm():
            yield _FakeHandle()

        return _cm()

    def flush(self) -> None:
        return


class _FakeHandle:
    def update(self, **kwargs) -> None:
        return
