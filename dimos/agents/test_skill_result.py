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

"""Tests for SkillResult, the agent_encode wire contract, and skill_timing."""

from __future__ import annotations

from enum import Enum, auto
import json
import time

import pytest

from dimos.agents.skill_result import CommonSkillError, SkillResult, skill_timing


class _DomainError(Enum):
    """Stand-in for a domain-specific error namespace (e.g. `ManipulationError`).

    Used to verify ``SkillResult`` is not coupled to ``CommonSkillError``.
    """

    SOMETHING_BROKE = auto()


class TestFactories:
    def test_ok_factory_packs_kwargs_into_metadata(self):
        """`ok(message, **kwargs)` routes kwargs to the metadata dict — non-obvious."""
        result = SkillResult.ok("done", planning_ms=12.3, attempts=2)
        assert result.metadata == {"planning_ms": 12.3, "attempts": 2}

    def test_fail_accepts_arbitrary_enum(self):
        """`error_code` is typed `Enum`, not `CommonSkillError`, so domains can BYO."""
        result = SkillResult.fail(_DomainError.SOMETHING_BROKE, "boom")
        assert result.error_code is _DomainError.SOMETHING_BROKE
        assert not result.is_success()


class TestAgentEncode:
    """Pins the wire contract used by the MCP server's ``agent_encode`` hook."""

    def test_success_payload_shape(self):
        result = SkillResult.ok("picked")
        result.duration_ms = 123.456

        encoded = result.agent_encode()
        assert isinstance(encoded, list)
        assert encoded[0]["type"] == "text"

        payload = json.loads(encoded[0]["text"])
        assert payload == {
            "success": True,
            "message": "picked",
            "error_code": None,
            "duration_ms": 123.5,
        }

    def test_failure_serializes_enum_via_name_cross_domain(self):
        """Failure encodes ``error_code.name`` (not ``.value``) and accepts foreign enums."""
        result = SkillResult.fail(_DomainError.SOMETHING_BROKE, "x")

        payload = json.loads(result.agent_encode()[0]["text"])
        assert payload["success"] is False
        assert payload["error_code"] == "SOMETHING_BROKE"
        assert payload["message"] == "x"

    def test_metadata_included_when_present(self):
        result = SkillResult.ok("done", attempts=3)
        payload = json.loads(result.agent_encode()[0]["text"])
        assert payload["metadata"] == {"attempts": 3}

    def test_metadata_omitted_when_empty(self):
        """Empty metadata is dropped from the wire to keep the payload small."""
        result = SkillResult.ok("done")
        payload = json.loads(result.agent_encode()[0]["text"])
        assert "metadata" not in payload


class TestSkillTiming:
    def test_duration_reflects_real_elapsed_time(self):
        """`skill_timing` measures wall time — not just stamps a fixed value."""
        sleep_ms = 50

        with skill_timing() as stamp:
            time.sleep(sleep_ms / 1000.0)
            result = stamp(SkillResult.ok())

        # Lower bound: sleep guarantees at least sleep_ms elapsed.
        # Upper bound is intentionally loose to avoid CI flakiness.
        assert result.duration_ms >= sleep_ms
        assert result.duration_ms < sleep_ms * 10  # sanity: not absurdly large

    def test_stamp_does_not_mutate_input(self):
        """Stamping returns a new instance; the input keeps its original duration.

        Critical for composed skills (``drop_on``, ``pick_and_place``) that
        stamp a result produced by an inner skill — mutating in place would
        silently overwrite the inner skill's ``duration_ms``.
        """
        sentinel_duration = 1234.5
        inner = SkillResult.ok("inner")
        inner.duration_ms = sentinel_duration  # simulate inner skill already stamped

        with skill_timing() as stamp:
            outer = stamp(inner)

        assert outer is not inner
        assert inner.duration_ms == sentinel_duration  # untouched
        # The outer stamp computes a real elapsed (very small) — distinct from the sentinel.
        assert outer.duration_ms != sentinel_duration


class _ListHandler:
    """Captures ``logger.info`` calls.

    The dimos logger sets ``propagate=False``, so pytest's ``caplog`` fixture
    can't see its records via the root logger. Instead we monkeypatch the
    logger's ``info`` method with this collector.
    """

    def __init__(self) -> None:
        self.messages: list[str] = []

    def __call__(self, msg: str, *args, **kwargs) -> None:
        self.messages.append(msg if not args else msg % args)


def _patch_logger(monkeypatch) -> _ListHandler:
    from dimos.agents import skill_result as sr

    handler = _ListHandler()
    monkeypatch.setattr(sr.logger, "info", handler)
    return handler


class TestSkillTimingLogging:
    def test_no_log_when_name_omitted(self, monkeypatch):
        """Anonymous timing stays quiet — preserves the original opt-in API."""
        handler = _patch_logger(monkeypatch)
        with skill_timing() as stamp:
            stamp(SkillResult.ok())
        assert handler.messages == []

    def test_log_format_on_success(self, monkeypatch):
        """Successful stamp emits `SKILL <name> result=OK duration_ms=<n.n>`."""
        handler = _patch_logger(monkeypatch)
        with skill_timing("set_gripper") as stamp:
            stamp(SkillResult.ok("done"))
        assert len(handler.messages) == 1
        msg = handler.messages[0]
        assert msg.startswith("SKILL set_gripper result=OK duration_ms=")

    def test_log_format_on_failure(self, monkeypatch):
        """Failure stamp emits the error_code's name (not OK, not a string repr)."""
        handler = _patch_logger(monkeypatch)
        with skill_timing("pick") as stamp:
            stamp(SkillResult.fail(CommonSkillError.ROBOT_NOT_FOUND, "x"))
        assert len(handler.messages) == 1
        msg = handler.messages[0]
        assert msg.startswith("SKILL pick result=ROBOT_NOT_FOUND duration_ms=")

    def test_each_stamp_call_logs_separately(self, monkeypatch):
        """One log line per return point — important for skills that branch."""
        handler = _patch_logger(monkeypatch)
        with skill_timing("move_to_pose") as stamp:
            stamp(SkillResult.ok("first"))
            stamp(SkillResult.fail(CommonSkillError.EXECUTION_FAILED, "second"))
        assert len(handler.messages) == 2

    def test_exception_path_logs_and_reraises(self, monkeypatch):
        """An uncaught exception in the skill body emits ``result=EXCEPTION`` and re-raises."""
        handler = _patch_logger(monkeypatch)
        with pytest.raises(RuntimeError, match="boom"), skill_timing("pick"):
            raise RuntimeError("boom")
        msgs = [m for m in handler.messages if "SKILL pick" in m]
        assert len(msgs) == 1
        assert msgs[0].startswith("SKILL pick result=EXCEPTION duration_ms=")

    def test_exception_path_silent_when_name_omitted(self, monkeypatch):
        """Anonymous timing still emits no log on exception — preserves opt-in API."""
        handler = _patch_logger(monkeypatch)
        with pytest.raises(ValueError), skill_timing():
            raise ValueError("crash")
        assert handler.messages == []
