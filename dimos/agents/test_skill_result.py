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

"""Tests for SkillResult, the agent_encode wire contract, and the @skill decorator's
auto-timing/logging behavior.
"""

from __future__ import annotations

import json
import logging
import time

import pytest

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult

# The @skill decorator logs through this stdlib logger name (set by
# setup_logger() from the module's file path).
_ANNOTATION_LOGGER = "dimos/agents/annotation.py"


class TestFactories:
    def test_ok_factory_packs_kwargs_into_metadata(self):
        """`ok(message, **kwargs)` routes kwargs to the metadata dict — non-obvious."""
        result = SkillResult.ok("done", planning_ms=12.3, attempts=2)
        assert result.metadata == {"planning_ms": 12.3, "attempts": 2}

    def test_fail_stores_string_code(self):
        """`fail` accepts a plain string; codes are Literal strings at runtime."""
        result = SkillResult.fail("ROBOT_NOT_FOUND", "no arm")
        assert not result.is_success()
        assert result.error_code == "ROBOT_NOT_FOUND"
        assert result.message == "no arm"


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

    def test_failure_payload_carries_code_string_verbatim(self):
        """Failure encodes ``error_code`` straight into the JSON — no conversion ceremony."""
        result = SkillResult.fail("EXECUTION_TIMEOUT", "took too long")

        payload = json.loads(result.agent_encode()[0]["text"])
        assert payload["success"] is False
        assert payload["error_code"] == "EXECUTION_TIMEOUT"
        assert payload["message"] == "took too long"

    def test_metadata_included_when_present(self):
        result = SkillResult.ok("done", attempts=3)
        payload = json.loads(result.agent_encode()[0]["text"])
        assert payload["metadata"] == {"attempts": 3}

    def test_metadata_omitted_when_empty(self):
        """Empty metadata is dropped from the wire to keep the payload small."""
        result = SkillResult.ok("done")
        payload = json.loads(result.agent_encode()[0]["text"])
        assert "metadata" not in payload


@pytest.fixture
def skill_logs(caplog):
    """Capture ``@skill`` decorator log lines via pytest's ``caplog``.

    The dimos logger is structlog over a stdlib logger with
    ``propagate=False``
    """
    lg = logging.getLogger(_ANNOTATION_LOGGER)
    lg.addHandler(caplog.handler)
    caplog.set_level(logging.INFO, logger=_ANNOTATION_LOGGER)
    try:
        yield caplog
    finally:
        lg.removeHandler(caplog.handler)


def _skill_lines(caplog, needle: str) -> list[str]:
    """Rendered log messages containing ``needle`` (e.g. 'SKILL pick')."""
    return [r.getMessage() for r in caplog.records if needle in r.getMessage()]


class TestSkillDecoratorTiming:
    """The ``@skill`` decorator auto-stamps ``duration_ms`` and logs.

    These tests run synthetic ``@skill``-decorated functions to verify the
    decorator's contract without touching manipulation infrastructure.
    """

    def test_decorator_stamps_duration_ms_on_skillresult(self):
        @skill
        def my_skill() -> SkillResult:
            time.sleep(0.05)
            return SkillResult.ok("done")

        result = my_skill()
        assert isinstance(result, SkillResult)
        assert result.duration_ms >= 50

    def test_decorator_does_not_modify_non_skillresult_returns(self):
        """Skills returning non-SkillResult values still log, but the return is untouched."""

        @skill
        def my_skill() -> str:
            return "plain string"

        assert my_skill() == "plain string"

    def test_logs_success_with_function_name(self, skill_logs):
        @skill
        def set_gripper() -> SkillResult:
            return SkillResult.ok("done")

        set_gripper()
        msgs = _skill_lines(skill_logs, "SKILL set_gripper")
        assert len(msgs) == 1
        assert "SKILL set_gripper result=OK duration_ms=" in msgs[0]

    def test_logs_failure_with_error_code(self, skill_logs):
        @skill
        def pick() -> SkillResult:
            return SkillResult.fail("ROBOT_NOT_FOUND", "x")

        pick()
        msgs = _skill_lines(skill_logs, "SKILL pick")
        assert len(msgs) == 1
        assert "SKILL pick result=ROBOT_NOT_FOUND duration_ms=" in msgs[0]

    def test_exception_path_logs_and_reraises(self, skill_logs):
        """An uncaught exception emits ``result=EXCEPTION`` and re-raises."""

        @skill
        def boom() -> SkillResult:
            raise RuntimeError("nope")

        with pytest.raises(RuntimeError, match="nope"):
            boom()

        msgs = _skill_lines(skill_logs, "SKILL boom")
        assert len(msgs) == 1
        assert "SKILL boom result=EXCEPTION duration_ms=" in msgs[0]

    def test_failed_result_without_code_logs_failed_not_ok(self, skill_logs):
        """success=False is authoritative even when error_code is unset."""

        @skill
        def half_broken() -> SkillResult:
            return SkillResult(success=False)  # no error_code set

        half_broken()
        msgs = _skill_lines(skill_logs, "SKILL half_broken")
        assert len(msgs) == 1
        assert "result=FAILED" in msgs[0]

    def test_non_skillresult_return_logs_unknown(self, skill_logs):
        """A bare string return can't be verified — don't claim result=OK."""

        @skill
        def legacy() -> str:
            return "Error: something the decorator can't interpret"

        legacy()
        msgs = _skill_lines(skill_logs, "SKILL legacy")
        assert len(msgs) == 1
        assert "result=UNKNOWN" in msgs[0]

    def test_decorator_does_not_mutate_returned_skillresult(self):
        """The decorator returns a fresh SkillResult instance — the body's return
        object keeps its original duration_ms (whatever it was before)."""
        sentinel = SkillResult.ok("done")
        sentinel.duration_ms = 999.0

        @skill
        def my_skill() -> SkillResult:
            return sentinel

        result = my_skill()
        assert result is not sentinel
        assert sentinel.duration_ms == 999.0  # untouched
        # Decorator overwrites with actual measured elapsed (very small).
        assert result.duration_ms != 999.0
