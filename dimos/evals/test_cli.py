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

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest
from typer.testing import CliRunner

from dimos.cli.dimos import main as app
from dimos.evals import runner as runner_module
from dimos.evals.cli import run_provenance
from dimos.evals.runner import EvalRunner
from dimos.evals.suites import examples
from dimos.evals.types import EvalCase

SUITE_MODULE = "dimos.evals.suites.examples"
AGENT_MODULE = "dimos.evals.agents.question_answer"


class FailingEnvironment:
    artifacts: tuple[str, ...] = ()
    has_robot = False

    def preflight(self, agent: Any) -> None:
        raise RuntimeError("offline preflight")

    def start(self, modules: str, trace_dir: Path | None = None) -> Any:
        raise AssertionError("invalid test setup: execution reached")

    def settle(self, budget_s: float) -> None:
        pass

    def stop(self) -> None:
        pass


def _case(case_id: str, tag: str = "image") -> EvalCase:
    return EvalCase(
        id=case_id,
        inputs="?",
        environment=FailingEnvironment(),
        grade=lambda outcome: 1.0,
        tags=frozenset({tag}),
    )


@pytest.fixture
def cli_out_dir(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Path:
    out_dir = tmp_path / "evals"

    class LocalEvalRunner(EvalRunner):
        def __init__(self, **kwargs: Any) -> None:
            super().__init__(out_dir=out_dir, **kwargs)

    monkeypatch.setattr(runner_module, "EvalRunner", LocalEvalRunner)
    return out_dir


def test_agent_provenance_redacts_secrets_and_unserializable_values() -> None:
    provenance = run_provenance(
        {"kind": "suite_module", "value": "suite"},
        "agent",
        {"config": {"api_key": "never-write-me"}, "runtime": object()},
    )

    assert provenance["agent"]["kwargs"] is None
    assert provenance["agent"]["unavailable_reason"]
    assert "never-write-me" not in json.dumps(provenance)


def test_run_cli_records_exact_selection_and_agent_inputs(
    monkeypatch: pytest.MonkeyPatch, cli_out_dir: Path
) -> None:
    monkeypatch.setattr(
        examples,
        "SUITE",
        [_case("ignored", "other"), _case("case-a"), _case("case-b"), _case("limited")],
    )

    result = CliRunner().invoke(
        app,
        [
            "evals",
            "run",
            SUITE_MODULE,
            "--agent",
            AGENT_MODULE,
            "--set",
            "model=test-model",
            "--set",
            "frames_per_stream=8",
            "--tags",
            "image",
            "--limit",
            "2",
        ],
    )

    assert result.exit_code == 0
    run_dir = next(cli_out_dir.glob("run-*"))
    manifest = json.loads((run_dir / "manifest.json").read_text())
    assert manifest["source"] == {"kind": "suite_module", "value": SUITE_MODULE}
    assert manifest["selection"] == {
        "tags": ["image"],
        "limit": 2,
        "case_ids": ["case-a", "case-b"],
    }
    assert manifest["agent"] == {
        "module": AGENT_MODULE,
        "kwargs": {"model": "test-model", "frames_per_stream": 8},
    }
    assert not (run_dir / "case-a").exists(), "preflight-only cases have no case directory"
