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

import json
from pathlib import Path
import shlex
import sys

import pytest
from typer.testing import CliRunner

from dimos.autoresearch.session import Session
from dimos.cli.commands.autoresearch import autoresearch_app

runner = CliRunner()


def invoke(*args: str) -> str:
    result = runner.invoke(autoresearch_app, list(args), catch_exceptions=False)
    assert result.exit_code == 0, result.output
    return result.output


def test_agent_cli_compares_text_artifacts_freezes_and_exports(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    Path("evaluate.py").write_text(
        "import json, sys\n"
        "from pathlib import Path\n"
        "trial = json.load(sys.stdin)\n"
        "text = Path(trial['artifact']).read_text()\n"
        "assert Path('project-data.txt').read_text() == 'benchmark'\n"
        "Path(trial['directory'], 'report.txt').write_text(text)\n"
        "print(json.dumps({'score': len(text) + trial['index'], 'evidence': {'report': 'report.txt'}}))\n"
    )
    Path("project-data.txt").write_text("benchmark")
    invoke(
        "init",
        "session",
        "--task",
        "Shorten a message",
        "--metric",
        "characters",
        "--minimize",
        "--candidates",
        "2",
        "--dev-trials",
        "2",
        "--heldout-trials",
        "2",
        "--evaluator",
        "evaluate.py",
    )
    artifact = Path("prompt.txt")
    artifact.write_text("long text")
    assert invoke("submit", "session", str(artifact), "--hypothesis", "baseline").strip() == "1"
    artifact.write_text("short")
    invoke("submit", "session", str(artifact), "--hypothesis", "cut words")
    session = str(tmp_path / "session")
    monkeypatch.chdir(tmp_path.parent)
    invoke("run", session, "1")
    invoke("run", session, "2")
    board = json.loads(invoke("status", session))["leaderboard"]
    assert [(s["candidate"], s["dev_score"]) for s in board] == [(2, 5.5), (1, 9.5)]
    frozen = json.loads(invoke("freeze", session, "2"))["session"]
    assert (frozen["frozen_candidate"], frozen["heldout_score"]) == (2, 5.5)
    results = json.loads(invoke("results", session, "2"))
    assert [(r["split"], r["index"]) for r in results["trials"]] == [
        ("dev", 0),
        ("dev", 1),
        ("heldout", 0),
        ("heldout", 1),
    ]
    last = results["trials"][-1]
    assert (Path(last["directory"]) / last["evidence"]["report"]).read_text() == "short"
    usage = tmp_path / "usage.md"
    usage.write_text("Use the supplied text for compact replies.")
    skill = Path(
        invoke(
            "export", session, "--root", str(tmp_path / "skills"), "--instructions", str(usage)
        ).strip()
    )
    assert (skill / "assets/candidate/prompt.txt").read_text() == "short"
    assert usage.read_text() in (skill / "SKILL.md").read_text()
    assert json.loads((skill / "assets/validation.json").read_text())["heldout_score"] == 5.5


@pytest.mark.parametrize(
    ("response", "error", "message"),
    [
        ("sys.exit(7)", RuntimeError, "exited with code 7"),
        ("print('missing score')", ValueError, "invalid evaluator result"),
    ],
)
def test_evaluator_failure_keeps_output_and_index_pending(
    tmp_path, monkeypatch, response, error, message
):
    monkeypatch.chdir(tmp_path)
    invoke(
        "init",
        "s",
        "--task",
        "Measure",
        "--metric",
        "quality",
        "--dev-trials",
        "1",
        "--evaluator",
        shlex.join(
            [sys.executable, "-c", f"import sys; sys.stderr.write('missing input'); {response}"]
        ),
    )
    Path("candidate.json").write_text("{}")
    invoke("submit", "s", "candidate.json", "--hypothesis", "initial")
    with pytest.raises(error, match=message):
        runner.invoke(autoresearch_app, ["run", "s", "1"], catch_exceptions=False)
    session = Session(Path("s"))
    assert session.pending(1, "dev") == [0]
    attempt = session.candidate_dir(1) / "trials/dev-00"
    assert (attempt / "stderr.log").read_text() == "missing input"
    if error is ValueError:
        assert (attempt / "evaluation.json").read_text().strip() == "missing score"


@pytest.mark.parametrize("command", [" ", "'unterminated", "''"])
def test_init_rejects_invalid_evaluator_without_creating_session(tmp_path, command):
    directory = tmp_path / "session"
    result = runner.invoke(
        autoresearch_app,
        [
            "init",
            str(directory),
            "--task",
            "Measure",
            "--metric",
            "quality",
            "--evaluator",
            command,
        ],
    )
    assert result.exit_code == 2
    assert "--evaluator" in result.output
    assert not directory.exists()
