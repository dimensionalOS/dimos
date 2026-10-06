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

from pathlib import Path
from subprocess import CalledProcessError, CompletedProcess, run

import pytest
import yaml

from dimos.utils import commit_message


@pytest.mark.parametrize(
    "identity",
    [
        "Claude Opus 5 (1M context) <noreply@anthropic.com>",
        "CODEX <codex@users.noreply.github.com>",
        "OpenAI Codex <noreply@openai.com>",
        "Assistant <noreply@anthropic.com>",
        "Assistant <noreply@openai.com>",
        "Assistant <noreply@cursor.com>",
        "Claude <bot@example.com>",
        "Claude Code <bot@example.com>",
        "Copilot <bot@example.com>",
        "GitHub Copilot <copilot@github.com>",
        "Cursor <cursoragent@cursor.com>",
        "Cursor Agent <bot@example.com>",
        "Gemini <bot@example.com>",
        "Google Gemini <bot@example.com>",
        "Windsurf <bot@example.com>",
        "Devin <bot@example.com>",
        "Aider <bot@example.com>",
    ],
)
@pytest.mark.parametrize("trailer", ["Co-Authored-By:", "co-authored-by:", "\tCO-AUTHORED-BY :"])
def test_ci_and_local_filter_reject_same_bot_identities(identity, trailer):
    message = f"Subject\n\n{trailer} {identity}\n"
    assert commit_message.AI_COAUTHOR.search(message) is not None
    assert commit_message.filter_text(message)[0] == "Subject\n\n"


@pytest.mark.parametrize(
    "message",
    [
        "Co-authored-by: Alice Smith <alice@example.com>",
        "Co-authored-by: Claude Smith <claude@example.com>",
        "Co-authored-by: Claude <claude@personal.example>",
        "Co-authored-by: Devin <devin@personal.example>",
        "Co-authored-by: Gemini <gemini@personal.example>",
        "Co-authored-by: Alice <alice@openai.com>",
        "Co-authored-by: Alice <alice@anthropic.com>",
        "Co-authored-by: Gemini Patel <gemini@example.com>",
        "Co-authored-by: Claudette <claudette@example.com>",
        "Co-authored-by: Alice <alice@google.com>",
        "Co-authored-by: Alice <alice@openai.com.example.org>",
        "Fix Claude and Codex integration",
        "Document Co-authored-by: Claude trailers",
        "Co-authored-by: Alice <alice@example.com>\n\nFix Codex integration",
    ],
)
def test_preserves_human_coauthors_and_prose(message):
    assert commit_message.AI_COAUTHOR.search(message) is None
    assert commit_message.filter_text(message) == (message, None)


@pytest.mark.parametrize(
    "sentence",
    [
        "This report was Generated with a local pipeline.",
        "This report was generated with a local pipeline.",
        "Generated with a local pipeline, these fixtures are deterministic.",
        "Generated with Claude templates, then manually edited.",
    ],
)
def test_rewrite_preserves_generated_with_prose(tmp_path, sentence):
    message = f"Subject\n\n{sentence}\nKeep these details.\n"
    path = tmp_path / "COMMIT_EDITMSG"
    path.write_text(message)
    assert commit_message.rewrite_file(path) == 0
    assert path.read_text() == message


@pytest.mark.parametrize(
    "signature",
    [
        "Generated with Codex",
        "Generated with [Claude Code](https://claude.ai/code)",
        "🤖 Generated with [Claude Code](https://claude.ai/code)",
    ],
)
def test_rewrite_removes_structured_ai_signature(tmp_path, signature):
    path = tmp_path / "COMMIT_EDITMSG"
    path.write_text(f"Subject\n\n{signature}\n")
    assert commit_message.rewrite_file(path) == 0
    assert path.read_text() == "Subject\n\n"


def test_rewrite_removes_generated_signature_case_insensitively(tmp_path):
    path = tmp_path / "COMMIT_EDITMSG"
    path.write_text("Subject\n\ngEnErAtEd WiTh Codex\nfooter\n")
    assert commit_message.rewrite_file(path) == 0
    assert path.read_text() == "Subject\n\n"


def test_range_reports_earlier_bot_commit_even_with_clean_head(monkeypatch, mocker, capsys):
    monkeypatch.setenv("PRE_COMMIT_FROM_REF", "base")
    monkeypatch.setenv("PRE_COMMIT_TO_REF", "head")
    run = mocker.patch.object(
        commit_message.subprocess,
        "run",
        side_effect=[
            CompletedProcess([], 0, stdout="badcommit\nhead\n"),
            CompletedProcess([], 0, stdout="Change\n\nCo-authored-by: Copilot <bot@example.com>"),
            CompletedProcess([], 0, stdout="Clean head"),
        ],
    )
    assert commit_message.check_commits() == 1
    assert "badcommit: AI co-author:" in capsys.readouterr().err
    assert run.call_args_list[0].args[0] == ["git", "rev-list", "--reverse", "base..head"]


@pytest.mark.parametrize("base, head", [(None, None), ("base", None), (None, "head")])
def test_missing_range_fails_without_running_git(monkeypatch, mocker, base, head):
    for key, value in [("PRE_COMMIT_FROM_REF", base), ("PRE_COMMIT_TO_REF", head)]:
        monkeypatch.delenv(key, raising=False)
        if value is not None:
            monkeypatch.setenv(key, value)
    run = mocker.patch.object(commit_message.subprocess, "run")
    assert commit_message.check_commits() == 1
    run.assert_not_called()


def test_git_failure_is_reported(monkeypatch, mocker, capsys):
    monkeypatch.setenv("PRE_COMMIT_FROM_REF", "base")
    monkeypatch.setenv("PRE_COMMIT_TO_REF", "head")
    mocker.patch.object(
        commit_message.subprocess,
        "run",
        side_effect=CalledProcessError(128, "git", stderr="unknown revision"),
    )
    assert commit_message.check_commits() == 1
    assert "unknown revision" in capsys.readouterr().err


def test_clean_range_passes(monkeypatch, mocker, capsys):
    monkeypatch.setenv("PRE_COMMIT_FROM_REF", "base")
    monkeypatch.setenv("PRE_COMMIT_TO_REF", "head")
    mocker.patch.object(
        commit_message.subprocess,
        "run",
        side_effect=[
            CompletedProcess([], 0, stdout="head\n"),
            CompletedProcess([], 0, stdout="Co-authored-by: Alice <alice@example.com>"),
        ],
    )
    assert commit_message.check_commits() == 0
    assert capsys.readouterr().err == ""


@pytest.mark.parametrize(
    "message, base_has_policy, expected, error",
    [
        ("Fix recording reader", True, 0, ""),
        ("Fix recording reader\n\nCo-authored-by: Alice <alice@example.com>", True, 0, ""),
        (
            "Fix recording reader\n\nCo-authored-by: Codex <noreply@openai.com>",
            True,
            1,
            "AI co-author:",
        ),
        ("Fix recording reader", False, 1, "Update this branch from main and retry."),
    ],
)
def test_workflow_uses_only_base_policy(
    tmp_path, monkeypatch, message, base_has_policy, expected, error
):
    workflow_path = Path(__file__).resolve().parents[2] / ".github/workflows/ci.yml"
    workflow = yaml.safe_load(workflow_path.read_text())
    script = next(
        step["run"]
        for step in workflow["jobs"]["commit-messages"]["steps"]
        if step["name"] == "Check incoming commit messages"
    )
    policy = Path(commit_message.__file__).read_text()
    monkeypatch.chdir(tmp_path)
    for role in ("AUTHOR", "COMMITTER"):
        monkeypatch.setenv(f"GIT_{role}_NAME", "Policy test")
        monkeypatch.setenv(f"GIT_{role}_EMAIL", "policy@example.com")
    run(["git", "init", "--quiet"], check=True)
    tree = run(
        ["git", "mktree"], input="", text=True, capture_output=True, check=True
    ).stdout.strip()
    policy_path = tmp_path / "dimos/utils/commit_message.py"
    policy_path.parent.mkdir(parents=True)
    if base_has_policy:
        policy_path.write_text(policy)
        run(["git", "add", str(policy_path)], check=True)
        tree = run(["git", "write-tree"], text=True, capture_output=True, check=True).stdout.strip()
    base = run(
        ["git", "commit-tree", tree, "-m", "Base"], text=True, capture_output=True, check=True
    ).stdout.strip()
    # A PR can replace its policy with a no-op, but CI must still use the base.
    policy_path.write_text("raise SystemExit(0)\n")
    run(["git", "add", str(policy_path)], check=True)
    tree = run(["git", "write-tree"], text=True, capture_output=True, check=True).stdout.strip()
    rejected_or_allowed = run(
        ["git", "commit-tree", tree, "-p", base, "-m", message],
        text=True,
        capture_output=True,
        check=True,
    ).stdout.strip()
    head = run(
        ["git", "commit-tree", tree, "-p", rejected_or_allowed, "-m", "Clean head"],
        text=True,
        capture_output=True,
        check=True,
    ).stdout.strip()
    monkeypatch.setenv("PRE_COMMIT_FROM_REF", base)
    monkeypatch.setenv("PRE_COMMIT_TO_REF", head)
    monkeypatch.setenv("RUNNER_TEMP", str(tmp_path))

    result = run(["bash", "-e", "-c", script], text=True, capture_output=True)

    assert result.returncode == expected
    assert error in result.stderr
