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

from subprocess import CalledProcessError, CompletedProcess

import pytest

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


def test_rewrite_removes_generated_signature_case_insensitively(tmp_path):
    path = tmp_path / "COMMIT_EDITMSG"
    path.write_text("Subject\n\ngEnErAtEd WiTh tool\nfooter\n")
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
