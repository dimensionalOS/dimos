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
import subprocess
import sys

from filter_commit_message import AI_COAUTHOR
import pytest

SCRIPT = Path(__file__).with_name("filter_commit_message.py")


@pytest.mark.parametrize(
    ("message", "rejected"),
    [
        ("Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>", True),
        ("CO-AUTHORED-BY: CODEX <codex@users.noreply.github.com>", True),
        ("Co-Authored-By: OpenAI Codex <noreply@openai.com>", True),
        ("\tco-authored-by : Assistant <noreply@anthropic.com>", True),
        ("Co-authored-by: Assistant <noreply@openai.com>", True),
        ("Co-authored-by: Claude <bot@example.com>", True),
        ("Co-authored-by: Codex <bot@example.com>", True),
        ("Co-authored-by: Alice Smith <alice@example.com>", False),
        ("Co-authored-by: Claudette <claudette@example.com>", False),
        ("Fix Claude and Codex integration", False),
        ("Document Co-authored-by: Claude trailers", False),
        ("Generated with a code generator", False),
        ("Co-authored-by: Alice <alice@example.com>\n\nFix Codex integration", False),
    ],
)
def test_ai_coauthor_detection(message: str, rejected: bool) -> None:
    assert bool(AI_COAUTHOR.search(f"Subject\n\n{message}\n")) == rejected


@pytest.fixture
def repository(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", "/dev/null")
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")
    for role in ("AUTHOR", "COMMITTER"):
        monkeypatch.setenv(f"GIT_{role}_NAME", "Test User")
        monkeypatch.setenv(f"GIT_{role}_EMAIL", "test@example.com")
    monkeypatch.delenv("PRE_COMMIT_FROM_REF", raising=False)
    monkeypatch.delenv("PRE_COMMIT_TO_REF", raising=False)
    subprocess.run(["git", "init", "--quiet"], check=True)


def commit(message: str, *parents: str) -> str:
    tree = subprocess.run(
        ["git", "mktree"], input="", capture_output=True, text=True, check=True
    ).stdout.strip()
    args = ["git", "-c", "commit.gpgsign=false", "commit-tree", tree, "-m", message]
    for parent in parents:
        args.extend(["-p", parent])
    return subprocess.run(args, capture_output=True, text=True, check=True).stdout.strip()


def check_range(
    monkeypatch: pytest.MonkeyPatch, base: str, head: str
) -> subprocess.CompletedProcess[str]:
    monkeypatch.setenv("PRE_COMMIT_FROM_REF", base)
    monkeypatch.setenv("PRE_COMMIT_TO_REF", head)
    return subprocess.run([sys.executable, str(SCRIPT), "--check"], capture_output=True, text=True)


def test_checks_older_commits_and_merge_parents(
    repository: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    base = commit("Base")
    claude = commit("Change\n\nCo-authored-by: Claude <noreply@anthropic.com>", base)
    codex = commit("Other change\n\nCo-authored-by: Codex <noreply@openai.com>", base)
    head = commit("Clean merge message", claude, codex)

    result = check_range(monkeypatch, base, head)

    assert result.returncode == 1
    assert claude[:12] in result.stderr
    assert codex[:12] in result.stderr
    assert head[:12] not in result.stderr


def test_excludes_base_history_and_allows_human_coauthors(
    repository: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    ancestor = commit("Existing\n\nCo-authored-by: Claude <noreply@anthropic.com>")
    base = commit("Main-only\n\nCo-authored-by: Codex <noreply@openai.com>", ancestor)
    head = commit("Change\n\nCo-authored-by: Alice <alice@example.com>", ancestor)

    result = check_range(monkeypatch, base, head)

    assert result.returncode == 0
    assert result.stderr == ""


@pytest.mark.parametrize("base", ["missing-ref", ""])
def test_invalid_or_incomplete_range_fails(
    repository: None, monkeypatch: pytest.MonkeyPatch, base: str
) -> None:
    head = commit("Clean")

    result = check_range(monkeypatch, base, head)

    assert result.returncode == 1


def test_local_hook_without_range_is_noop(repository: None) -> None:
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--check"], capture_output=True, text=True
    )

    assert result.returncode == 0
