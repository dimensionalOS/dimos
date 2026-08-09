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

"""Getting the pinned scorer, without reaching the network in the default tier."""

from pathlib import Path
import subprocess

import pytest

import dimos.benchmark.space_qa.source as source_module
from dimos.benchmark.space_qa.source import SPACE_SOURCE_ENV, ensure_space_source, head_revision

ABSENT_REVISION = "0" * 40


def _checkout(path: Path) -> str:
    """A real one-commit repository, so the revision check runs against real git."""
    path.mkdir(parents=True, exist_ok=True)
    _git("init", "--quiet", "-b", "main", cwd=path)
    (path / "README.md").write_text("space", encoding="utf-8")
    _git("add", "README.md", cwd=path)
    _git(
        "-c",
        "user.email=tests@example.com",
        "-c",
        "user.name=tests",
        "-c",
        "commit.gpgsign=false",
        "commit",
        "--quiet",
        "--no-verify",
        "-m",
        "initial",
        cwd=path,
    )
    return head_revision(path)


def _git(*args: str, cwd: Path) -> None:
    subprocess.run(["git", *args], cwd=cwd, capture_output=True, check=True)


def test_an_override_at_the_pin_is_used_as_is(tmp_path, monkeypatch) -> None:
    checkout = tmp_path / "ml-space-benchmark"
    monkeypatch.setattr(source_module, "SPACE_REVISION", _checkout(checkout))
    monkeypatch.setenv(SPACE_SOURCE_ENV, str(checkout))

    assert ensure_space_source() == checkout.resolve()


def test_an_override_off_the_pin_names_both_revisions(tmp_path, monkeypatch) -> None:
    """A different scorer produces a different score, so the mismatch has to be loud."""
    checkout = tmp_path / "ml-space-benchmark"
    head = _checkout(checkout)
    monkeypatch.setattr(source_module, "SPACE_REVISION", ABSENT_REVISION)
    monkeypatch.setenv(SPACE_SOURCE_ENV, str(checkout))

    with pytest.raises(RuntimeError) as raised:
        ensure_space_source()
    assert head in str(raised.value)
    assert ABSENT_REVISION in str(raised.value)


def test_an_override_at_the_pin_with_an_edited_file_is_refused(tmp_path, monkeypatch) -> None:
    """`git checkout` moves HEAD and leaves edits alone, so the pin alone proves nothing."""
    checkout = tmp_path / "ml-space-benchmark"
    monkeypatch.setattr(source_module, "SPACE_REVISION", _checkout(checkout))
    monkeypatch.setenv(SPACE_SOURCE_ENV, str(checkout))
    (checkout / "README.md").write_text("space, edited", encoding="utf-8")

    with pytest.raises(RuntimeError) as raised:
        ensure_space_source()
    message = str(raised.value)
    assert "README.md" in message
    assert "working tree has been changed" in message
    # The override is the user's own checkout: never advise deleting it.
    assert f"unset {SPACE_SOURCE_ENV}" in message
    assert "delete the checkout" not in message


def test_a_cached_checkout_with_an_untracked_file_is_refused(tmp_path, monkeypatch) -> None:
    """A module dropped beside the scorer changes what imports, and edits nothing tracked."""
    cached = tmp_path / "cache" / "src" / "ml-space-benchmark"
    monkeypatch.setattr(source_module, "SPACE_REVISION", _checkout(cached))
    monkeypatch.setattr(source_module, "space_cache_root", lambda: tmp_path / "cache")
    monkeypatch.delenv(SPACE_SOURCE_ENV, raising=False)
    (cached / "space_patch.py").write_text("# not upstream's\n", encoding="utf-8")

    with pytest.raises(RuntimeError) as raised:
        ensure_space_source()
    message = str(raised.value)
    assert "space_patch.py" in message
    # The cached clone is disposable, so recloning is safe advice here.
    assert "delete the checkout and let the next run clone it again" in message


def test_an_override_that_is_not_a_checkout_is_refused(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv(SPACE_SOURCE_ENV, str(tmp_path / "nowhere"))

    with pytest.raises(FileNotFoundError, match=SPACE_SOURCE_ENV):
        ensure_space_source()


def test_a_cached_checkout_at_the_pin_is_reused_without_cloning(tmp_path, monkeypatch) -> None:
    cached = tmp_path / "cache" / "src" / "ml-space-benchmark"
    monkeypatch.setattr(source_module, "SPACE_REVISION", _checkout(cached))
    monkeypatch.setattr(source_module, "space_cache_root", lambda: tmp_path / "cache")
    monkeypatch.setattr(
        source_module, "_clone", lambda _checkout: pytest.fail("cloned an existing checkout")
    )
    monkeypatch.delenv(SPACE_SOURCE_ENV, raising=False)

    assert ensure_space_source() == cached


def test_a_missing_checkout_is_cloned_once(tmp_path, monkeypatch) -> None:
    cached = tmp_path / "cache" / "src" / "ml-space-benchmark"
    cloned = []
    monkeypatch.setattr(source_module, "space_cache_root", lambda: tmp_path / "cache")
    monkeypatch.setattr(source_module, "_clone", cloned.append)
    monkeypatch.delenv(SPACE_SOURCE_ENV, raising=False)

    assert ensure_space_source() == cached
    assert cloned == [cached]


def test_a_failing_git_invocation_reports_what_it_ran(tmp_path) -> None:
    with pytest.raises(RuntimeError, match="rev-parse HEAD` failed"):
        head_revision(tmp_path / "nowhere")
