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

import os
import shutil
import subprocess
import sys

from dimup.development import clone, develop
from dimup.process import Runner, SetupError
import pytest


def git(root, *args):
    return subprocess.check_output(["git", "-C", str(root), *args], text=True).strip()


@pytest.fixture
def upstream(tmp_path, monkeypatch):
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", os.devnull)
    monkeypatch.setenv("GIT_AUTHOR_NAME", "Test")
    monkeypatch.setenv("GIT_AUTHOR_EMAIL", "test@example.invalid")
    monkeypatch.setenv("GIT_COMMITTER_NAME", "Test")
    monkeypatch.setenv("GIT_COMMITTER_EMAIL", "test@example.invalid")
    root = tmp_path / "upstream"
    root.mkdir()
    git(root, "init", "--initial-branch=main")
    (root / "pyproject.toml").write_text(
        '[project]\nname="dimos"\n[project.optional-dependencies]\n'
        "all=[]\nspot=[]\ngraspgenx=[]\ndds=[]\nunitree-dds=[]\n"
    )
    (root / ".gitignore").write_text(".venv/\n.dimos/\n.envrc\n")
    git(root, "add", ".")
    git(root, "commit", "-m", "initial")
    git(root, "tag", "v1")
    (root / "change.txt").write_text("latest")
    git(root, "add", ".")
    git(root, "commit", "-m", "latest")
    git(root, "branch", "feat/test")
    monkeypatch.setattr("dimup.development.SDK_URL", str(root))
    return root


@pytest.mark.parametrize("ref", ["main", "feat/test", "v1", "commit"])
def test_clone_keeps_history_remotes_and_requested_revision(upstream, tmp_path, ref):
    selected = git(upstream, "rev-parse", "v1") if ref == "commit" else ref
    expected = git(upstream, "rev-parse", selected)
    root = tmp_path / "my checkout"
    result = clone(
        root, selected, Runner(tmp_path / "setup.log"), {**os.environ, "GIT_LFS_SKIP_SMUDGE": "1"}
    )
    assert result == expected
    assert git(root, "remote", "get-url", "origin") == str(upstream)
    assert git(root, "rev-parse", "--is-shallow-repository") == "false"
    assert git(root, "branch", "--show-current") == (ref if ref in {"main", "feat/test"} else "")
    assert git(root, "status", "--porcelain") == ""


def test_lfs_clone_skips_download_but_explicit_pull_works(upstream, tmp_path):
    if shutil.which("git-lfs") is None:
        pytest.skip("Git LFS is exercised in CI")
    git(upstream, "lfs", "install", "--local")
    git(upstream, "lfs", "track", "*.bin")
    content = b"local LFS fixture\n" * 100
    (upstream / "sample.bin").write_bytes(content)
    git(upstream, "add", ".")
    git(upstream, "commit", "-m", "data")
    root = tmp_path / "clone"
    env = {**os.environ, "GIT_LFS_SKIP_SMUDGE": "1"}
    clone(root, "main", Runner(tmp_path / "setup.log"), env)
    assert (
        (root / "sample.bin").read_text().startswith("version https://git-lfs.github.com/spec/v1")
    )
    subprocess.run(
        ["git", "-C", str(root), "lfs", "pull"], env=env, check=True, capture_output=True
    )
    assert (root / "sample.bin").read_bytes() == content


@pytest.mark.parametrize("kind", ["nonempty", "file", "symlink"])
def test_reject_destination_without_touching_it(tmp_path, kind):
    target = tmp_path / "existing"
    target.mkdir()
    note = target / "notes"
    note.write_text("keep")
    root = target
    if kind == "file":
        root = note
    elif kind == "symlink":
        root = tmp_path / "link"
        root.symlink_to(target, target_is_directory=True)
    with pytest.raises(SetupError, match="new or empty"):
        develop(root, "main")
    assert list(target.iterdir()) == [note]
    assert note.read_text() == "keep"


@pytest.fixture
def commands(monkeypatch):
    calls = []
    run = Runner.run

    def capture_install(self, stage, command, **kwargs):
        if stage in {
            "Install development dependencies · uv sync",
            "Install commit hooks",
            "Verify contributor checkout",
        }:
            calls.append((stage, command, kwargs))
            return ""
        return run(self, stage, command, **kwargs)

    monkeypatch.setattr(Runner, "run", capture_install)
    # All system provisioning is outside this test; Git remains a real subprocess.
    monkeypatch.setattr(
        "dimup.development.executable",
        lambda name: shutil.which("git")
        if name == "git"
        else f"/tools/{name}"
        if name != "deno"
        else pytest.fail("Deno is optional"),
    )
    return calls


def test_develop_prepares_empty_checkout_without_changing_tracked_files(
    upstream, tmp_path, commands, capsys
):
    root = tmp_path / "dimos"
    root.mkdir()
    develop(root, "main")
    stage, command, kwargs = commands[0]
    assert command == [
        "/tools/uv",
        "sync",
        "--locked",
        "--python",
        "3.12",
        "--no-default-groups",
        "--group",
        "tests",
        "--group",
        "lint",
        "--extra",
        "all",
        "--extra",
        "graspgenx",
        "--extra",
        "spot",
    ]
    assert kwargs["env"]["UV_PYTHON_PREFERENCE"] == "only-managed"
    assert kwargs["env"]["GIT_LFS_SKIP_SMUDGE"] == "1"
    assert commands[1][1][-4:] == ["--hook-type", "pre-commit", "--hook-type", "commit-msg"]
    assert commands[2][2]["cwd"] != root
    assert "-I" in commands[2][1]
    assert (root / ".dimos/activate.sh").is_file()
    assert (root / ".envrc").is_file()
    assert "git clone" in (root / ".dimos/setup.log").read_text()
    assert git(root, "status", "--porcelain") == ""
    assert "Ready · dimos" in capsys.readouterr().out


@pytest.mark.parametrize("failure", ["bad-ref", "install", "interrupt"])
def test_failure_keeps_checkout_and_log_without_ready_message(
    upstream, tmp_path, commands, monkeypatch, capsys, failure
):
    run = Runner.run

    def fail_install(self, stage, command, **kwargs):
        if stage == "Install development dependencies · uv sync":
            if failure == "interrupt":
                raise KeyboardInterrupt
            raise SetupError("dependency failure")
        return run(self, stage, command, **kwargs)

    monkeypatch.setattr(Runner, "run", fail_install)
    root = tmp_path / "failed"
    with pytest.raises(KeyboardInterrupt if failure == "interrupt" else SetupError):
        develop(root, "does-not-exist" if failure == "bad-ref" else "main")
    assert (root / ".git").is_dir()
    assert (root / ".dimos/setup.log").is_file()
    assert "Ready ·" not in capsys.readouterr().out


def test_dev_requires_directory():
    result = subprocess.run([sys.executable, "-m", "dimup.cli", "dev"], capture_output=True)
    assert result.returncode == 2
