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

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import sys
from threading import Barrier
from unittest.mock import Mock

from filelock import FileLock, Timeout
from git import Actor, Repo
import pytest
from pytest import MonkeyPatch
from pytest_mock import MockerFixture

from dimos.simulation.mujoco import menagerie

PINNED_COMMIT = "1b86ece576591213e2b666ebf59508454200ca97\n"


@pytest.fixture
def checkout(tmp_path: Path, monkeypatch: MonkeyPatch) -> Path:
    path = tmp_path / "external_deps" / "mujoco_menagerie"
    monkeypatch.setattr(menagerie, "menagerie_path", lambda: path)
    return path


@pytest.fixture
def download(checkout: Path, monkeypatch: MonkeyPatch, mocker: MockerFixture) -> Mock:
    """Stands in for the mujoco_playground helper, which skips a directory that exists."""

    def clone() -> None:
        if checkout.exists():
            return
        (checkout / ".git").mkdir(parents=True)
        (checkout / ".git" / "HEAD").write_text(PINNED_COMMIT)

    helper = mocker.Mock(side_effect=clone)
    monkeypatch.setitem(
        sys.modules,
        "mujoco_playground._src",
        mocker.Mock(mjx_env=mocker.Mock(ensure_menagerie_exists=helper)),
    )
    return helper


def test_downloads_once(download: Mock, monkeypatch: MonkeyPatch) -> None:
    menagerie.ensure_menagerie()
    # A finished checkout must not reach the slow mujoco_playground import.
    monkeypatch.setitem(sys.modules, "mujoco_playground._src", None)
    menagerie.ensure_menagerie()

    download.assert_called_once_with()


@pytest.mark.parametrize(
    "head", [None, "ref: refs/heads/main\n"], ids=["no-git-directory", "on-a-branch"]
)
def test_downloads_again_after_unfinished_download(
    checkout: Path, download: Mock, head: str | None
) -> None:
    leftover = checkout / "unitree_go1" / "go1.xml"
    leftover.parent.mkdir(parents=True)
    leftover.write_text("partial")
    if head:
        (checkout / ".git").mkdir()
        (checkout / ".git" / "HEAD").write_text(head)

    menagerie.ensure_menagerie()

    download.assert_called_once_with()
    assert not leftover.exists()
    assert (checkout / ".git" / "HEAD").read_text() == PINNED_COMMIT


def test_serializes_concurrent_downloads(
    checkout: Path, download: Mock, monkeypatch: MonkeyPatch
) -> None:
    barrier = Barrier(4)
    clone = download.side_effect

    def shared_path() -> Path:
        barrier.wait(timeout=5)
        return checkout

    def locked_clone() -> None:
        with pytest.raises(Timeout), FileLock(checkout.with_suffix(".lock"), timeout=0):
            pass
        clone()

    monkeypatch.setattr(menagerie, "menagerie_path", shared_path)
    download.side_effect = locked_clone
    with ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(lambda _: menagerie.ensure_menagerie(), range(4)))

    download.assert_called_once_with()


@pytest.mark.self_hosted  # needs mujoco_playground, which the default test group leaves out
def test_playground_helper_finishes_on_a_detached_head(tmp_path: Path) -> None:
    """ensure_menagerie relies on this to tell a finished download from a stopped one."""
    author = Actor("Test Author", "test@example.com")
    with Repo.init(tmp_path / "source") as repo:
        commit = repo.index.commit("Models", author=author, committer=author, skip_hooks=True)
    checkout = tmp_path / "mujoco_menagerie"

    # In a subprocess, because importing mujoco_playground loads torch and jax.
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys\n"
            "from mujoco_playground._src import mjx_env\n"
            "mjx_env._clone_with_progress(*sys.argv[1:])",
            str(tmp_path / "source"),
            str(checkout),
            commit.hexsha,
        ],
        check=True,
    )

    assert menagerie._is_cloned(checkout)
