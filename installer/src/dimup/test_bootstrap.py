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

import io
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

import pytest


def bootstrap(tmp_path, *, ref=None, uv_present=False, failure=""):
    bins = tmp_path / "bin"
    bins.mkdir()
    for name in ("bash", "sh", "env", "cat", "cp", "mkdir", "mktemp", "rm", "tar", "gzip"):
        (bins / name).symlink_to(shutil.which(name))
    archive = tmp_path / "source.tar.gz"
    with tarfile.open(archive, "w:gz") as source:
        content = b"[project]\nname = 'dimup'\n"
        info = tarfile.TarInfo("dimos-source/installer/pyproject.toml")
        info.size = len(content)
        source.addfile(info, io.BytesIO(content))
    if failure == "extract":
        archive.write_bytes(b"invalid archive")
    (bins / "curl").write_text(
        '#!/bin/sh\ncase "$*" in\n'
        '*astral.sh*) cat "$TEST_UV_INSTALLER" ;;\n'
        '*) echo "$*" >> "$TEST_URLS"; [ "$TEST_FAILURE" != download ] || exit 9; '
        'while [ "$1" != "--output" ]; do shift; done; cp "$TEST_ARCHIVE" "$2" ;;\n'
        "esac\n"
    )
    (bins / "curl").chmod(0o755)
    uv = tmp_path / "uv"
    uv.write_text(
        '#!/bin/sh\nif [ "$1 $2" = "tool dir" ]; then echo "$HOME/.local/bin"; '
        'else for arg do last=$arg; done; test -f "$last/pyproject.toml" || exit 1; '
        'echo "install dimup" >> "$TEST_LOG"; '
        '[ "$TEST_FAILURE" != install ] || exit 10; fi\n'
    )
    uv.chmod(0o755)
    dimup = tmp_path / "dimup"
    dimup.write_text(
        '#!/bin/sh\necho "dimup $*" >> "$TEST_LOG"\n[ "$TEST_FAILURE" != setup ] || exit 11\n'
    )
    dimup.chmod(0o755)
    home_bin = tmp_path / "home/.local/bin"
    home_bin.mkdir(parents=True)
    shutil.copy(dimup, home_bin / "dimup")
    if uv_present:
        shutil.copy(uv, bins / "uv")
    installer = tmp_path / "uv-install.sh"
    installer.write_text(
        'mkdir -p "$HOME/.local/bin"\n'
        'cp "$TEST_UV" "$TEST_DIMUP" "$HOME/.local/bin/"\n'
        'echo "install uv" >> "$TEST_LOG"\n'
    )
    log = tmp_path / "commands"
    env = {
        **os.environ,
        "PATH": str(bins),
        "HOME": str(tmp_path / "home"),
        "TMPDIR": str(tmp_path),
        "TEST_UV_INSTALLER": str(installer),
        "TEST_ARCHIVE": str(archive),
        "TEST_UV": str(uv),
        "TEST_DIMUP": str(dimup),
        "TEST_LOG": str(log),
        "TEST_URLS": str(tmp_path / "urls"),
        "TEST_FAILURE": failure,
    }
    env.pop("DIMUP_REF", None)
    if ref is not None:
        env["DIMUP_REF"] = ref
    result = subprocess.run(
        [str(bins / "bash"), str(Path(__file__).resolve().parents[2] / "bootstrap.sh")],
        env=env,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        check=False,
    )
    commands = log.read_text().splitlines() if log.exists() else []
    assert list(tmp_path.glob("tmp.*")) == []
    return result, commands, (tmp_path / "urls").read_text()


@pytest.mark.parametrize("ref", [None, "feat/test-installer", "a" * 40])
@pytest.mark.parametrize("uv_present", [False, True])
def test_source_bootstrap_selects_ref_and_prepares_machine(tmp_path, ref, uv_present):
    result, commands, urls = bootstrap(tmp_path, ref=ref, uv_present=uv_present)
    assert result.returncode == 0, result.stderr
    assert commands == ([] if uv_present else ["install uv"]) + ["install dimup", "dimup setup"]
    assert f"https://codeload.github.com/dimensionalOS/dimos/tar.gz/{ref or 'main'}" in urls
    assert "Installed dimup" in result.stdout


@pytest.mark.parametrize(
    ("failure", "commands"),
    [
        ("download", []),
        ("extract", []),
        ("install", ["install dimup"]),
        ("setup", ["install dimup", "dimup setup"]),
    ],
)
def test_bootstrap_stops_at_failure(tmp_path, failure, commands):
    result, actual, _ = bootstrap(tmp_path, uv_present=True, failure=failure)
    assert result.returncode != 0
    assert actual == commands
    assert "Installed dimup" not in result.stdout
