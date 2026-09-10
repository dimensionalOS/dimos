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

from hashlib import sha256
import io
import shutil
import tarfile
import os
from pathlib import Path
import subprocess


def bootstrap(tmp_path, valid):
    template = Path(__file__).resolve().parents[2] / "bootstrap.sh"
    bins = tmp_path / "bin"
    bins.mkdir()
    (bins / "curl").write_text(
        '#!/bin/sh\nwhile [ "$1" != "--output" ]; do shift; done\nprintf wheel > "$2"\n'
    )
    (bins / "uv").write_text(
        '#!/bin/sh\nif [ "$1 $2" = "tool dir" ]; then echo "$TEST_BIN"; else echo "$*" >> "$TEST_LOG"; fi\n'
    )
    (bins / "dimup").write_text('#!/bin/sh\necho "dimup $*" >> "$TEST_LOG"\n')
    for path in bins.iterdir():
        path.chmod(0o755)
    script = tmp_path / "bootstrap.sh"
    digest = sha256(b"wheel").hexdigest() if valid else "0" * 64
    script.write_text(
        template.read_text()
        .replace("@DIMUP_WHEEL_URL@", "https://example.com/dimup-0.1.0-py3-none-any.whl")
        .replace("@DIMUP_WHEEL_SHA@", digest)
    )
    log = tmp_path / "commands"
    env = {
        **os.environ,
        "PATH": f"{bins}:/usr/bin:/bin",
        "TEST_BIN": str(bins),
        "TEST_LOG": str(log),
    }
    result = subprocess.run(
        ["bash", str(script)],
        env=env,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        check=False,
    )
    return result, log


def test_bootstrap_installs_verified_wheel_and_runs_setup(tmp_path):
    result, log = bootstrap(tmp_path, True)
    assert result.returncode == 0, result.stderr
    assert "tool install --force --python 3.12" in log.read_text()
    assert "dimup setup" in log.read_text()


def test_bootstrap_rejects_wrong_checksum_before_installing(tmp_path):
    result, log = bootstrap(tmp_path, False)
    assert result.returncode != 0
    assert "checksum mismatch" in result.stderr
    assert not log.exists()


def test_repository_bootstrap_installs_uv_then_dimup_from_source(tmp_path):
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
    (bins / "curl").write_text(
        '#!/bin/sh\ncase "$*" in\n'
        '*astral.sh*) cat "$TEST_UV_INSTALLER" ;;\n'
        '*) while [ "$1" != "--output" ]; do shift; done; cp "$TEST_ARCHIVE" "$2" ;;\n'
        'esac\n'
    )
    (bins / "curl").chmod(0o755)
    uv = tmp_path / "uv"
    uv.write_text(
        '#!/bin/sh\nif [ "$1 $2" = "tool dir" ]; then echo "$HOME/.local/bin"; '
        'else for arg do last=$arg; done; test -f "$last/pyproject.toml" || exit 1; '
        'echo "install dimup" >> "$TEST_LOG"; fi\n'
    )
    uv.chmod(0o755)
    dimup = tmp_path / "dimup"
    dimup.write_text('#!/bin/sh\necho "dimup $*" >> "$TEST_LOG"\n')
    dimup.chmod(0o755)
    installer = tmp_path / "uv-install.sh"
    installer.write_text(
        'mkdir -p "$HOME/.local/bin"\n'
        'cp "$TEST_UV" "$TEST_DIMUP" "$HOME/.local/bin/"\n'
        'echo "install uv" >> "$TEST_LOG"\n'
    )
    log = tmp_path / "commands"
    result = subprocess.run(
        [str(bins / "bash"), str(Path(__file__).resolve().parents[2] / "bootstrap.sh")],
        env={
            **os.environ,
            "PATH": str(bins),
            "HOME": str(tmp_path / "home"),
            "TEST_UV_INSTALLER": str(installer),
            "TEST_ARCHIVE": str(archive),
            "TEST_UV": str(uv),
            "TEST_DIMUP": str(dimup),
            "TEST_LOG": str(log),
        },
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert log.read_text().splitlines() == ["install uv", "install dimup", "dimup setup"]
