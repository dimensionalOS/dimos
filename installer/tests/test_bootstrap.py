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

import hashlib
import os
from pathlib import Path
import shlex
import subprocess

import pytest

BOOTSTRAP = Path(__file__).resolve().parents[1] / "bootstrap.sh"


@pytest.mark.parametrize("arch", ["x86_64", "aarch64"])
@pytest.mark.parametrize("corrupt", [False, True])
def test_bootstrap_verifies_forwards_arguments_and_cleans_up(tmp_path, arch, corrupt):
    assets = tmp_path / "assets"
    assets.mkdir()
    output = tmp_path / "arguments"
    binary = assets / f"create-dimos-linux-{arch}"
    binary.write_text(f"#!/bin/sh\nprintf '%s\\n' \"$@\" > {shlex.quote(str(output))}\n")
    digest = hashlib.sha256(binary.read_bytes()).hexdigest()
    (assets / "SHA256SUMS").write_text(f"{'0' * 64 if corrupt else digest}  {binary.name}\n")
    commands = tmp_path / "commands"
    commands.mkdir()
    (commands / "uname").write_text(
        f'#!/bin/sh\nif [ "$1" = -s ]; then echo Linux; else echo {arch}; fi\n'
    )
    (commands / "curl").write_text(f"""#!/bin/bash
set -e
for arg in "$@"; do
    if [[ "$arg" == https:* ]]; then asset=${{arg##*/}}; fi
    destination=$arg
done
cp {shlex.quote(str(assets))}/"$asset" "$destination"
""")
    for command in commands.iterdir():
        command.chmod(0o755)
    script = tmp_path / "create.sh"
    script.write_text(BOOTSTRAP.read_text().replace("@DIMOS_RELEASE@", "v0.1.0"))
    temporary = tmp_path / "temporary"
    temporary.mkdir()
    env = {**os.environ, "PATH": f"{commands}:{os.environ['PATH']}", "TMPDIR": str(temporary)}
    result = subprocess.run(
        ["bash", str(script), "project with spaces", "$(touch unwanted)"],
        env=env,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        start_new_session=True,
        check=False,
    )
    assert list(temporary.iterdir()) == []
    if corrupt:
        assert result.returncode != 0
        assert not output.exists()
    else:
        assert result.returncode == 0, result.stderr
        assert output.read_text().splitlines() == ["project with spaces", "$(touch unwanted)"]
