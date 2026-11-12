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
import shutil
import subprocess

import pytest


@pytest.mark.parametrize("relative_root", [False, True])
def test_graph_smoke_check_runs_in_installed_project(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, relative_root: bool
) -> None:
    scripts = tmp_path / "checkout" / "scripts"
    scripts.mkdir(parents=True)
    shutil.copyfile(Path(__file__).with_name("test-install.sh"), scripts / "test-install.sh")
    # Replace installation with a no-op; this test only exercises the smoke harness.
    (scripts / "install.sh").write_text("exit 0\n")
    install_root = tmp_path / "installed project"
    project = install_root / "library"
    bin_dir = project / ".venv" / "bin"
    bin_dir.mkdir(parents=True)
    cli = bin_dir / "dimos"
    # Record the working directory at the requested graph output path.
    cli.write_text('#!/bin/bash\nset -eu\nprintf "%s\\n" "$PWD" > "$4"\n')
    cli.chmod(0o755)
    monkeypatch.setenv(
        "INSTALL_TEST_ROOT", "../installed project" if relative_root else str(install_root)
    )

    subprocess.run(
        ["bash", str(scripts / "test-install.sh"), "library"],
        cwd=scripts.parent,
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    )

    assert (install_root / "logs" / "graph.svg").read_text().strip() == str(project)
