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

import pytest

from dimos_build import ensure_web_dist


def test_bundled_assets_need_no_deno(tmp_path, monkeypatch):
    for project, artifact in (("sdk", "sdk.js"), ("cockpit", "index.html")):
        dist = tmp_path / "web" / project / "dist"
        dist.mkdir(parents=True)
        (dist / artifact).write_text("built")
    monkeypatch.setattr("shutil.which", lambda name: None)
    ensure_web_dist(tmp_path)


def test_missing_deno_explains_machine_setup(tmp_path, monkeypatch):
    monkeypatch.setattr("shutil.which", lambda name: None)
    with pytest.raises(RuntimeError, match="dimup setup"):
        ensure_web_dist(tmp_path)


def test_source_build_produces_both_bundles(tmp_path, monkeypatch):
    commands = []

    def build(command, *, check):
        commands.append(command)
        project = Path(command[3])
        dist = project / "dist"
        dist.mkdir(parents=True)
        (dist / ("sdk.js" if project.name == "sdk" else "index.html")).write_text("built")

    monkeypatch.setattr("shutil.which", lambda name: "/bin/deno")
    monkeypatch.setattr(subprocess, "run", build)
    ensure_web_dist(tmp_path)
    assert [Path(command[3]).name for command in commands] == ["sdk", "cockpit"]
