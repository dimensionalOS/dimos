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
import runpy
import shutil
import subprocess
import sys

import pytest
from setuptools import Distribution
from setuptools.command.build_py import build_py as setuptools_build_py


@pytest.mark.parametrize("bundled", [False, True])
def test_python_build_copies_optional_assets_without_frontend_tools(tmp_path, monkeypatch, bundled):
    repository = Path(__file__).resolve().parents[2]
    shutil.copy(repository / "setup.py", tmp_path / "setup.py")
    source = tmp_path / "web"
    (source / "relay").mkdir(parents=True)
    (source / "relay/main.ts").write_text("relay")
    (source / "deno.json").write_text("{}")
    if bundled:
        for project, artifact in (("sdk", "sdk.js"), ("cockpit", "index.html")):
            dist = source / project / "dist"
            dist.mkdir(parents=True)
            (dist / artifact).write_text("prebuilt")
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(sys, "path", sys.path.copy())
    monkeypatch.setattr("setuptools.setup", lambda **kwargs: None)
    namespace = runpy.run_path(str(tmp_path / "setup.py"))
    monkeypatch.setattr(setuptools_build_py, "run", lambda self: None)
    monkeypatch.setitem(
        namespace["build_py"].run.__globals__, "bundle_native_sources", lambda *args: None
    )

    def unexpected_command(*args, **kwargs):
        pytest.fail("Python packaging must not invoke frontend tools")

    monkeypatch.setattr(subprocess, "run", unexpected_command)
    command = namespace["build_py"](Distribution())
    command.build_lib = str(tmp_path / "build")
    command.run()
    packaged = tmp_path / "build/dimos/web/relay_bridge/_relay_dist"
    assert (packaged / "relay/main.ts").read_text() == "relay"
    assert (packaged / "cockpit/dist/index.html").exists() is bundled
    assert (packaged / "sdk/dist/sdk.js").exists() is bundled
    assert "run" not in namespace["sdist"].__dict__
