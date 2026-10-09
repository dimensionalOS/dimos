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
from pathlib import Path
import subprocess
import sys
import tarfile

import pytest


def test_clean_source_wheel_sdist_and_editable_install(tmp_path):
    wheelhouse = os.environ.get("DIMOS_MESSAGE_WHEELHOUSE")
    if not wheelhouse:
        pytest.skip("Set DIMOS_MESSAGE_WHEELHOUSE for isolated install acceptance")
    wheelhouse = str(Path(wheelhouse).resolve())
    environment = {key: value for key, value in os.environ.items() if key != "PYTHONPATH"}
    environment.update(
        PIP_NO_INDEX="1", PIP_FIND_LINKS=wheelhouse, CC="/bin/false", CXX="/bin/false"
    )
    tools = tmp_path / "tools"
    tools.mkdir()
    for name in ("cargo", "rustc", "cmake", "c++"):
        executable = tools / name
        executable.write_text("#!/bin/sh\nexit 97\n")
        executable.chmod(0o755)
    environment["PATH"] = str(tools) + os.pathsep + environment["PATH"]
    venv = tmp_path / "venv"
    subprocess.run([sys.executable, "-m", "venv", str(venv)], check=True)
    python = str(venv / "bin/python")
    app = tmp_path / "app"
    definition = app / "interfaces/story_msgs/msg/DeviceReading.msg"
    definition.parent.mkdir(parents=True)
    definition.write_text("std_msgs/Header header\nfloat64 value\n")
    (app / "pyproject.toml").write_text("""[build-system]
requires = ["dimos-message-build==0.1.0"]
build-backend = "dimos_message_build.backend"
[project]
name = "story-messages"
version = "0.1.0"
[tool.dimos.messages]
languages = ["python", "cpp", "rust"]
""")

    def pip(*args):
        subprocess.run([python, "-m", "pip", *args], cwd=tmp_path, env=environment, check=True)

    def verify(code):
        subprocess.run([python, "-I", "-c", code], cwd=tmp_path, env=environment, check=True)

    pip("install", str(app))
    verify("""
from importlib.metadata import distributions
from dimos_generated.std_msgs.msg import Header
from story_messages.story_msgs.msg import DeviceReading
message = DeviceReading(header=Header(frame_id='sensor'), value=20.5)
assert type(message.header) is Header
assert DeviceReading.decode(message.encode()).value == 20.5
assert 'dimos' not in {dist.metadata['Name'] for dist in distributions()}
""")
    assert not list((app / "build").rglob("target"))
    # Produce a wheel, then prove its install needs no compiler or backend.
    dist = tmp_path / "dist"
    pip("wheel", "--no-deps", "--wheel-dir", str(dist), str(app))
    pip("uninstall", "-y", "story-messages")
    wheel = next(dist.glob("story_messages-*.whl"))
    old_path = environment["PATH"]
    environment["PATH"] = str(venv / "bin")
    pip("install", "--only-binary=:all:", str(wheel))
    verify(
        "from story_messages.story_msgs.msg import DeviceReading; assert DeviceReading(value=7).value == 7"
    )
    environment["PATH"] = old_path
    # Source distribution builds outside the original project and has no generated output.
    pip("install", "dimos-message-build==0.1.0")
    subprocess.run(
        [
            python,
            "-c",
            "from dimos_message_build.backend import build_sdist; build_sdist('../sdist')",
        ],
        cwd=app,
        env=environment,
        check=True,
    )
    source = next((tmp_path / "sdist").glob("*.tar.gz"))
    with tarfile.open(source) as archive:
        assert not any("/build/" in name for name in archive.getnames())
    pip("install", "--force-reinstall", str(source))
    verify(
        "from story_messages.story_msgs.msg import DeviceReading; assert DeviceReading(value=8).value == 8"
    )
    pip("install", "-e", str(app))
    definition.write_text(definition.read_text() + 'string unit "C"\n')
    pip("install", "-e", str(app))
    verify(
        "from story_messages.story_msgs.msg import DeviceReading; assert DeviceReading().unit == 'C'"
    )
    definition.rename(definition.with_name("RenamedReading.msg"))
    pip("install", "-e", str(app))
    verify(
        "import story_messages.story_msgs.msg as m; assert not hasattr(m, 'DeviceReading'); assert m.RenamedReading().unit == 'C'"
    )
