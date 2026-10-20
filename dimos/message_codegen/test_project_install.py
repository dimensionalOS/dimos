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

"""Real isolated pip source/wheel/sdist/editable acceptance for message authors."""

import os
from pathlib import Path
import shutil
import subprocess
from tempfile import TemporaryDirectory
import venv

import pytest

ROOT = Path(__file__).resolve().parents[2]


def run(python, *arguments, cwd, env):
    result = subprocess.run(
        [str(python), *arguments], check=False, cwd=cwd, env=env, capture_output=True, text=True
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout


@pytest.fixture(scope="module")
def installed_toolchain(tmp_path_factory):
    location = os.environ.get("DIMOS_EXTERNAL_WHEELHOUSE")
    if not location:
        pytest.skip("Set DIMOS_EXTERNAL_WHEELHOUSE to the complete review wheelhouse")
    wheelhouse = Path(location).resolve()
    parent = Path(os.environ.get("DIMOS_EXTERNAL_TEST_ROOT", tmp_path_factory.getbasetemp()))
    parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix="message-install-", dir=parent) as directory:
        folder = Path(directory)
        environment = folder / "venv"
        venv.EnvBuilder(with_pip=True).create(environment)
        python = environment / "bin/python"
        temporary = folder / "pip-tmp"
        temporary.mkdir()
        env = {
            **os.environ,
            "PIP_NO_INDEX": "1",
            "PIP_FIND_LINKS": str(wheelhouse),
            "TMPDIR": str(temporary),
        }
        env.pop("PYTHONPATH", None)
        run(python, "-m", "pip", "install", "dimos[message-build]==0.0.14", cwd=folder, env=env)
        forbidden = folder / "no-native-tools"
        forbidden.mkdir()
        for name in ("cmake", "cargo", "rustc", "c++", "g++", "cc"):
            script = forbidden / name
            script.write_text("#!/bin/sh\nexit 97\n")
            script.chmod(0o755)
        env.update(
            PATH=str(forbidden) + os.pathsep + str(environment / "bin") + os.pathsep + env["PATH"],
            CC="/bin/false",
            CXX="/bin/false",
        )
        yield python, env, folder


@pytest.mark.parametrize("mode", ["source", "wheel", "sdist", "editable"])
def test_clean_isolated_message_install_and_builtin_identity(installed_toolchain, tmp_path, mode):
    python, env, _ = installed_toolchain
    project = tmp_path / "project"
    shutil.copytree(ROOT / "examples/external-messages", project)
    source = project
    if mode == "wheel":
        run(
            python,
            "-m",
            "pip",
            "wheel",
            "--no-deps",
            "--wheel-dir",
            str(tmp_path / "dist"),
            ".",
            cwd=project,
            env=env,
        )
        source = next((tmp_path / "dist").glob("acme_telemetry-*.whl"))
    elif mode == "sdist":
        run(
            python,
            "-c",
            "from dimos.message_codegen.backend import build_sdist; build_sdist('dist')",
            cwd=project,
            env=env,
        )
        source = next((project / "dist").glob("*.tar.gz"))
    run(
        python,
        "-m",
        "pip",
        "install",
        "--force-reinstall",
        "--no-deps",
        *(["-e"] if mode == "editable" else []),
        str(source),
        cwd=tmp_path,
        env=env,
    )
    run(
        python,
        "-I",
        "-c",
        """from importlib.metadata import entry_points
from acme_telemetry.telemetry_msgs.msg import DeviceReading
from acme_telemetry_schemas.provider import package_root
from dimos.messages.std_msgs.msg import Header
from dimos.messages.builtin_interfaces.msg import Time
from dimos_cdrgen.registry import encode, decode
value = DeviceReading(Header(Time(17, 23), "map"), 7, 2.5, "sensor")
assert type(value.header) is Header
assert decode(encode(value), DeviceReading) == value
assert type(decode(encode(value), DeviceReading).header) is Header
assert len([ep for ep in entry_points(group='dimos.messages') if ep.name == 'acme_telemetry']) == 1
assert (package_root() / 'schemas/std_msgs/msg/Header.msg').is_file()
assert (package_root() / 'native-project/Cargo.toml').is_file()
assert not (package_root() / 'cpp').exists()
assert not (package_root() / 'rust').exists()
""",
        cwd=tmp_path,
        env=env,
    )
    run(python, "-m", "pip", "check", cwd=tmp_path, env=env)
    if mode == "editable":
        definition = project / "interfaces/telemetry_msgs/msg/DeviceReading.msg"
        definition.write_text(definition.read_text() + "uint32 status\n")
        run(python, "-m", "pip", "install", "--no-deps", "-e", str(project), cwd=tmp_path, env=env)
        run(
            python,
            "-I",
            "-c",
            """from acme_telemetry.telemetry_msgs.msg import DeviceReading
from dimos.messages.std_msgs.msg import Header
from dimos.messages.builtin_interfaces.msg import Time
value = DeviceReading(Header(Time(0, 0), 'map'), 7, 2.5, 'sensor', 9)
assert value.status == 9
""",
            cwd=tmp_path,
            env=env,
        )
        definition.rename(definition.with_name("RenamedReading.msg"))
        run(python, "-m", "pip", "install", "--no-deps", "-e", str(project), cwd=tmp_path, env=env)
        run(
            python,
            "-I",
            "-c",
            """from acme_telemetry.telemetry_msgs import msg
assert hasattr(msg, 'RenamedReading')
assert not hasattr(msg, 'DeviceReading')
""",
            cwd=tmp_path,
            env=env,
        )
        assert not (project / "build/messages/cpp").exists()
        assert not (project / "build/messages/rust").exists()
