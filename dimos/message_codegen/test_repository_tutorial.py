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
import re
import shutil
import subprocess
import sys

import pytest

try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10
    import tomli as tomllib


def test_builtin_tutorial_builds_new_definition_without_runtime(tmp_path):
    wheelhouse = os.environ.get("DIMOS_MESSAGE_WHEELHOUSE")
    if not wheelhouse:
        pytest.skip("Set message wheelhouse for clean source acceptance")
    root = Path(__file__).resolve().parents[2]
    tutorial = (root / "docs/development/messages-in-repository.md").read_text()
    definition = re.search(r"<!-- builtin-message: [^>]+ -->\n```text\n(.*?)```", tutorial, re.S)
    check = re.search(r"<!-- builtin-value-check -->\n```python skip\n(.*?)```", tutorial, re.S)
    assert definition is not None and check is not None
    checkout = tmp_path / "checkout"
    ignore = shutil.ignore_patterns("build", "dist", "*.egg-info", "__pycache__", "target")
    shutil.copytree(
        root / "dimos/message_codegen", checkout / "dimos/message_codegen", ignore=ignore
    )
    (checkout / "dimos/__init__.py").write_text("")
    package = checkout / "packages/dimos-generated"
    shutil.copytree(root / "packages/dimos-generated", package, ignore=ignore)
    source = checkout / "dimos/message_codegen/schemas/dimos_msgs/msg/DeviceReading.msg"
    assert not source.exists()
    source.write_text(definition.group(1))
    environment = {key: value for key, value in os.environ.items() if key != "PYTHONPATH"}
    environment.update(
        PATH=os.pathsep.join(
            directory
            for directory in os.environ["PATH"].split(os.pathsep)
            if Path(directory) != Path(sys.executable).parent
        ),
        PIP_NO_INDEX="1",
        PIP_FIND_LINKS=str(Path(wheelhouse).resolve()),
        CC="/bin/false",
        CXX="/bin/false",
    )
    (checkout / "scripts").mkdir()
    shutil.copyfile(
        root / "scripts/generate_builtin_messages.py",
        checkout / "scripts/generate_builtin_messages.py",
    )
    shutil.copyfile(root / "pyproject.toml", checkout / "pyproject.toml")
    subprocess.run(
        [sys.executable, "-m", "scripts.generate_builtin_messages"],
        check=True,
        cwd=checkout,
        env=environment,
    )
    venv = tmp_path / "venv"
    subprocess.run([sys.executable, "-m", "venv", str(venv)], check=True)
    python = str(venv / "bin/python")
    dist = tmp_path / "dist"
    uv = shutil.which("uv")
    assert uv is not None, "Prepare uv before running the repository tutorial"
    subprocess.run(
        [
            uv,
            "build",
            str(package),
            "--python",
            python,
            "--out-dir",
            str(dist),
            "--no-index",
            "--find-links",
            str(Path(wheelhouse).resolve()),
        ],
        cwd=tmp_path,
        env=environment,
        check=True,
    )
    wheel = next(dist.glob("dimos_generated-*.whl"))
    subprocess.run(
        [python, "-m", "pip", "install", str(wheel)],
        cwd=tmp_path,
        env=environment,
        check=True,
    )
    validation = (
        check.group(1)
        + """
from importlib.metadata import distributions
assert 'dimos' not in {dist.metadata['Name'] for dist in distributions()}
from dimos_message_build.registry import schema
assert 'MSG: std_msgs/Header' in schema(DeviceReading.__msgtype__)
"""
    )
    subprocess.run([python, "-I", "-c", validation], cwd=tmp_path, env=environment, check=True)


def test_checkout_sync_sees_generated_source_changes_without_reinstall(tmp_path):
    wheelhouse = os.environ.get("DIMOS_MESSAGE_WHEELHOUSE")
    if not wheelhouse:
        pytest.skip("Set message wheelhouse for clean checkout acceptance")
    root = Path(__file__).resolve().parents[2]
    config = tomllib.loads((root / "pyproject.toml").read_text())
    source = config["tool"]["uv"]["sources"]["dimos-generated"]
    checkout = tmp_path / "checkout"
    package = checkout / source["path"]
    shutil.copytree(
        root / source["path"],
        package,
        ignore=shutil.ignore_patterns("build", "dist", "*.egg-info", "__pycache__"),
    )
    (checkout / "pyproject.toml").write_text(
        '[project]\nname = "checkout-probe"\nversion = "0.1.0"\n'
        f'requires-python = "=={sys.version_info.major}.{sys.version_info.minor}.*"\n'
        'dependencies = ["dimos-generated==0.1.0"]\n[tool.uv.sources]\n'
        f'dimos-generated = {{ path = "{source["path"]}", editable = {str(source.get("editable", False)).lower()} }}\n'
    )
    environment = {
        key: value
        for key, value in os.environ.items()
        if key not in {"PYTHONPATH", "VIRTUAL_ENV", "UV_PROJECT_ENVIRONMENT"}
    }
    environment.update(CC="/bin/false", CXX="/bin/false", UV_PYTHON_DOWNLOADS="never")
    uv = shutil.which("uv")
    assert uv is not None, "Prepare uv before running checkout acceptance"
    subprocess.run(
        [
            uv,
            "sync",
            "--python",
            sys.executable,
            "--no-index",
            "--find-links",
            str(Path(wheelhouse).resolve()),
        ],
        cwd=checkout,
        env=environment,
        check=True,
    )
    python = str(checkout / ".venv/bin/python")
    validation = (
        "import dimos_generated; "
        "from dimos_generated.geometry_msgs.msg import Point; "
        "from dimos_message_build.registry import encode, decode; "
        "assert decode(encode(Point(x=1.25, y=0., z=0.)), Point).x == 1.25; "
        "assert dimos_generated.CHECKOUT_PROBE == 17"
    )
    # Editable installation must expose refreshed generated package exports.
    # The isolated wheel tutorial above verifies actual .msg -> source generation.
    # Native rosbags classes are not declarations in _types.py to rewrite.
    exports = package / "src/dimos_generated/__init__.py"
    exports.write_text(exports.read_text() + "\nCHECKOUT_PROBE = 17\n")
    subprocess.run([uv, "sync", "--frozen", "--offline"], cwd=checkout, env=environment, check=True)
    subprocess.run([python, "-I", "-c", validation], cwd=checkout, env=environment, check=True)
