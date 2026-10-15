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

"""Cold offline source, wheel, sdist and editable installs need no native tools."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

import pytest

CHECK = """
import pickle
from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.std_msgs.msg import Header
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_cdrgen.registry import encode, decode, schema
from dimos_generated_schemas.provider import message_types, package_root
from pathlib import Path
h = Header(Time(17, 23), 'map')
m = PoseStamped(h, Pose(Point(1., 2., 3.), Quaternion(0., 0., 0., 1.)))
r = decode(encode(m), PoseStamped)
assert type(r.header) is Header and r == m
assert type(pickle.loads(pickle.dumps(m)).header) is Header
assert 'MSG: std_msgs/Header' in schema(m.__msgtype__)
assert len(message_types()) == 83
assert 'visualization_msgs/msg/Marker' not in message_types()
root = Path(package_root())
assert (root / 'schema-sources.json').is_file()
assert (root / 'rust/Cargo.toml').is_file()
assert (root / 'cpp/CMakeLists.txt').is_file()
"""


def test_cold_offline_source_sdist_wheel_and_editable(tmp_path):
    dependency_wheels = os.environ.get("DIMOS_MESSAGE_WHEELHOUSE")
    if not dependency_wheels:
        if os.environ.get("DIMOS_MESSAGE_INSTALL_REQUIRED") == "1":
            pytest.fail("DIMOS_MESSAGE_WHEELHOUSE must contain the offline build dependencies")
        pytest.skip("Prepare an offline message dependency wheelhouse for installation acceptance")
    root = Path(__file__).resolve().parents[2]
    wheels = tmp_path / "wheels"
    shutil.copytree(dependency_wheels, wheels)
    environment = {key: value for key, value in os.environ.items() if key != "PYTHONPATH"}
    environment.update(
        PIP_NO_INDEX="1", PIP_FIND_LINKS=str(wheels), PIP_DISABLE_PIP_VERSION_CHECK="1"
    )

    def run(command, cwd=tmp_path):
        result = subprocess.run(command, cwd=cwd, env=environment, capture_output=True, text=True)
        assert result.returncode == 0, result.stdout + result.stderr

    checkout = tmp_path / "checkout"
    project = checkout / "packages/dimos-generated"
    shutil.copytree(
        root / "packages/dimos-generated",
        project,
        ignore=shutil.ignore_patterns("build", "dist", "*.egg-info", "canonical", "__pycache__"),
    )
    blocked = tmp_path / "blocked"
    blocked.mkdir()
    for tool in ["cc", "c++", "gcc", "g++", "cmake", "cargo", "rustc", "rustup"]:
        path = blocked / tool
        path.write_text("#!/bin/sh\necho 'unexpected native tool' >&2\nexit 97\n")
        path.chmod(0o755)
    environment.update(
        PATH=str(blocked) + os.pathsep + environment["PATH"],
        CC=str(blocked / "cc"),
        CXX=str(blocked / "c++"),
    )
    venv = tmp_path / "venv"
    run([sys.executable, "-m", "venv", str(venv)])
    python = str(venv / "bin/python")
    run([python, "-m", "pip", "install", str(project)])
    run([python, "-I", "-c", CHECK])
    run([python, "setup.py", "sdist", "--dist-dir", str(tmp_path / "sdists")], cwd=project)
    archive = next((tmp_path / "sdists").glob("*.tar.gz"))
    with tarfile.open(archive) as bundle:
        names = bundle.getnames()
        assert any(name.endswith("catalog.json") for name in names)
        assert not any("/build/" in name or "/src/" in name for name in names)
    run([python, "-m", "pip", "install", "--force-reinstall", str(archive)])
    run([python, "-I", "-c", CHECK])
    run(
        [
            python,
            "-m",
            "pip",
            "wheel",
            "--no-deps",
            "--wheel-dir",
            str(tmp_path / "output"),
            str(archive),
        ]
    )
    wheel = next((tmp_path / "output").glob("dimos_generated-*.whl"))
    run([python, "-m", "pip", "install", "--force-reinstall", str(wheel)])
    run([python, "-I", "-c", CHECK])
    run([python, "-m", "pip", "install", "-e", str(project)])
    run([python, "-I", "-c", CHECK])
    stats = project / "interfaces/dimos_msgs/msg/VideoStats.msg"
    stats.write_text(stats.read_text().replace("float64 e2e_latency_ms", "float32 e2e_latency_ms"))
    run([python, "-m", "pip", "install", "-e", str(project)])
    run(
        [
            python,
            "-I",
            "-c",
            CHECK + "\nassert 'float32 e2e_latency_ms' in schema('dimos_msgs/msg/VideoStats')\n",
        ]
    )

    stats = project / "interfaces/dimos_msgs/msg/VideoStats.msg"
    stats.rename(stats.with_name("RenamedStats.msg"))
    catalog = project / "catalog.json"
    data = json.loads(catalog.read_text())
    data["roots"]["dimos_msgs/msg/RenamedStats"] = data["roots"].pop("dimos_msgs/msg/VideoStats")
    catalog.write_text(json.dumps(data))
    run([python, "-m", "pip", "install", "-e", str(project)])
    run(
        [
            python,
            "-I",
            "-c",
            CHECK
            + "\nfrom dimos_generated.dimos_msgs import msg\nassert hasattr(msg, 'RenamedStats') and not hasattr(msg, 'VideoStats')\n",
        ]
    )
