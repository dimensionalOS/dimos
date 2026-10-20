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

"""Acceptance for built-in messages owned by the main DimOS distribution.

Build the root wheel and sdist first, then set DIMOS_BUNDLE_DIST and
DIMOS_BUNDLE_WHEELHOUSE. Python dependencies use only that local wheelhouse. Git-source acceptance
also downloads verified schema archives and frozen Web dependencies.
"""

import os
from pathlib import Path
import subprocess
import tarfile
import venv
import zipfile

import pytest


def run(*command, cwd, env=None):
    result = subprocess.run(command, cwd=cwd, env=env, capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout


@pytest.fixture(scope="module")
def artifacts():
    if "DIMOS_BUNDLE_DIST" not in os.environ:
        pytest.skip("Set DIMOS_BUNDLE_DIST and DIMOS_BUNDLE_WHEELHOUSE for artifact acceptance")
    dist = Path(os.environ["DIMOS_BUNDLE_DIST"]).resolve()
    return (
        next(dist.glob("dimos-*.whl")),
        next(dist.glob("dimos-*.tar.gz")),
        Path(os.environ["DIMOS_BUNDLE_WHEELHOUSE"]).resolve(),
        next(dist.glob("dimos-*.whl")).name.split("-")[1],
    )


def test_main_wheel_owns_messages_and_schema_closure(artifacts):
    wheel, sdist, _, _ = artifacts
    with zipfile.ZipFile(wheel) as archive:
        names = set(archive.namelist())
        metadata = archive.read(next(n for n in names if n.endswith("/METADATA"))).decode()
        assert "Requires-Dist: dimos-generated" not in metadata
        assert "dimos/core/module.py" in names
        assert "dimos/web/relay_bridge/_relay_dist/sdk/dist/sdk.js" in names
        assert "dimos/web/relay_bridge/_relay_dist/cockpit/dist/index.html" in names
        assert not any(name.startswith("dimos_generated") for name in names)
        assert "dimos/messages/std_msgs/msg/__init__.py" in names
        assert "dimos/messages_schemas/schemas/std_msgs/msg/Header.msg" in names
        assert "dimos/messages_schemas/schemas/builtin_interfaces/msg/Time.msg" in names
        assert "dimos/messages_schemas/package/cpp/CMakeLists.txt" in names
        assert "dimos/messages_schemas/package/rust/Cargo.toml" in names
    with tarfile.open(sdist) as archive:
        names = {name.split("/", 1)[-1] for name in archive.getnames()}
        assert "packages/dimos-generated/interfaces/std_msgs/msg/Header.msg" in names
        assert "packages/dimos-generated/interfaces/licenses/dimos_msgs-LICENSE" in names
        assert "packages/dimos-generated/catalog.json" in names


@pytest.mark.parametrize("mode", ["wheel", "source", "editable", "git"])
def test_installed_messages_and_external_header_identity(tmp_path, artifacts, mode):
    wheel, sdist, wheelhouse, version = artifacts
    environment = tmp_path / "venv"
    venv.EnvBuilder(with_pip=True).create(environment)
    python = str(environment / "bin/python")
    pip = [python, "-m", "pip", "install", "--no-index", "--find-links", str(wheelhouse)]
    run(*pip, "dimos-cdrgen==0.1.1", "setuptools>=70", "wheel", "pybind11", cwd=tmp_path)
    if mode == "git":
        root = Path(__file__).resolve().parents[2]
        head = run("git", "rev-parse", "HEAD", cwd=root).strip()
        source = f"git+{root.as_uri()}@{head}"
    elif mode == "wheel":
        source = wheel
    else:
        with tarfile.open(sdist) as archive:
            archive.extractall(tmp_path / "source", filter="data")
        source = next((tmp_path / "source").iterdir())
    env = {**os.environ, "DIMOS_OFFLINE": "0" if mode == "git" else "1"}
    env.pop("DIMOS_ALLOW_MISSING_COCKPIT", None)
    run(
        *pip,
        "--no-deps",
        *(["-e"] if mode == "editable" else []),
        str(source),
        cwd=tmp_path,
        env=env,
    )
    # The wheel demo deliberately rejects editable paths outside site-packages.
    if mode != "editable":
        example = (
            Path(__file__).resolve().parents[2] / "examples/message-package/demo_installed_wheel.py"
        )
        output = run(python, "-I", str(example), cwd=tmp_path)
        assert "Catalog: 73 types" in output
        assert "PASS: dimos_msgs/msg/LineSegments3D" in output
    project = tmp_path / "external"
    definition = project / "interfaces/demo_msgs/msg/Reading.msg"
    definition.parent.mkdir(parents=True)
    definition.write_text("std_msgs/Header header\nfloat64 value\n")
    (project / "pyproject.toml").write_text(
        """[build-system]
requires = ["setuptools>=70", "wheel", "dimos-cdrgen==0.1.1", "dimos==0.0.14"]
build-backend = "setuptools.build_meta"
[project]
name = "external-reading"
version = "0.1.0"
dependencies = ["dimos==0.0.14", "dimos-cdrgen==0.1.1"]
[tool.dimos.messages]
python-module = "my_robot.messages"
[tool.dimos.messages.dependencies]
"dimos.messages" = "0.0.14"
""".replace("0.0.14", version)
    )
    (project / "setup.py").write_text(
        "from pathlib import Path\nfrom setuptools import setup\n"
        "from dimos_cdrgen.source_package import setup_arguments\n"
        "setup(**setup_arguments(Path(__file__).resolve().parent))\n"
    )
    run(*pip, "--no-deps", "--no-build-isolation", str(project), cwd=tmp_path)
    run(
        python,
        "-I",
        "-c",
        """from importlib.metadata import distribution, entry_points, PackageNotFoundError
import json
from importlib.util import find_spec
from dimos.messages.std_msgs.msg import Header
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages_schemas.provider import message_types, package_root
from my_robot.messages.demo_msgs.msg import Reading
from my_robot.messages_schemas.provider import package_root as external_root
from dimos_cdrgen.registry import encode, decode
assert distribution("dimos").version == "0.0.14"
assert find_spec("dimos_generated") is None
try:
    distribution("dimos-generated")
except PackageNotFoundError:
    pass
else:
    raise AssertionError("Built-ins must have only the DimOS distribution owner")
assert len(message_types()) == 73
assert len([ep for ep in entry_points(group="dimos.messages") if ep.name == "dimos"]) == 1
header = Header(Time(17, 23), "map")
value = Reading(header=header, value=21.5)
assert type(value.header) is Header
assert type(decode(encode(value), Reading).header) is Header
assert decode(encode(value), Reading) == value
metadata = json.loads((package_root() / "message-package.json").read_text())
assert metadata["version"] == "0.0.14"
assert (external_root() / "schemas/std_msgs/msg/Header.msg").is_file()
assert (external_root() / "schemas/builtin_interfaces/msg/Time.msg").is_file()
assert not (external_root() / "cpp/std_msgs").exists()
assert all("dimos-generated" not in req for req in distribution("external-reading").requires)
""".replace("0.0.14", version),
        cwd=tmp_path,
    )
