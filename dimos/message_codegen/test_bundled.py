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

Ordinary test runs skip when ``DIMOS_BUNDLE_DIST`` is unset. The dedicated
artifact job (see ``.github/workflows/ci.yml``) sets ``DIMOS_BUNDLE_DIST``,
``DIMOS_BUNDLE_WHEELHOUSE`` and ``DIMOS_BUNDLE_REQUIRED=1``; in that mode a
missing wheel, sdist or wheelhouse is a failure, never a skip.

Expected layout of ``DIMOS_BUNDLE_DIST``: the ``dimos-*.whl`` and
``dimos-*.tar.gz`` produced by ``python -m build``. ``DIMOS_BUNDLE_WHEELHOUSE``
must contain the build/runtime dependencies used below, so the checks never
reach PyPI and never install a second DimOS distribution.
"""

import json
import os
from pathlib import Path
import subprocess
import tarfile
import venv
import zipfile

import pytest

ROOT = Path(__file__).resolve().parents[2]
WHEELHOUSE_REQUIREMENTS = ("dimos-cdrgen==0.1.2", "setuptools>=70", "wheel", "pybind11", "numpy")


def run(*command, cwd, env=None):
    result = subprocess.run(command, cwd=cwd, env=env, capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout


def _required() -> bool:
    return os.environ.get("DIMOS_BUNDLE_REQUIRED") == "1"


@pytest.fixture(scope="module")
def artifacts():
    if "DIMOS_BUNDLE_DIST" not in os.environ:
        if _required():
            pytest.fail("DIMOS_BUNDLE_REQUIRED=1 but DIMOS_BUNDLE_DIST is unset")
        pytest.skip("Set DIMOS_BUNDLE_DIST and DIMOS_BUNDLE_WHEELHOUSE for artifact acceptance")
    dist = Path(os.environ["DIMOS_BUNDLE_DIST"]).resolve()
    wheels = sorted(dist.glob("dimos-*.whl"))
    sdists = sorted(dist.glob("dimos-*.tar.gz"))
    wheelhouse = Path(os.environ.get("DIMOS_BUNDLE_WHEELHOUSE", dist)).resolve()
    missing = [
        str(path)
        for path, found in (
            (dist / "dimos-*.whl", wheels),
            (dist / "dimos-*.tar.gz", sdists),
            (wheelhouse, wheelhouse.is_dir()),
        )
        if not found
    ]
    if missing and _required():
        pytest.fail("missing artifact inputs: " + ", ".join(missing))
    assert len(wheels) == 1 and len(sdists) == 1, (wheels, sdists)
    return wheels[0], sdists[0], wheelhouse


def test_main_wheel_owns_messages_and_schema_closure(artifacts):
    wheel, sdist, _ = artifacts
    with zipfile.ZipFile(wheel) as archive:
        names = set(archive.namelist())
        metadata = archive.read(next(n for n in names if n.endswith("/METADATA"))).decode()
        assert "Requires-Dist: dimos-generated" not in metadata
        assert "dimos/core/module.py" in names
        assert "dimos/web/relay_bridge/_relay_dist/relay/main.ts" in names
        if os.environ.get("DIMOS_ALLOW_MISSING_COCKPIT") != "1":
            assert "dimos/web/relay_bridge/_relay_dist/sdk/dist/sdk.js" in names
            assert "dimos/web/relay_bridge/_relay_dist/cockpit/dist/index.html" in names
        for license in (ROOT / "packages/dimos-messages/interfaces/licenses").glob("*"):
            assert any(
                ".dist-info/" in name and name.endswith("/" + license.name) for name in names
            ), license.name
        assert not any(name.startswith("dimos_generated") for name in names)
        assert "dimos_messages/std_msgs/msg/__init__.py" in names
        assert "dimos_messages_schemas/schemas/std_msgs/msg/Header.msg" in names
        assert "dimos_messages_schemas/schemas/builtin_interfaces/msg/Time.msg" in names
        assert "dimos_messages_schemas/package/cpp/CMakeLists.txt" in names
        assert "dimos_messages_schemas/package/rust/Cargo.toml" in names
    with tarfile.open(sdist) as archive:
        names = {name.split("/", 1)[-1] for name in archive.getnames()}
        assert "packages/dimos-messages/interfaces/std_msgs/msg/Header.msg" in names
        assert "packages/dimos-messages/interfaces/licenses/dimos_msgs-LICENSE" in names
        assert "packages/dimos-messages/schema-sources.json" in names


def _external_project(project: Path, version: str) -> None:
    """Write a throwaway external message package matching the local DimOS build."""
    definition = project / "interfaces/demo_msgs/msg/Reading.msg"
    definition.parent.mkdir(parents=True)
    definition.write_text("std_msgs/Header header\nfloat64 value\n")
    (project / "pyproject.toml").write_text(
        f"""[build-system]
requires = ["setuptools>=70", "wheel", "dimos-cdrgen==0.1.2", "dimos=={version}"]
build-backend = "setuptools.build_meta"
[project]
name = "external-reading"
version = "0.1.0"
dependencies = ["dimos=={version}", "dimos-cdrgen==0.1.2"]
[tool.dimos.messages]
python-module = "my_robot.messages"
[tool.dimos.messages.dependencies]
"dimos_messages" = "{version}"
"""
    )
    (project / "setup.py").write_text(
        "from pathlib import Path\nfrom setuptools import setup\n"
        "from dimos_cdrgen.source_package import setup_arguments\n"
        "setup(**setup_arguments(Path(__file__).resolve().parent))\n"
    )


def _assert_installed(python: str, version: str) -> None:
    script = f"""from importlib.metadata import distribution, entry_points, PackageNotFoundError
import json
from importlib.util import find_spec
from dimos_messages.std_msgs.msg import Header
from dimos_messages.builtin_interfaces.msg import Time
from dimos_messages.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos_messages.geometry_msgs.msg import Point
from dimos_messages_schemas.provider import message_types, package_root
from my_robot.messages.demo_msgs.msg import Reading
from my_robot.messages_schemas.provider import package_root as external_root
from dimos_cdrgen.registry import decode, encode
from pathlib import Path

owner = distribution("dimos")
assert owner.version == {version!r}, owner.version
assert find_spec("dimos_generated") is None
try:
    distribution("dimos-generated")
except PackageNotFoundError:
    pass
else:
    raise AssertionError("built-ins must keep a single DimOS distribution owner")

types = message_types()
required = {{
    "std_msgs/msg/Header",
    "builtin_interfaces/msg/Time",
    "sensor_msgs/msg/Image",
    "geometry_msgs/msg/Point",
    "dimos_msgs/msg/LineSegments3D",
    "dimos_msgs/msg/LineSegment3D",
}}
assert required <= set(types), sorted(set(types) - required)
assert len(types) == len(set(types.values())), "provider registered duplicate classes"
assert len([ep for ep in entry_points(group="dimos.messages") if ep.name == "dimos_messages"]) == 1

metadata = json.loads((package_root() / "message-package.json").read_text())
assert metadata["version"] == {version!r}, metadata["version"]
assert metadata["python_module"] == "dimos_messages", metadata

schemas = json.loads((package_root() / "schemas.json").read_text())
assert schemas, "schemas.json is empty"
for name, text in schemas.items():
    path = package_root() / "schemas" / (name + ".msg")
    assert path.is_file(), path
    for line in text.splitlines():
        field = line.split("#", 1)[0].strip()
        if not field or field.startswith("="):
            continue
        tokens = field.split()
        if not tokens:
            continue
        reference = tokens[0].rstrip("[]")
        if "/" not in reference:
            continue
        package, message = reference.split("/", 1)
        assert (package_root() / "schemas" / package / "msg" / (message + ".msg")).is_file(), (name, reference)

header = Header(Time(17, 23), "map")
value = Reading(header=header, value=21.5)
assert type(value.header) is Header
assert type(decode(encode(value), Reading).header) is Header
assert decode(encode(value), Reading) == value

segments = LineSegments3D(header=header, segments=[LineSegment3D(start=Point(1.0, 2.0, 3.0), end=Point(4.0, 5.0, 6.0), weight=0.5)])
restored = decode(encode(segments), LineSegments3D)
assert restored == segments
assert type(restored.header) is Header and type(restored.segments[0].start) is Point

assert (external_root() / "schemas/std_msgs/msg/Header.msg").is_file()
assert (external_root() / "schemas/builtin_interfaces/msg/Time.msg").is_file()
assert not (external_root() / "cpp/std_msgs").exists()
assert all("dimos-generated" not in req for req in distribution("external-reading").requires)
"""
    (Path(python).parent.parent / "check_installed.py").write_text(script)
    run(
        python,
        "-I",
        str(Path(python).parent.parent / "check_installed.py"),
        cwd=Path(python).parent.parent,
    )


@pytest.mark.parametrize("mode", ["wheel", "source", "editable", "git"])
def test_installed_messages_and_external_header_identity(tmp_path, artifacts, mode):
    wheel, sdist, wheelhouse = artifacts
    # Owner version comes from the generated manifest inside the artifact, not a literal.
    with zipfile.ZipFile(wheel) as archive:
        initializer = next(
            n
            for n in archive.namelist()
            if n.endswith("dimos_messages_schemas/package/message-package.json")
        )
        version = json.loads(archive.read(initializer))["version"]

    environment = tmp_path / "venv"
    venv.EnvBuilder(with_pip=True).create(environment)
    python = str(environment / "bin/python")
    pip = [python, "-m", "pip", "install", "--no-index", "--find-links", str(wheelhouse)]
    run(*pip, *WHEELHOUSE_REQUIREMENTS, cwd=tmp_path)
    if mode == "git":
        head = run("git", "rev-parse", "HEAD", cwd=ROOT).strip()
        # Bundle HEAD so a partial (blob:none) checkout still yields a complete
        # git source tree for pip without contacting the upstream remote.
        bundle = tmp_path / "dimos.bundle"
        run("git", "bundle", "create", str(bundle), "HEAD", cwd=ROOT)
        clone = tmp_path / "git-source"
        run("git", "clone", "--quiet", str(bundle), str(clone), cwd=tmp_path)
        source = f"git+{clone.as_uri()}@{head}"
    elif mode == "wheel":
        source = wheel
    else:
        with tarfile.open(sdist) as archive:
            archive.extractall(tmp_path / "source", filter="data")
        source = next((tmp_path / "source").iterdir())
    env = {
        **os.environ,
        "DIMOS_OFFLINE": "0" if mode == "git" else "1",
        "PIP_NO_INDEX": "1",
        "PIP_FIND_LINKS": str(wheelhouse),
    }
    run(
        *pip,
        "--no-deps",
        *(["-e"] if mode == "editable" else []),
        str(source),
        cwd=tmp_path,
        env=env,
    )
    if mode != "editable":
        example = ROOT / "examples/message-package/demo_installed_wheel.py"
        output = run(python, "-I", str(example), cwd=tmp_path)
        assert "PASS: dimos_msgs/msg/LineSegments3D" in output
    project = tmp_path / "external"
    _external_project(project, version)
    run(*pip, "--no-deps", "--no-build-isolation", str(project), cwd=tmp_path)
    _assert_installed(python, version)
