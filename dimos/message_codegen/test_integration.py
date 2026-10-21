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

"""Verify the DimOS adapter and one installed consumer of published cdrgen."""

import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from dimos.message_codegen import native_build


def run(*command: str, cwd: Path) -> None:
    subprocess.run(command, cwd=cwd, check=True, capture_output=True, text=True)


@pytest.mark.parametrize("layout", ["installed", "checkout", "absent"])
def test_sdk_discovery(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, layout: str) -> None:
    prefix = tmp_path / "prefix"
    prefix.mkdir()
    (prefix / "native-prefixes.json").write_text(json.dumps([str(prefix)]))
    (prefix / "native-target.json").write_text('{"values": {}, "toolchain_sha256": null}')
    package = tmp_path / "dimos"
    sdk = package / "_native/cpp" if layout == "installed" else tmp_path / "native/cpp"
    if layout != "absent":
        sdk.mkdir(parents=True)
        (sdk / "dimos_nativeConfig.cmake").write_text("")
    monkeypatch.setattr(
        native_build,
        "find_spec",
        lambda _: SimpleNamespace(submodule_search_locations=[str(package)]),
    )
    toolchain = native_build.write_cmake_toolchain(prefix, tmp_path / "toolchain.cmake")
    text = toolchain.read_text()
    assert str(prefix) in text
    sdk_lines = [
        line for line in text.splitlines() if line.startswith("list(APPEND CMAKE_PREFIX_PATH")
    ]
    assert sdk_lines == ([] if layout == "absent" else [f'list(APPEND CMAKE_PREFIX_PATH "{sdk}")'])


def test_installed_message_consumer(tmp_path: Path) -> None:
    project = tmp_path / "consumer"
    source = project / "interfaces/consumer_msgs/msg/Tick.msg"
    source.parent.mkdir(parents=True)
    source.write_text("int32 value\n")
    (project / "pyproject.toml").write_text("""[build-system]
requires = ["setuptools>=77", "wheel", "dimos-cdrgen==0.1.1"]
build-backend = "setuptools.build_meta"
[project]
name = "dimos-cdr-consumer"
version = "0.1.0"
dependencies = ["dimos-cdrgen==0.1.1"]
[tool.dimos.messages]
source = "interfaces"
""")
    (project / "setup.py").write_text(
        "from pathlib import Path\nfrom setuptools import setup\n"
        "from dimos_cdrgen.source_package import setup_arguments\n"
        "setup(**setup_arguments(Path(__file__).resolve().parent))\n"
    )
    run(sys.executable, "-m", "build", "--no-isolation", cwd=project)
    wheel = next((project / "dist").glob("*.whl"))
    installed = tmp_path / "installed"
    run(
        "uv",
        "pip",
        "install",
        "--python",
        sys.executable,
        "--no-deps",
        "--target",
        str(installed),
        str(wheel),
        cwd=tmp_path,
    )
    run(
        sys.executable,
        "-I",
        "-c",
        f"""import sys
sys.path.insert(0, {str(installed)!r})
import pickle
from dimos_cdr_consumer.consumer_msgs.msg import Tick
from dimos_cdrgen.registry import decode, encode
value = Tick(value=17)
assert encode(value) == bytes.fromhex("0001000011000000")
assert type(decode(encode(value), Tick)) is Tick
assert type(pickle.loads(pickle.dumps(value))) is Tick
assert decode(encode(value), Tick).value == 17
""",
        cwd=tmp_path,
    )
