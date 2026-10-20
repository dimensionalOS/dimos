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
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

from dimos_cdrgen.generate import generate
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


@pytest.fixture(scope="session")
def native_build_python(tmp_path_factory: pytest.TempPathFactory) -> Path:
    # Native generator prerequisites are build tools, isolated from DimOS runtime deps.
    # In particular, native build dependencies must not downgrade the runtime lock.
    environment = tmp_path_factory.mktemp("cdr-native-toolchain") / "venv"
    run("uv", "venv", "--python", sys.executable, str(environment), cwd=environment.parent)
    python = environment / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    offline_args = ("--offline",) if os.environ.get("DIMOS_OFFLINE") == "1" else ()
    run(
        "uv",
        "pip",
        "install",
        *offline_args,
        "--python",
        str(python),
        "dimos-cdrgen[native]==0.1.1",
        cwd=environment.parent,
    )
    return python


@pytest.mark.native_e2e
def test_cpp_consumer_uses_dimos_header(tmp_path: Path, native_build_python: Path) -> None:
    source = tmp_path / "interfaces/consumer_msgs/msg/Tick.msg"
    source.parent.mkdir(parents=True)
    source.write_text("int32 value\n")
    generated = tmp_path / "generated"
    generate(
        [source.parents[2]], generated, ["consumer_msgs/msg/Tick"], "dimos_consumer", shared=True
    )
    prefix_file = tmp_path / "native-prefix.txt"
    run(
        str(native_build_python),
        "-c",
        """import json
import os
from pathlib import Path
import sys
from dimos_cdrgen.native_build import prepare_cpp
sources = {name: Path(path) for name, path in json.loads(os.environ.get("DIMOS_NATIVE_SOURCE_DIRS", "{}")).items()}
prefix = prepare_cpp(
    Path(sys.argv[1]), cache=Path(sys.argv[2]),
    offline=os.environ.get("DIMOS_OFFLINE") == "1", source_dirs=sources or None,
)
Path(sys.argv[3]).write_text(str(prefix))
""",
        str(generated),
        os.environ.get("DIMOS_NATIVE_TEST_CACHE", str(tmp_path / "native-cache")),
        str(prefix_file),
        cwd=tmp_path,
    )
    prefix = Path(prefix_file.read_text())
    toolchain = native_build.write_cmake_toolchain(prefix, tmp_path / "toolchain.cmake")
    include = Path(__file__).resolve().parents[2] / "native/cpp/include"
    (tmp_path / "CMakeLists.txt").write_text(f'''cmake_minimum_required(VERSION 3.20)
project(dimos_cdr_consumer LANGUAGES CXX)
find_package(dimos_consumer CONFIG REQUIRED)
add_executable(consumer main.cpp)
target_compile_features(consumer PRIVATE cxx_std_17)
target_include_directories(consumer PRIVATE "{include}")
target_link_libraries(consumer PRIVATE dimos_consumer::messages)
''')
    (tmp_path / "main.cpp").write_text("""#include <consumer_msgs/msg/tick.hpp>
#include <dimos/native/cdr_codec.hpp>
#include <vector>
int main() {
    const std::vector<uint8_t> expected{0, 1, 0, 0, 17, 0, 0, 0};
    auto value = dimos::native::cdr_decode<consumer_msgs::msg::Tick>(expected);
    return value.value == 17 && dimos::native::cdr_encode(value) == expected ? 0 : 1;
}
""")
    run(
        "cmake",
        "-S",
        str(tmp_path),
        "-B",
        str(tmp_path / "build"),
        f"-DCMAKE_TOOLCHAIN_FILE={toolchain}",
        cwd=tmp_path,
    )
    run("cmake", "--build", str(tmp_path / "build"), "--parallel", "2", cwd=tmp_path)
    run(str(tmp_path / "build/consumer"), cwd=tmp_path)
