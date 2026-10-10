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

import json
import os
from pathlib import Path
import subprocess
import tarfile

import pytest

from dimos.message_codegen.definitions import Definitions
from dimos.message_codegen.native_build import prepare_cpp, write_cmake_toolchain
from dimos.message_codegen.project import Project


def test_builtin_native_artifact_version_and_relocated_consumer(tmp_path):
    location = os.environ.get("DIMOS_BUILTIN_PACKAGE")
    if not location:
        pytest.skip("Set DIMOS_BUILTIN_PACKAGE for native source artifact acceptance")
    output = Path(location).resolve()
    root = Path(__file__).resolve().parents[2]
    version = Project.load(root / "packages/dimos-generated").version
    metadata = json.loads((output / "message-package.json").read_text())
    assert metadata["version"] == version
    assert metadata["shared"]
    assert set(metadata["owned"]) == {message.name for message in Definitions([]).resolve()}
    assert (output / "dist" / f"dimos-generated-messages-{version}.crate").is_file()
    relocated = tmp_path / "relocated"
    relocated.mkdir()
    with tarfile.open(output / "dist" / f"dimos-messages-sources-{version}.tar.gz") as archive:
        archive.extractall(relocated, filter="data")
    assert json.loads((relocated / "message-package.json").read_text()) == metadata
    sources = {
        name: Path(path)
        for name, path in json.loads(os.environ.get("DIMOS_NATIVE_SOURCE_DIRS", "{}")).items()
    }
    cache = os.environ.get("DIMOS_NATIVE_TEST_CACHE")
    prefix = prepare_cpp(
        relocated,
        cache=Path(cache) if cache else None,
        offline=os.environ.get("DIMOS_OFFLINE") == "1",
        source_dirs=sources or None,
    )
    toolchain = write_cmake_toolchain(prefix, tmp_path / "toolchain.cmake")
    source = tmp_path / "app"
    source.mkdir()
    (source / "CMakeLists.txt").write_text(f"""cmake_minimum_required(VERSION 3.20)
project(external_builtin LANGUAGES CXX)
find_package(dimos_generated {version} EXACT CONFIG REQUIRED)
add_executable(check main.cpp)
target_compile_features(check PRIVATE cxx_std_17)
target_include_directories(check PRIVATE "{root}/native/cpp/include")
target_link_libraries(check PRIVATE dimos_generated::messages)
""")
    (source / "main.cpp").write_text("""#include <std_msgs/msg/header.hpp>
#include <dimos/native/cdr_codec.hpp>
int main() {
 std_msgs::msg::Header h; h.frame_id = "outside-checkout"; h.stamp.sec = 17;
 auto result = dimos::native::cdr_decode<std_msgs::msg::Header>(dimos::native::cdr_encode(h));
 return result.frame_id == h.frame_id && result.stamp.sec == 17 ? 0 : 1;
}
""")
    build = tmp_path / "build"
    subprocess.run(
        [
            "cmake",
            "-S",
            str(source),
            "-B",
            str(build),
            f"-DCMAKE_TOOLCHAIN_FILE={toolchain}",
        ],
        check=True,
    )
    subprocess.run(["cmake", "--build", str(build), "--parallel", "2"], check=True)
    subprocess.run([str(build / "check")], check=True)
