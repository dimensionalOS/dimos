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
from dimos.message_codegen.project import Project


def test_builtin_native_artifact_version_and_relocated_consumer(tmp_path):
    location = os.environ.get("DIMOS_BUILTIN_PACKAGE")
    fastcdr = os.environ.get("DIMOS_FASTCDR_PREFIX")
    if not location or not fastcdr:
        pytest.skip("Set DIMOS_BUILTIN_PACKAGE and DIMOS_FASTCDR_PREFIX for artifact acceptance")
    output = Path(location).resolve()
    root = Path(__file__).resolve().parents[2]
    version = Project.load(root / "packages/dimos-generated").version
    metadata = json.loads((output / "message-package.json").read_text())
    assert metadata["version"] == version
    assert metadata["shared"]
    assert set(metadata["owned"]) == {message.name for message in Definitions([]).resolve()}
    assert (output / "dist" / f"dimos-generated-messages-{version}.crate").is_file()
    prefix = tmp_path / "prefix"
    prefix.mkdir()
    with tarfile.open(output / "dist" / f"dimos-messages-cmake-{version}.tar.gz") as archive:
        archive.extractall(prefix, filter="data")
    source = tmp_path / "app"
    source.mkdir()
    (source / "CMakeLists.txt").write_text(f"""cmake_minimum_required(VERSION 3.20)
project(external_builtin LANGUAGES CXX)
find_package(dimos_generated {version} EXACT CONFIG REQUIRED)
add_executable(check main.cpp)
target_link_libraries(check PRIVATE dimos_generated::messages)
""")
    (source / "main.cpp").write_text("""#include <dimos_generated/messages.hpp>
int main() {
 std_msgs::msg::Header h; h.frame_id = "outside-checkout"; h.stamp.sec = 17;
 auto result = dimos::cdr::decode<std_msgs::msg::Header>(dimos::cdr::encode(h));
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
            f"-DCMAKE_PREFIX_PATH={prefix};{Path(fastcdr).resolve()}",
        ],
        check=True,
    )
    subprocess.run(["cmake", "--build", str(build), "--parallel", "2"], check=True)
    subprocess.run([str(build / "check")], check=True)
