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

"""Add DimOS SDK discovery to the published CDR generator toolchain."""

from importlib.util import find_spec
import json
from pathlib import Path

from dimos_cdrgen.native_build import write_cmake_toolchain as library_toolchain


def write_cmake_toolchain(prefix: Path, destination: Path) -> Path:
    """Export the library toolchain and retain optional DimOS SDK discovery."""
    result = library_toolchain(prefix, destination)
    spec = find_spec("dimos")
    if spec is not None and spec.submodule_search_locations:
        package = Path(next(iter(spec.submodule_search_locations)))
        for sdk in [package / "_native/cpp", package.parent / "native/cpp"]:
            if (sdk / "dimos_nativeConfig.cmake").is_file():
                with result.open("a") as output:
                    output.write("list(APPEND CMAKE_PREFIX_PATH " + json.dumps(str(sdk)) + ")\n")
                break
    return result
