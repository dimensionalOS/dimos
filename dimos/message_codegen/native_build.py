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


def stage_rust_module_source(
    project_root: Path,
    module_relative_path: str,
    destination: Path,
    *,
    message_source: Path | None = None,
) -> Path:
    """Stage a standalone crate with the SDK and prepared generated dependencies.

    Keep repository-relative Cargo paths intact. Preparation is explicit and
    separate; this function never downloads sources or installs build tools.
    """
    import shutil
    from tempfile import TemporaryDirectory

    module_path = Path(module_relative_path)
    if module_path.is_absolute() or not module_path.parts or ".." in module_path.parts:
        raise ValueError("Module path must be relative to the project root")
    project_root = project_root.resolve()
    module = project_root / module_path
    sdk = project_root / "native/rust"
    generated = message_source or project_root / "build/message-source/rust"
    for source in [module, sdk / "dimos-module", sdk / "dimos-module-macros", generated]:
        if not (source / "Cargo.toml").is_file():
            raise FileNotFoundError(f"Missing prepared Rust source: {source}")
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    ignore = shutil.ignore_patterns("target", ".git", "__pycache__", "result")
    with TemporaryDirectory(prefix=".native-source-", dir=destination.parent) as temporary:
        stage = Path(temporary) / "source"
        shutil.copytree(sdk, stage / "native/rust", ignore=ignore)
        shutil.copytree(generated, stage / "build/message-source/rust", ignore=ignore)
        shutil.copytree(module, stage / module_path, ignore=ignore, dirs_exist_ok=True)
        stage.rename(destination)
    return destination
