#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
story_build="$PWD/build/message-codegen/user-story"
story_python="${DIMOS_CODEGEN_PYTHON:-$PWD/.venv/bin/python}"
"$story_python" - <<'PY'
from importlib.util import find_spec
import json
import os
from pathlib import Path
import shutil
import subprocess
from dimos.message_codegen.build import local_cargo_dependency
from dimos.message_codegen.generate import generate
from dimos.message_codegen.native_build import prepare_cpp, write_cmake_toolchain
from dimos.message_codegen.ownership import Dependency
root = Path.cwd()
output = root / "build/message-codegen/user-story"
spec = find_spec("dimos_generated_schemas")
if spec is None or spec.origin is None:
    raise RuntimeError("Install matching built-in message schemas before this acceptance run")
dependency = Dependency.load(Path(spec.origin).parent / "package")
generate([root / "examples/message-codegen/user-story"], output,
         ["story_msgs/msg/DeviceReading"], "story_messages", shared=True,
         dependencies=(dependency,))
sources = {name: Path(path) for name, path in json.loads(os.environ.get("DIMOS_NATIVE_SOURCE_DIRS", "{}")).items()}
cache = os.environ.get("DIMOS_NATIVE_TEST_CACHE")
prefix = prepare_cpp(output, cache=Path(cache) if cache else None,
                     offline=os.environ.get("DIMOS_OFFLINE") == "1", source_dirs=sources or None)
toolchain = write_cmake_toolchain(prefix, output / "toolchain.cmake")
consumer = output / "consumer"
consumer.mkdir(exist_ok=True)
(consumer / "CMakeLists.txt").write_text(f'''cmake_minimum_required(VERSION 3.20)
project(story_consumer LANGUAGES CXX)
find_package(story_messages CONFIG REQUIRED)
add_executable(cpp-consumer "{root}/examples/message-codegen/user-story/consumer.cpp")
target_compile_features(cpp-consumer PRIVATE cxx_std_17)
target_include_directories(cpp-consumer PRIVATE "{root}/native/cpp/include")
target_link_libraries(cpp-consumer PRIVATE story_messages::messages)
set_target_properties(cpp-consumer PROPERTIES RUNTIME_OUTPUT_DIRECTORY "{output}")
''')
subprocess.run(["cmake", "--fresh", "-S", str(consumer), "-B", str(consumer / "build"), f"-DCMAKE_TOOLCHAIN_FILE={toolchain}"], check=True)
subprocess.run(["cmake", "--build", str(consumer / "build"), "--parallel", "2"], check=True)
# Python-only dependency wheels carry schemas; reconstruct native sources without
# changing their type ownership or rebuilding the DimOS runtime.
builtin = output / "dimos_generated"
generate([dependency.root / "schemas"], builtin, list(dependency.owned), dependency.module,
         version=dependency.version, shared=True, languages=("rust",))
manifest = output / "rust/Cargo.toml"
manifest.write_text(local_cargo_dependency(manifest.read_text(), dependency.module).replace('../dimos_generated', '../dimos_generated/rust'))
(output / "rust/src/bin").mkdir(exist_ok=True)
shutil.copy2(root / "examples/message-codegen/user-story/consumer.rs", output / "rust/src/bin/story-consumer.rs")
PY
mkdir -p "$story_build/evidence"
cargo build --offline --manifest-path "$story_build/rust/Cargo.toml" --bin story-consumer
"${DIMOS_RUNTIME_PYTHON:-$story_python}" -m pip install --no-deps --no-build-isolation \
  --target "$story_build/site" "$story_build/python"
PYTHONPATH="$story_build/site:$PWD${PYTHONPATH:+:$PYTHONPATH}" \
  "${DIMOS_RUNTIME_PYTHON:-$story_python}" examples/message-codegen/user-story/demo_module.py --build "$story_build" \
  | tee "$story_build/evidence/terminal.txt"
