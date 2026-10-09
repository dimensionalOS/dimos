#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# Explicit maintainer native build. Python install/import never invokes this.
"${DIMOS_CODEGEN_PYTHON:-$PWD/.venv/bin/python}" - <<'PY'
import json
import os
from pathlib import Path
from dimos.message_codegen.generate import generate
from dimos.message_codegen.native_build import prepare_cpp, write_cmake_toolchain
output = Path("build/message-codegen/native")
generate([], output, shared=True, languages=("cpp", "rust"))
# Optional verified local source cache for offline acceptance; not a user project flag.
sources = {name: Path(path) for name, path in json.loads(os.environ.get("DIMOS_NATIVE_SOURCE_DIRS", "{}")).items()}
cache = os.environ.get("DIMOS_NATIVE_TEST_CACHE")
prefix = prepare_cpp(output, cache=Path(cache) if cache else None,
                     offline=os.environ.get("DIMOS_OFFLINE") == "1", source_dirs=sources or None)
print(write_cmake_toolchain(prefix, output / "toolchain.cmake").resolve())
PY
