#!/usr/bin/env bash
set -euo pipefail

# Build the built-in C++ package with the same pinned support builder as dimos build.
# Requires dimos-cdrgen[native]; no ROS installation or runtime compilation.
cd "$(dirname "$0")/.."
"${DIMOS_CODEGEN_PYTHON:-python3}" - <<'BUILD'
import json
import os
from pathlib import Path

from dimos_cdrgen.generate import generate
from dimos_cdrgen.native_build import installed_prefixes, prepare_cpp
from dimos_cdrgen.registry import schema_roots
from dimos.message_codegen.native_build import write_cmake_toolchain

output = Path("build/message-codegen/native").resolve()
generate(list(schema_roots()), output, shared=True)
sources = {name: Path(path) for name, path in json.loads(os.environ.get("DIMOS_NATIVE_SOURCE_DIRS", "{}")).items()}
cache = os.environ.get("DIMOS_NATIVE_CACHE")
prefix = prepare_cpp(
    output, cache=Path(cache) if cache else None, offline=os.environ.get("DIMOS_OFFLINE") == "1",
    source_dirs=sources or None
)
write_cmake_toolchain(prefix, output / "toolchain.cmake")
(output / "prefixes.txt").write_text(";".join(installed_prefixes(prefix)))
BUILD
