#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# -ne 0 ]]; then
  echo "Message version comes from packages/dimos-generated/pyproject.toml; no version argument is accepted" >&2
  exit 2
fi
"${DIMOS_CODEGEN_PYTHON:-$PWD/.venv/bin/python}" - <<'PY'
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tomllib
from dimos.message_codegen.generate import generate
output = Path("build/message-codegen/release")
version = tomllib.loads(Path("packages/dimos-generated/pyproject.toml").read_text())["project"]["version"]
generate([], output, version=version, shared=True, languages=("cpp", "rust"))
subprocess.run(["cargo", "package", "--manifest-path", str(output / "rust/Cargo.toml"), "--allow-dirty", *(["--offline"] if os.environ.get("DIMOS_OFFLINE") == "1" else [])], check=True)
dist = output / "dist"
dist.mkdir(exist_ok=True)
# Native ROSIDL requires generated typesupport/support libraries, not just headers.
# Ship relocatable source inputs; the shared native preparation core builds them.
archive = dist / f"dimos-messages-sources-{version}.tar.gz"
with tarfile.open(archive, "w:gz") as bundle:
    for name in ["cpp", "schemas", "schemas.json", "message-package.json"]:
        bundle.add(output / name, arcname=name)
shutil.copy2(output / f"rust/target/package/dimos-generated-messages-{version}.crate", dist)
print(archive.resolve())
PY
