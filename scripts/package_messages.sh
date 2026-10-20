#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# -ne 0 ]]; then
  echo "Message version comes from pyproject.toml; no version argument is accepted" >&2
  exit 2
fi
"${DIMOS_CODEGEN_PYTHON:-$PWD/.venv/bin/python}" - <<'PY'
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tomllib
from dimos_cdrgen.source_package import prepare
output = Path("build/message-codegen/release")
version = tomllib.loads(Path("pyproject.toml").read_text())["project"]["version"]
prepare(Path.cwd(), output)
resources = output / "python/dimos/messages_schemas/package"
metadata = output / "metadata"
metadata.mkdir(exist_ok=True)
for file in resources.iterdir():
    if file.is_file() and file.name not in {"message-package.json", "schemas.json"}:
        shutil.copy2(file, metadata / file.name)
shutil.copytree(metadata, output / "rust/source-metadata", dirs_exist_ok=True)
subprocess.run(["cargo", "package", "--manifest-path", str(output / "rust/Cargo.toml"), "--allow-dirty", *(["--offline"] if os.environ.get("DIMOS_OFFLINE") == "1" else [])], check=True)
dist = output / "dist"
dist.mkdir(exist_ok=True)
# Native ROSIDL requires generated typesupport/support libraries, not just headers.
# Ship relocatable source inputs; the shared native preparation core builds them.
archive = dist / f"dimos-messages-sources-{version}.tar.gz"
with tarfile.open(archive, "w:gz") as bundle:
    for name in ["cpp", "schemas", "schemas.json", "message-package.json", "metadata"]:
        bundle.add(output / name, arcname=name)
shutil.copy2(output / f"rust/target/package/dimos-generated-messages-{version}.crate", dist)
print(archive.resolve())
PY
