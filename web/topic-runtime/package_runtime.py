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

"""Package a verified runtime for Desktop's DIMOS_TOPIC_RUNTIME_BUNDLE build input.

Python and zstd are build tools only. The installed Desktop embeds and extracts
the archive itself, before starting any DimOS installation or robot process.
"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile


def checksum(path: Path) -> str:
    with path.open("rb") as content:
        return hashlib.file_digest(content, "sha256").hexdigest()


def normalized(info: tarfile.TarInfo) -> tarfile.TarInfo:
    info.uid = info.gid = info.mtime = 0
    info.uname = info.gname = ""
    info.pax_headers = {}
    info.mode = 0o777 if info.issym() else 0o755 if info.isdir() or info.mode & 0o111 else 0o644
    return info


def package(source: Path, output: Path) -> None:
    source = source.resolve()
    manifest = json.loads((source / "runtime.json").read_text())
    for name, expected in manifest["files"].items():
        path = (source / name).resolve()
        if not path.is_relative_to(source) or checksum(path) != expected:
            raise ValueError(f"runtime checksum mismatch: {name}")
    if output.exists():
        raise ValueError("output exists; use a new release artifact")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".package-runtime-", dir=output.parent) as scratch:
        archive = Path(scratch) / "runtime.tar"
        with tarfile.open(archive, "w", format=tarfile.PAX_FORMAT) as bundle:
            for path in sorted(source.rglob("*")):
                relative = path.relative_to(source)
                if "__pycache__" in relative.parts or path.suffix == ".pyc":
                    continue
                if not path.resolve().is_relative_to(source):
                    raise ValueError(f"runtime path escapes bundle: {relative}")
                bundle.add(path, arcname=str(relative), recursive=False, filter=normalized)
        subprocess.run(["zstd", "-q", "-10", str(archive), "-o", str(output)], check=True)
    print(
        json.dumps(
            {
                "archive": str(output),
                "sha256": checksum(output),
                "manifestSha256": checksum(source / "runtime.json"),
            }
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    package(args.source, args.output)
