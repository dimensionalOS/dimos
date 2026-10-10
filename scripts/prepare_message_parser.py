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

"""Explicit preparation only: verify upstream, package unchanged parser sources."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path, PurePosixPath
import shutil
import tarfile
from tempfile import TemporaryDirectory
from typing import Any

import requests

ROOT = Path(__file__).resolve().parents[1]
TOOLKIT = ROOT / "dimos/message_codegen"


def prepare(
    cache: Path, output: Path, *, archive: Path | None = None, offline: bool = False
) -> Path:
    lock: dict[str, Any] = json.loads((TOOLKIT / "parser-source.json").read_text())
    cache.mkdir(parents=True, exist_ok=True)
    cached = cache / f"rosidl-{lock['revision']}.tar.gz"
    if archive is None and not cached.is_file():
        if offline:
            raise FileNotFoundError(f"Offline parser source missing: {cached}; prepare it first")
        with requests.get(lock["url"], stream=True, timeout=60) as response:
            response.raise_for_status()
            payload = b"".join(response.iter_content(1024 * 1024))
    else:
        payload = (archive or cached).read_bytes()
    if sha256(payload).hexdigest() != lock["sha256"]:
        raise ValueError("ROSIDL archive SHA256 mismatch; source was not installed")
    if archive is not None or not cached.exists():
        cached.write_bytes(payload)
    source = cache / "source"
    prefix = f"rosidl-{lock['revision']}/"
    with TemporaryDirectory(dir=cache) as temporary:
        stage = Path(temporary)
        with tarfile.open(cached) as bundle:
            for member in bundle.getmembers():
                if member.name.rstrip("/") == prefix.rstrip("/"):
                    continue
                if not member.name.startswith(prefix):
                    raise ValueError("Unexpected upstream archive root")
                relative = PurePosixPath(member.name[len(prefix) :])
                if ".." in relative.parts or relative.is_absolute():
                    raise ValueError("Unsafe upstream archive path")
                if str(relative) != "LICENSE" and not str(relative).startswith("rosidl_adapter/"):
                    continue
                if member.isdir():
                    continue
                if not member.isfile():
                    raise ValueError("Non-regular upstream parser source")
                stream = bundle.extractfile(member)
                assert stream is not None
                target = stage / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                with stream:
                    target.write_bytes(stream.read())
        parser = stage / "rosidl_adapter/rosidl_adapter/parser.py"
        if sha256(parser.read_bytes()).hexdigest() != lock["parser_sha256"]:
            raise ValueError("Upstream parser SHA256 mismatch")
        if source.exists():
            shutil.rmtree(source)
        shutil.copytree(stage, source)
    output.mkdir(parents=True, exist_ok=True)
    package = output / "rosidl_adapter"
    if package.exists():
        shutil.rmtree(package)
    shutil.copytree(source / "rosidl_adapter/rosidl_adapter", package)
    shutil.copyfile(source / "LICENSE", package / "LICENSE")
    shutil.copyfile(source / "rosidl_adapter/package.xml", package / "package.xml")
    return source


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=Path, default=ROOT / "build/message-codegen/parser-source")
    parser.add_argument("--output", type=Path, default=TOOLKIT / ".upstream")
    parser.add_argument(
        "--archive", type=Path, help="Previously downloaded, hash-verified project source"
    )
    parser.add_argument("--offline", action="store_true")
    args = parser.parse_args()
    print(prepare(args.cache, args.output, archive=args.archive, offline=args.offline))


if __name__ == "__main__":
    main()
