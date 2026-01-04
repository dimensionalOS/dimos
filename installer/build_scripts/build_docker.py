#!/usr/bin/env python3
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

"""Build and run the installer Docker image with incremental hashing."""

import asyncio
import hashlib
from pathlib import Path
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
INSTALLER_ROOT = SCRIPT_DIR.parent
DOCKERFILE = INSTALLER_ROOT / "pyz_app" / "bundled_files" / "Dockerfile"
HASH_FILE = SCRIPT_DIR / ".dockerfile.hash.ignore"
IMAGE_NAME = "mystery"


async def _run_cmd(args: list[str], *, inherit_io: bool = False) -> None:
    kwargs = {}
    if inherit_io:
        kwargs.update(
            stdin=sys.stdin,
            stdout=sys.stdout,
            stderr=sys.stderr,
        )
    proc = await asyncio.create_subprocess_exec(*args, **kwargs)
    code = await proc.wait()
    if code != 0:
        raise RuntimeError(f"Command {' '.join(args)} failed with exit code {code}")


async def _hash_file(path: Path) -> str:
    data = await asyncio.to_thread(path.read_bytes)
    return hashlib.sha256(data).hexdigest()


async def main() -> None:
    current_hash = await _hash_file(DOCKERFILE)
    previous_hash = HASH_FILE.read_text().strip() if HASH_FILE.exists() else None
    needs_build = previous_hash != current_hash

    if needs_build:
        print(f"Building image {IMAGE_NAME}...")
        await _run_cmd(
            [
                "docker",
                "build",
                "-t",
                IMAGE_NAME,
                "-f",
                str(DOCKERFILE),
                str(INSTALLER_ROOT),
            ],
            inherit_io=True,
        )
        await asyncio.to_thread(HASH_FILE.write_text, current_hash)
    else:
        print(f"Dockerfile unchanged; skipping build for {IMAGE_NAME}.")

    await _run_cmd(
        [
            "docker",
            "run",
            "-it",
            "--rm",
            "-v",
            f"{INSTALLER_ROOT}:/app",
            "-w",
            "/app",
            IMAGE_NAME,
        ],
        inherit_io=True,
    )


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
