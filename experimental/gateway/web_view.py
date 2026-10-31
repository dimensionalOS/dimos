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

"""The blueprint view's dist, rebuilt like the cockpit's (relay_process) when its sources change; a wheel's is prebuilt."""

from __future__ import annotations

import asyncio
import hashlib
import os
from pathlib import Path
import shutil
import tempfile
import threading
import time

from dimos.utils.deno import ensure_deno
from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge import relay_process
from experimental.gateway import constants

logger = setup_logger()

_lock = asyncio.Lock()


def app_dir() -> Path:
    """Its own Deno project (deno.json, deno.lock, node_modules), not a member of web/'s workspace: a wheel
    ships web/ without experimental/, and Deno refuses a workspace whose member is missing."""
    return Path(__file__).parent / "blueprint_view"


def _sources(app: Path) -> list[Path]:
    files = []
    for path in sorted(app.rglob("*")):
        parts = path.relative_to(app).parts
        skipped = {"dist", "node_modules"} & set(parts) or any(
            part.startswith(".") for part in parts
        )
        if path.is_file() and not skipped and ".test." not in path.name:
            files.append(path)
    return files


def stamp(app: Path) -> str:
    digest = hashlib.sha256()
    for path in _sources(app):
        digest.update(str(path.relative_to(app)).encode() + b"\0")
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()


def deno_on_path() -> str:
    """`ensure_deno`, with its folder put on this process's PATH: the vite build calls `deno info` through
    @deno/vite-plugin, which looks `deno` up on PATH (the Alfred Jetson has only the downloaded one)."""
    deno = ensure_deno()
    folder = str(Path(deno).parent)
    if folder not in os.environ.get("PATH", "").split(os.pathsep):
        os.environ["PATH"] = os.pathsep.join(filter(None, [folder, os.environ.get("PATH", "")]))
    return deno


def ensure_built(app: Path) -> Path | None:
    """The dist, rebuilt first if a source changed; None when there's none to serve."""
    # TODO(core): relay_process should expose its build helpers
    dist = app / "dist"
    if (app / "src").is_dir():
        cancel = threading.Event()
        deadline = time.monotonic() + constants.VIEW_BUILD_TIMEOUT_S
        with relay_process._build_lock(app, deadline, cancel) as locked:
            wanted = stamp(app)
            if locked and relay_process._read_stamp(dist) != wanted:
                logger.warning("blueprint view is missing or stale; building it")
                out = Path(tempfile.mkdtemp(prefix=".dist-new-", dir=app))
                out.chmod(0o755)
                try:
                    cmd = [deno_on_path(), "run", "--frozen", "-A", "npm:vite", "build"]
                    if relay_process._run_build(
                        [*cmd, "--outDir", str(out)], app, deadline, cancel
                    ):
                        (out / relay_process._STAMP_NAME).write_text(wanted)
                        relay_process._swap_dist(app, out)
                finally:
                    shutil.rmtree(out, ignore_errors=True)
    return dist if (dist / "index.html").is_file() else None


async def built_dist() -> Path | None:
    async with _lock:
        return await asyncio.to_thread(ensure_built, app_dir())
