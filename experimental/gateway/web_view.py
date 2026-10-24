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
from pathlib import Path
import shutil
import tempfile
import threading
import time

from dimos.utils.deno import ensure_deno
from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge import relay_process
from dimos.web.relay_bridge.locate import find_web_dir
from experimental.gateway import constants

logger = setup_logger()

_lock = asyncio.Lock()


def app_dir() -> Path:
    return find_web_dir() / "blueprint_view"


def _sources(web_dir: Path, app: Path) -> list[Path]:
    files = [path for path in (web_dir / "deno.json", web_dir / "deno.lock") if path.is_file()]
    for path in sorted(app.rglob("*")):
        parts = path.relative_to(app).parts
        skipped = {"dist", "node_modules"} & set(parts) or any(
            part.startswith(".") for part in parts
        )
        if path.is_file() and not skipped and ".test." not in path.name:
            files.append(path)
    return files


def stamp(web_dir: Path, app: Path) -> str:
    digest = hashlib.sha256()
    for path in _sources(web_dir, app):
        digest.update(str(path.relative_to(web_dir)).encode() + b"\0")
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()


def ensure_built(app: Path) -> Path | None:
    """The dist, rebuilt first if a source changed; None when there's none to serve."""
    # TODO(core): relay_process should expose its build helpers
    dist = app / "dist"
    if (app / "src").is_dir():
        web_dir = app.parent
        cancel = threading.Event()
        deadline = time.monotonic() + constants.VIEW_BUILD_TIMEOUT_S
        with relay_process._build_lock(web_dir, deadline, cancel) as locked:
            wanted = stamp(web_dir, app)
            if locked and relay_process._read_stamp(dist) != wanted:
                logger.warning("blueprint view is missing or stale; building it")
                out = Path(tempfile.mkdtemp(prefix=".dist-new-", dir=app))
                out.chmod(0o755)
                try:
                    cmd = [ensure_deno(), "run", "--frozen", "-A", "npm:vite", "build"]
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
