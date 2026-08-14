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

from __future__ import annotations

import atexit
import collections
from collections.abc import Iterator
import contextlib
from dataclasses import dataclass
import fcntl
import hashlib
import json
import os
from pathlib import Path
import queue
import shutil
import signal
import subprocess
import tempfile
import threading
import time
from typing import IO

from dimos.utils.deno import ensure_deno
from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge.locate import (
    WEB_DIR_BUILD_ENV_VAR,
    WEB_DIR_ENV_VAR,
    build_allowed,
    find_cockpit_dist,
    find_web_dir,
    relay_run_cmd,
)

logger = setup_logger()

_STDERR_TAIL_LINES = 60

_COCKPIT_BUILD_TIMEOUT_S = 600.0
# SIGTERM-to-SIGKILL grace when a build child is cancelled or times out.
_BUILD_KILL_GRACE_S = 5.0
_BUILD_POLL_S = 0.2

# Input-hash stamp a successful build writes into its dist; a mismatch with
# the current inputs means the dist is stale.
_STAMP_NAME = ".buildstamp"

# Interpreter exit must kill in-flight builds: plain atexit callbacks run
# before the executor threads driving them are joined, so exit stays bounded
# by the kill grace instead of the build timeout.
_live_build_cancels: set[threading.Event] = set()


def _cancel_builds_at_exit() -> None:
    for cancel in list(_live_build_cancels):
        cancel.set()


atexit.register(_cancel_builds_at_exit)


def ensure_cockpit_dist(
    web_dir: Path,
    timeout: float = _COCKPIT_BUILD_TIMEOUT_S,
    cancel: threading.Event | None = None,
) -> Path | None:
    """Path to the built Cockpit app, building it first when possible.

    Wheel installs ship a pre-built dist and no sources: return it (or None on
    a cockpit-less wheel). Checkouts rebuild when the dist's input stamp (a
    hash over the cockpit/shared sources, workspace config, and lockfile) no
    longer matches; the build runs under a cross-process lock, into a
    temporary directory published atomically, so concurrent starts serialize
    and a failed build leaves the served dist untouched (the relay still runs,
    and serves the debug page when there is no dist at all). Setting `cancel`
    kills the build child within a bounded grace.
    """
    dist = find_cockpit_dist(web_dir)
    cockpit = web_dir / "cockpit"
    if not (cockpit / "src").is_dir():
        return dist
    if not build_allowed(web_dir):
        logger.warning(
            f"{WEB_DIR_ENV_VAR} is set; serving its cockpit as-is "
            f"(set {WEB_DIR_BUILD_ENV_VAR}=1 to allow building that tree)"
        )
        return dist
    if cancel is None:
        cancel = threading.Event()
    deadline = time.monotonic() + timeout
    _live_build_cancels.add(cancel)
    try:
        with _build_lock(cockpit, deadline, cancel) as locked:
            if not locked:
                logger.warning("another cockpit build is running; serving the existing dist")
                return find_cockpit_dist(web_dir)
            stamp = _input_stamp(web_dir)
            dist = find_cockpit_dist(web_dir)  # a previous lock holder may have rebuilt
            if dist is not None and _read_stamp(dist) == stamp:
                return dist
            state = "missing" if dist is None else "stale"
            logger.warning(f"cockpit dist is {state}; building it (takes a minute)")
            for leftover in cockpit.glob(".dist-*"):  # crashed builds; ours is the lock
                shutil.rmtree(leftover, ignore_errors=True)
            out_dir = Path(tempfile.mkdtemp(prefix=".dist-new-", dir=cockpit))
            out_dir.chmod(0o755)  # mkdtemp gives 0o700; this becomes the dist
            try:
                # Fixed argv (what cockpit/deno.json's build task runs), not
                # the tree's task definition; --outDir keeps a failed build
                # away from the served dist.
                cmd = [
                    ensure_deno(),
                    "run",
                    "--frozen",
                    "-A",
                    "npm:vite",
                    "build",
                    "--outDir",
                    str(out_dir),
                ]
                if _run_build(cmd, cockpit, deadline, cancel):
                    (out_dir / _STAMP_NAME).write_text(stamp)
                    _swap_dist(cockpit, out_dir)
                    logger.info("cockpit dist built")
            except Exception:
                logger.exception("cockpit build did not run")
            finally:
                shutil.rmtree(out_dir, ignore_errors=True)
            return find_cockpit_dist(web_dir)
    finally:
        _live_build_cancels.discard(cancel)


@contextlib.contextmanager
def _build_lock(cockpit: Path, deadline: float, cancel: threading.Event) -> Iterator[bool]:
    """Cross-process flock; yields False when it stays busy until the deadline."""
    with (cockpit / ".build.lock").open("w") as lock_file:
        while True:
            try:
                fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                if cancel.is_set() or time.monotonic() >= deadline:
                    yield False
                    return
                time.sleep(_BUILD_POLL_S)
                continue
            yield True  # closing the file releases the lock
            return


def _input_stamp(web_dir: Path) -> str:
    """Hash of the build inputs: file set plus content, so deletions rebuild
    while test-only edits, generated output, and hidden files do not."""
    digest = hashlib.sha256()
    for path in _build_inputs(web_dir):
        digest.update(str(path.relative_to(web_dir)).encode())
        digest.update(b"\0")
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()


def _build_inputs(web_dir: Path) -> list[Path]:
    files = [path for path in (web_dir / "deno.json", web_dir / "deno.lock") if path.is_file()]
    for tree in (web_dir / "cockpit", web_dir / "shared"):
        if not tree.is_dir():
            continue
        for path in sorted(tree.rglob("*")):
            parts = path.relative_to(tree).parts
            if any(part.startswith(".") for part in parts):
                continue
            if {"dist", "node_modules"} & set(parts[:-1]):
                continue
            if parts[-1].endswith(("_test.ts", ".test.ts", ".test.tsx")):
                continue
            if path.is_file():
                files.append(path)
    return files


def _read_stamp(dist: Path) -> str | None:
    try:
        return (dist / _STAMP_NAME).read_text()
    except OSError:
        return None


def _run_build(cmd: list[str], cwd: Path, deadline: float, cancel: threading.Event) -> bool:
    """Run the build child in its own process group so cancellation and the
    deadline kill the whole tree (deno -> vite -> esbuild workers)."""
    with tempfile.TemporaryFile() as output:
        proc = subprocess.Popen(
            cmd, cwd=cwd, stdout=output, stderr=subprocess.STDOUT, start_new_session=True
        )
        while (code := proc.poll()) is None:
            if cancel.is_set():
                _kill_group(proc)
                logger.warning("cockpit build cancelled")
                return False
            if time.monotonic() >= deadline:
                _kill_group(proc)
                logger.error("cockpit build timed out")
                return False
            time.sleep(_BUILD_POLL_S)
        if code != 0:
            output.seek(0)
            tail = "\n".join(output.read().decode(errors="replace").strip().splitlines()[-20:])
            logger.error(f"cockpit build failed (exit {code}); output tail:\n{tail}")
            return False
        return True


def _kill_group(proc: subprocess.Popen[bytes]) -> None:
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except OSError:
        pass  # already gone
    try:
        proc.wait(timeout=_BUILD_KILL_GRACE_S)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except OSError:
            pass
        proc.wait()


def _swap_dist(cockpit: Path, new_dist: Path) -> None:
    """Publish atomically: renames, so readers never see a half-written dist."""
    dist = cockpit / "dist"
    old = cockpit / ".dist-old"
    if dist.exists():
        os.rename(dist, old)
    os.rename(new_dist, dist)
    shutil.rmtree(old, ignore_errors=True)


@dataclass
class RelayReadyInfo:
    http_port: int
    wt_url: str
    cert_hash: str
    v: int
    # True when the relay serves a built Cockpit at /; set by RelayProcess.
    cockpit: bool = False

    @property
    def debug_url(self) -> str:
        return f"http://127.0.0.1:{self.http_port}/debug.html"

    @property
    def open_url(self) -> str:
        """What a browser should open: the Cockpit, or the debug page without one."""
        return f"http://127.0.0.1:{self.http_port}/" if self.cockpit else self.debug_url


class RelayProcess:
    """Relay child process with a parsed ready line and clean teardown."""

    def __init__(
        self,
        *,
        port: int = 0,
        host: str = "127.0.0.1",
        web_dir: Path | None = None,
        cockpit_dir: Path | None = None,
        timeout: float = 20.0,
    ) -> None:
        self._port = port
        self._host = host
        self._web_dir = web_dir
        self._cockpit_dir = cockpit_dir
        self._timeout = timeout
        self._process: subprocess.Popen[str] | None = None
        self._threads: list[threading.Thread] = []
        self._ready_queue: queue.Queue[RelayReadyInfo] = queue.Queue(maxsize=1)
        self._stderr_tail: collections.deque[str] = collections.deque(maxlen=_STDERR_TAIL_LINES)
        self.info: RelayReadyInfo | None = None

    def start(self) -> RelayReadyInfo:
        deno = ensure_deno()
        web_dir = self._web_dir or find_web_dir()
        # No build here: start() must stay cheap (tests spawn many relays).
        # The build-if-needed step is ensure_cockpit_dist(), called by the
        # RelayBridgeModule before spawning.
        cockpit_dir = self._cockpit_dir or find_cockpit_dist(web_dir)
        cmd = relay_run_cmd(
            deno, web_dir, "--port", str(self._port), "--host", self._host, cockpit_dir=cockpit_dir
        )
        logger.info(f"starting relay: {' '.join(cmd)}")
        env = os.environ | {"NO_COLOR": "1"}
        self._process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env
        )
        assert self._process.stdout is not None and self._process.stderr is not None
        self._threads = [
            threading.Thread(target=self._read_stdout, args=(self._process.stdout,), daemon=True),
            threading.Thread(target=self._read_stderr, args=(self._process.stderr,), daemon=True),
        ]
        for thread in self._threads:
            thread.start()
        deadline = time.monotonic() + self._timeout
        while self.info is None:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                code = self._process.poll()
                self.stop()
                stderr = "\n".join(self._stderr_tail)
                state = f"exited with {code}" if code is not None else "still running"
                raise RuntimeError(
                    f"relay produced no ready line within {self._timeout} s ({state}); "
                    f"stderr tail:\n{stderr}"
                )
            try:
                self.info = self._ready_queue.get(timeout=min(0.1, remaining))
            except queue.Empty:
                code = self._process.poll()
                if code is None:
                    continue
                self.stop()
                stderr = "\n".join(self._stderr_tail)
                raise RuntimeError(
                    f"relay exited with {code} before producing a ready line; "
                    f"stderr tail:\n{stderr}"
                ) from None
        self.info.cockpit = cockpit_dir is not None
        logger.info(f"relay ready: {self.info}")
        return self.info

    def poll(self) -> int | None:
        """Child exit code; None while running (or after stop()/before start())."""
        return None if self._process is None else self._process.poll()

    def is_running(self) -> bool:
        """True while a started child is alive."""
        return self._process is not None and self._process.poll() is None

    def stop(self) -> None:
        if self._process is None:
            return
        process, self._process = self._process, None
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)
        # The child is dead: its pipes are at EOF, so the reader threads have
        # finished. Join them and close the pipes so no file object leaks.
        for thread in self._threads:
            thread.join(timeout=1)
        self._threads.clear()
        for stream in (process.stdout, process.stderr):
            if stream is not None:
                stream.close()

    def __enter__(self) -> RelayReadyInfo:
        return self.start()

    def __exit__(self, *exc_info: object) -> None:
        self.stop()

    def _read_stdout(self, stream: IO[str]) -> None:
        for line in stream:
            line = line.rstrip()
            if not line:
                continue
            if self.info is None and line.startswith("{"):
                try:
                    data = json.loads(line)
                except ValueError:
                    data = None
                if isinstance(data, dict) and data.get("event") == "ready":
                    self._ready_queue.put(
                        RelayReadyInfo(
                            http_port=int(data["httpPort"]),
                            wt_url=str(data["wtUrl"]),
                            cert_hash=str(data["certHash"]),
                            v=int(data["v"]),
                        )
                    )
                    continue
            logger.debug(f"[relay stdout] {line}")

    def _read_stderr(self, stream: IO[str]) -> None:
        for line in stream:
            line = line.rstrip()
            if line:
                self._stderr_tail.append(line)
                logger.debug(f"[relay stderr] {line}")
