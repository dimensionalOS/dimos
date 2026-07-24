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

"""Spawn the Deno relay as a child process (used by tests and the smoke demo;
T2 grows this into the RelayBridge module's managed relay process)."""

from __future__ import annotations

import collections
from dataclasses import dataclass
import json
import os
from pathlib import Path
import queue
import subprocess
import threading
from typing import IO

from dimos.utils.deno import ensure_deno
from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge.locate import find_web_dir, relay_run_cmd

logger = setup_logger()

_STDERR_TAIL_LINES = 60


@dataclass
class RelayReadyInfo:
    http_port: int
    wt_url: str
    cert_hash: str
    v: int

    @property
    def debug_url(self) -> str:
        return f"http://127.0.0.1:{self.http_port}/debug.html"


class RelayProcess:
    """Relay child process with a parsed ready line and clean teardown."""

    def __init__(
        self,
        *,
        port: int = 0,
        host: str = "127.0.0.1",
        web_dir: Path | None = None,
        timeout: float = 20.0,
    ) -> None:
        self._port = port
        self._host = host
        self._web_dir = web_dir
        self._timeout = timeout
        self._process: subprocess.Popen[str] | None = None
        self._threads: list[threading.Thread] = []
        self._ready_queue: queue.Queue[RelayReadyInfo] = queue.Queue(maxsize=1)
        self._stderr_tail: collections.deque[str] = collections.deque(maxlen=_STDERR_TAIL_LINES)
        self.info: RelayReadyInfo | None = None

    def start(self) -> RelayReadyInfo:
        deno = ensure_deno()
        web_dir = self._web_dir or find_web_dir()
        cmd = relay_run_cmd(deno, web_dir, "--port", str(self._port), "--host", self._host)
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
        try:
            self.info = self._ready_queue.get(timeout=self._timeout)
        except queue.Empty:
            code = self._process.poll()
            self.stop()
            stderr = "\n".join(self._stderr_tail)
            state = f"exited with {code}" if code is not None else "still running"
            raise RuntimeError(
                f"relay produced no ready line within {self._timeout} s ({state}); "
                f"stderr tail:\n{stderr}"
            ) from None
        logger.info(f"relay ready: {self.info}")
        return self.info

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
