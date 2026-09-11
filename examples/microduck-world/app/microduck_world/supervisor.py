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

"""One independently restartable DimOS runtime per robot visitor generation."""

import json
import os
import signal
import subprocess
import sys
import threading
import time
from contextlib import suppress
from dataclasses import dataclass
from typing import IO, Any

import requests
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.utils.logging_config import setup_logger
from microduck_world.relay import relay_settings
from microduck_world.robot_io import VISITOR_IDS
from microduck_world.scene import PROJECT_ROOT

logger = setup_logger()


@dataclass
class RobotProcess:
    generation: str
    process: subprocess.Popen[bytes]
    log: IO[bytes]

    def close(self) -> None:
        try:
            self.process.send_signal(signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            self.process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGKILL)
            self.process.wait(timeout=5)
        finally:
            # A crashed/timed-out parent can leave live workers behind even when
            # wait() returns immediately. Every runtime owns its process group.
            with suppress(ProcessLookupError):
                os.killpg(self.process.pid, signal.SIGKILL)
            self.log.close()


class RobotSupervisor(Module):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._children: dict[str, RobotProcess] = {}

    @rpc
    def start(self) -> None:
        super().start()
        self._thread = threading.Thread(target=self._run, name="robot-supervisor", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=65)
            if self._thread.is_alive():
                raise RuntimeError("Robot runtimes did not stop")
        super().stop()

    def _run(self) -> None:
        settings = relay_settings()
        wanted: dict[str, str] = {}
        last_ok = 0.0
        retry: dict[str, float] = {}
        try:
            with requests.Session() as http:
                http.trust_env = False
                while not self._stop.is_set():
                    try:
                        response = http.get(
                            settings.upstream_url + "/internal/assignments", timeout=1
                        )
                        response.raise_for_status()
                        value = response.json()
                        if not isinstance(value, dict) or not all(
                            k in VISITOR_IDS and isinstance(v, str) and v for k, v in value.items()
                        ):
                            raise ValueError("Invalid robot assignments")
                        wanted = value
                        last_ok = time.monotonic()
                    except (requests.RequestException, ValueError):
                        if time.monotonic() - last_ok > 3:
                            wanted = {}
                    for id, child in list(self._children.items()):
                        if child.generation != wanted.get(id) or child.process.poll() is not None:
                            child.close()
                            del self._children[id]
                    for id, generation in wanted.items():
                        if id in self._children or time.monotonic() < retry.get(id, 0):
                            continue
                        log = (PROJECT_ROOT / f"logs/{id}.log").open("ab")
                        process = subprocess.Popen(
                            [
                                sys.executable,
                                "-m",
                                "microduck_world.run_robot",
                                "--robot",
                                id,
                                "--generation",
                                generation,
                            ],
                            cwd=PROJECT_ROOT,
                            stdout=log,
                            stderr=subprocess.STDOUT,
                            start_new_session=True,
                        )
                        self._children[id] = RobotProcess(generation, process, log)
                        retry[id] = time.monotonic() + 10
                        logger.info("Started robot runtime", robot=id, pid=process.pid)
                    (PROJECT_ROOT / "state/robot-runtimes.json").write_text(
                        json.dumps(
                            {
                                id: {"generation": c.generation, "pid": c.process.pid}
                                for id, c in self._children.items()
                            }
                        )
                    )
                    self._stop.wait(0.5)
        finally:
            for child in self._children.values():
                child.close()
            self._children.clear()
