# Copyright 2025-2026 Dimensional Inc.
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

import os
from pathlib import Path
import subprocess
import threading
import time
from typing import IO

from dimos.core.global_config import GlobalConfig
from dimos.simulation.dimsim.deno_utils import ensure_deno, ensure_playwright_chromium
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_VIDEO_RATE = 50
_LIDAR_RATE = 100
_DIMSIM_DIR = Path(__file__).resolve().parents[3] / "misc" / "DimSim"


class DimSimProcess:
    def __init__(self, global_config: GlobalConfig) -> None:
        self.global_config = global_config
        self.process: subprocess.Popen[bytes] | None = None

    def start(self) -> None:
        deno_path = ensure_deno()
        base_cmd = _deno_cmd(deno_path, _DIMSIM_DIR)

        scene = self.global_config.dimsim_scene
        port = self.global_config.dimsim_port
        headless = self.global_config.dimsim_headless

        if headless:
            ensure_playwright_chromium(deno_path)
        _kill_port_holder(port)

        render = os.environ.get("DIMSIM_RENDER", "").strip()
        if not render:
            render = "cpu" if os.environ.get("CI") else "gpu"

        cmd = [
            *base_cmd,
            "dev",
            "--scene",
            scene,
            "--port",
            str(port),
            "--no-depth",
            *(("--headless",) if headless else ()),
            "--render",
            render,
            "--image-rate",
            str(_VIDEO_RATE),
            "--lidar-rate",
            str(_LIDAR_RATE),
        ]

        if not headless:
            logger.info(
                f"Open http://localhost:{port} in your browser; sensors won't publish until that tab is loaded."
            )

        self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        self._start_log_reader()

    def stop(self) -> None:
        if self.process:
            if self.process.stderr:
                self.process.stderr.close()
            try:
                self.process.terminate()
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger.warning("DimSim process did not stop gracefully, killing")
                self.process.kill()
                self.process.wait(timeout=2)
            except Exception as e:
                logger.error(f"Error stopping DimSim process: {e}")
            self.process = None

    def _start_log_reader(self) -> None:
        assert self.process is not None

        def _reader(stream: IO[bytes] | None, label: str) -> None:
            if stream is None:
                return
            for raw in stream:
                line = raw.decode("utf-8", errors="replace").rstrip()
                if line:
                    logger.info(f"[dimsim {label}] {line}")

        for stream, label in [
            (self.process.stdout, "out"),
            (self.process.stderr, "err"),
        ]:
            t = threading.Thread(target=_reader, args=(stream, label), daemon=True)
            t.start()


def _kill_port_holder(port: int) -> None:
    """Kill any process listening on the given port."""
    try:
        result = subprocess.run(
            ["lsof", "-ti", f":{port}"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        pids = result.stdout.strip()
        if pids:
            for pid in pids.splitlines():
                logger.info(f"Killing stale process {pid} on port {port}")
                subprocess.run(["kill", pid], timeout=5)
            time.sleep(0.5)
    except Exception as e:
        logger.warning(f"Failed to check/kill port {port}: {e}")


def _deno_cmd(deno_path: str, repo_dir: Path) -> list[str]:
    cli_ts = repo_dir / "cli" / "cli.ts"
    return [deno_path, "run", "--allow-all", "--unstable-net", str(cli_ts)]
