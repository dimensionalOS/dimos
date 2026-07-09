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
import platform
import subprocess
import sys
import threading
import time
from typing import IO

from dimos.constants import STATE_DIR
from dimos.core.global_config import GlobalConfig
from dimos.simulation.dimsim.deno_utils import ensure_deno, ensure_playwright_chromium
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_VIDEO_RATE = 50
_LIDAR_RATE = 1000
_DIMSIM_REPO_URL = "https://github.com/paul-nechifor/DimSim.git"
_DIMSIM_REPO_BRANCH = "run-from-repo"

# dimsim's deno LCM bridge fragments large sensor messages (lidar/pointcloud)
# into UDP datagrams of up to ~65 KB. macOS caps a single UDP datagram at
# ``net.inet.udp.maxdgram`` (default 9216 bytes), so every lidar publish is
# dropped with EMSGSIZE ("Message too long"). Raise the cap past the largest
# fragment. Linux's default is already 65535, so this only matters on macOS.
_LCM_MAX_DATAGRAM = 65535


class DimSimProcess:
    def __init__(self, global_config: GlobalConfig) -> None:
        self.global_config = global_config
        self.process: subprocess.Popen[bytes] | None = None

    def start(self) -> None:
        ensure_macos_udp_datagram_size()
        deno_path = ensure_deno()
        repo_dir = _ensure_repo()
        base_cmd = _deno_cmd(deno_path, repo_dir)

        scene = self.global_config.dimsim_scene
        port = self.global_config.dimsim_port

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
            "--headless",
            "--render",
            render,
            "--image-rate",
            str(_VIDEO_RATE),
            "--lidar-rate",
            str(_LIDAR_RATE),
        ]

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


def ensure_macos_udp_datagram_size() -> None:
    """Raise ``net.inet.udp.maxdgram`` on macOS so dimsim LCM lidar isn't dropped.

    macOS defaults the coordination transport to zenoh, so the usual LCM
    autoconfig (``BufferConfiguratorMacOS``) never runs -- but the dimsim bridge
    always publishes sensors over LCM. Configure it here, at the dimsim entry
    point.

    No-op off macOS and when the limit is already large enough. Raising it needs
    root: uses passwordless/cached sudo when available, prompts once on an
    interactive terminal, and otherwise logs the manual command rather than
    blocking (e.g. the detached ``dimos run`` subprocess spawned under pytest).
    """
    if platform.system() != "Darwin":
        return
    if (_read_maxdgram() or 0) >= _LCM_MAX_DATAGRAM:
        return

    arg = f"net.inet.udp.maxdgram={_LCM_MAX_DATAGRAM}"
    if _run_ok(["sudo", "-n", "sysctl", "-w", arg]):  # passwordless / cached creds
        return
    if sys.stdin.isatty() and _run_ok(["sudo", "sysctl", "-w", arg]):  # prompt once
        return
    logger.warning(
        "dimsim lidar exceeds macOS's UDP datagram limit and frames will be dropped; "
        "raise it with: sudo sysctl -w %s",
        arg,
    )


def _read_maxdgram() -> int | None:
    try:
        result = subprocess.run(
            ["sysctl", "-n", "net.inet.udp.maxdgram"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode != 0:
        return None
    try:
        return int(result.stdout.strip())
    except ValueError:
        return None


def _run_ok(cmd: list[str]) -> bool:
    try:
        return subprocess.run(cmd, capture_output=True, timeout=30).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


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


def _ensure_repo() -> Path:
    repo_dir = STATE_DIR / "dimsim_repo"
    if (repo_dir / ".git").exists():
        return repo_dir
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    logger.info(f"Cloning DimSim into {repo_dir}")
    subprocess.run(
        [
            "git",
            "clone",
            "--depth",
            "1",
            "--branch",
            _DIMSIM_REPO_BRANCH,
            _DIMSIM_REPO_URL,
            str(repo_dir),
        ],
        check=True,
    )
    return repo_dir


def _deno_cmd(deno_path: str, repo_dir: Path) -> list[str]:
    cli_ts = repo_dir / "dimos-cli" / "cli.ts"
    return [deno_path, "run", "--allow-all", "--unstable-net", str(cli_ts)]
