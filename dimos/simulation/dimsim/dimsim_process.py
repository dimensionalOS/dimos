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
import shutil
import stat
import subprocess
import tempfile
import threading
import time
from typing import IO
import urllib.request
import zipfile

from dimos.constants import STATE_DIR
from dimos.core.global_config import GlobalConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_VIDEO_RATE = 50
_LIDAR_RATE = 1000
_DIMSIM_REPO_URL = "https://github.com/paul-nechifor/DimSim.git"
_DIMSIM_REPO_BRANCH = "run-from-repo"
_DENO_VERSION = "v2.6.10"


class DimSimProcess:
    def __init__(self, global_config: GlobalConfig) -> None:
        self.global_config = global_config
        self.process: subprocess.Popen[bytes] | None = None

    def start(self) -> None:
        deno_path = _ensure_deno()
        repo_dir = _ensure_repo()
        base_cmd = _deno_cmd(deno_path, repo_dir)

        scene = self.global_config.dimsim_scene
        port = self.global_config.dimsim_port

        _ensure_playwright_chromium(deno_path)
        _kill_port_holder(port)

        render = os.environ.get("DIMSIM_RENDER", "gpu").strip()
        if os.environ.get("CI"):
            render = "cpu"

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


def _ensure_playwright_chromium(deno_path: str) -> None:
    subprocess.run(
        [deno_path, "run", "--allow-all", "npm:playwright@1.58.2", "install", "chromium"],
        check=True,
    )


def _deno_triple() -> str:
    system = platform.system()
    machine = platform.machine().lower()
    if system == "Linux":
        if machine in ("x86_64", "amd64"):
            return "x86_64-unknown-linux-gnu"
        if machine in ("aarch64", "arm64"):
            return "aarch64-unknown-linux-gnu"
    elif system == "Darwin":
        if machine in ("x86_64", "amd64"):
            return "x86_64-apple-darwin"
        if machine in ("arm64", "aarch64"):
            return "aarch64-apple-darwin"
    elif system == "Windows" and machine in ("amd64", "x86_64"):
        return "x86_64-pc-windows-msvc"
    raise RuntimeError(
        f"Unsupported platform for deno auto-install: {system} {machine}. "
        "Install deno manually from https://deno.com/"
    )


def _ensure_deno() -> str:
    which = shutil.which("deno")
    if which:
        return which

    exe_name = "deno.exe" if platform.system() == "Windows" else "deno"
    deno_dir = STATE_DIR / "deno" / _DENO_VERSION
    deno_path = deno_dir / exe_name
    if deno_path.exists():
        return str(deno_path)

    triple = _deno_triple()
    url = f"https://github.com/denoland/deno/releases/download/{_DENO_VERSION}/deno-{triple}.zip"
    logger.info(f"Downloading deno {_DENO_VERSION} from {url}")
    deno_dir.mkdir(parents=True, exist_ok=True)
    try:
        with tempfile.TemporaryDirectory(dir=str(deno_dir.parent)) as tmp:
            tmp_path = Path(tmp)
            zip_path = tmp_path / "deno.zip"
            with urllib.request.urlopen(url, timeout=60) as resp, open(zip_path, "wb") as f:
                shutil.copyfileobj(resp, f)
            with zipfile.ZipFile(zip_path) as z:
                z.extractall(tmp_path)
            extracted = tmp_path / exe_name
            if not extracted.exists():
                raise RuntimeError(f"deno binary not found in archive from {url}")
            extracted.chmod(extracted.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
            extracted.replace(deno_path)
    except Exception as e:
        raise RuntimeError(
            f"deno is required to run DimSim from source. Auto-download failed: {e}. "
            "Install manually from https://deno.com/"
        ) from e

    return str(deno_path)
