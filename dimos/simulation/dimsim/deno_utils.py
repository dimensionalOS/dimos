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

from pathlib import Path
import platform
import shutil
import stat
import subprocess
import tempfile
import urllib.request
import zipfile

from dimos.constants import STATE_DIR
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DENO_VERSION = "v2.6.10"


def ensure_deno() -> str:
    which = shutil.which("deno")
    if which:
        return which

    exe_name = "deno.exe" if platform.system() == "Windows" else "deno"
    deno_dir = STATE_DIR / "deno" / DENO_VERSION
    deno_path = deno_dir / exe_name
    if deno_path.exists():
        return str(deno_path)

    triple = _deno_triple()
    url = f"https://github.com/denoland/deno/releases/download/{DENO_VERSION}/deno-{triple}.zip"
    logger.info(f"Downloading deno {DENO_VERSION} from {url}")
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


def ensure_playwright_chromium(deno_path: str) -> None:
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
