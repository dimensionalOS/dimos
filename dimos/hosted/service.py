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

"""Running the Host daemon: its runtime files, a systemd user unit, or a detached process."""

from __future__ import annotations

from collections.abc import Callable
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any
import uuid

from dimos.constants import DIMOS_PROJECT_ROOT, STATE_DIR

UNIT_NAME = "dimos-host.service"
HOST_ID_PATH = STATE_DIR / "hosted" / "host_id"
HOST_LOCK_PATH = STATE_DIR / "hosted" / "host.lock"
STOP_TIMEOUT = 20.0


def load_host_id(path: Path = HOST_ID_PATH) -> str:
    """This machine's persistent Host ID, created on first use."""
    try:
        host_id = path.read_text().strip()
    except FileNotFoundError:
        path.parent.mkdir(parents=True, exist_ok=True)
        host_id = uuid.uuid4().hex
        try:
            with path.open("x") as identity_file:
                identity_file.write(f"{host_id}\n")
        except FileExistsError:
            host_id = path.read_text().strip()
    if not host_id:
        raise ValueError(f"Host identity file is empty: {path}")
    return host_id


def runtime_dir() -> Path:
    """Where the running daemon leaves its endpoint, pid and log; never /tmp."""
    base = os.environ.get("XDG_RUNTIME_DIR")
    path = Path(base) / "dimos" if base else Path.home() / ".local" / "state" / "dimos"
    path.mkdir(parents=True, exist_ok=True)
    return path


def host_file() -> Path:
    return runtime_dir() / "host.json"


def pid_file() -> Path:
    return runtime_dir() / "host.pid"


def log_file() -> Path:
    return runtime_dir() / "host.log"


def unit_path() -> Path:
    return Path.home() / ".config" / "systemd" / "user" / UNIT_NAME


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def write_host_file(info: dict[str, Any]) -> None:
    path = host_file()
    path.with_suffix(".tmp").write_text(json.dumps({**info, "pid": os.getpid()}))
    path.with_suffix(".tmp").replace(path)


def remove_host_file() -> None:
    path = host_file()
    try:
        if json.loads(path.read_text()).get("pid") == os.getpid():
            path.unlink()
    except (OSError, ValueError):
        pass


def local_host() -> dict[str, Any] | None:
    """The running local daemon's record (host_id, name, client_endpoint), if any."""
    try:
        info: dict[str, Any] = json.loads(host_file().read_text())
    except (OSError, ValueError):
        return None
    return info if _alive(int(info.get("pid", 0))) else None


def dimos_executable() -> str:
    """This venv's dimos entry point, absolute."""
    return str(Path(sys.executable).with_name("dimos"))


def render_unit(executable: str, working_directory: Path) -> str:
    return f"""[Unit]
Description=DimOS Host daemon (zenoh router + fragment supervisor)
After=network-online.target
Wants=network-online.target

[Service]
ExecStart={executable} host start --foreground
WorkingDirectory={working_directory}
# The venv's tools (dimos-viewer, rerun) are spawned by name.
Environment=PATH={Path(executable).parent}:/usr/local/bin:/usr/bin:/bin
Restart=on-failure
RestartSec=3
KillSignal=SIGINT
TimeoutStopSec=30

[Install]
WantedBy=default.target
"""


Run = Callable[[list[str]], subprocess.CompletedProcess[str]]


def _systemctl(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["systemctl", "--user", *args], capture_output=True, text=True)


def unit_current() -> bool:
    """Whether the installed unit is what ``install`` would write now."""
    return unit_path().read_text() == render_unit(dimos_executable(), DIMOS_PROJECT_ROOT)


def install(run: Run = _systemctl) -> Path:
    path = unit_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render_unit(dimos_executable(), DIMOS_PROJECT_ROOT))
    run(["daemon-reload"])
    run(["enable", UNIT_NAME])
    if run(["is-active", UNIT_NAME]).returncode == 0:
        run(["restart", UNIT_NAME])
    return path


def uninstall(run: Run = _systemctl) -> None:
    run(["disable", "--now", UNIT_NAME])
    unit_path().unlink(missing_ok=True)
    run(["daemon-reload"])


def installed() -> bool:
    return unit_path().exists()


def enabled(run: Run = _systemctl) -> bool:
    return run(["is-enabled", UNIT_NAME]).returncode == 0


def _pid() -> int | None:
    try:
        pid = int(pid_file().read_text())
    except (OSError, ValueError):
        return None
    return pid if _alive(pid) else None


def start(run: Run = _systemctl, spawn: Callable[..., Any] = subprocess.Popen) -> str:
    if installed():
        if (pid := _pid()) is not None:
            return f"a detached Host runs (pid {pid}); `dimos host stop` it before the unit"
        result = run(["start", UNIT_NAME])
        return f"started {UNIT_NAME}" if result.returncode == 0 else result.stderr.strip()
    if (pid := _pid()) is not None:
        return f"already running (pid {pid})"
    with log_file().open("ab") as log:
        process = spawn(
            [dimos_executable(), "host", "start", "--foreground"],
            cwd=DIMOS_PROJECT_ROOT,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    pid_file().write_text(str(process.pid))
    return f"started (pid {process.pid}), log {log_file()}"


def stop(run: Run = _systemctl, timeout: float = STOP_TIMEOUT) -> str:
    if installed() and _pid() is None:
        result = run(["stop", UNIT_NAME])
        return f"stopped {UNIT_NAME}" if result.returncode == 0 else result.stderr.strip()
    pid = _pid()
    if pid is None:
        pid_file().unlink(missing_ok=True)
        return "not running"
    os.kill(pid, signal.SIGINT)
    deadline = time.monotonic() + timeout
    while _alive(pid) and time.monotonic() < deadline:
        time.sleep(0.1)
    if _alive(pid):
        os.kill(pid, signal.SIGKILL)
    pid_file().unlink(missing_ok=True)
    return f"stopped (pid {pid})"


def restart(run: Run = _systemctl) -> str:
    if installed() and _pid() is None:
        result = run(["restart", UNIT_NAME])
        return f"restarted {UNIT_NAME}" if result.returncode == 0 else result.stderr.strip()
    stop(run)
    return start(run)


def status(run: Run = _systemctl) -> str:
    if installed():
        return run(["is-active", UNIT_NAME]).stdout.strip() + f" ({UNIT_NAME})"
    pid = _pid()
    return f"running (pid {pid})" if pid is not None else "not running"
