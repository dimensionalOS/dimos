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

"""This machine's Host: identity, revision, lock and record, systemd unit, process control."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import asdict, dataclass
import fcntl
from functools import lru_cache
import hashlib
from importlib.metadata import version as package_version
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from typing import TYPE_CHECKING, Any
import uuid

from dimos.constants import DIMOS_PROJECT_ROOT, RUNTIME_DIR, STATE_DIR
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hosted.daemon import HostDaemon

logger = setup_logger()

UNIT_NAME = "dimos-host.service"
HOST_ID_PATH = STATE_DIR / "hosted" / "host_id"
HOST_FILE = RUNTIME_DIR / "host.json"
# Held by the running daemon for its lifetime; the record is only trusted while it is.
LOCK_FILE = RUNTIME_DIR / "host.lock"
LOG_FILE = RUNTIME_DIR / "host.log"
# Above 2 x HostConfig.stop_timeout: the daemon terminates, then kills, its fragments first.
STOP_TIMEOUT = 40.0


@dataclass(frozen=True, slots=True)
class HostRecord:
    """What the running local daemon leaves for the CLI and doctors."""

    host_id: str
    name: str
    pid: int
    listen: tuple[str, ...]
    client_endpoint: str


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


def _git(*args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(DIMOS_PROJECT_ROOT), *args], capture_output=True, text=True, check=True
    ).stdout


@lru_cache(maxsize=1)
def code_revision() -> str:
    """The commit this checkout runs, plus a digest of its uncommitted diff; else the package version."""
    try:
        commit = _git("rev-parse", "HEAD").strip()
        diff = _git("diff", "HEAD")
    except (OSError, subprocess.CalledProcessError):
        return package_version("dimos")
    return f"{commit}+{hashlib.sha256(diff.encode()).hexdigest()[:8]}" if diff else commit


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


@contextmanager
def host_lock() -> Iterator[None]:
    """Be this machine's one Host daemon for the block, or raise if another is."""
    LOCK_FILE.parent.mkdir(parents=True, exist_ok=True)
    with LOCK_FILE.open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("A Host daemon is already running on this machine") from None
        yield


def _locked() -> bool:
    """Whether some process holds the Host lock."""
    try:
        with LOCK_FILE.open("r") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except FileNotFoundError:
        return False
    except BlockingIOError:
        return True
    return False


def write_host_file(record: HostRecord) -> None:
    HOST_FILE.parent.mkdir(parents=True, exist_ok=True)
    HOST_FILE.with_suffix(".tmp").write_text(json.dumps(asdict(record)))
    HOST_FILE.with_suffix(".tmp").replace(HOST_FILE)


def local_host() -> HostRecord | None:
    """The running local daemon's record, if any."""
    if not _locked():
        return None
    try:
        info = json.loads(HOST_FILE.read_text())
        return HostRecord(**{**info, "listen": tuple(info["listen"])})
    except (OSError, ValueError, TypeError, KeyError):
        return None


def run_foreground(daemon: HostDaemon, stop: threading.Event) -> None:
    """Run ``daemon`` as this machine's Host: hold the lock and leave the record."""
    with host_lock():
        record = HostRecord(
            daemon.host_id,
            daemon.config.name,
            os.getpid(),
            tuple(daemon.listen),
            daemon.client_endpoint,
        )
        write_host_file(record)
        try:
            daemon.run(stop)
        finally:
            HOST_FILE.unlink(missing_ok=True)


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
TimeoutStopSec=45

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


def _unit_active(run: Run) -> bool:
    return installed() and run(["is-active", UNIT_NAME]).returncode == 0


def start(run: Run = _systemctl, spawn: Callable[..., Any] = subprocess.Popen) -> str:
    """Start the Host through its unit if installed, else detached; the daemon writes its record."""
    if (record := local_host()) is not None:
        return f"already running (pid {record.pid})"
    if installed():
        result = run(["start", UNIT_NAME])
        return f"started {UNIT_NAME}" if result.returncode == 0 else result.stderr.strip()
    LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    with LOG_FILE.open("ab") as log:
        process = spawn(
            [dimos_executable(), "host", "start", "--foreground"],
            cwd=DIMOS_PROJECT_ROOT,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    return f"started (pid {process.pid}), log {LOG_FILE}"


def stop(run: Run = _systemctl, timeout: float = STOP_TIMEOUT) -> str:
    if _unit_active(run):
        result = run(["stop", UNIT_NAME])
        return f"stopped {UNIT_NAME}" if result.returncode == 0 else result.stderr.strip()
    record = local_host()
    if record is None:
        return "not running"
    os.kill(record.pid, signal.SIGINT)
    deadline = time.monotonic() + timeout
    while _alive(record.pid) and time.monotonic() < deadline:
        time.sleep(0.1)
    if _alive(record.pid):
        os.kill(record.pid, signal.SIGKILL)
    HOST_FILE.unlink(missing_ok=True)
    return f"stopped (pid {record.pid})"


def restart(run: Run = _systemctl) -> str:
    if installed():
        result = run(["restart", UNIT_NAME])
        return f"restarted {UNIT_NAME}" if result.returncode == 0 else result.stderr.strip()
    stop(run)
    return start(run)


def status(run: Run = _systemctl) -> str:
    if installed():
        return run(["is-active", UNIT_NAME]).stdout.strip() + f" ({UNIT_NAME})"
    record = local_host()
    return f"running (pid {record.pid})" if record is not None else "not running"
