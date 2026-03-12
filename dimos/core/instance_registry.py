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

"""Instance registry for tracking named DimOS daemon processes.

Every running DimOS instance is identified by a global name (default:
the blueprint name).  Metadata lives under ``~/.dimos/instances/<name>/``.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import os
from pathlib import Path
import signal
import time

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def dimos_home() -> Path:
    """Return DIMOS_HOME (``~/.dimos`` by default, overridable via env var)."""
    env = os.environ.get("DIMOS_HOME")
    if env:
        return Path(env)
    return Path.home() / ".dimos"


def _instances_dir() -> Path:
    return dimos_home() / "instances"


@dataclass
class InstanceInfo:
    """Metadata for a running DimOS instance."""

    name: str
    pid: int
    blueprint: str
    started_at: str  # ISO 8601
    run_dir: str
    grpc_port: int = 9877
    original_argv: list[str] = field(default_factory=list)
    config_overrides: dict[str, object] = field(default_factory=dict)


def _current_json_path(name: str) -> Path:
    return _instances_dir() / name / "current.json"


def is_pid_alive(pid: int) -> bool:
    """Check whether a process with the given PID is still running."""
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def register(info: InstanceInfo) -> None:
    """Write ``current.json`` for the given instance."""
    path = _current_json_path(info.name)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(asdict(info), indent=2))


def unregister(name: str) -> None:
    """Delete ``current.json`` for the given instance."""
    _current_json_path(name).unlink(missing_ok=True)


def get(name: str) -> InstanceInfo | None:
    """Read ``current.json``, verify the pid is alive, return info or None.

    Auto-cleans stale entries (pid dead).
    """
    path = _current_json_path(name)
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text())
        info = InstanceInfo(**data)
    except Exception:
        path.unlink(missing_ok=True)
        return None
    if not is_pid_alive(info.pid):
        path.unlink(missing_ok=True)
        return None
    return info


def list_running() -> list[InstanceInfo]:
    """Scan all ``instances/*/current.json`` and return live instances."""
    base = _instances_dir()
    if not base.exists():
        return []
    results: list[InstanceInfo] = []
    for child in sorted(base.iterdir()):
        cj = child / "current.json"
        if not cj.exists():
            continue
        try:
            data = json.loads(cj.read_text())
            info = InstanceInfo(**data)
        except Exception:
            cj.unlink(missing_ok=True)
            continue
        if is_pid_alive(info.pid):
            results.append(info)
        else:
            cj.unlink(missing_ok=True)
    return results


def get_sole_running() -> InstanceInfo | None:
    """Return the single running instance, or None if 0.

    Raises ``SystemExit`` with a helpful message if 2+ are running.
    """
    running = list_running()
    if len(running) == 0:
        return None
    if len(running) == 1:
        return running[0]
    names = ", ".join(r.name for r in running)
    raise SystemExit(
        f"Multiple instances running ({names}). Specify a name explicitly."
    )


def stop(name: str, force: bool = False) -> tuple[str, bool]:
    """Stop a named instance.  Returns (message, success)."""
    info = get(name)
    if info is None:
        return ("No running instance with that name", False)

    sig = signal.SIGKILL if force else signal.SIGTERM
    sig_name = "SIGKILL" if force else "SIGTERM"

    try:
        os.kill(info.pid, sig)
    except ProcessLookupError:
        unregister(name)
        return ("Process already dead, cleaned registry", True)

    if not force:
        for _ in range(50):  # 5 seconds
            if not is_pid_alive(info.pid):
                break
            time.sleep(0.1)
        else:
            try:
                os.kill(info.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            else:
                for _ in range(20):
                    if not is_pid_alive(info.pid):
                        break
                    time.sleep(0.1)
            unregister(name)
            return (f"Escalated to SIGKILL after {sig_name} timeout", True)

    unregister(name)
    return (f"Stopped with {sig_name}", True)


def make_run_dir(name: str) -> Path:
    """Create ``instances/<name>/runs/<YYYYMMDD-HHMMSS>/`` and return its path."""
    ts = time.strftime("%Y%m%d-%H%M%S")
    run_dir = _instances_dir() / name / "runs" / ts
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def latest_run_dir(name: str) -> Path | None:
    """Return the most recent run directory for the given instance, or None."""
    runs_dir = _instances_dir() / name / "runs"
    if not runs_dir.exists():
        return None
    dirs = sorted(runs_dir.iterdir(), reverse=True)
    return dirs[0] if dirs else None
