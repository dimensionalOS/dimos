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

"""Compatibility shim — delegates to instance_registry.

.. deprecated::
    Use ``dimos.core.instance_registry`` directly.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import os
from pathlib import Path
import re
import signal
import time

from dimos.core.instance_registry import (
    is_pid_alive,
    list_running,
    stop as _stop_by_name,
)

# Re-export
__all__ = [
    "LOG_BASE_DIR",
    "REGISTRY_DIR",
    "RunEntry",
    "check_port_conflicts",
    "cleanup_stale",
    "generate_run_id",
    "get_most_recent",
    "is_pid_alive",
    "list_runs",
    "stop_entry",
]


def _get_state_dir() -> Path:
    """XDG_STATE_HOME compliant state directory for dimos."""
    xdg = os.environ.get("XDG_STATE_HOME")
    if xdg:
        return Path(xdg) / "dimos"
    return Path.home() / ".local" / "state" / "dimos"


REGISTRY_DIR = _get_state_dir() / "runs"
LOG_BASE_DIR = _get_state_dir() / "logs"


@dataclass
class RunEntry:
    """Legacy RunEntry — kept for test and migration compatibility.

    New code should use ``InstanceInfo`` from ``instance_registry``.
    """

    run_id: str
    pid: int
    blueprint: str
    started_at: str
    log_dir: str
    cli_args: list[str] = field(default_factory=list)
    config_overrides: dict[str, object] = field(default_factory=dict)
    grpc_port: int = 9877
    original_argv: list[str] = field(default_factory=list)

    # Alias for instance_registry compat
    @property
    def name(self) -> str:
        return self.blueprint

    @property
    def run_dir(self) -> str:
        return self.log_dir

    @property
    def registry_path(self) -> Path:
        return REGISTRY_DIR / f"{self.run_id}.json"

    def save(self) -> None:
        """Persist this entry to disk (legacy format)."""
        REGISTRY_DIR.mkdir(parents=True, exist_ok=True)
        self.registry_path.write_text(json.dumps(asdict(self), indent=2))

    def remove(self) -> None:
        """Delete this entry from disk."""
        self.registry_path.unlink(missing_ok=True)

    @classmethod
    def load(cls, path: Path) -> RunEntry:
        """Load a RunEntry from a JSON file."""
        data = json.loads(path.read_text())
        return cls(**data)


def generate_run_id(blueprint: str) -> str:
    ts = time.strftime("%Y%m%d-%H%M%S")
    safe_name = re.sub(r"[^a-zA-Z0-9_-]", "-", blueprint)
    return f"{ts}-{safe_name}"


def list_runs(alive_only: bool = True) -> list[RunEntry]:
    """List runs.  Checks both new instance registry and legacy format."""
    # Check new instance registry first
    new_entries = list_running()
    results: list[RunEntry] = []
    for info in new_entries:
        results.append(
            RunEntry(
                run_id=info.name,
                pid=info.pid,
                blueprint=info.blueprint,
                started_at=info.started_at,
                log_dir=info.run_dir,
                grpc_port=info.grpc_port,
                original_argv=info.original_argv,
                config_overrides=info.config_overrides,
            )
        )

    # Also check legacy registry dir (don't create it if it doesn't exist)
    if not REGISTRY_DIR.exists():
        return results
    seen_pids = {r.pid for r in results}
    for f in sorted(REGISTRY_DIR.glob("*.json")):
        try:
            entry = RunEntry.load(f)
        except Exception:
            f.unlink()
            continue
        if entry.pid in seen_pids:
            continue
        if alive_only and not is_pid_alive(entry.pid):
            entry.remove()
            continue
        results.append(entry)

    return results


def get_most_recent(alive_only: bool = True) -> RunEntry | None:
    runs = list_runs(alive_only=alive_only)
    return runs[-1] if runs else None


def cleanup_stale() -> int:
    REGISTRY_DIR.mkdir(parents=True, exist_ok=True)
    removed = 0
    for f in list(REGISTRY_DIR.glob("*.json")):
        try:
            entry = RunEntry.load(f)
            if not is_pid_alive(entry.pid):
                entry.remove()
                removed += 1
        except Exception:
            f.unlink()
            removed += 1
    return removed


def check_port_conflicts(grpc_port: int = 9877) -> RunEntry | None:
    for entry in list_runs(alive_only=True):
        if entry.grpc_port == grpc_port:
            return entry
    return None


def stop_entry(entry: RunEntry, force: bool = False) -> tuple[str, bool]:
    """Stop a DimOS instance by RunEntry."""
    # Try new registry first
    msg, ok = _stop_by_name(entry.name, force=force)
    if ok:
        # Also clean legacy entry if present
        entry.remove()
        return msg, ok

    # Fall back to direct PID kill (legacy)
    sig = signal.SIGKILL if force else signal.SIGTERM
    sig_name = "SIGKILL" if force else "SIGTERM"

    try:
        os.kill(entry.pid, sig)
    except ProcessLookupError:
        entry.remove()
        return ("Process already dead, cleaning registry", True)

    if not force:
        for _ in range(50):
            if not is_pid_alive(entry.pid):
                break
            time.sleep(0.1)
        else:
            try:
                os.kill(entry.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            else:
                for _ in range(20):
                    if not is_pid_alive(entry.pid):
                        break
                    time.sleep(0.1)
            entry.remove()
            return (f"Escalated to SIGKILL after {sig_name} timeout", True)

    entry.remove()
    return (f"Stopped with {sig_name}", True)
