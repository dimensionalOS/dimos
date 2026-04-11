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

"""Run registry for tracking DimOS daemon processes."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import os
from pathlib import Path
import re
import signal
import time

from dimos.constants import STATE_DIR
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

REGISTRY_DIR = STATE_DIR / "runs"


@dataclass
class RunEntry:
    """Metadata for a single DimOS run (daemon or foreground)."""

    run_id: str
    pid: int
    blueprint: str
    started_at: str
    log_dir: str
    cli_args: list[str] = field(default_factory=list)
    config_overrides: dict[str, object] = field(default_factory=dict)
    grpc_port: int = 9877
    original_argv: list[str] = field(default_factory=list)

    @property
    def registry_path(self) -> Path:
        return REGISTRY_DIR / f"{self.run_id}.json"

    def save(self) -> None:
        """Persist this entry to disk."""
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
    """Generate a human-readable, timestamp-prefixed run ID."""
    ts = time.strftime("%Y%m%d-%H%M%S")
    safe_name = re.sub(r"[^a-zA-Z0-9_-]", "-", blueprint)
    return f"{ts}-{safe_name}"


def is_pid_alive(pid: int) -> bool:
    """Check whether a process with the given PID is still running."""
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        # Process exists but we can't signal it — still alive.
        return True


def list_runs(alive_only: bool = True) -> list[RunEntry]:
    """List all registered runs, optionally filtering to alive processes."""
    REGISTRY_DIR.mkdir(parents=True, exist_ok=True)
    entries: list[RunEntry] = []
    for f in sorted(REGISTRY_DIR.glob("*.json")):
        try:
            entry = RunEntry.load(f)
        except Exception:
            logger.warning("Corrupt registry entry, removing", path=str(f))
            f.unlink()
            continue

        if alive_only and not is_pid_alive(entry.pid):
            logger.info("Cleaning stale run entry", run_id=entry.run_id, pid=entry.pid)
            entry.remove()
            continue
        entries.append(entry)
    return entries


def cleanup_stale() -> int:
    """Remove registry entries for dead processes. Returns count removed."""
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
    """Check if any alive run is using the gRPC port. Returns conflicting entry or None."""
    for entry in list_runs(alive_only=True):
        if entry.grpc_port == grpc_port:
            return entry
    return None


def get_most_recent(alive_only: bool = True) -> RunEntry | None:
    """Return the most recently created run entry, or None."""
    runs = list_runs(alive_only=alive_only)
    return runs[-1] if runs else None


def stop_entry(entry: RunEntry, force: bool = False) -> tuple[str, bool]:
    """Stop a DimOS instance by registry entry.

    Returns (message, success) for the CLI to display.
    """
    sig = signal.SIGKILL if force else signal.SIGTERM
    sig_name = "SIGKILL" if force else "SIGTERM"

    try:
        os.kill(entry.pid, sig)
    except ProcessLookupError:
        entry.remove()
        return ("Process already dead, cleaning registry", True)

    if not force:
        for _ in range(50):  # 5 seconds
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
