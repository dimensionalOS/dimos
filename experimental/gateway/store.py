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

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any

from dimos.constants import DIMOS_PROJECT_ROOT, STATE_DIR
from experimental.gateway import shared_config

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib


def gateway_dir() -> Path:
    return STATE_DIR / "gateway"


def logs_dir() -> Path:
    return gateway_dir() / "logs"


def setting(section: str, key: str) -> Any:
    """A value from the shared `$DIMOS_HOME/config.yaml` (default ~/.dimos/config.yaml), None when unset."""
    return shared_config.setting(section, key)


# the checkout the gateway serves (new_state), which may not be the gateway's own (Desktop's pinned copy)
_served: Path = DIMOS_PROJECT_ROOT


def serve(dimos_dir: Path) -> None:
    global _served
    _served = dimos_dir


def takes_record_dir(dimos_dir: Path) -> bool:
    """Whether the checkout's dimos has `--record-dir` (GlobalConfig.record_dir); one without it
    rejects the flag and always records to its own default directory."""
    try:
        config = (dimos_dir / "dimos" / "core" / "global_config.py").read_text()
    except OSError:
        return False
    return "record_dir:" in config


def recordings_dir(dimos_dir: Path | None = None) -> Path:
    """Where the checkout's runs record, which `GET /history/recordings` is the one answer to.

    `recordings.dir` from the shared config.yaml (relative to its folder) when the checkout takes
    `--record-dir`, which `launches.start` then hands every launch, so the answer and the path a
    run writes to cannot drift apart; else what dimos picks by itself: the checkout's `recordings/`,
    or the state directory's for an installed package. Without `dimos_dir`, the served checkout's.
    """
    dimos_dir = dimos_dir or _served
    configured = setting("recordings", "dir")
    if configured and takes_record_dir(dimos_dir):
        return (shared_config.dimos_home() / Path(str(configured)).expanduser()).absolute()
    if (dimos_dir / ".git").exists():
        return dimos_dir.absolute() / "recordings"
    return STATE_DIR / "recordings"


def shared_settings() -> dict[str, Any]:
    """What `GET /config` answers, and what the `config` event says has changed: the shared
    `config.yaml`'s settings as the gateway resolves them for each use.

    The settings fixed when the gateway started (`zenoh.namespace`, `dimos-gateway.socket`, where a
    flag wins over the file) are not here: they cannot change while it runs, and the running ones are
    `GET /info`'s. So everything here is read fresh, and a change to any of it is news.
    """
    return {"path": str(shared_config.config_file()), "recordingsDir": str(recordings_dir())}


def write_atomic(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "w") as handle:
            handle.write(text)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def venv_dir(dimos_dir: Path) -> Path:
    return dimos_dir / ".venv"


def python_for(dimos_dir: Path) -> str:
    python = venv_dir(dimos_dir) / "bin" / "python"
    return str(python) if python.exists() else sys.executable


def dimos_bin(dimos_dir: Path) -> Path:
    return venv_dir(dimos_dir) / "bin" / "dimos"


def checkout_version(dimos_dir: Path) -> tuple[bool, str | None]:
    try:
        project = tomllib.loads((dimos_dir / "pyproject.toml").read_text()).get("project", {})
    except (OSError, tomllib.TOMLDecodeError):
        return False, None
    if project.get("name") != "dimos":
        return False, None
    version = project.get("version")
    return True, str(version) if version else None


def git(dimos_dir: Path, *args: str) -> str | None:
    try:
        done = subprocess.run(["git", "-C", str(dimos_dir), *args], capture_output=True, text=True)
    except OSError:
        return None
    return done.stdout.strip() or None if done.returncode == 0 else None


def info(dimos_dir: Path) -> dict[str, Any]:
    from experimental.gateway.state import API_VERSION

    found, version = checkout_version(dimos_dir)
    return {
        "dir": str(dimos_dir),
        "found": found,
        "installed": found and dimos_bin(dimos_dir).exists(),
        "version": version,
        "api": API_VERSION,
        "commit": git(dimos_dir, "rev-parse", "--short=10", "HEAD"),
        "tag": git(
            dimos_dir, "describe", "--tags", "--exact-match", "--match", "desktop-gateway-v*"
        ),
    }
