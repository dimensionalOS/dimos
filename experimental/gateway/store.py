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
import sys
import tempfile
import threading
from typing import Any

from dimos.constants import RECORDINGS_DIR, STATE_DIR

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

_settings_lock = threading.Lock()


def gateway_dir() -> Path:
    return STATE_DIR / "gateway"


def logs_dir() -> Path:
    return gateway_dir() / "logs"


def recordings_dir() -> Path:
    configured = os.environ.get("DIMOS_RECORDINGS_DIR")
    return Path(configured).expanduser() if configured else RECORDINGS_DIR


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


def settings_file() -> Path:
    return gateway_dir() / "settings.json"


def _settings() -> dict[str, Any]:
    value = read_json(settings_file())
    return value if isinstance(value, dict) else {}


def _section(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def global_config_overrides() -> dict[str, Any]:
    with _settings_lock:
        return dict(_section(_settings().get("global_config")))


def set_global_config_overrides(values: dict[str, Any]) -> None:
    with _settings_lock:
        settings = _settings()
        settings["global_config"] = {k: v for k, v in values.items() if v is not None}
        write_atomic(settings_file(), json.dumps(settings, indent=2))


def module_config(blueprint: str) -> dict[str, dict[str, Any]]:
    with _settings_lock:
        saved = _section(_section(_settings().get("module_config")).get(blueprint))
    return {module: dict(fields) for module, fields in saved.items() if isinstance(fields, dict)}


def set_module_config(blueprint: str, modules: dict[str, dict[str, Any]]) -> None:
    with _settings_lock:
        settings = _settings()
        saved = _section(settings.get("module_config"))
        if modules:
            saved[blueprint] = modules
        else:
            saved.pop(blueprint, None)
        settings["module_config"] = saved
        write_atomic(settings_file(), json.dumps(settings, indent=2))


def venv_dir(dimos_dir: Path) -> Path:
    return dimos_dir / ".venv"


def venv_python(dimos_dir: Path) -> Path:
    return venv_dir(dimos_dir) / "bin" / "python"


def python_for(dimos_dir: Path) -> str:
    python = venv_python(dimos_dir)
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


def info(dimos_dir: Path) -> dict[str, Any]:
    found, version = checkout_version(dimos_dir)
    return {
        "dir": str(dimos_dir),
        "found": found,
        "installed": found and dimos_bin(dimos_dir).exists(),
        "version": version,
    }
