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

"""Where the dimos gateway keeps things, Desktop's config.yaml it shares, and the checkout's info."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
import fcntl
import os
from pathlib import Path
import shutil
import sys
import tempfile
import threading
from typing import Any

import requests
import yaml

from dimos.constants import STATE_DIR

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

# what Desktop's dimos.yaml `requires.dimos` says, when Desktop passes it (else every version is in range)
RANGE_ENV = "DESKTOP_DIMOS_RANGE"


def dimos_home() -> Path:
    """Desktop's home: DIMOS_HOME, else ~/.dimos."""
    home = os.environ.get("DIMOS_HOME")
    return Path(home) if home else Path.home() / ".dimos"


def gateway_dir() -> Path:
    """The gateway's own state (upload queue, launch record, logs); moved from `server/`, its name before."""
    path = STATE_DIR / "gateway"
    old = STATE_DIR / "server"
    if not path.exists() and old.is_dir():
        try:
            old.rename(path)
        except OSError:
            return old
    return path


def logs_dir() -> Path:
    return gateway_dir() / "logs"


def legacy_dir() -> Path:
    """Where Desktop's Rust dimos gateway kept the same files."""
    return dimos_home() / "desktop"


def state_file(name: str) -> Path:
    """`gateway_dir()/name`, first copied from Desktop's Rust gateway when only it has one."""
    path = gateway_dir() / name
    legacy = legacy_dir() / name
    if not path.exists() and legacy.is_file():
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(legacy, path)
    return path


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


_config_lock = threading.RLock()


@contextmanager
def config_transaction() -> Iterator[None]:
    with _config_lock:
        path = config_file().with_suffix(".lock")
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a") as handle:
            fcntl.flock(handle, fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(handle, fcntl.LOCK_UN)


def expand(path: str) -> Path:
    return Path(path).expanduser()


def config_file() -> Path:
    return dimos_home() / "config.yaml"


def load_desktop_config() -> dict[str, Any]:
    """Desktop's config.yaml (it owns the file; the gateway reads `dimos:` and `recordings:`)."""
    try:
        import yaml

        data = yaml.safe_load(config_file().read_text())
    except (OSError, ImportError):
        return {}
    return data if isinstance(data, dict) else {}


def replace_launch_settings(key: str, values: dict[str, Any], blueprint: str | None = None) -> None:
    """Desktop owns its config file while it runs; offline updates use a locked transaction."""
    with config_transaction():
        current = load_desktop_config()
        section = _section(current, "dimos")
        desktop_url = os.environ.get("DESKTOP_URL", "").rstrip("/")
        if desktop_url:
            # PUT /api/config merges objects. Null leaves remove overrides at launch and on gateway reads.
            old = _section(section, key)
            if blueprint is None:
                change: dict[str, Any] = {name: None for name in old if name not in values}
                change.update(values)
            else:
                previous = _section(old, blueprint)
                change = {
                    module: {field: None for field in fields}
                    for module, fields in previous.items()
                    if isinstance(fields, dict)
                }
                for module, fields in values.items():
                    change.setdefault(module, {}).update(fields)
                change = {blueprint: change}
            # Reset object-valued arguments before replacing them: Desktop deep-merges dictionaries.
            if blueprint is None:
                reset: dict[str, Any] = {
                    name: None
                    for name, value in values.items()
                    if isinstance(value, dict) and isinstance(old.get(name), dict)
                }
            else:
                reset_modules = {
                    module: {
                        field: None
                        for field, value in fields.items()
                        if isinstance(value, dict)
                        and isinstance(_section(previous, module).get(field), dict)
                    }
                    for module, fields in values.items()
                }
                reset = {
                    blueprint: {
                        module: fields for module, fields in reset_modules.items() if fields
                    }
                }
                if not reset[blueprint]:
                    reset = {}
            try:
                if reset:
                    cleared = requests.put(
                        desktop_url + "/api/config", json={"dimos": {key: reset}}, timeout=10
                    )
                    cleared.raise_for_status()
                response = requests.put(
                    desktop_url + "/api/config", json={"dimos": {key: change}}, timeout=10
                )
                response.raise_for_status()
            except requests.RequestException as failure:
                if reset:
                    if blueprint is None:
                        restore = {name: old[name] for name in reset}
                    else:
                        restore = {
                            blueprint: {
                                module: {field: previous[module][field] for field in fields}
                                for module, fields in reset[blueprint].items()
                            }
                        }
                    try:
                        reset_again = requests.put(
                            desktop_url + "/api/config", json={"dimos": {key: reset}}, timeout=10
                        )
                        reset_again.raise_for_status()
                        restored = requests.put(
                            desktop_url + "/api/config", json={"dimos": {key: restore}}, timeout=10
                        )
                        restored.raise_for_status()
                    except requests.RequestException as rollback_failure:
                        raise RuntimeError(
                            "Desktop settings save and rollback failed; previous object overrides may be cleared"
                        ) from rollback_failure
                raise failure
            return
        if blueprint is None:
            section[key] = values
        else:
            saved = dict(_section(section, key))
            if values:
                saved[blueprint] = values
            else:
                saved.pop(blueprint, None)
            if saved:
                section[key] = saved
            else:
                section.pop(key, None)
        current["dimos"] = section
        header = "# dimOS Desktop settings. Edited by Desktop's Settings app too.\n"
        write_atomic(config_file(), header + yaml.safe_dump(current, sort_keys=False))


def _section(config: dict[str, Any], name: str) -> dict[str, Any]:
    value = config.get(name)
    return value if isinstance(value, dict) else {}


# GlobalConfig values Desktop's launches start from, under the saved overrides: nothing opens (no native
# Rerun window, no browser tab) to take the screen from Desktop; Rerun's web viewer is served (:9878) to open on demand.
LAUNCH_GLOBAL_DEFAULTS: dict[str, Any] = {"rerun_open": "none", "rerun_web": True}


def global_config_overrides() -> dict[str, Any]:
    with _config_lock:
        overrides = _section(load_desktop_config(), "dimos").get("global_config")
    return (
        {key: value for key, value in overrides.items() if value is not None}
        if isinstance(overrides, dict)
        else {}
    )


def set_global_config_overrides(overrides: dict[str, Any]) -> None:
    replace_launch_settings(
        "global_config", {key: value for key, value in overrides.items() if value is not None}
    )


def module_config(blueprint: str) -> dict[str, dict[str, Any]]:
    with _config_lock:
        saved = _section(_section(load_desktop_config(), "dimos"), "module_config").get(blueprint)
    if not isinstance(saved, dict):
        return {}
    modules = {
        module: {field: value for field, value in fields.items() if value is not None}
        for module, fields in saved.items()
        if isinstance(fields, dict)
    }
    return {module: fields for module, fields in modules.items() if fields}


def set_module_config(blueprint: str, modules: dict[str, dict[str, Any]]) -> None:
    replace_launch_settings("module_config", modules, blueprint)


def ignore_version_range() -> bool:
    return bool(_section(load_desktop_config(), "dimos").get("ignore_version_range"))


def recordings_dir() -> Path:
    """config.yaml's `recordings.dir`, else the folder Desktop gives its apps ($DIMOS_RECORDINGS_DIR, set when Desktop
    starts this gateway), else where dimos itself records (its RECORDINGS_DIR: the checkout's recordings/, or
    <state>/dimos/recordings for a library install)."""
    from dimos.constants import RECORDINGS_DIR

    configured = _section(load_desktop_config(), "recordings").get("dir") or os.environ.get(
        "DIMOS_RECORDINGS_DIR"
    )
    return expand(configured) if configured else RECORDINGS_DIR


def venv_dir(dimos_dir: Path) -> Path:
    """The checkout's virtualenv (scripts/install.sh and `uv sync` make it there)."""
    return dimos_dir / ".venv"


def dimos_bin(dimos_dir: Path) -> Path:
    return venv_dir(dimos_dir) / "bin" / "dimos"


@dataclass
class Info:
    dir: str
    found: bool
    installed: bool
    version: str | None
    range: str
    in_range: bool

    def to_json(self) -> dict[str, Any]:
        return {
            "dir": self.dir,
            "found": self.found,
            "installed": self.installed,
            "version": self.version,
            "range": self.range,
            "inRange": self.in_range,
        }


def checkout_version(dimos_dir: Path) -> tuple[bool, str | None]:
    """(a dimos checkout is there, its pyproject version)."""
    try:
        project = tomllib.loads((dimos_dir / "pyproject.toml").read_text()).get("project", {})
    except (OSError, tomllib.TOMLDecodeError):
        return False, None
    if project.get("name") != "dimos":
        return False, None
    version = project.get("version")
    return True, str(version) if version else None


def satisfies(version: str, range_text: str) -> bool:
    """`>=0.0.14b1 <0.1` (Desktop's space-separated form) or comma-separated; prereleases count."""
    from packaging.specifiers import InvalidSpecifier, SpecifierSet
    from packaging.version import InvalidVersion, Version

    try:
        specifiers = SpecifierSet(",".join(range_text.split()), prereleases=True)
        return Version(version) in specifiers
    except (InvalidSpecifier, InvalidVersion):
        return False


def info(dimos_dir: Path) -> Info:
    found, version = checkout_version(dimos_dir)
    range_text = os.environ.get(RANGE_ENV, "")
    in_range = version is not None and (not range_text or satisfies(version, range_text))
    return Info(
        dir=str(dimos_dir),
        found=found,
        installed=found and dimos_bin(dimos_dir).exists(),
        version=version,
        range=range_text,
        in_range=in_range,
    )


def flag_fields() -> set[str]:
    """The GlobalConfig fields `dimos` takes as a root flag (global_options.py builds them, and skips some types)."""
    import inspect

    from dimos.cli.commands.global_options import create_dynamic_callback

    callback = create_dynamic_callback()  # type: ignore[no-untyped-call]
    return set(inspect.signature(callback).parameters) - {"ctx"}


def check_overrides(overrides: dict[str, Any]) -> None:
    """ValueError naming what `dimos` would refuse: a key that isn't one of its GlobalConfig flags (a typo, or a field
    renamed since it was saved), or a value GlobalConfig rejects. None values are skipped (they mean "unset")."""
    from pydantic import ValidationError

    from dimos.core.global_config import GlobalConfig

    unknown = sorted(set(overrides) - flag_fields())
    if unknown:
        raise ValueError(f"not a dimos GlobalConfig setting: {', '.join(unknown)}")
    try:
        # model_validate: the values alone, without the environment and .env a GlobalConfig() reads
        GlobalConfig.model_validate({k: v for k, v in overrides.items() if v is not None})
    except ValidationError as error:
        problems = "; ".join(f"{'.'.join(map(str, e['loc']))}: {e['msg']}" for e in error.errors())
        raise ValueError(f"bad GlobalConfig value: {problems}") from error


def global_config_flags(overrides: dict[str, Any], explicit_bools: bool = False) -> list[str]:
    """dimos's root flags for GlobalConfig `overrides` (`--key=value`, `--flag`/`--no-flag`), as its typer callback
    reads them; `=` keeps a value from being read as a flag's optional value (`--simulation` alone means mujoco).
    `explicit_bools`: `--flag=true` (after `run`, a bare flag would take the next item as its true/false)."""
    import json

    flags: list[str] = []
    for key, value in sorted(overrides.items()):
        flag = "--" + key.replace("_", "-")
        if value is None:
            continue
        if value is True:
            flags.append(f"{flag}=true" if explicit_bools else flag)
        elif value is False:
            flags.append("--no-" + key.replace("_", "-"))
        else:
            flags.append(f"{flag}={value if isinstance(value, str) else json.dumps(value)}")
    return flags
