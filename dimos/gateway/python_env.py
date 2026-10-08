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

"""The python an agent runs to use dimos's python API: dimos's own interpreter, checked to import the checkout's dimos."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import threading
from typing import Any

from dimos.gateway import config

PROBE_TIMEOUT_S = 30.0
# what the probe prints: the interpreter's version and where `dimos` came from
PROBE = "import json, sys, dimos; print(json.dumps({'version': sys.version.split()[0], 'file': dimos.__file__}))"
EXAMPLE_CODE = "import dimos; print(dimos.__file__)"

_cache: dict[str, dict[str, Any]] = {}
_lock = threading.Lock()


class NoPythonError(Exception):
    """No candidate python imports the checkout's dimos."""


def candidates(dimos_dir: Path) -> list[str]:
    """The gateway's own python when it is the checkout's venv, then the checkout's venv python, then the gateway's."""
    venv_python = str(config.venv_dir(dimos_dir).absolute() / "bin" / "python")
    own = os.path.abspath(sys.executable)
    in_venv = Path(sys.prefix).resolve() == config.venv_dir(dimos_dir).resolve()
    found = [own] if in_venv else []
    found += [venv_python, own]
    return list(dict.fromkeys(found))


def clean_env(extra: dict[str, str]) -> dict[str, str]:
    """The gateway's environment without its own python settings, plus `extra`: what an agent's shell is like."""
    env = {
        k: v
        for k, v in os.environ.items()
        if k not in ("PYTHONPATH", "PYTHONHOME", "PYTHONSTARTUP")
    }
    return {**env, **extra}


def probe(python: str, extra: dict[str, str]) -> dict[str, Any] | None:
    """Run `python -c PROBE` from / (so the working folder isn't on sys.path); None when it fails."""
    try:
        done = subprocess.run(
            [python, "-c", PROBE],
            env=clean_env(extra),
            cwd="/",
            capture_output=True,
            text=True,
            timeout=PROBE_TIMEOUT_S,
        )
        return json.loads(done.stdout.strip().splitlines()[-1]) if done.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired, ValueError, IndexError):
        return None


def from_checkout(file: str, dimos_dir: Path) -> bool:
    return Path(file).resolve().is_relative_to(dimos_dir.resolve())


def find(dimos_dir: Path) -> dict[str, Any]:
    """The first candidate that imports the checkout's dimos as is, else with PYTHONPATH=<checkout>."""
    tried: list[str] = []
    for python in candidates(dimos_dir):
        if not (os.path.isfile(python) and os.access(python, os.X_OK)):
            tried.append(f"{python}: not there")
            continue
        for extra in ({}, {"PYTHONPATH": str(dimos_dir.absolute())}):
            answer = probe(python, extra)
            if answer and from_checkout(answer["file"], dimos_dir):
                return describe(python, extra, answer["version"], dimos_dir)
        tried.append(f"{python}: can't import dimos from {dimos_dir}")
    raise NoPythonError("no python imports dimos: " + "; ".join(tried))


def describe(python: str, env: dict[str, str], version: str, dimos_dir: Path) -> dict[str, Any]:
    prefix = "".join(f"{k}={shlex.quote(v)} " for k, v in env.items())
    return {
        "python": python,
        "command": [python],
        "dimosDir": str(dimos_dir.absolute()),
        "version": version,
        "dimosVersion": config.checkout_version(dimos_dir)[1],
        "env": env,
        "example": f"{prefix}{shlex.quote(python)} -c {shlex.quote(EXAMPLE_CODE)}",
    }


def python_command(dimos_dir: Path) -> dict[str, Any]:
    """`find`, cached per checkout while its python is still there (a failure isn't cached)."""
    key = str(dimos_dir.absolute())
    with _lock:
        cached = _cache.get(key)
        if cached is None or not os.path.isfile(cached["python"]):
            cached = _cache[key] = find(dimos_dir)
        return cached
