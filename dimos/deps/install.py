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

"""Install dependency bundles into the virtualenv that runs this process, using uv.

Two sources, no resolution of our own:

- a source checkout (editable install): ``uv sync`` against the checkout's ``uv.lock``;
- an installed release: ``uv pip install`` of the ``pylock.<bundle>-<backend>.toml`` files
  exported from that same lock at build time (``dimos/deps/export_locks.py``).
"""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys

from packaging.version import InvalidVersion, Version
import typer

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.run_registry import list_runs
from dimos.deps.backend import Backend
from dimos.deps.bundles import LOCKS_DIR, lock_path

MIN_UV_VERSION = Version("0.9.25")
UV_INSTALL_HINT = "curl -LsSf https://astral.sh/uv/install.sh | sh"
ONNX_GPU_DISTRIBUTION = "onnxruntime-gpu"
# Variables that would point uv at another interpreter, project or environment, or change
# the locked/inexact behaviour required here. Credentials, proxies and index settings stay.
LEAKING_VARIABLES = (
    "VIRTUAL_ENV",
    "CONDA_PREFIX",
    "UV_PYTHON",
    "UV_PROJECT",
    "UV_PROJECT_ENVIRONMENT",
    "UV_FROZEN",
    "UV_LOCKED",
    "UV_NO_SYNC",
    "UV_FIND_LINKS",
    "UV_OFFLINE",
    "UV_SYSTEM_PYTHON",
)
PROVIDERS = {"cpu": "CPUExecutionProvider", "cuda": "CUDAExecutionProvider"}
# Both scripts run in a fresh interpreter: packages changed under the preparing process.
_VERIFY_SCRIPT = """\
import sys
import cv2
if not hasattr(cv2, "legacy"):
    sys.exit("cv2 has no 'legacy' module: opencv-contrib-python was overwritten by another OpenCV build")
import onnxruntime
providers = onnxruntime.get_available_providers()
if sys.argv[1] not in providers:
    sys.exit(f"onnxruntime lacks {sys.argv[1]}; available providers: {providers}")
print("verified: opencv-contrib and onnxruntime " + sys.argv[1])
"""
_VERSION_SCRIPT = """\
import importlib.metadata, sys
try:
    print(importlib.metadata.version(sys.argv[1]))
except importlib.metadata.PackageNotFoundError:
    pass
"""


class PrepareError(Exception):
    """Preparation cannot proceed; the message is shown to the user."""


@dataclass(frozen=True)
class Target:
    """The interpreter and virtualenv running this process."""

    python: Path
    prefix: Path
    base_prefix: Path

    @classmethod
    def current(cls) -> Target:
        return cls(
            python=Path(sys.executable),
            prefix=Path(sys.prefix).resolve(),
            base_prefix=Path(sys.base_prefix).resolve(),
        )

    @property
    def is_virtualenv(self) -> bool:
        return self.prefix != self.base_prefix


def checkout_root() -> Path | None:
    """The source checkout this installation runs from, or None for an installed release."""
    pyproject = DIMOS_PROJECT_ROOT / "pyproject.toml"
    if not pyproject.is_file() or not (DIMOS_PROJECT_ROOT / "uv.lock").is_file():
        return None
    if not re.search(r'^name\s*=\s*"dimos"\s*$', pyproject.read_text(), re.MULTILINE):
        return None
    return DIMOS_PROJECT_ROOT


def find_uv() -> list[str]:
    """The uv command (``uv`` on PATH, else ``python -m uv``), checked against the minimum."""
    which = shutil.which("uv")
    command = [which] if which else [sys.executable, "-m", "uv"]
    try:
        output = subprocess.run(
            [*command, "--version"], capture_output=True, text=True, check=True
        ).stdout
    except (OSError, subprocess.CalledProcessError) as e:
        raise PrepareError(
            f"uv is required but was not found; install it: {UV_INSTALL_HINT}"
        ) from e
    version = parse_uv_version(output)
    if version < MIN_UV_VERSION:
        raise PrepareError(
            f"uv {version} is too old; dimos needs uv >= {MIN_UV_VERSION}: {UV_INSTALL_HINT}"
        )
    return command


def parse_uv_version(output: str) -> Version:
    """``uv 0.11.15 (x86_64-unknown-linux-gnu)`` -> ``Version("0.11.15")``."""
    parts = output.split()
    try:
        return Version(parts[1])
    except (IndexError, InvalidVersion) as e:
        raise PrepareError(f"could not read the uv version from {output!r}") from e


def refuse_system_python(target: Target) -> None:
    if target.is_virtualenv:
        return
    raise PrepareError(
        f"{target.python} is not a virtualenv interpreter. dimos prepare only installs into an "
        "ordinary virtualenv: create one (scripts/install.sh, or `uv venv && source "
        ".venv/bin/activate`), install dimos into it, and rerun from there."
    )


def refuse_active_runs(target: Target) -> None:
    """A running DimOS instance in this environment must be stopped before packages change."""
    for entry in list_runs(alive_only=True):
        if entry.environment in ("", str(target.prefix)):
            raise PrepareError(
                f"DimOS run {entry.run_id} (PID {entry.pid}) is using this environment; "
                "stop it first: dimos stop"
            )


def uv_environment(target: Target, offline: bool) -> dict[str, str]:
    env = {key: value for key, value in os.environ.items() if key not in LEAKING_VARIABLES}
    env["UV_PROJECT_ENVIRONMENT"] = str(target.prefix)
    env["UV_PYTHON_DOWNLOADS"] = "never"
    # `uv pip install -r pylock.toml` is a preview feature in uv 0.11 (warning only).
    env["UV_PREVIEW_FEATURES"] = "pylock"
    if offline:
        env["UV_OFFLINE"] = "1"
    return env


def sync_command(
    uv: list[str],
    root: Path,
    target: Target,
    bundles: list[str],
    backend: Backend,
    offline: bool,
    reinstall: str | None = None,
) -> list[str]:
    command = [
        *uv,
        "sync",
        "--locked",
        "--inexact",
        "--no-default-groups",
        "--project",
        str(root),
        "--python",
        str(target.python),
    ]
    if offline:
        command.append("--offline")
    for extra in [*bundles, backend]:
        command += ["--extra", extra]
    if reinstall:
        command += ["--reinstall-package", reinstall]
    return command


def pip_install_command(
    uv: list[str], target: Target, lock: Path, offline: bool, reinstall: str | None = None
) -> list[str]:
    command = [*uv, "pip", "install", "--python", str(target.python), "-r", str(lock)]
    if offline:
        command.append("--offline")
    if reinstall:
        command += ["--reinstall-package", reinstall]
    return command


def installed_version(target: Target, distribution: str) -> str | None:
    result = subprocess.run(
        [str(target.python), "-c", _VERSION_SCRIPT, distribution],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout.strip() or None


def refuse_backend_switch(target: Target, backend: Backend) -> None:
    """A CUDA environment is not converted in place; that path is untested."""
    if backend == "cpu" and installed_version(target, ONNX_GPU_DISTRIBUTION):
        raise PrepareError(
            f"this environment was prepared with --backend cuda ({ONNX_GPU_DISTRIBUTION} is "
            "installed); use --backend cuda, or create a fresh virtualenv for a CPU-only setup"
        )


def run_installer(command: list[str], env: dict[str, str]) -> None:
    typer.echo(f"$ {shlex.join(command)}")
    result = subprocess.run(command, env=env)
    if result.returncode:
        raise PrepareError(f"installer exited with status {result.returncode}")


def verify_providers(target: Target, backend: Backend) -> None:
    result = subprocess.run(
        [str(target.python), "-c", _VERIFY_SCRIPT, PROVIDERS[backend]],
        capture_output=True,
        text=True,
    )
    if result.returncode:
        raise PrepareError(
            "verification failed after installation:\n" + (result.stderr or result.stdout).strip()
        )
    typer.echo(result.stdout.strip())


def prepare(bundles: list[str], backend: Backend, offline: bool) -> None:
    """Install ``bundles`` plus the ``backend`` extra into the current virtualenv."""
    target = Target.current()
    refuse_system_python(target)
    refuse_active_runs(target)
    uv = find_uv()
    refuse_backend_switch(target, backend)
    env = uv_environment(target, offline)
    root = checkout_root()
    if root is not None:
        commands = [sync_command(uv, root, target, bundles, backend, offline)]
        repair = sync_command(
            uv, root, target, bundles, backend, offline, reinstall=ONNX_GPU_DISTRIBUTION
        )
    else:
        locks = [lock_path(bundle, backend) for bundle in bundles]
        missing = [lock.name for lock in locks if not lock.is_file()]
        if missing:
            raise PrepareError(
                f"this dimos installation lacks its lock artifacts in {LOCKS_DIR}: "
                f"{', '.join(missing)}. The package was built without them; reinstall a "
                "release that ships them, or prepare from a source checkout."
            )
        commands = [pip_install_command(uv, target, lock, offline) for lock in locks]
        repair = pip_install_command(
            uv, target, locks[-1], offline, reinstall=ONNX_GPU_DISTRIBUTION
        )
    for command in commands:
        run_installer(command, env)
    if backend == "cuda":
        # chromadb and faster-whisper depend on the CPU onnxruntime, which unpacks into the
        # same `onnxruntime/` tree as onnxruntime-gpu. Whichever uv installed last wins, so
        # lay the GPU build down again (same locked version, no new resolution).
        typer.echo(f"Re-layering {ONNX_GPU_DISTRIBUTION} over the CPU onnxruntime")
        run_installer(repair, env)
    verify_providers(target, backend)
