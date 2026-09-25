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

"""Launching MuJoCo's ``mjpython`` interpreter, which native viewers require on macOS.

``mujoco.viewer.launch_passive`` needs the Cocoa main thread, which CPython does not
give it. MuJoCo ships ``mjpython`` to own that thread and hand the window off, so on
macOS any process that opens a native viewer must be started by ``mjpython`` rather
than ``sys.executable``.
"""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import sysconfig

_PERSISTENT_UI_DOMAIN = "org.mujoco.mjpython"


def mjpython_executable() -> Path:
    """Resolve the ``mjpython`` shipped alongside the running interpreter.

    Resolved next to ``sys.executable`` rather than through ``PATH`` so it keeps working
    when the venv is not activated, and so it can never pick up a different environment's
    MuJoCo.
    """
    if sys.platform != "darwin":
        raise RuntimeError(f"mjpython is only used on macOS, not {sys.platform!r}")

    mjpython = Path(sys.executable).with_name("mjpython")
    if not mjpython.is_file() or not os.access(mjpython, os.X_OK):
        raise RuntimeError(
            f"MuJoCo's mjpython launcher is missing from this environment: {mjpython}. "
            "Install the 'sim' dependency group to get it."
        )
    return mjpython


def mjpython_env(base_env: dict[str, str] | None = None) -> dict[str, str]:
    """Return an environment in which ``mjpython`` can load libpython.

    ``mjpython`` dlopens the interpreter binary, whose install name is
    ``@rpath/libpython3.X.dylib``. uv-installed Pythons resolve that rpath relative to
    their real prefix, which is outside the venv, so inside a venv the dlopen fails with
    "Library not loaded: @rpath/libpython3.X.dylib" and mjpython dies before running any
    dimos code. Prepending the interpreter's own LIBDIR makes it resolvable.
    """
    env = dict(os.environ if base_env is None else base_env)
    if sys.platform != "darwin":
        return env

    # Guard the empty string: Path("") is Path("."), which would silently put the
    # working directory on the dylib search path.
    configured_libdir = sysconfig.get_config_var("LIBDIR")
    if not configured_libdir:
        return env

    libdir = Path(configured_libdir)
    if libdir.is_dir():
        existing = env.get("DYLD_LIBRARY_PATH", "")
        env["DYLD_LIBRARY_PATH"] = f"{libdir}:{existing}" if existing else str(libdir)
    return env


def allow_mjpython_to_start_after_a_crash() -> None:
    """Stop AppKit from blocking every future ``mjpython`` launch on an unseen prompt.

    ``mjpython`` is an app bundle, so macOS records crash history for it. Any unclean
    exit -- Ctrl-C on a running sim is enough -- makes the next launch block forever in
    ``promptToIgnorePersistentStateWithCrashHistory``, waiting on a "reopen windows?"
    dialog that a headless-launched process can never show. It hangs before running a
    single line of Python, so the only symptom is the sim never coming up.
    """
    if sys.platform != "darwin":
        return

    subprocess.run(
        [
            "defaults",
            "write",
            _PERSISTENT_UI_DOMAIN,
            "ApplePersistenceIgnoreState",
            "-bool",
            "true",
        ],
        check=False,
        capture_output=True,
        timeout=30,
    )


def prepare_mjpython_launch() -> tuple[Path, dict[str, str]]:
    """Return the ``mjpython`` to run and the environment it needs to run in."""
    mjpython = mjpython_executable()
    allow_mjpython_to_start_after_a_crash()
    return mjpython, mjpython_env()
