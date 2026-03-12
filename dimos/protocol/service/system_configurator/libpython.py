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

"""Ensure libpython is available in the venv for MuJoCo's mjpython on macOS.

When Python is installed via uv, mjpython fails because it expects
libpython at .venv/lib/ but uv places it in its own managed directory.
This configurator creates a symlink so mjpython can find the library.
"""

from __future__ import annotations

import logging
from pathlib import Path
import platform
import sys

from dimos.protocol.service.system_configurator.base import SystemConfigurator

logger = logging.getLogger(__name__)


class LibPythonConfiguratorMacOS(SystemConfigurator):
    """Create a libpython symlink in the venv lib dir if missing (macOS only)."""

    critical = False

    def __init__(self) -> None:
        self._missing: list[tuple[Path, Path]] = []  # (symlink_target, real_dylib)

    def check(self) -> bool:
        if platform.system() != "Darwin":
            return True

        self._missing.clear()
        venv_lib = Path(sys.prefix) / "lib"
        real_lib = Path(sys.executable).resolve().parent.parent / "lib"

        for dylib in real_lib.glob("libpython*.dylib"):
            target = venv_lib / dylib.name
            if not target.exists():
                self._missing.append((target, dylib))

        return not self._missing

    def explanation(self) -> str | None:
        if not self._missing:
            return None
        lines = []
        for symlink_path, real_path in self._missing:
            lines.append(f"- Symlink {symlink_path} -> {real_path} (for mjpython)")
        return "\n".join(lines)

    def fix(self) -> None:
        for symlink_path, real_path in self._missing:
            try:
                symlink_path.parent.mkdir(parents=True, exist_ok=True)
                if symlink_path.is_symlink():
                    symlink_path.unlink()
                symlink_path.symlink_to(real_path)
                logger.warning("Created symlink %s -> %s for mjpython", symlink_path, real_path)
            except OSError as error:
                logger.warning(
                    "Failed to create symlink %s -> %s: %s", symlink_path, real_path, error
                )
