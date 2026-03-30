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

"""
Prompts that are safe to call in modules (despite potentially no stdin, or potentially a TUI that eats/controls the stdin)
"""

from __future__ import annotations

import os
import subprocess
import sys
from typing import Any


def confirm(message: str, *, default: bool = True) -> bool:
    """Ask yes/no.

    In non-interactive mode (no tty), returns *default* without prompting
    — useful for daemons and CI where stdin is unavailable.

    In interactive mode, no default is pre-selected so the user must
    explicitly type ``y`` or ``n``.  This prevents accidental Enter-mashing
    from silently triggering system changes (some of which require sudo).
    """
    if not sys.stdin.isatty():
        return default
    import typer

    # No default in interactive mode — require explicit y/n
    return typer.confirm(message)


def sudo_run(*args: Any, **kwargs: Any) -> subprocess.CompletedProcess[str]:
    """Run a command, prepending sudo if not already root."""
    try:
        is_root = os.geteuid() == 0
    except AttributeError:
        is_root = False
    if is_root:
        return subprocess.run(list(args), **kwargs)
    return subprocess.run(["sudo", *args], **kwargs)
