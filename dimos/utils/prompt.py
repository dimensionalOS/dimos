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

"""Unified prompts for DimOS.

Usage::

    from dimos.utils.prompt import confirm, sudo_prompt

    if confirm("Apply these changes now?", question_id="autoconf"):
        ...

    if sudo_prompt("Need sudo for multicast setup"):
        # sudo credentials are now cached
        subprocess.run(["sudo", "route", "add", ...])

When running inside ``dio`` (the DimOS TUI), prompts are rendered as
modal popups.  Otherwise styled terminal prompts are shown via *rich*.

Prompts are safe to call in modules (despite potentially no stdin, or
potentially a TUI that eats/controls the stdin).
"""

from __future__ import annotations

import getpass
import os
import subprocess
import sys
import threading
from typing import Any

# Global hooks -- set by the dio app when it is running so that prompts
# can route through the TUI instead of stdin.
_dio_confirm_hook: Any = None  # callable(message, default) -> bool | None
_dio_sudo_hook: Any = None  # callable(message) -> bool | None
_lock = threading.Lock()

# In-memory answer cache keyed by question_id
_answer_cache: dict[str, bool] = {}


def set_dio_hook(hook: Any) -> None:
    """Register the dio TUI confirm handler (called by DIOApp on startup)."""
    global _dio_confirm_hook
    with _lock:
        _dio_confirm_hook = hook


def set_dio_sudo_hook(hook: Any) -> None:
    """Register the dio TUI sudo handler (called by DIOApp on startup)."""
    global _dio_sudo_hook
    with _lock:
        _dio_sudo_hook = hook


def clear_dio_hook() -> None:
    """Unregister all dio TUI handlers (called by DIOApp on shutdown)."""
    global _dio_confirm_hook, _dio_sudo_hook
    with _lock:
        _dio_confirm_hook = None
        _dio_sudo_hook = None


def confirm(
    message: str,
    *,
    default: bool = False,
    question_id: str | None = None,
) -> bool:
    """Ask the user a yes/no question and return the answer.

    Parameters
    ----------
    message:
        The question to display.
    default:
        The value returned when the user presses Enter without typing.
        In non-interactive mode (no tty), returns *default* without prompting.
    question_id:
        Optional stable identifier. If provided, the answer is cached
        in memory and subsequent calls with the same id return the
        cached value without prompting again.

    Returns
    -------
    bool
        ``True`` for yes, ``False`` for no.
    """
    if question_id is not None:
        with _lock:
            if question_id in _answer_cache:
                return _answer_cache[question_id]

    with _lock:
        hook = _dio_confirm_hook

    if hook is not None:
        result = hook(message, default)
        if result is not None:
            answer = bool(result)
            if question_id is not None:
                with _lock:
                    _answer_cache[question_id] = answer
            return answer

    # Non-interactive stdin (piped, /dev/null, etc.) — auto-accept default
    if not sys.stdin.isatty():
        answer_str = "yes" if default else "no"
        print(f"assuming {answer_str} for: {message}")
        if question_id is not None:
            with _lock:
                _answer_cache[question_id] = default
        return default

    # Fallback: nice terminal prompt via rich
    result = _terminal_confirm(message, default)
    if question_id is not None:
        with _lock:
            _answer_cache[question_id] = result
    return result


def sudo_prompt(message: str = "sudo password required") -> bool:
    """Prompt for a sudo password and cache the credentials.

    In dio, this shows a password input modal. Outside dio, it uses
    getpass in the terminal. Either way, the password is passed to
    ``sudo -S true`` to validate and cache credentials.

    Returns
    -------
    bool
        ``True`` if sudo credentials are now cached, ``False`` if the
        user cancelled or the password was wrong.
    """
    # Check if sudo is already cached (no password needed)
    result = subprocess.run(
        ["sudo", "-n", "true"],
        capture_output=True,
    )
    if result.returncode == 0:
        return True

    # Try TUI hook first (works even when stdin is not a tty)
    with _lock:
        hook = _dio_sudo_hook

    if hook is not None:
        hook_result = hook(message)
        if hook_result is not None:
            return bool(hook_result)

    # Non-interactive stdin — can't prompt for password
    if not sys.stdin.isatty():
        print(f"assuming no for: {message} (cannot prompt for password non-interactively)")
        return False

    return _terminal_sudo(message)


def sudo_run(*args: Any, **kwargs: Any) -> subprocess.CompletedProcess[str]:
    """Run a command, prepending sudo if not already root."""
    try:
        is_root = os.geteuid() == 0
    except AttributeError:
        is_root = False
    if is_root:
        return subprocess.run(list(args), **kwargs)
    return subprocess.run(["sudo", *args], **kwargs)


def _terminal_confirm(message: str, default: bool) -> bool:
    """Rich-powered terminal yes/no prompt."""
    try:
        from rich.console import Console
        from rich.panel import Panel
        from rich.text import Text

        console = Console()

        body = Text(message, style="bold #b5e4f4")

        console.print()
        console.print(
            Panel(
                body,
                border_style="#00eeee",
                padding=(1, 3),
                title="[bold #00eeee]confirm[/bold #00eeee]",
                title_align="left",
            )
        )

        if default:
            console.print("[bold #00eeee]y[/bold #00eeee][#404040]/n:[/#404040] ", end="")
        else:
            console.print(
                "[#404040]y/[/#404040][bold #00eeee]n[/bold #00eeee][#404040]:[/#404040] ", end=""
            )

        try:
            answer = input().strip().lower()
        except (EOFError, KeyboardInterrupt):
            return default
        if not answer:
            return default
        return answer in ("y", "yes")
    except Exception:
        # Absolute last resort
        hint = "[Y/n]" if default else "[y/N]"
        try:
            answer = input(f"{message} {hint} ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return default
        if not answer:
            return default
        return answer in ("y", "yes")


def _terminal_sudo(message: str) -> bool:
    """Terminal sudo prompt using getpass."""
    try:
        from rich.console import Console
        from rich.panel import Panel
        from rich.text import Text

        console = Console()
        body = Text(message, style="bold #b5e4f4")
        console.print()
        console.print(
            Panel(
                body,
                border_style="#ffcc00",
                padding=(1, 3),
                title="[bold #ffcc00]sudo[/bold #ffcc00]",
                title_align="left",
            )
        )
    except Exception:
        print(message)

    for attempt in range(3):
        try:
            password = getpass.getpass("\033[93mPassword:\033[0m ")
        except (EOFError, KeyboardInterrupt):
            return False
        if not password:
            return False

        result = subprocess.run(
            ["sudo", "-S", "true"],
            input=password + "\n",
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            return True
        print(
            "\033[91mIncorrect password, try again.\033[0m"
            if attempt < 2
            else "\033[91mIncorrect password.\033[0m"
        )
    return False
