#!/usr/bin/env python3
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

from __future__ import annotations

from typing import TYPE_CHECKING, Union

from InquirerPy import inquirer

if TYPE_CHECKING:
    from collections.abc import Iterable

from .installer_status import installer_status

# Manual ANSI helpers (basic 8-color + bold/dim)
RESET = "\x1b[0m"
BOLD = "\x1b[1m"
DIM = "\x1b[2m"
FG_RED = "\x1b[31m"
FG_GREEN = "\x1b[32m"
FG_YELLOW = "\x1b[33m"
FG_CYAN = "\x1b[36m"


def _color_help(text, style):
    # there is a smarter way to do this, but this works for now
    return text.replace(RESET, RESET + style)


def red(text: str) -> str:
    return f"{FG_RED}{_color_help(text, FG_RED)}{RESET}"


def cyan(text: str) -> str:
    return f"{FG_CYAN}{_color_help(text, FG_CYAN)}{RESET}"


def clear_screen() -> None:
    print("\x1b[2J")


def header(text: str) -> None:
    padding = 2
    content = f"{text}"
    width = len(content) + padding * 2
    top = f"{BOLD}{FG_GREEN}┌{'─' * width}┐{RESET}"
    mid = f"{BOLD}{FG_GREEN}│{RESET}{' ' * padding}{_color_help(content, BOLD + FG_GREEN)}{' ' * padding}{BOLD}{FG_GREEN}│{RESET}"
    bottom = f"{BOLD}{FG_GREEN}└{'─' * width}┘{RESET}"
    print("\n" * 2)
    print(top)
    print(mid)
    print(bottom)
    print()


def sub_header(text: str) -> None:
    print(f"{BOLD}{FG_YELLOW}{_color_help(text, BOLD + FG_YELLOW)}{RESET}")


def boring_log(text: str) -> None:
    print(f"{DIM}{_color_help(text, DIM)}{RESET}")


def error(text: str) -> None:
    print(f"{FG_RED}{_color_help(text, FG_RED)}{RESET}")


def warning(text: str) -> None:
    print(f"{FG_YELLOW}Warning: {RESET}{_color_help(text, FG_YELLOW)}{RESET}")


def highlight(text: str) -> str:
    return f"{FG_CYAN}{_color_help(text, FG_CYAN)}{RESET}"


def confirm(text: str) -> None:
    if installer_status.get("non_interactive"):
        print(f"""- continuing past '{text}\'""")
        return
    input(f"{FG_YELLOW}{text}{RESET}")


def ask_yes_no(question: str) -> bool:
    if installer_status.get("non_interactive"):
        return True
    return bool(inquirer.confirm(message=question, default=True).execute())


def _normalize_options(
    options: Union[Iterable[str], dict[str, str]],
) -> tuple[list[str], list[str]]:
    if isinstance(options, dict):
        keys = list(options.keys())
        values = [options[k] for k in keys]
    else:
        values = list(options)
        keys = values
    return keys, values


def pick_one(message: str, *, options: Iterable[str] | dict[str, str]):
    keys, values = _normalize_options(options)
    choice = inquirer.select(
        message=message,
        choices=values,
        cycle=True,
        pointer="❯",  # noqa: RUF001
        multiselect=False,
        border=True,
        qmark="?",
    ).execute()
    # Map back to key (handles dict or list case)
    return keys[values.index(choice)]


def pick_many(message: str, *, options: Iterable[str] | dict[str, str]) -> list[str]:
    keys, values = _normalize_options(options)
    selected = inquirer.checkbox(
        message=message,
        choices=values,
        cycle=True,
        border=True,
        pointer="❯",  # noqa: RUF001
        instruction="Space to toggle, Enter to confirm",
    ).execute()
    return [keys[values.index(v)] for v in selected]


__all__ = [
    "ask_yes_no",
    "boring_log",
    "clear_screen",
    "confirm",
    "error",
    "header",
    "highlight",
    "pick_many",
    "pick_one",
    "sub_header",
    "warning",
]
