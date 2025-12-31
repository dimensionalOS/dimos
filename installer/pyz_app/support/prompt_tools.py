#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

from typing import TYPE_CHECKING, List, Tuple, Union

from InquirerPy import inquirer
from rich.console import Console

if TYPE_CHECKING:
    from collections.abc import Iterable

console = Console()


def clear_screen() -> None:
    print("\x1b[2J")


def header(text: str) -> None:
    console.print(f"[bold green]{text}[/]")


def sub_header(text: str) -> None:
    console.print(f"[bold yellow]{text}[/]")


def boring_log(text: str) -> None:
    console.print(f"[dim]{text}[/]")


def error(text: str) -> None:
    console.print(f"[red]{text}[/]")


def warning(text: str) -> None:
    console.print(f"[yellow]{text}[/]")


def highlight(text: str) -> str:
    return f"[cyan]{text}[/]"


def confirm(text: str) -> bool:
    return bool(inquirer.confirm(message=text, default=True).execute())


def prompt(text: str) -> str:
    return inquirer.text(message=text).execute()


def ask_yes_no(question: str) -> bool:
    return confirm(question)


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
        pointer="❯",
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
        pointer="❯",
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
    "prompt",
    "sub_header",
    "warning",
]
