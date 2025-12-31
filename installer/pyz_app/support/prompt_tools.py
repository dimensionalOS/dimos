#!/usr/bin/env python3
from __future__ import annotations

from typing import Iterable

from rich.console import Console
from rich.prompt import Confirm, Prompt

console = Console()


def clear_screen() -> None:
    console.print("\x1B[2J", end="")


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
    return Confirm.ask(text, default=True)


def prompt(text: str) -> str:
    return Prompt.ask(text)


def ask_yes_no(question: str) -> bool:
    while True:
        answer = Prompt.ask(question + " (y/n)").strip().lower()
        if answer in {"y", "yes"}:
            return True
        if answer in {"n", "no"}:
            return False
        console.print("[yellow][ please respond with y/n, yes/no, or use CTRL+C to cancel ][/]")


def pick_one(message: str, *, options: Iterable[str] | dict[str, str]):
    if isinstance(options, dict):
        keys = list(options.keys())
        values = [options[k] for k in keys]
    else:
        values = list(options)
        keys = values

    console.print(message)
    for idx, label in enumerate(values, start=1):
        console.print(f"  {idx}. {label}")

    while True:
        choice = Prompt.ask("Enter number", default="1").strip()
        if not choice.isdigit():
            console.print("[yellow]Please enter a valid number.[/]")
            continue
        idx = int(choice)
        if 1 <= idx <= len(keys):
            return keys[idx - 1]
        console.print("[yellow]Choice out of range; try again.[/]")


def pick_many(message: str, *, options: Iterable[str]) -> list[str]:
    values = list(options)
    console.print(message)
    for idx, label in enumerate(values, start=1):
        console.print(f"  {idx}. {label}")

    console.print("[dim]Enter numbers separated by commas (blank = none).[/]")
    while True:
        resp = Prompt.ask("Selection", default="").strip()
        if resp == "":
            return []
        parts = [p.strip() for p in resp.split(",") if p.strip()]
        if all(part.isdigit() for part in parts):
            chosen: list[str] = []
            for part in parts:
                idx = int(part)
                if 1 <= idx <= len(values):
                    chosen.append(values[idx - 1])
                else:
                    console.print(f"[yellow]{part} is out of range.[/]")
                    break
            else:
                seen = set()
                deduped: list[str] = []
                for c in chosen:
                    if c not in seen:
                        deduped.append(c)
                        seen.add(c)
                return deduped
        console.print("[yellow]Please enter comma-separated numbers from the list.[/]")


__all__ = [
    "ask_yes_no",
    "boring_log",
    "clear_screen",
    "confirm",
    "error",
    "header",
    "highlight",
    "pick_one",
    "pick_many",
    "prompt",
    "sub_header",
    "warning",
]
