# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Skill registry — load skill descriptions and bodies from ./skills/*.md.

A "skill" is a step-by-step procedure for solving a particular kind of
problem. Skills are stored as Markdown files with a YAML-ish header:

    ---
    name: <slug>
    description: <one-line description used to decide whether to load it>
    ---
    <body — the actual procedure>

The agent sees only names + descriptions by default. It loads a skill's
full body on demand via the `load_skill` tool, so context isn't bloated
by skills irrelevant to the current question.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re

SKILLS_DIR = Path(__file__).parent / "skills"

_HEADER_RE = re.compile(r"^---\s*\n(.*?)\n---\s*\n", re.DOTALL)


@dataclass(frozen=True)
class Skill:
    name: str
    description: str
    body: str
    path: Path


def _parse(path: Path) -> Skill:
    text = path.read_text()
    name = path.stem
    description = ""
    body = text
    m = _HEADER_RE.match(text)
    if m:
        for line in m.group(1).strip().splitlines():
            if ":" in line:
                k, v = line.split(":", 1)
                k = k.strip().lower()
                v = v.strip()
                if k == "name":
                    name = v
                elif k == "description":
                    description = v
        body = text[m.end() :]
    return Skill(name=name, description=description, body=body, path=path)


def list_skills() -> list[Skill]:
    if not SKILLS_DIR.exists():
        return []
    return [_parse(p) for p in sorted(SKILLS_DIR.glob("*.md"))]


def load_skill(name: str) -> Skill | None:
    for s in list_skills():
        if s.name == name:
            return s
    return None
