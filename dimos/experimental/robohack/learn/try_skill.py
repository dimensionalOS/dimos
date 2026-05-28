#!/usr/bin/env python3
"""
Minimal, TESTED hello-world for the DimOS @skill decorator.

This is the single most important seam for the hackathon: a method decorated
with @skill becomes both an LLM tool and (when on a Module in a blueprint) an
MCP tool. The decorator just wraps the function with timing/context tracking
and logs a `SKILL <name> result=... duration_ms=...` line on every call.

Verified working against installed dimos 0.0.12.post2 (PyPI) on 2026-05-26.

Run:
    source /home/kristjan/code/dimos/.venv/bin/activate
    python learn/try_skill.py
"""

from dimos.agents.annotation import skill


class DemoSkills:
    """A plain object, just to exercise the decorator without the Module
    worker machinery. In real use these live on a `Module` subclass — see
    patrol_dog_skills.py."""

    @skill
    def greet(self, name: str = "world") -> str:
        """Say hello to NAME.

        The docstring is NOT optional — the LLM agent reads it (plus the typed
        params) to decide when and how to call this skill. Treat it as the
        tool's prompt.
        """
        return f"hello, {name}"

    @skill
    def add(self, a: float = 0.0, b: float = 0.0) -> str:
        """Add two numbers and report the sum."""
        return f"{a} + {b} = {a + b}"


if __name__ == "__main__":
    d = DemoSkills()
    print("greet ->", repr(d.greet("patrol dog")))
    print("add   ->", repr(d.add(2, 3)))
    print("\nNote: a `SKILL greet result=... duration_ms=...` line is emitted")
    print("by the dimos logger on each call (look in the log output above).")
