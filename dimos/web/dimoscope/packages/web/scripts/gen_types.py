#!/usr/bin/env python3
"""Generate the browser SDK's typed maps from dimos blueprints — statically, no gateway/robot.

Imports one or more scenario blueprints (side-effect-free: class definition only, never instantiated),
reads each `PORTS` list for topic↔type pairs and reflects every `@rpc` Module method for command
signatures, then MERGES them into a single `dimos.topics.gen.ts` (DimosTopics + DimosCommands) the app
consumes. The client lifts the `{args, ret}` command descriptors into a callable
`modules.<Target>.<method>()` (RpcFn in client.ts).

  uv run python packages/web/scripts/gen_types.py scenarios/nav.py
  uv run python packages/web/scripts/gen_types.py scenarios/*.py --out app/src/dimos.topics.gen.ts
"""

from __future__ import annotations

import argparse
import importlib.util
import inspect
from pathlib import Path
import sys
import types
import typing

# Python → TS scalar map for @rpc arg/return types.
_SCALARS = {bool: "boolean", int: "number", float: "number", str: "string"}
_UNION = (typing.Union, types.UnionType)


def _ts_type(ann: object, pkgs: set[str], *, ret: bool = False) -> str:
    """Map a Python annotation to a TS type, registering any @dimos/msgs package it needs in `pkgs`.
    Handles scalars, dimos message classes (→ `pkg.Name`), `list[X]` (→ `X[]`), and `Optional`/unions
    (→ `... | null`); anything else → `unknown`."""
    if ann is None or ann is type(None):
        return "void" if ret else "null"
    if ann is inspect.Parameter.empty:  # unannotated
        return "void" if ret else "unknown"
    if ann in _SCALARS:
        return _SCALARS[ann]  # type: ignore[index]
    name = getattr(ann, "msg_name", None)  # a dimos message class → "pkg.Name"
    if isinstance(name, str) and "." in name:
        pkgs.add(name.split(".")[0])
        return name
    origin, args = typing.get_origin(ann), typing.get_args(ann)
    if origin is list:
        return f"{_ts_type(args[0], pkgs) if args else 'unknown'}[]"
    if origin in _UNION:
        non_none = [a for a in args if a is not type(None)]
        parts = list(dict.fromkeys(_ts_type(a, pkgs) for a in non_none)) or ["unknown"]
        ts = " | ".join(parts)
        return f"{ts} | null" if len(non_none) != len(args) else ts
    return "unknown"


def _load(path: Path):
    """Import a blueprint by file path (its dir on sys.path so `common` resolves); __main__ never runs."""
    sys.path.insert(0, str(path.resolve().parent))
    spec = importlib.util.spec_from_file_location(path.stem, path)
    if not spec or not spec.loader:
        raise SystemExit(f"gen_types: cannot load {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _collect(
    mod: object,
    pkgs: set[str],
    topics: dict[str, str | None],
    conflicts: dict[str, set[str]],
    commands: dict[str, list[str]],
) -> None:
    """Merge one blueprint module's PORTS (→ topics) and every own @rpc Module (→ commands) into the
    shared accumulators. A file may contribute topics, commands, both, or neither (e.g. common.py)."""
    from dimos.core.module import Module  # local import: only needed once dimos is on the path

    # Topics: (attr, topic, MsgClass) → topic: <pkg>.<Name> (msg_name is already "pkg.Name"). PORTS is
    # optional — a blueprint may define its topics only in __main__ (bench.py); those stay untyped.
    for _attr, topic, cls in getattr(mod, "PORTS", None) or ():
        name = getattr(cls, "msg_name", None)
        ts = name if (isinstance(name, str) and "." in name) else None
        if ts:
            pkgs.add(ts.split(".")[0])
        if topic in topics and topics[topic] != ts and topics[topic] is not None and ts is not None:
            conflicts.setdefault(topic, set()).add(ts)  # keep the first; note the divergence
        topics.setdefault(topic, ts)

    # Commands: EVERY Module subclass defined in this file → target: { method: { args:[T..]; ret:T } }.
    for target in vars(mod).values():
        if not (
            inspect.isclass(target)
            and issubclass(target, Module)
            and target is not Module
            and target.__module__ == mod.__name__  # type: ignore[attr-defined]
        ):
            continue
        method_rows: list[str] = []
        # vars(target) = methods declared ON the blueprint (skips inherited Module @rpc plumbing whose
        # annotations reference unresolved names). Read annotations straight off the signature.
        for mname, fn in sorted(vars(target).items()):
            if mname.startswith("_") or not callable(fn) or not getattr(fn, "__rpc__", False):
                continue
            sig = inspect.signature(fn)
            args = [_ts_type(p.annotation, pkgs) for n, p in sig.parameters.items() if n != "self"]
            ret = _ts_type(sig.return_annotation, pkgs, ret=True)
            method_rows.append(f'    "{mname}": {{ args: [{", ".join(args)}]; ret: {ret} }};')
        if method_rows:
            commands[target.__name__] = method_rows


def generate(paths: list[Path]) -> str:
    pkgs: set[str] = set()
    topics: dict[str, str | None] = {}  # topic → "pkg.Name" (or None for untyped)
    conflicts: dict[str, set[str]] = {}  # topic → other types seen (kept the first)
    commands: dict[str, list[str]] = {}  # target class name → its method rows
    for path in paths:
        _collect(_load(path), pkgs, topics, conflicts, commands)

    topic_rows: list[str] = []
    for topic, ts in topics.items():
        note = f"  // conflict: also seen as {', '.join(sorted(conflicts[topic]))}" if topic in conflicts else ""
        topic_rows.append(f'  "{topic}": {ts};{note}' if ts else f'  "{topic}": unknown;{note}')

    cmd_block = (
        "\n".join(f'  "{t}": {{\n' + "\n".join(rows) + "\n  };" for t, rows in commands.items())
        if commands
        else "  // (no @rpc methods)"
    )

    imports = f'import type {{ {", ".join(sorted(pkgs))} }} from "@dimos/msgs";\n\n' if pkgs else ""
    sources = ", ".join(p.as_posix() for p in paths)
    return (
        f"// AUTO-GENERATED by `deno task gen-types` from {sources}. DO NOT EDIT.\n"
        f"// {len(topics)} topics · {len(commands)} command targets (static — no gateway).\n\n"
        f"{imports}"
        f"export interface DimosTopics {{\n" + "\n".join(topic_rows) + "\n}\n\n"
        f"export interface DimosCommands {{\n{cmd_block}\n}}\n\n"
        f"export type TopicName = keyof DimosTopics;\n"
        f"export type MsgOf<K extends TopicName> = DimosTopics[K];\n"
    )


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Generate merged DimosTopics/DimosCommands from one or more dimos blueprints."
    )
    ap.add_argument(
        "blueprints", nargs="+", help="paths to blueprint .py files (e.g. scenarios/*.py)"
    )
    ap.add_argument("--out", help="write to this file instead of stdout")
    a = ap.parse_args()
    ts = generate([Path(p) for p in a.blueprints])
    if a.out:
        Path(a.out).write_text(ts)
        print(f"gen_types: wrote {a.out}", file=sys.stderr)
    else:
        sys.stdout.write(ts)


if __name__ == "__main__":
    main()
