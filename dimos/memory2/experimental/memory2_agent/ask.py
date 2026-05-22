#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Ask the memory2 agent one question.

Usage:
    export OPENAI_API_KEY=...   # plus GOOGLE_API_KEY if using --model gemini-*
    python -m dimos.memory2.experimental.memory2_agent.ask --db PATH "How many rooms did you walk through?"
    python -m dimos.memory2.experimental.memory2_agent.ask --db PATH --model gpt-4.1-mini "Where is the biggest room?"
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time


def _save_trace(path_str: str, args, res, elapsed: float) -> None:
    """Write the run record to disk. JSON if path endswith .json, else text."""
    path = Path(path_str)
    path.parent.mkdir(parents=True, exist_ok=True)
    record = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "question": args.question,
        "model": args.model,
        "db": args.db,
        "iterations": res.iterations,
        "elapsed_s": round(elapsed, 2),
        "tool_calls": res.tool_calls,
        "error": res.error,
        "final_answer": res.final_answer,
    }
    if path.suffix.lower() == ".json":
        path.write_text(json.dumps(record, indent=2, default=str))
        return
    lines = [
        f"timestamp:   {record['timestamp']}",
        f"model:       {record['model']}",
        f"db:          {record['db']}",
        f"iterations:  {record['iterations']}",
        f"elapsed:     {record['elapsed_s']}s",
        "",
        "Question:",
        record["question"],
        "",
        f"Tools used ({len(record['tool_calls'])}):",
    ]
    for tc in record["tool_calls"]:
        lines.append(f"  - {tc['name']}({tc['args']})")
    lines.append("")
    if record["error"]:
        lines.append(f"ERROR: {record['error']}")
    else:
        lines.append("Final answer:")
        lines.append(record["final_answer"])
    path.write_text("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("question", help="The question to ask the agent")
    parser.add_argument(
        "--model",
        default="gpt-5.5",
        help="Chat model (default: gpt-5.5; also try gpt-4.1-mini, gpt-4o, gemini-2.5-flash)",
    )
    parser.add_argument(
        "--db",
        required=True,
        help="Recording .db path (required).",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Only print the final answer (no tool trace).",
    )
    parser.add_argument(
        "--save",
        metavar="PATH",
        help="Save the full trace (question, model, tool calls, final answer, "
        "timestamps) to a file. Extension '.json' writes JSON, anything "
        "else writes a human-readable text trace.",
    )
    args = parser.parse_args()

    if not Path(args.db).exists():
        print(f"recording not found at {args.db}", file=sys.stderr)
        return 2
    if not os.environ.get("OPENAI_API_KEY"):
        print("OPENAI_API_KEY not set", file=sys.stderr)
        return 2

    # Imports lazy so --help is fast.
    from dimos.memory2.experimental.memory2_agent.agent import run_question
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.models.embedding.clip import CLIPModel

    store = SqliteStore(path=args.db)
    clip = CLIPModel()

    try:
        if not args.quiet:
            print(f"Q: {args.question}")
            print(f"model: {args.model}")
            print()

        t_start = time.time()
        res = run_question(store, clip, args.question, model=args.model)
        elapsed = time.time() - t_start

        if res.error:
            print(f"ERROR: {res.error}", file=sys.stderr)
            if args.save:
                _save_trace(args.save, args, res, elapsed)
            return 1

        if not args.quiet:
            print(f"Tools used ({len(res.tool_calls)}):")
            for tc in res.tool_calls:
                print(f"  - {tc['name']}({tc['args']})")
            print()
            print(f"Iterations: {res.iterations}  ({elapsed:.1f}s wall)")
            print()
            print("Final answer:")
            print(res.final_answer)
        else:
            print(res.final_answer)

        if args.save:
            _save_trace(args.save, args, res, elapsed)
            if not args.quiet:
                print(f"\n(trace saved to {args.save})")

        return 0
    finally:
        store.stop()


if __name__ == "__main__":
    raise SystemExit(main())
