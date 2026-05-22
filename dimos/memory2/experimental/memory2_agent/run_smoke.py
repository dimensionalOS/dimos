# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""End-to-end smoke test for the memory2 LangChain agent.

Usage:
    export OPENAI_API_KEY=...
    python -m dimos.memory2.experimental.memory2_agent.run_smoke --db PATH
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

from dimos.memory2.experimental.memory2_agent.agent import run_question
from dimos.memory2.store.sqlite import SqliteStore
from dimos.models.embedding.clip import CLIPModel

QUESTIONS = [
    # Orientation
    "How many streams does this memory store have, and how long was the recording?",
    # Map semantic search
    "Where in the room is somewhere I could sit down?",
    # Map tag filter
    "List every door in the map and where each one is.",
    # Map spatial query
    "What objects are near the point (1.5, 12)? Use a 3 m radius.",
    # Ego-centric semantic search
    "Did the robot see any people during the recording? If yes, roughly where was it standing when it saw them?",
    # Cross-stream — combines map + ego
    "Is there a coffee machine in the room, and did the robot walk close to it?",
    # Spatial / directional ego-memory — uses show_map + recall_view together.
    # At t=22s the robot has just rounded the top of its loop; this asks it to
    # locate itself, then surface the actual recorded views in two relative
    # directions and reason about whether they differ.
    (
        "At t=22s, show me where the robot was on the map, then show me what "
        "the robot saw directly forward and what it saw to its right at that "
        "moment. Describe each view in one sentence."
    ),
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", type=Path, required=True, help="Recording .db path (required).")
    args = parser.parse_args()

    if not args.db.exists():
        print(f"recording not found at {args.db}", file=sys.stderr)
        return 2
    if not os.environ.get("OPENAI_API_KEY"):
        print("OPENAI_API_KEY not set", file=sys.stderr)
        return 2

    store = SqliteStore(path=str(args.db))
    clip = CLIPModel()

    model = os.environ.get("MEMORY2_AGENT_MODEL", "gpt-4.1-mini")
    print(f"[smoke] model = {model}\n")

    for i, q in enumerate(QUESTIONS, 1):
        print(f"=== Q{i}: {q}")
        result = run_question(store, clip, q, model=model)
        if result.error:
            print(f"  ERROR: {result.error}\n")
            continue
        print(f"  Tools used ({len(result.tool_calls)}):")
        for tc in result.tool_calls:
            print(f"    - {tc['name']}({tc['args']})")
        print(f"  Answer: {result.final_answer}\n")

    store.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
