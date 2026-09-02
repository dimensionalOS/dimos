# Copyright 2026 Dimensional Inc.
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

"""Ask the agent about a recording, as of one moment in it.

    python -m scripts.ask recording.db "where is the backpack" --at 61

Pause the viewer, read the number off its timeline, ask here. ``--at`` is
required for the same reason ``as_of`` is: the asking time decides the answer,
so nothing picks it for you.

Prints and exits. The store is opened read-only and nothing is logged anywhere:
a question is about the recording, not part of it.

**The asking time is the harness's, not the model's.** The tool overwrites
whatever ``as_of`` the model puts in its query. A model free to choose it can
answer a question about 1:00 from what the robot saw at 7:00, and the answer
reads exactly as well.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import textwrap
import warnings

from dotenv import load_dotenv

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.experimental.memory_belief.api import (
    PROJECTIONS,
    SELECTS,
    QueryError,
    describe_store,
    execute,
)
from dimos.memory.store.sqlite import SqliteStore

# The model key lives in the project's .env, which nothing has loaded by the
# time a script runs.
load_dotenv(DIMOS_PROJECT_ROOT / ".env")

SYSTEM = (
    "You answer questions about what a robot saw, using belief_query. "
    "Compose the whole question into ONE call when you can. "
    "You are answering as of a fixed moment: the store will not show you anything "
    "later, and that is correct, not a fault to work around. "
    "Call the tool before saying anything is absent or out of vocabulary: the "
    "store decides that, not your guess about what a robot would detect. "
    "If status is unknown, say what is unknown and why -- do not guess. "
    "Report positions as coordinates in metres; this store holds no room names. "
    "Results carry no timestamps. Asked when something was seen, say the store "
    "cannot tell you -- do not answer where instead. "
    "High dispersion_m means the sightings are too scattered to be one object. "
    "Answer in one or two sentences."
)


def parse_at(value: str, t_start: float) -> float:
    """Turn what the viewer shows into an absolute asking time.

    The renderer logs a ``time`` timeline of seconds since the recording began,
    so the number on a paused playhead is an offset -- ``61`` or ``1:01``.
    """
    text = value.strip()
    if ":" in text:
        minutes, seconds = text.split(":", 1)
        return t_start + int(minutes) * 60 + float(seconds)
    return t_start + float(text)


def build_tool(store, as_of, seen):  # type: ignore[no-untyped-def]
    from langchain_core.tools import StructuredTool

    facts = describe_store(store)

    def belief_query(query: dict) -> str:
        q = dict(query or {})
        # Overwritten, never defaulted. The moment is the caller's.
        q["as_of"] = as_of
        seen["calls"] += 1
        try:
            envelope = execute(store, q)
        except QueryError as exc:
            return json.dumps({"status": "error", "message": str(exc)})
        for row in (envelope.get("result") or {}).get("positions", []):
            if row.get("entity_id"):
                seen["cited"].add(row["entity_id"])
        return json.dumps(envelope, default=str)[:4000]

    return StructuredTool.from_function(
        func=belief_query,
        name="belief_query",
        description=f"""Ask where things are, as of a fixed moment.

  time: {facts["time_unit"]}
        answering as of {as_of:.0f}; nothing after it is visible
  vocabulary: {facts["vocabulary_size"]} terms; a term outside it returns
        OUT_OF_VOCABULARY with nearer terms, which are worth retrying

select: one of {list(SELECTS)}
where: clauses, ANDed. {{"op":"label","value":"chair"}} and
       {{"op":"time_range","t1":..,"t2":..}}
project: one of {list(PROJECTIONS)}

Returns status(ok|unknown|error), reason, result, and quality(rows, support,
dispersion_m, coherence, deduplicated). status=error means the query was
malformed; fix it and call again rather than reporting an absence.""",
    )


WIDTH = 76


def wrapped(text: str, tag: str) -> str:
    """One tagged block: the tag on the first line, the rest hanging under it."""
    lines = [
        wrap
        for para in text.strip().splitlines()
        for wrap in (textwrap.wrap(para, WIDTH - 3) or [""])
    ]
    return "\n".join(f"{tag if i == 0 else '':<3}{line}" for i, line in enumerate(lines))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("store", type=Path)
    parser.add_argument("question")
    parser.add_argument(
        "--at", required=True, help="asking time as the viewer's timeline reads it: 61, or 1:01"
    )
    parser.add_argument("--model", default="gpt-4o")
    args = parser.parse_args(argv)

    # langchain_core's import installs a filter that unmutes its own pending
    # deprecations, so ours has to be registered after it to win. The one this
    # silences is about a default this script never sets, and it would land
    # above the answer.
    import langchain_core  # noqa: F401

    warnings.filterwarnings("ignore", category=PendingDeprecationWarning)

    from langchain.agents import create_agent
    from langchain_core.messages import HumanMessage, SystemMessage
    from langchain_openai import ChatOpenAI

    store = SqliteStore(path=str(args.store), must_exist=True)
    try:
        facts = describe_store(store)
        if facts["t_start"] is None:
            print(f"{args.store} holds no belief; run detect_recording first", file=sys.stderr)
            return 1

        as_of = parse_at(args.at, facts["t_start"])
        print(
            f"\n{args.store.name}   t+{as_of - facts['t_start']:.0f}s "
            f"of {facts['t_end'] - facts['t_start']:.0f}s\n"
        )
        print(wrapped(args.question, "Q"), end="\n\n")

        seen: dict = {"calls": 0, "cited": set()}
        agent = create_agent(
            model=ChatOpenAI(model=args.model, temperature=0),
            tools=[build_tool(store, as_of, seen)],
            system_prompt=SYSTEM,
        )
        result = agent.invoke({"messages": [SystemMessage(SYSTEM), HumanMessage(args.question)]})
    finally:
        store.stop()

    print(wrapped(result["messages"][-1].content, "A"), end="\n\n")
    # Zero tool calls means it answered from the question alone, which is the
    # one failure a plausible sentence hides.
    print(f"{'':<3}{seen['calls']} tool call(s) · {len(seen['cited'])} entities cited\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
