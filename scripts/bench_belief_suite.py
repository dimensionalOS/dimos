#!/usr/bin/env python3
"""Run the reachable benchmark through the belief agent, one Langfuse trace per task.

    python -m scripts.bench_belief_suite --store moshi_8min.db --suite suite.json

Every task opens its own trace named ``bench/<id>`` and carries the audit's own
verdict as metadata, so the report afterwards is assembled from what Langfuse
recorded rather than from what this script believed at the time. The distinction
matters: a run that scores itself can only report the failures it anticipated.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time
import uuid

from dotenv import load_dotenv

from dimos.constants import DIMOS_PROJECT_ROOT

load_dotenv(DIMOS_PROJECT_ROOT / ".env", override=True)
# The repo's .env names the endpoint LANGFUSE_BASE_URL; the SDK reads LANGFUSE_HOST.
if os.environ.get("LANGFUSE_BASE_URL") and not os.environ.get("LANGFUSE_HOST"):
    os.environ["LANGFUSE_HOST"] = os.environ["LANGFUSE_BASE_URL"]

from langchain.agents import create_agent
from langchain_core.messages import HumanMessage, SystemMessage
from langchain_core.tools import StructuredTool
from langchain_openai import ChatOpenAI

from dimos.experimental.memory_belief.api import (
    QueryError,
    describe_store,
    execute,
)
from dimos.memory.store.sqlite import SqliteStore
from dimos.utils.tracing import agent_trace, langchain_handler

SYS = (
    "You answer questions about what a robot saw, using belief_query. "
    "Compose the whole question into ONE call when you can. "
    "Read the quality block: high dispersion_m means the sightings are too scattered to "
    "be one object, so the position is an average of several. If status is unknown, say "
    "what is unknown and why -- do not guess. "
    "A status of 'error' means your query was malformed, not that the robot saw nothing: "
    "the message says what is wrong, so fix the query and call again before answering. "
    "Same for OUT_OF_VOCABULARY when it offers nearer terms -- try one. "
    "Positions are metres in the map frame, not room names -- report them as coordinates. "
    "If the store cannot answer at all, say so plainly. Answer in one or two sentences."
)


#: Characters of tool output the model sees. The first traced run cut the JSON at
#: this length with a slice, which severed it mid-array and dropped the `quality`
#: block the prompt tells the model to trust -- every successful query in that run
#: arrived unparseable. Shrinking the result list instead keeps the envelope whole.
BUDGET = 4000


def _fit(envelope: dict, budget: int = BUDGET) -> str:
    """Serialise the envelope, shrinking long result lists rather than the JSON.

    Reports what it dropped. Silently keeping the first twenty of two hundred
    positions is how "where are the chairs" becomes "where are the first twenty
    chairs found", with nothing in the answer to say so.
    """

    def dump(env: dict) -> str:
        return json.dumps(env, ensure_ascii=False, default=str)

    text = dump(envelope)
    if len(text) <= budget:
        return text

    result = envelope.get("result")
    lists = (
        [(k, v) for k, v in result.items() if isinstance(v, list)]
        if isinstance(result, dict)
        else ([("result", result)] if isinstance(result, list) else [])
    )
    if not lists:
        # Nothing list-shaped to shrink: drop the result entirely rather than
        # return a broken envelope, and say that is what happened.
        trimmed = {
            **envelope,
            "result": None,
            "truncated": {"result": "omitted, too large to send"},
        }
        return dump(trimmed)

    for keep in (200, 100, 50, 25, 12, 6, 3, 1):
        shrunk = dict(result) if isinstance(result, dict) else None
        note = {}
        for key, value in lists:
            if len(value) > keep:
                note[key] = f"showing {keep} of {len(value)}"
                if shrunk is not None:
                    shrunk[key] = value[:keep]
        payload = shrunk if shrunk is not None else lists[0][1][:keep]
        if not note:
            continue
        trimmed = {**envelope, "result": payload, "truncated": note}
        text = dump(trimmed)
        if len(text) <= budget:
            return text
    return dump({**envelope, "result": None, "truncated": {"result": "omitted, too large to send"}})


def build_tool(store, as_of, counter):  # type: ignore[no-untyped-def]
    def belief_query(query: dict) -> str:
        counter["calls"] += 1
        q = dict(query or {})
        # Forced, not defaulted. `setdefault` let the model supply its own
        # asking time, and a model that asks about a moment after the one it
        # was asked at answers from observations that had not happened.
        q["as_of"] = as_of
        try:
            envelope = execute(store, q)
        except QueryError as exc:
            counter["errors"] += 1
            return json.dumps({"status": "error", "message": str(exc)}, ensure_ascii=False)
        counter["status"].append(envelope.get("status"))
        return _fit(envelope)

    facts = describe_store(store)
    return StructuredTool.from_function(
        func=belief_query,
        name="belief_query",
        description=f"""Ask where things are. ONE call carries the whole question.

THIS RECORDING'S CONSTANTS -- read from the store, not assumed:
  time: {facts["time_unit"]}
        this recording spans {facts["t_start"]:.0f} .. {facts["t_end"]:.0f}
  vocabulary: {facts["vocabulary_size"]} terms; a term outside it returns
        OUT_OF_VOCABULARY with the nearest terms, which are worth retrying

select: "entities" -- the only one. A thing, not one sighting of it.
where: a list of clauses, ANDed. Two operators:
  {{"op":"label","value":"chair"}}
  {{"op":"time_range","t1":{facts["t_start"]:.0f},"t2":{facts["t_end"]:.0f}}}
project: "locate" -- the only one. Position, label, support, dispersion per thing.

Returns an envelope: status(ok|unknown|error), reason, result, quality(rows,
support, dispersion_m, coherence, deduplicated), `truncated` when a long result
list was shortened, and on an empty result a diagnostic naming the clause that
emptied it. status=error means the query was malformed and the message says how
to fix it -- fix it and retry rather than reporting it as an absence.

This store answers where things are and when they were seen. It holds no room
names, no people's identities, no schedule and no event log; a question needing
one of those cannot be answered from here, and saying so is the right answer.""",
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--store", required=True, type=Path)
    parser.add_argument("--suite", required=True, type=Path)
    parser.add_argument("--as-of", type=float, default=1787859665.0)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--model", default="gpt-4o")
    args = parser.parse_args(argv)

    suite = json.loads(args.suite.read_text())[: args.limit]
    run_id = uuid.uuid4().hex[:8]
    store = SqliteStore(path=str(args.store), must_exist=True)
    model = ChatOpenAI(model=args.model, temperature=0)
    handler = langchain_handler()
    print(f"run {run_id}: {len(suite)} tasks, tracing={'on' if handler else 'OFF'}", flush=True)

    results = []
    for i, task in enumerate(suite, 1):
        counter = {"calls": 0, "errors": 0, "status": []}
        tool = build_tool(store, args.as_of, counter)
        agent = create_agent(model=model, tools=[tool], system_prompt=SYS)
        started = time.perf_counter()
        answer = error = None
        try:
            with agent_trace(
                f"bench/{task['id']}",
                run_id=run_id,
                task_id=task["id"],
                audit_verdict=task["audit_verdict"],
                now_built=",".join(task.get("now_built") or []),
                still_missing=",".join(task.get("still_missing") or []) or "none",
            ):
                out = agent.invoke(
                    {"messages": [SystemMessage(SYS), HumanMessage(task["task"])]},
                    config={"callbacks": [handler]} if handler else None,
                )
                answer = out["messages"][-1].content
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
        row = {
            **task,
            "run_id": run_id,
            "trace_name": f"bench/{task['id']}",
            "answer": answer,
            "error": error,
            "tool_calls": counter["calls"],
            "tool_errors": counter["errors"],
            "tool_status": counter["status"],
            "seconds": round(time.perf_counter() - started, 2),
        }
        results.append(row)
        mark = "!" if error else ("." if counter["calls"] else "0")
        print(
            f"  [{i:3d}/{len(suite)}] {task['id']} {mark} {counter['calls']}call {row['seconds']}s",
            flush=True,
        )
        args.out.write_text(
            json.dumps({"run_id": run_id, "results": results}, ensure_ascii=False, indent=1)
        )

    store.stop()
    # Flush before exiting: traces are batched, and a process that ends first
    # leaves the last few tasks missing from the very report they are the point of.
    try:
        from dimos.utils.tracing import _client

        client = _client()
        if client is not None:
            client.flush()
            print("langfuse flushed")
    except Exception as exc:
        print(f"langfuse flush failed: {exc}")
    print(f"\nrun_id {run_id} -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
