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

"""FRANK's mailbox: chat events and idle wake-ups from the FRANK server.

The agent driving FRANK never gets prompts pushed into it, so it polls here.
`wait` blocks until an event arrives and prints it as one line of JSON.

    uv run python dimos/experimental/frank/tools/inbox.py wait --timeout 90

Server URL from `FRANK_URL` (default http://127.0.0.1:7790), token from
`FRANK_AGENT_TOKEN` if the server is reachable off-loopback.

Exit codes: 0 got something, 2 can't reach the server, 3 nothing waiting.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Any

import requests

DEFAULT_URL = "http://127.0.0.1:7790"
MAX_WAIT_S = 120.0  # the contract caps /agent/events at 120


class FrankError(Exception):
    """The server is unreachable or answered with an error."""


def base_url() -> str:
    return os.environ.get("FRANK_URL", DEFAULT_URL).rstrip("/")


def _headers() -> dict[str, str]:
    token = os.environ.get("FRANK_AGENT_TOKEN")
    return {"Authorization": f"Bearer {token}"} if token else {}


def _request(method: str, path: str, *, timeout: float = 15.0, **kwargs: Any) -> requests.Response:
    url = f"{base_url()}{path}"
    try:
        resp = requests.request(method, url, headers=_headers(), timeout=timeout, **kwargs)
    except requests.ConnectionError as exc:
        raise FrankError(
            f"no FRANK server at {base_url()} — start it with "
            "`uv run python dimos/experimental/frank/app/server.py`"
        ) from exc
    except requests.Timeout as exc:
        raise FrankError(f"the FRANK server at {base_url()} did not answer in time") from exc
    except requests.RequestException as exc:
        raise FrankError(f"can't talk to the FRANK server at {base_url()}: {exc}") from exc
    if resp.status_code >= 400:
        raise FrankError(f"{method} {path} failed: {resp.status_code} {resp.text[:200].strip()}")
    return resp


def _json(method: str, path: str, **kwargs: Any) -> Any:
    resp = _request(method, path, **kwargs)
    try:
        return resp.json()
    except ValueError as exc:
        raise FrankError(f"{method} {path} did not return JSON: {resp.text[:200].strip()}") from exc


# --- operations, also importable ------------------------------------------------


def wait_event(timeout: float = 90.0) -> dict[str, Any]:
    """Block until one event arrives. Returns `{"type": "none"}` on timeout."""
    wait = max(0.0, min(float(timeout), MAX_WAIT_S))
    event = _json("GET", "/agent/events", params={"wait": wait}, timeout=wait + 15.0)
    if not isinstance(event, dict) or "type" not in event:
        raise FrankError(f"unexpected event from the server: {json.dumps(event)[:200]}")
    return event


def send(person_id: str, text: str) -> dict[str, Any]:
    """Say something to a person. Also pushed to their phone."""
    return _json("POST", "/agent/send", json={"person_id": person_id, "text": text})


def people() -> list[dict[str, Any]]:
    """Everyone FRANK has met, newest facts first."""
    got = _json("GET", "/agent/people")
    return list(got) if isinstance(got, list) else list(got.get("people", []))


def history(person_id: str, since_minutes: int = 60) -> list[dict[str, Any]]:
    """Recent messages both ways with one person."""
    got = _json(
        "GET",
        f"/agent/people/{person_id}/history",
        params={"since_minutes": int(since_minutes)},
    )
    return list(got) if isinstance(got, list) else list(got.get("messages", []))


def selfie(person_id: str, path: str) -> str:
    """Save a person's enrollment photo to `path`. Returns the path."""
    resp = _request("GET", f"/agent/people/{person_id}/selfie.jpg")
    with open(path, "wb") as fh:
        fh.write(resp.content)
    return path


def sighting(
    person_id: str,
    confidence: float,
    pose: tuple[float, float, float] | None = None,
    **fields: Any,
) -> dict[str, Any]:
    """Tell the server you just saw someone, optionally where.

    `fields` carries the watcher's extras (bearing_deg, range_m, face_px, in_view, x, y).
    Returns `{"ok": true, "event": "found" | "seen" | null}`.
    """
    body: dict[str, Any] = {"person_id": person_id, "confidence": float(confidence), **fields}
    if pose is not None:
        body["pose"] = {"x": pose[0], "y": pose[1], "yaw": pose[2]}
    return _json("POST", "/agent/sightings", json=body)


def world() -> dict[str, Any]:
    """Who is where right now: in view, last seen, position, bearing and range."""
    got = _json("GET", "/agent/world")
    if not isinstance(got, dict):
        raise FrankError("unexpected /agent/world response")
    return got


def watch_for(person_id: str, say: str | None = None) -> dict[str, Any]:
    """Ask to be woken with a `found` event the moment this person is recognized."""
    return _json("POST", "/agent/watch", json={"person_id": person_id, "say": say})


def unwatch(person_id: str) -> dict[str, Any]:
    """Stop looking for this person."""
    return _json("DELETE", f"/agent/watch/{person_id}")


def watches() -> list[dict[str, Any]]:
    """People FRANK is currently looking for."""
    got = _json("GET", "/agent/watch")
    return list(got.get("watches", [])) if isinstance(got, dict) else list(got)


def done(task_id: str, outcome: str = "done", note: str | None = None) -> dict[str, Any]:
    """Close a wake. Outcome is done / not_found / skipped."""
    body: dict[str, Any] = {"outcome": outcome}
    if note:
        body["note"] = note
    return _json("POST", f"/agent/tasks/{task_id}/done", json=body)


# --- printing -------------------------------------------------------------------


def _minutes_ago(ts: Any) -> str:
    if not isinstance(ts, (int, float)) or ts <= 0:
        return "-"
    return str(int(max(0.0, time.time() - float(ts)) // 60))


def _print_people(rows: list[dict[str, Any]]) -> None:
    if not rows:
        print("no people yet")
        return
    table = [("id", "name", "chat_min", "seen_min", "follow_ups")]
    table += [
        (
            str(r.get("person_id", "?")),
            str(r.get("name", "?")),
            _minutes_ago(r.get("last_chat_ts")),
            _minutes_ago(r.get("last_seen_ts")),
            str(r.get("follow_ups_today", 0)),
        )
        for r in rows
    ]
    widths = [max(len(row[i]) for row in table) for i in range(len(table[0]))]
    for row in table:
        print("  ".join(cell.ljust(w) for cell, w in zip(row, widths, strict=False)).rstrip())


def _ago(now: float, ts: Any) -> str:
    if not isinstance(ts, (int, float)) or ts <= 0:
        return "never"
    d = max(0.0, now - float(ts))
    return f"{d:.0f} s ago" if d < 90 else f"{d / 60:.0f} min ago"


def bearing_word(bearing_deg: float) -> str:
    """A bearing the agent can say out loud. Positive is to FRANK's left."""
    b = float(bearing_deg)
    if abs(b) < 12:
        return "ahead"
    side = "left" if b > 0 else "right"
    return f"ahead-{side}" if abs(b) < 55 else side


def where_words(row: dict[str, Any]) -> str:
    """ "1.8 m ahead-left" for someone in view, else where they were last."""
    if row.get("in_view") and row.get("range_m") is not None:
        rng = f"{float(row['range_m']):.1f} m"
        return f"{rng} {bearing_word(row.get('bearing_deg') or 0.0)}"
    if row.get("x") is not None:
        return f"last at ({float(row['x']):.1f}, {float(row['y'] or 0.0):.1f})"
    return "position unknown"


def format_world(state: dict[str, Any]) -> str:
    """The short table both `inbox.py world` and loop.py's world block print."""
    rows = list(state.get("people") or [])
    now = float(state.get("as_of") or time.time())
    clock = time.strftime("%H:%M:%S", time.localtime(now))
    if not rows:
        return f"nobody enrolled yet (as of {clock})"
    table = [("name", "state", "position", "chat")]
    for r in rows:
        state_s = "in view" if r.get("in_view") else f"seen {_ago(now, r.get('last_seen_ts'))}"
        where = where_words(r)
        if r.get("in_view") and r.get("x") is not None:
            where += f" at ({float(r['x']):.1f}, {float(r.get('y') or 0.0):.1f})"
        table.append(
            (
                f"{r.get('name', '?')} [{r.get('person_id', '?')}]",
                state_s,
                where,
                _ago(now, r.get("last_chat_ts")),
            )
        )
    widths = [max(len(row[i]) for row in table) for i in range(4)]
    body = "\n".join(
        "  ".join(c.ljust(w) for c, w in zip(row, widths, strict=False)).rstrip() for row in table
    )
    return f"as of {clock}\n{body}"


def _name_of(person_id: str) -> str:
    """The person's name if the server knows it, else their id."""
    try:
        for row in people():
            if row.get("person_id") == person_id:
                return str(row.get("name") or person_id)
    except FrankError:
        pass
    return person_id


def _print_history(messages: list[dict[str, Any]], name: str) -> None:
    if not messages:
        print("no messages")
        return
    for msg in messages:
        ts = msg.get("ts")
        clock = (
            time.strftime("%H:%M", time.localtime(ts)) if isinstance(ts, (int, float)) else "--:--"
        )
        who = "FRANK" if msg.get("from") == "frank" else name
        print(f"{clock}  {who}: {msg.get('text', '')}")


# --- cli ------------------------------------------------------------------------


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_wait = sub.add_parser("wait", help="block for one event, print it as JSON")
    p_wait.add_argument("--timeout", type=float, default=90.0, help="seconds to wait, max 120")

    p_send = sub.add_parser("send", help="say something to a person")
    p_send.add_argument("person_id")
    p_send.add_argument("text")

    sub.add_parser("people", help="everyone FRANK has met")

    sub.add_parser("world", help="who is where right now")

    p_watch = sub.add_parser("watch-for", help="be woken when this person is recognized")
    p_watch.add_argument("person_id")
    p_watch.add_argument(
        "--say", help="speak this exact line immediately when the person is recognized"
    )

    p_unwatch = sub.add_parser("unwatch", help="stop looking for this person")
    p_unwatch.add_argument("person_id")

    p_hist = sub.add_parser("history", help="recent messages with one person")
    p_hist.add_argument("person_id")
    p_hist.add_argument("--since", type=int, default=60, help="minutes back")

    p_selfie = sub.add_parser("selfie", help="save a person's enrollment photo")
    p_selfie.add_argument("person_id")
    p_selfie.add_argument("path")

    p_sight = sub.add_parser("sighting", help="record that you just saw someone")
    p_sight.add_argument("person_id")
    p_sight.add_argument("confidence", type=float)
    p_sight.add_argument("--pose", type=float, nargs=3, metavar=("X", "Y", "YAW"))

    p_done = sub.add_parser("done", help="close a wake task")
    p_done.add_argument("task_id")
    p_done.add_argument(
        "outcome", nargs="?", default="done", choices=["done", "not_found", "skipped"]
    )
    p_done.add_argument("--note")

    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        if args.command == "wait":
            event = wait_event(args.timeout)
            print(json.dumps(event, separators=(",", ":")))
            return 3 if event.get("type") == "none" else 0

        if args.command == "send":
            send(args.person_id, args.text)
            print(f"sent to {args.person_id}: {args.text}")
        elif args.command == "people":
            _print_people(people())
        elif args.command == "world":
            print(format_world(world()))
        elif args.command == "watch-for":
            watch_for(args.person_id, args.say)
            print(
                f"looking for {_name_of(args.person_id)} [{args.person_id}] — you'll get a `found` event when the watcher sees them (10 min)"
            )
        elif args.command == "unwatch":
            got = unwatch(args.person_id)
            print(
                f"stopped looking for {args.person_id}"
                if got.get("ok")
                else f"was not looking for {args.person_id}"
            )
        elif args.command == "history":
            _print_history(history(args.person_id, args.since), _name_of(args.person_id))
        elif args.command == "selfie":
            print(selfie(args.person_id, args.path))
        elif args.command == "sighting":
            pose = tuple(args.pose) if args.pose else None
            sighting(args.person_id, args.confidence, pose)  # type: ignore[arg-type]
            where = f" at {pose}" if pose else ""
            print(f"saw {args.person_id} (confidence {args.confidence}){where}")
        elif args.command == "done":
            done(args.task_id, args.outcome, args.note)
            print(f"closed {args.task_id}: {args.outcome}")
    except FrankError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
