# Copyright 2025-2026 Dimensional Inc.
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

"""PACK MIND conductor — a shared semantic memory for a team of Unitree Go2s.

The conductor is the ONLY shared layer. Each dog runs its own full
``unitree-go2-agentic`` stack (own map, own memory) and exposes an MCP HTTP
endpoint. The conductor holds the roster, an append-only event blackboard, the
mission state machine, and a movement lock. It talks to each dog over MCP
JSON-RPC. It NEVER exchanges coordinates between dogs — cross-dog handoff is by
zone NAME only.

Run without hardware (drives the whole story from the dashboard buttons)::

    uv run python dimos/experimental/pack_mind/conductor.py --mock

Run against real dogs::

    uv run python dimos/experimental/pack_mind/conductor.py \\
        --dog alpha=10.0.0.10 --dog bravo=10.0.0.11 --dog charlie=10.0.0.12

Then open http://localhost:8080 for the dashboard.

NOTE: MCP tool argument names (``navigate_with_text``, ``look_out_for``,
``speak``) are passed through verbatim. Verify them against the skill
signatures in dimos/agents/skills/ on the first real-hardware test (G0).
"""

from __future__ import annotations

import argparse
import json
import threading
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Literal, cast

import requests

DogId = Literal["alpha", "bravo", "charlie"]
ZoneId = Literal["zone_a", "zone_b", "zone_c"]
MissionState = Literal[
    "IDLE",
    "SCOUT_ALPHA",
    "FOUND_EVENT_RECORDED",
    "QUERY_BRAVO",
    "BRAVO_NAVIGATING",
    "VERIFYING",
    "DONE",
]

# MCP call timeouts in seconds (plan section 5). Never block the dashboard on these.
TIMEOUT_SPEAK = 8.0
TIMEOUT_LIGHT = 15.0
TIMEOUT_NAV = 90.0

_HERE = Path(__file__).parent


@dataclass
class Dog:
    """A pack member. ``zone`` is the named zone it was last sent to — never a coordinate."""

    id: DogId
    name: str
    mcp_url: str
    role: str
    status: str = "idle"
    zone: ZoneId | None = None


@dataclass(frozen=True)
class MemoryEvent:
    """An immutable blackboard entry. The shared memory is an append-only list of these.

    There is deliberately no ``x``/``y``/``pose`` field. If you add one, you are
    building the wrong project.
    """

    id: str
    ts: str
    robot: DogId
    type: str
    text: str
    object: str | None = None
    zone: ZoneId | None = None
    source: str = "system"
    confidence: float | None = None


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _new_id() -> str:
    return f"evt-{uuid.uuid4().hex[:8]}"


class Conductor:
    """Holds all shared state and orchestrates the deterministic mission state machine."""

    def __init__(self, dogs: list[Dog], mock: bool) -> None:
        self._dogs: dict[DogId, Dog] = {d.id: d for d in dogs}
        self._events: list[MemoryEvent] = []
        self._mission: MissionState = "IDLE"
        self._mock = mock
        self._state_lock = threading.Lock()  # guards roster + blackboard + mission
        self._move_lock = threading.Lock()  # only one dog moves at a time
        self._acting: DogId | None = None

    # -- shared memory -------------------------------------------------------

    def append_event(
        self,
        robot: DogId,
        type_: str,
        text: str,
        *,
        object_: str | None = None,
        zone: ZoneId | None = None,
        source: str = "system",
        confidence: float | None = None,
    ) -> MemoryEvent:
        event = MemoryEvent(
            id=_new_id(),
            ts=_now(),
            robot=robot,
            type=type_,
            text=text,
            object=object_,
            zone=zone,
            source=source,
            confidence=confidence,
        )
        with self._state_lock:
            self._events.append(event)
        return event

    def _latest_zone_for(self, object_: str) -> ZoneId | None:
        """The whole semantic-memory trick: any dog can answer from the blackboard."""
        with self._state_lock:
            for event in reversed(self._events):
                if event.type == "object_found" and event.object == object_ and event.zone:
                    return event.zone
        return None

    def _set_mission(self, state: MissionState) -> None:
        with self._state_lock:
            self._mission = state

    # -- MCP transport -------------------------------------------------------

    def call_tool(
        self, dog_id: DogId, tool_name: str, args: dict[str, Any], timeout: float
    ) -> str | None:
        """Call one MCP tool on one dog. Returns text content, or None on failure.

        On timeout/error, appends a ``tool_timeout`` event and returns None so the
        caller can fall back to a scripted beat. Never raises into the request thread.
        """
        dog = self._dogs.get(dog_id)
        if dog is None:
            return None

        if self._mock:
            self.append_event(
                dog_id, "mock_call", f"[mock] {tool_name}({args})", source="mock"
            )
            return f"[mock {dog_id}] {tool_name} ok"

        payload = {
            "jsonrpc": "2.0",
            "id": _new_id(),
            "method": "tools/call",
            "params": {"name": tool_name, "arguments": args},
        }
        try:
            resp = requests.post(dog.mcp_url, json=payload, timeout=timeout)
            resp.raise_for_status()
            data: Any = resp.json()
            return _extract_text(data)
        except requests.RequestException as exc:
            self.append_event(
                dog_id, "tool_timeout", f"{tool_name} failed: {exc}", source="system"
            )
            return None

    def _speak(self, dog_id: DogId, text: str) -> None:
        self.call_tool(dog_id, "speak", {"text": text, "blocking": False}, TIMEOUT_SPEAK)

    # -- mission state machine ----------------------------------------------

    def start_act1(self) -> None:
        self._set_mission("SCOUT_ALPHA")
        with self._state_lock:
            if "alpha" in self._dogs:
                self._dogs["alpha"].status = "scouting"
        self.append_event("alpha", "mission", "Alpha scouting for the red backpack.")

    def inject_found(self, dog_id: DogId, object_: str, zone: ZoneId) -> MemoryEvent:
        """Operator fallback == real detection: identical blackboard effect."""
        event = self.append_event(
            dog_id,
            "object_found",
            f"{dog_id.title()} found {object_} in {zone}.",
            object_=object_,
            zone=zone,
            source="operator",
            confidence=0.92,
        )
        with self._state_lock:
            if dog_id in self._dogs:
                self._dogs[dog_id].status = "found_target"
        self._set_mission("FOUND_EVENT_RECORDED")
        self._run_async(lambda: self._speak(dog_id, f"Found {object_} in {zone}."))
        return event

    def ask_where(self, dog_id: DogId, object_: str) -> str:
        """The winning moment: a dog answers from SHARED memory, not its own."""
        self._set_mission("QUERY_BRAVO")
        zone = self._latest_zone_for(object_)
        if zone is None:
            answer = f"I have no pack memory of the {object_} yet."
        else:
            finder = self._finder_of(object_)
            who = finder.title() if finder else "A teammate"
            answer = f"{who} found it in {zone}. Follow me."
        self.append_event(dog_id, "answer", answer, object_=object_, zone=zone)
        self._run_async(lambda: self._speak(dog_id, answer))
        return answer

    def send_dog_to_memory(self, dog_id: DogId, object_: str) -> bool:
        """Acquire the movement lock and navigate by zone NAME (the dog's own frame)."""
        zone = self._latest_zone_for(object_)
        if zone is None:
            self.append_event(dog_id, "blocked", f"No memory of {object_}; cannot navigate.")
            return False
        if not self._move_lock.acquire(blocking=False):
            self.append_event(dog_id, "blocked", "Another dog is moving; movement lock held.")
            return False
        self._acting = dog_id
        self._set_mission("BRAVO_NAVIGATING")
        with self._state_lock:
            if dog_id in self._dogs:
                self._dogs[dog_id].status = "navigating"
                self._dogs[dog_id].zone = zone
        self.append_event(dog_id, "navigating", f"{dog_id.title()} acting on pack memory -> {zone}.", zone=zone)

        def _go() -> None:
            try:
                self.call_tool(dog_id, "navigate_with_text", {"query": zone}, TIMEOUT_NAV)
                with self._state_lock:
                    if dog_id in self._dogs:
                        self._dogs[dog_id].status = "arrived"
                self.append_event(dog_id, "arrived", f"{dog_id.title()} reached {zone}.", zone=zone)
            finally:
                self._acting = None
                self._move_lock.release()

        self._run_async(_go)
        return True

    def verify_at_zone(self, dog_id: DogId, object_: str) -> None:
        self._set_mission("VERIFYING")
        zone = self._latest_zone_for(object_)

        def _verify() -> None:
            # Primary path uses the VLM lookout; mock/fallback just confirms.
            self.call_tool(
                dog_id, "look_out_for", {"description_of_things": [object_]}, TIMEOUT_LIGHT
            )
            self.append_event(
                dog_id, "verified", "Confirmed. Pack memory was correct.", object_=object_, zone=zone
            )
            self._run_async(lambda: self._speak(dog_id, "Confirmed. Pack memory was correct."))
            self._set_mission("DONE")
            with self._state_lock:
                if dog_id in self._dogs:
                    self._dogs[dog_id].status = "done"

        self._run_async(_verify)

    def emergency_stop(self) -> None:
        if self._move_lock.locked():
            try:
                self._move_lock.release()
            except RuntimeError:
                pass
        self._acting = None
        with self._state_lock:
            for dog in self._dogs.values():
                dog.status = "idle"
        self._set_mission("IDLE")
        self.append_event("alpha", "estop", "Operator emergency stop. All dogs holding.", source="operator")

    # -- helpers -------------------------------------------------------------

    def _finder_of(self, object_: str) -> DogId | None:
        with self._state_lock:
            for event in reversed(self._events):
                if event.type == "object_found" and event.object == object_:
                    return event.robot
        return None

    def _run_async(self, fn: Any) -> None:
        threading.Thread(target=fn, daemon=True).start()

    def snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            return {
                "mission": self._mission,
                "acting": self._acting,
                "mock": self._mock,
                "roster": [asdict(d) for d in self._dogs.values()],
                "events": [asdict(e) for e in reversed(self._events)],
            }


def _extract_text(data: Any) -> str | None:
    """Pull text out of an MCP tools/call response, tolerating shape variation."""
    if not isinstance(data, dict):
        return None
    result = data.get("result", data)
    if isinstance(result, dict):
        content = result.get("content")
        if isinstance(content, list):
            parts = [c.get("text", "") for c in content if isinstance(c, dict)]
            joined = " ".join(p for p in parts if p)
            return joined or None
    return None


# -- HTTP server -------------------------------------------------------------


class _Server(ThreadingHTTPServer):
    conductor: Conductor


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args: Any) -> None:  # silence default logging
        return

    @property
    def _conductor(self) -> Conductor:
        return cast(_Server, self.server).conductor

    def _send(self, code: int, body: bytes, content_type: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path: Path, content_type: str) -> None:
        if not path.is_file():
            self._send(404, b"not found", "text/plain")
            return
        self._send(200, path.read_bytes(), content_type)

    def do_GET(self) -> None:  # noqa: N802 (stdlib API)
        if self.path in ("/", "/index.html"):
            self._send_file(_HERE / "dashboard.html", "text/html; charset=utf-8")
        elif self.path == "/static/style.css":
            self._send_file(_HERE / "static" / "style.css", "text/css; charset=utf-8")
        elif self.path == "/state":
            body = json.dumps(self._conductor.snapshot()).encode()
            self._send(200, body, "application/json")
        else:
            self._send(404, b"not found", "text/plain")

    def do_POST(self) -> None:  # noqa: N802 (stdlib API)
        if self.path != "/action":
            self._send(404, b"not found", "text/plain")
            return
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            req: Any = json.loads(raw)
        except json.JSONDecodeError:
            self._send(400, b'{"ok": false, "error": "bad json"}', "application/json")
            return
        result = self._dispatch(req if isinstance(req, dict) else {})
        self._send(200, json.dumps(result).encode(), "application/json")

    def _dispatch(self, req: dict[str, Any]) -> dict[str, Any]:
        action = req.get("action")
        c = self._conductor
        obj = cast(str, req.get("object", "red backpack"))
        dog = cast(DogId, req.get("dog", "bravo"))
        zone = cast(ZoneId, req.get("zone", "zone_b"))
        if action == "start_act1":
            c.start_act1()
        elif action == "inject_found":
            c.inject_found(cast(DogId, req.get("dog", "alpha")), obj, zone)
        elif action == "ask_where":
            return {"ok": True, "answer": c.ask_where(dog, obj)}
        elif action == "send_dog":
            return {"ok": c.send_dog_to_memory(dog, obj)}
        elif action == "verify":
            c.verify_at_zone(dog, obj)
        elif action == "estop":
            c.emergency_stop()
        else:
            return {"ok": False, "error": f"unknown action: {action}"}
        return {"ok": True}


def _parse_dog(spec: str) -> Dog:
    """Parse ``alpha=10.0.0.10`` or ``alpha=10.0.0.10:9990`` into a Dog."""
    name, _, addr = spec.partition("=")
    name = name.strip().lower()
    if name not in ("alpha", "bravo", "charlie"):
        raise argparse.ArgumentTypeError(f"dog id must be alpha/bravo/charlie, got {name!r}")
    host, _, port = addr.partition(":")
    port = port or "9990"
    roles = {"alpha": "scout", "bravo": "guide", "charlie": "guard"}
    return Dog(
        id=cast(DogId, name),
        name=name.title(),
        mcp_url=f"http://{host}:{port}/mcp",
        role=roles[name],
    )


def _default_dogs() -> list[Dog]:
    roles = {"alpha": "scout", "bravo": "guide"}
    return [
        Dog(id=cast(DogId, n), name=n.title(), mcp_url=f"http://mock/{n}/mcp", role=r)
        for n, r in roles.items()
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description="PACK MIND conductor")
    parser.add_argument(
        "--dog",
        action="append",
        type=_parse_dog,
        default=[],
        metavar="ID=HOST[:PORT]",
        help="A dog to control, e.g. alpha=10.0.0.10. Repeatable.",
    )
    parser.add_argument("--mock", action="store_true", help="No hardware: log calls instead of HTTP.")
    parser.add_argument("--port", type=int, default=8080, help="Dashboard/API port.")
    args = parser.parse_args()

    dogs: list[Dog] = list(args.dog) or _default_dogs()
    mock = args.mock or not args.dog  # no real dogs given -> mock
    conductor = Conductor(dogs, mock=mock)

    server = _Server(("0.0.0.0", args.port), _Handler)
    server.conductor = conductor
    mode = "MOCK (no hardware)" if mock else f"{len(dogs)} dog(s)"
    print(f"PACK MIND conductor up on http://localhost:{args.port}  [{mode}]")
    print(f"  roster: {', '.join(f'{d.id}->{d.mcp_url}' for d in dogs)}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down")
        server.shutdown()


if __name__ == "__main__":
    main()
