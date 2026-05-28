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

"""PACK MIND — paced LIVE narrative for the projector dashboard.

One command: starts the coordinator + dashboard AND plays the full v4 story with
pauses, so you just open the browser and watch the shared memory animate. Drives
the real coordinator (the dashboard polls the same state), deterministically — no
hardware, no racing mock dogs.

    uv run python -m dimos.experimental.pack_mind.demo_pack_live --pace 2.0
    # then open http://localhost:8090 on the projector

The story it plays (the inheritance climax — two dogs, no overlap, a dog drops,
the survivor inherits and finishes):
  start_search → Alpha claims the target zone → Bravo clears the others (no
  overlap) → Alpha goes OFFLINE (its zone reclaimed) → Bravo inherits it and finds
  the object there.
"""

from __future__ import annotations

import argparse
import threading
import time

from dimos.experimental.pack_mind.pack_coordinator_server import (
    DEFAULT_COORDINATOR_PORT,
    make_server,
)


def _narrate(tag: str, msg: str) -> None:
    print(f"  {tag:<26} {msg}", flush=True)


def play(target: str, target_zone: str, zones: list[str], port: int, host: str, pace: float) -> None:
    others = [z for z in zones if z != target_zone]
    # Alpha prefers the target zone (so it holds it unfinished); Bravo prefers the rest.
    prefs = {"alpha": [target_zone, *others], "bravo": [*others, target_zone]}
    server = make_server(host, port, zones, prefs)
    actual = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    c = server.coordinator

    print(f"\nPACK MIND live demo on http://{host}:{actual}")
    print(f"  → open that in the projector browser now. Starting in {pace * 2:.0f}s…\n")
    time.sleep(pace * 2)

    print("=== mission: find the red kit (two dogs, one shared memory) ===")
    _narrate("MISSION", c.start_search(target))
    time.sleep(pace)

    _narrate("ALPHA CLAIMS", c.assign_zone("alpha") or "-")  # the target zone, unfinished
    time.sleep(pace)

    while True:
        z = c.assign_zone("bravo")
        if not z:
            break
        time.sleep(pace)
        c.report_cleared("bravo", z)
        _narrate("BRAVO CLEARED", f"{z} (no overlap — never alpha's ground)")
        time.sleep(pace * 0.5)

    print("\n=== the real test: lose a dog mid-mission ===")
    time.sleep(pace)
    _narrate("ALPHA OFFLINE", c.release_dog("alpha"))
    time.sleep(pace)

    inherited = c.assign_zone("bravo")
    _narrate("BRAVO INHERITS", f"{inherited} — alpha started this zone, never finished")
    time.sleep(pace)
    f = c.report_finding("bravo", target, inherited or "")
    _narrate("BRAVO FINDS IT", f"{f.object} in {f.zone} — mission complete")

    print("\nAlpha never finished. Bravo inherited its ground and found it. — PACK MIND")
    print("(dashboard stays live; Ctrl-C to exit)\n")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        server.shutdown()


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PACK MIND paced live dashboard demo")
    p.add_argument("--target", default="red kit")
    p.add_argument("--target-zone", default="south", help="where the object is")
    p.add_argument("--zones", default="north,east,south,west")
    p.add_argument("--port", type=int, default=DEFAULT_COORDINATOR_PORT)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--pace", type=float, default=2.0, help="seconds between beats")
    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    zones = [z.strip() for z in args.zones.split(",") if z.strip()]
    if args.target_zone not in zones:
        raise SystemExit(f"--target-zone {args.target_zone!r} must be one of {zones}")
    play(args.target, args.target_zone, zones, args.port, args.host, args.pace)
