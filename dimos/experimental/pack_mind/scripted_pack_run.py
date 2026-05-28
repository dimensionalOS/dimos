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

"""PACK MIND — live STABLE path (the can't-miss demo runner, one process per laptop).

Drives THIS dog's identity against the REAL coordinator (no LLM, no nav planner) and
optionally moves the REAL dog via velocity teleop, so the no-overlap + inheritance
story plays on the projector dashboard with zero autonomy fragility. The coordination
story is identical with or without ``--drive``.

    # Laptop-A (alpha holds the target zone, you drop it on cue):
    uv run python -m dimos.experimental.pack_mind.scripted_pack_run --role alpha --leader \\
        --target "red kit" --target-zone south --hold-zone --drop-on-enter --drive
    # Laptop-B (bravo sweeps, inherits, finds):
    uv run python -m dimos.experimental.pack_mind.scripted_pack_run --role bravo \\
        --target "red kit" --target-zone south --find --keep-polling --drive

Movement (``--drive``) publishes Twist to ``/cmd_vel`` exactly like ``demo_drive_go2``,
so a ``dimos run unitree-go2-pack`` daemon must already hold the WebRTC link on this
laptop (LCM is host-local). See LIVE_RUNBOOK.md §4a.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
import time
from typing import Any

import requests

_TIMEOUT = 5.0
_POLL_SECS = 1.0  # how often a sweeper re-asks while waiting to inherit

Driver = Callable[[], None]


def _post(url: str, path: str, payload: dict[str, Any]) -> dict[str, Any] | None:
    try:
        r = requests.post(f"{url}{path}", json=payload, timeout=_TIMEOUT)
        r.raise_for_status()
        data: Any = r.json()
        return data if isinstance(data, dict) else None
    except (requests.RequestException, ValueError) as exc:
        print(f"  [warn] {path} failed: {exc}")
        return None


def _get(url: str, path: str, params: dict[str, str]) -> dict[str, Any] | None:
    try:
        r = requests.get(f"{url}{path}", params=params, timeout=_TIMEOUT)
        r.raise_for_status()
        data: Any = r.json()
        return data if isinstance(data, dict) else None
    except (requests.RequestException, ValueError) as exc:
        print(f"  [warn] {path} failed: {exc}")
        return None


def _make_driver(drive_secs: float) -> Driver:
    """A /cmd_vel velocity-burst driver (host-local LCM), modelled on demo_drive_go2."""
    from dimos.core.transport import LCMTransport
    from dimos.msgs.geometry_msgs.Twist import Twist
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

    transport: LCMTransport[Twist] = LCMTransport("/cmd_vel", Twist)

    def burst(vx: float, wz: float, secs: float) -> None:
        deadline = time.time() + secs
        while time.time() < deadline:
            transport.broadcast(
                None,
                Twist(linear=Vector3(x=vx, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=wz)),
            )
            time.sleep(0.1)
        transport.broadcast(
            None, Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        )

    # a visible "I'm searching this zone" wiggle: forward, turn, forward.
    def choreograph() -> None:
        seg = max(0.4, drive_secs / 3)
        burst(0.3, 0.0, seg)
        burst(0.0, 0.5, seg)
        burst(0.3, 0.0, seg)

    return choreograph


def run(args: argparse.Namespace) -> None:
    url: str = args.url.rstrip("/")
    dog: str = args.role
    target: str = args.target
    target_zone: str = args.target_zone
    pace: float = args.pace
    driver: Driver | None = _make_driver(args.drive_secs) if args.drive else None

    if args.leader:
        status = _post(url, "/start_search", {"target": target})
        print(f"  MISSION  {status.get('status') if status else '(coordinator unreachable)'}")
        time.sleep(pace)

    while True:
        stop = _get(url, "/should_stop", {"dog": dog})
        if stop and stop.get("stop"):
            print(f"  [{dog}] pack found it — converging. STOP.")
            return

        assigned = _post(url, "/assign_zone", {"dog": dog})
        zone = assigned.get("zone") if assigned else None

        if not zone:
            if args.keep_polling:
                stop = _get(url, "/should_stop", {"dog": dog})
                if stop and stop.get("stop"):
                    print(f"  [{dog}] pack found it — STOP.")
                    return
                print(f"  [{dog}] no free zone — waiting to inherit a dropped teammate's ground…")
                time.sleep(_POLL_SECS)
                continue
            print(f"  [{dog}] no zone left — search over. STOP.")
            return

        print(f"  [{dog}] CLAIMS {zone}")
        if driver:
            driver()  # visible search wiggle in place of real nav
        else:
            time.sleep(pace)

        # the dog that HOLDS the target zone: keep it claimed (still searching) and
        # wait for the operator to drop us, so a teammate inherits it.
        if args.hold_zone and zone == target_zone:
            print(f"  [{dog}] HOLDING {zone} (the object is here, still searching)…")
            if args.drop_on_enter:
                input(f"  >>> press ENTER to DROP {dog} (simulate losing the robot) <<< ")
            else:
                time.sleep(pace * 2)
            ok = _post(url, "/release_dog", {"dog": dog})
            print(f"  [{dog}] OFFLINE — {ok.get('status') if ok else 'released'};"
                  " its ground returns to the pack.")
            return

        # the finder: report the find when it reaches the target zone.
        if args.find and zone == target_zone:
            res = _post(url, "/report_finding", {"dog": dog, "object": target, "zone": zone})
            found = res.get("finding") if res else None
            print(f"  [{dog}] FOUND {target} in {zone} -> {found}. Pack STOP.")
            return

        cleared = _post(url, "/report_cleared", {"dog": dog, "zone": zone})
        print(f"  [{dog}] cleared {zone} (no overlap) — {cleared.get('status') if cleared else '?'}")
        time.sleep(pace * 0.4)


def _parse() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PACK MIND live stable per-laptop runner")
    p.add_argument("--role", required=True, help="this dog's name, e.g. alpha")
    p.add_argument("--url", default="http://127.0.0.1:8090", help="coordinator base URL")
    p.add_argument("--target", default="red kit")
    p.add_argument("--target-zone", default="south", help="where the object is")
    p.add_argument("--leader", action="store_true", help="call start_search (resets ledger)")
    p.add_argument("--hold-zone", action="store_true",
                   help="when assigned the target zone, hold it (don't clear) until dropped")
    p.add_argument("--find", action="store_true",
                   help="when assigned the target zone, report the finding")
    p.add_argument("--keep-polling", action="store_true",
                   help="when no free zone, keep re-asking (to inherit a dropped teammate's zone)")
    p.add_argument("--drop-on-enter", action="store_true",
                   help="while holding, wait for ENTER then release this dog and exit")
    p.add_argument("--drive", action="store_true", help="actually move the dog via /cmd_vel teleop")
    p.add_argument("--drive-secs", type=float, default=1.5, help="seconds of motion per zone")
    p.add_argument("--pace", type=float, default=2.0, help="seconds between beats (no-drive)")
    return p.parse_args()


def main() -> None:
    try:
        run(_parse())
    except KeyboardInterrupt:
        print("\n  interrupted — coordinator state preserved.")


if __name__ == "__main__":
    main()
