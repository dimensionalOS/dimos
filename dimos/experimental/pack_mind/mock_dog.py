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

"""PACK MIND mock dog — a hardware-free client that drives one dog's search loop.

No DimOS Module, no robot, just ``requests`` against a running coordinator. Run
the server plus two of these (one with ``--find-prob 1.0``) to demonstrate the
full no-overlap + stop-on-find story with zero robots::

    uv run python dimos/experimental/pack_mind/pack_coordinator_server.py --port 8090 &
    uv run python dimos/experimental/pack_mind/mock_dog.py --dog alpha --url http://localhost:8090
    uv run python dimos/experimental/pack_mind/mock_dog.py --dog bravo --url http://localhost:8090 --find-prob 1.0

Each dog loops: ask for a zone; stop if the pack already found the target; with
probability ``--find-prob`` report a finding (and stop), otherwise report the
zone cleared; repeat until no zone is left.
"""

from __future__ import annotations

import argparse
import random
import time
from typing import Any

import requests

DEFAULT_TARGET = "red object"
_REQUEST_TIMEOUT = 5.0
_MIN_SLEEP = 0.05
_MAX_SLEEP = 0.25


def _post(url: str, path: str, payload: dict[str, Any]) -> dict[str, Any] | None:
    try:
        resp = requests.post(f"{url}{path}", json=payload, timeout=_REQUEST_TIMEOUT)
        resp.raise_for_status()
        data: Any = resp.json()
        return data if isinstance(data, dict) else None
    except (requests.RequestException, ValueError) as exc:
        print(f"[{path}] request failed: {exc}")
        return None


def _get(url: str, path: str, params: dict[str, str]) -> dict[str, Any] | None:
    try:
        resp = requests.get(f"{url}{path}", params=params, timeout=_REQUEST_TIMEOUT)
        resp.raise_for_status()
        data: Any = resp.json()
        return data if isinstance(data, dict) else None
    except (requests.RequestException, ValueError) as exc:
        print(f"[{path}] request failed: {exc}")
        return None


def run_dog(
    dog: str, url: str, target: str, find_prob: float, rng: random.Random
) -> None:
    url = url.rstrip("/")
    while True:
        stop = _get(url, "/should_stop", {"dog": dog})
        if stop is not None and stop.get("stop"):
            print(f"[{dog}] pack already found the target — converging, stopping.")
            return

        assigned = _post(url, "/assign_zone", {"dog": dog})
        zone = assigned.get("zone") if assigned else None
        if not zone:
            print(f"[{dog}] no zone assigned (search over or exhausted) — stopping.")
            return
        print(f"[{dog}] assigned zone: {zone}")

        time.sleep(rng.uniform(_MIN_SLEEP, _MAX_SLEEP))

        if rng.random() < find_prob:
            result = _post(
                url, "/report_finding", {"dog": dog, "object": target, "zone": zone}
            )
            finding = result.get("finding") if result else None
            print(f"[{dog}] FOUND {target} in {zone} -> {finding} — stopping.")
            return

        cleared = _post(url, "/report_cleared", {"dog": dog, "zone": zone})
        status = cleared.get("status") if cleared else "(unreachable)"
        print(f"[{dog}] cleared {zone}: {status}")


def main() -> None:
    parser = argparse.ArgumentParser(description="PACK MIND mock dog client")
    parser.add_argument("--dog", required=True, help="This dog's name, e.g. alpha.")
    parser.add_argument(
        "--url",
        default="http://localhost:8090",
        help="Coordinator base URL.",
    )
    parser.add_argument("--target", default=DEFAULT_TARGET, help="What to 'find'.")
    parser.add_argument(
        "--find-prob",
        type=float,
        default=0.0,
        help="Per-zone probability of reporting a finding (1.0 = always find).",
    )
    parser.add_argument("--seed", type=int, default=None, help="RNG seed for repeatability.")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    run_dog(args.dog, args.url, args.target, args.find_prob, rng)


if __name__ == "__main__":
    main()
