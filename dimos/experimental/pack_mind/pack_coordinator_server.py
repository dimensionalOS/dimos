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

"""PACK MIND coordinator HTTP server — the shared MEANING, reachable over a LAN.

Wraps a single ``PackCoordinator`` (the pure, unit-tested ledger) behind a tiny
JSON/HTTP API so each dog's laptop agent can call it with ``requests``. Mirrors
the stdlib ``ThreadingHTTPServer`` + ``BaseHTTPRequestHandler`` pattern in
``conductor.py``. The coordinator shares zone NAMES and findings only — never
coordinates — so two dogs running independent SLAM frames never re-search the
same area.

Endpoints (all JSON; 200 on success, 400 on bad input)::

    GET  /state                                          -> snapshot()
    POST /start_search   {"target"}                      -> {"status"}
    POST /assign_zone    {"dog"}                          -> {"zone": str|null}
    POST /report_cleared {"dog", "zone"}                 -> {"status"}
    POST /report_finding {"dog", "object", "zone"}       -> {"finding": {...}|null}
    GET  /should_stop?dog=NAME                            -> {"stop": bool}

Run (binds 0.0.0.0 so dogs across the LAN can reach it)::

    uv run python dimos/experimental/pack_mind/pack_coordinator_server.py \\
        --port 8090 --zones north,east,south,west
"""

from __future__ import annotations

import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, cast
from urllib.parse import parse_qs, urlparse

_DASHBOARD_HTML = Path(__file__).with_name("pack_dashboard.html")

from dimos.experimental.pack_mind.pack_coordinator import PackCoordinator
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# New named constant — the LAN port the coordinator listens on. No hardcoded
# ports elsewhere; clients import this to build their default URL.
DEFAULT_COORDINATOR_PORT = 8090
DEFAULT_ZONES = ["north", "east", "south", "west"]


class _Server(ThreadingHTTPServer):
    coordinator: PackCoordinator


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args: Any) -> None:  # silence default logging
        return

    @property
    def _coordinator(self) -> PackCoordinator:
        return cast(_Server, self.server).coordinator

    def _send_json(self, code: int, payload: Any) -> None:
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict[str, Any] | None:
        """Read a JSON object body. Returns None on malformed/non-object input."""
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            data: Any = json.loads(raw)
        except json.JSONDecodeError:
            return None
        return data if isinstance(data, dict) else None

    def _serve_dashboard(self) -> None:
        try:
            body = _DASHBOARD_HTML.read_bytes()
        except OSError:
            self._send_json(500, {"error": "dashboard html not found"})
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802 (stdlib API)
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/dashboard"):
            self._serve_dashboard()
        elif parsed.path == "/state":
            self._send_json(200, self._coordinator.snapshot())
        elif parsed.path == "/where_is":
            f = self._coordinator.finding
            self._send_json(
                200,
                {
                    "found": f is not None,
                    "object": f.object if f else None,
                    "zone": f.zone if f else None,
                    "by": f.by if f else None,
                },
            )
        elif parsed.path == "/should_stop":
            params = parse_qs(parsed.query)
            dog_values = params.get("dog")
            if not dog_values or not dog_values[0]:
                self._send_json(400, {"error": "missing query param: dog"})
                return
            self._send_json(200, {"stop": self._coordinator.should_stop(dog_values[0])})
        else:
            self._send_json(404, {"error": f"unknown path: {parsed.path}"})

    def do_POST(self) -> None:  # noqa: N802 (stdlib API)
        path = urlparse(self.path).path
        req = self._read_json()
        if req is None:
            self._send_json(400, {"error": "bad json"})
            return
        if path == "/start_search":
            self._start_search(req)
        elif path == "/assign_zone":
            self._assign_zone(req)
        elif path == "/report_cleared":
            self._report_cleared(req)
        elif path == "/report_finding":
            self._report_finding(req)
        elif path == "/release_dog":
            self._release_dog(req)
        else:
            self._send_json(404, {"error": f"unknown path: {path}"})

    # -- handlers ------------------------------------------------------------

    def _start_search(self, req: dict[str, Any]) -> None:
        target = req.get("target")
        if not isinstance(target, str) or not target:
            self._send_json(400, {"error": "missing or invalid field: target"})
            return
        status = self._coordinator.start_search(target)
        logger.info("start_search", target=target)
        self._send_json(200, {"status": status})

    def _assign_zone(self, req: dict[str, Any]) -> None:
        dog = req.get("dog")
        if not isinstance(dog, str) or not dog:
            self._send_json(400, {"error": "missing or invalid field: dog"})
            return
        zone = self._coordinator.assign_zone(dog)
        logger.info("assign_zone", dog=dog, zone=zone)
        self._send_json(200, {"zone": zone})

    def _report_cleared(self, req: dict[str, Any]) -> None:
        dog = req.get("dog")
        zone = req.get("zone")
        if not isinstance(dog, str) or not dog or not isinstance(zone, str) or not zone:
            self._send_json(400, {"error": "missing or invalid fields: dog, zone"})
            return
        status = self._coordinator.report_cleared(dog, zone)
        logger.info("report_cleared", dog=dog, zone=zone)
        self._send_json(200, {"status": status})

    def _report_finding(self, req: dict[str, Any]) -> None:
        dog = req.get("dog")
        obj = req.get("object")
        zone = req.get("zone", "")
        if not isinstance(dog, str) or not dog or not isinstance(obj, str) or not obj:
            self._send_json(400, {"error": "missing or invalid fields: dog, object"})
            return
        # zone is best-effort: the coordinator falls back to the dog's claimed zone.
        zone = zone if isinstance(zone, str) else ""
        finding = self._coordinator.report_finding(dog, obj, zone)
        logger.info("report_finding", dog=dog, object=obj, zone=zone)
        self._send_json(
            200,
            {"finding": {"object": finding.object, "zone": finding.zone, "by": finding.by}},
        )

    def _release_dog(self, req: dict[str, Any]) -> None:
        dog = req.get("dog")
        if not isinstance(dog, str) or not dog:
            self._send_json(400, {"error": "missing or invalid field: dog"})
            return
        status = self._coordinator.release_dog(dog)
        logger.info("release_dog", dog=dog)
        self._send_json(200, {"status": status})


def make_server(
    host: str,
    port: int,
    zones: list[str],
    preferences: dict[str, list[str]] | None = None,
) -> _Server:
    """Build a ready-to-serve coordinator server. Pass port 0 for an ephemeral port."""
    server = _Server((host, port), _Handler)
    server.coordinator = PackCoordinator(zones, preferences)
    return server


def _parse_prefs(raw: str) -> dict[str, list[str]]:
    """Parse "alpha:north,east;bravo:south,west" into per-dog zone orderings."""
    prefs: dict[str, list[str]] = {}
    for chunk in raw.split(";"):
        chunk = chunk.strip()
        if not chunk or ":" not in chunk:
            continue
        dog, zone_csv = chunk.split(":", 1)
        zones = [z.strip() for z in zone_csv.split(",") if z.strip()]
        if dog.strip() and zones:
            prefs[dog.strip()] = zones
    return prefs


def main() -> None:
    parser = argparse.ArgumentParser(description="PACK MIND coordinator HTTP server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (0.0.0.0 for LAN).")
    parser.add_argument(
        "--port", type=int, default=DEFAULT_COORDINATOR_PORT, help="Listen port."
    )
    parser.add_argument(
        "--zones",
        default=",".join(DEFAULT_ZONES),
        help="Comma-separated zone names.",
    )
    parser.add_argument(
        "--prefs",
        default="",
        help='Per-dog zone order so dogs fan out, e.g. "alpha:north,east;bravo:south,west".',
    )
    args = parser.parse_args()

    zones = [z.strip() for z in args.zones.split(",") if z.strip()]
    if not zones:
        parser.error("--zones must contain at least one zone name")

    server = make_server(args.host, args.port, zones, _parse_prefs(args.prefs))
    actual_port = server.server_address[1]
    print(f"PACK MIND coordinator up on http://{args.host}:{actual_port}")
    print(f"  reachable on the LAN at http://<this-laptop-ip>:{actual_port}")
    print(f"  zones: {', '.join(zones)}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down")
        server.shutdown()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
