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

"""Prometheus metrics (HARDENING_PLAN D2).

Bare prometheus_client — the fastapi instrumentator pins a newer starlette
than fastapi 0.115 allows, and all we need is a registry + one middleware.

/metrics is intentionally NOT proxied by the Caddyfile, so it is reachable
only from the box (127.0.0.1:8450/metrics) — scrape it with an on-instance
agent (CloudWatch agent / node exporter sidecar), don't expose it publicly.
"""

import time

from fastapi import Request, Response
from prometheus_client import (
    CONTENT_TYPE_LATEST,
    Counter,
    Gauge,
    Histogram,
    generate_latest,
)

HTTP_REQUESTS = Counter(
    "teleop_http_requests_total",
    "HTTP requests by method, route template and status.",
    ["method", "route", "status"],
)
HTTP_LATENCY = Histogram(
    "teleop_http_request_seconds",
    "Request latency by route template.",
    ["route"],
    buckets=(0.005, 0.025, 0.1, 0.5, 1.0, 5.0, 30.0),
)
SESSIONS_BY_STATE = Gauge(
    "teleop_sessions",
    "Sessions by state, refreshed by the reaper loop (~10s).",
    ["state"],
)
ROBOT_EVICTIONS = Counter(
    "teleop_robot_evictions_total",
    "Robots reaped for stale heartbeat.",
)
OPERATOR_EVICTIONS = Counter(
    "teleop_operator_evictions_total",
    "Operators reaped for stale heartbeat.",
)
RATE_LIMIT_HITS = Counter(
    "teleop_rate_limit_hits_total",
    "Requests over a rate-limit bucket (passive: logged-only; enforced: 429).",
    ["route_class", "enforced"],
)


def install(app) -> None:
    """Request middleware + /metrics endpoint."""

    @app.middleware("http")
    async def _observe(request: Request, call_next):
        start = time.monotonic()
        response = await call_next(request)
        # Route template (e.g. /api/v1/sessions/{session_id}/join), not the
        # raw path — raw paths explode label cardinality with session ids.
        route = getattr(request.scope.get("route"), "path", None)
        if route:  # unmatched paths (404 probes) are deliberately not labeled
            HTTP_REQUESTS.labels(request.method, route, str(response.status_code)).inc()
            HTTP_LATENCY.labels(route).observe(time.monotonic() - start)
        return response

    @app.get("/metrics", include_in_schema=False)
    async def metrics() -> Response:
        return Response(content=generate_latest(), media_type=CONTENT_TYPE_LATEST)
