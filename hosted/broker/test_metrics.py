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

"""Metrics endpoint tests. Run: cd app && python test_metrics.py

No framework (matches repo). Verifies /metrics serves prometheus text,
request counters label by route template (not raw path), and the reaper's
session gauge refresh works against the real DB.
"""

import asyncio
import os
import tempfile

os.environ["ENVIRONMENT"] = "dev"
_db = tempfile.NamedTemporaryFile(suffix=".db", delete=False).name
os.environ["DATABASE_URL"] = f"sqlite+aiosqlite:///{_db}"

from fastapi import Header, HTTPException
from fastapi.testclient import TestClient
import main
from routers.sessions import _refresh_session_gauge
from services.auth import get_robot_owner
from services.cloudflare import cf_client

PASS = [0]


def check(name: str, cond: bool) -> None:
    print(("PASS" if cond else "FAIL"), name)
    PASS[0] += not cond


async def _owner(x_robot_api_key: str = Header(None)) -> str:
    if not x_robot_api_key:
        raise HTTPException(401, "no key")
    return x_robot_api_key


async def _fake_cf(_sdp: str) -> dict:
    return {"cf_session_id": "cf-fake", "sdp_answer": "v=0"}


main.app.dependency_overrides[get_robot_owner] = _owner
cf_client.create_session = _fake_cf  # type: ignore[assignment]

with TestClient(app=main.app) as c:
    r = c.get("/metrics")
    check("metrics endpoint serves", r.status_code == 200)
    check("prometheus content type", "text/plain" in r.headers["content-type"])
    check(
        "metric families present",
        "teleop_http_requests_total" in r.text and "teleop_sessions" in r.text,
    )

    # Generate traffic, confirm it lands under the route TEMPLATE label.
    c.get("/health")
    r2 = c.post(
        "/api/v1/sessions",
        json={"robot_id": "m1", "robot_name": "m1", "sdp_offer": "v=0"},
        headers={"X-Robot-API-Key": "alice"},
    )
    check("session create ok", r2.status_code == 201)

    body = c.get("/metrics").text
    check("health counted", 'route="/health"' in body)
    check("labels use route template", 'route="/api/v1/sessions"' in body)
    check("latency histogram present", "teleop_http_request_seconds_bucket" in body)

    # Reaper gauge refresh reads the real DB.
    asyncio.run(_refresh_session_gauge())
    body = c.get("/metrics").text
    check("session gauge reflects created session", 'teleop_sessions{state="idle"} 1.0' in body)

print(f"\n{'ALL PASS' if PASS[0] == 0 else str(PASS[0]) + ' FAILED'}")
os.unlink(_db)
raise SystemExit(PASS[0])
