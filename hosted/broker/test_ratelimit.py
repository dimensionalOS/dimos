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

"""Rate-limiter tests. Run: cd app && python test_ratelimit.py

No framework (matches repo). Covers the pure bucket math with a fake clock,
route classification, and the middleware in both passive and enforcing modes
via TestClient with auth stubbed like test_tenant_isolation.py.
"""

import os
import tempfile

os.environ["ENVIRONMENT"] = "dev"
_db = tempfile.NamedTemporaryFile(suffix=".db", delete=False).name
os.environ["DATABASE_URL"] = f"sqlite+aiosqlite:///{_db}"

from fastapi import Header, HTTPException
from fastapi.testclient import TestClient
import main
from ratelimit import LIMITS, RateLimiter, TokenBucket, classify
from services.auth import get_current_user, get_robot_owner
from services.cloudflare import cf_client

PASS = [0]


def check(name: str, cond: bool) -> None:
    print(("PASS" if cond else "FAIL"), name)
    PASS[0] += not cond


# ─── pure bucket math (fake clock) ───────────────────────────────────

b = TokenBucket(capacity=3, refill_per_sec=1.0, now=0.0)
check("burst up to capacity", all(b.allow(0.0) for _ in range(3)))
check("empty bucket denies", b.allow(0.0) is False)
check("retry-after positive when dry", b.retry_after_sec() >= 1)
check("refills over time", b.allow(1.1) is True)
check(
    "does not overfill past capacity",
    [b.allow(100.0) for _ in range(4)] == [True, True, True, False],
)

# ─── route classification ────────────────────────────────────────────

check("keys POST classed", classify("POST", "/api/v1/keys") == "keys_write")
check("keys DELETE classed", classify("DELETE", "/api/v1/keys/abc") == "keys_write")
check("robot create classed", classify("POST", "/api/v1/sessions") == "session_join")
check("join classed", classify("POST", "/api/v1/sessions/xyz/join") == "session_join")
check("turn classed", classify("GET", "/api/v1/sessions/turn-credentials") == "turn_credentials")
check("heartbeat exempt", classify("POST", "/api/v1/sessions/xyz/heartbeat") is None)
check("op-heartbeat exempt", classify("POST", "/api/v1/sessions/xyz/op-heartbeat") is None)
check("list exempt", classify("GET", "/api/v1/sessions") is None)
check("non-api exempt", classify("GET", "/health") is None)

# ─── limiter counts per caller ───────────────────────────────────────

clock = [0.0]
lim = RateLimiter(enforce=True, clock=lambda: clock[0])
cap = int(LIMITS["turn_credentials"][0])
for _ in range(cap):
    ok, _ra = lim.check("ip:1.2.3.4", "turn_credentials")
    assert ok
ok, ra = lim.check("ip:1.2.3.4", "turn_credentials")
check("caller denied past burst", ok is False and ra >= 1)
ok, _ = lim.check("ip:5.6.7.8", "turn_credentials")
check("other caller unaffected", ok is True)

# ─── middleware: passive lets through, enforce 429s ──────────────────


async def _owner(x_robot_api_key: str = Header(None)) -> str:
    if not x_robot_api_key:
        raise HTTPException(401, "no key")
    return x_robot_api_key


async def _user(authorization: str = Header(None)) -> dict:
    tok = (authorization or "").removeprefix("Bearer ").strip()
    if not tok:
        raise HTTPException(401, "no token")
    return {"sub": tok, "role": "operator"}


async def _fake_cf(_sdp: str) -> dict:
    return {"cf_session_id": "cf-fake", "sdp_answer": "v=0"}


main.app.dependency_overrides[get_robot_owner] = _owner
main.app.dependency_overrides[get_current_user] = _user
cf_client.create_session = _fake_cf  # type: ignore[assignment]

BURST = int(LIMITS["session_join"][0])

with TestClient(app=main.app) as c:

    def create(n: int) -> list[int]:
        return [
            c.post(
                "/api/v1/sessions",
                json={"robot_id": f"r{i}", "robot_name": f"r{i}", "sdp_offer": "v=0"},
                headers={"X-Robot-API-Key": "alice"},
            ).status_code
            for i in range(n)
        ]

    # Passive (default): even past the burst, nothing 429s.
    codes = create(BURST + 3)
    check("passive mode never 429s", all(sc == 201 for sc in codes))

with TestClient(app=main.app) as c:
    # Enforcing: same traffic gets cut off at the burst.
    main.rate_limiter.enforce = True
    try:
        codes = [
            c.post(
                "/api/v1/sessions",
                json={"robot_id": f"e{i}", "robot_name": f"e{i}", "sdp_offer": "v=0"},
                headers={"X-Robot-API-Key": "bob"},
            ).status_code
            for i in range(BURST + 3)
        ]
        check("enforce mode 429s past burst", codes.count(429) >= 1)
        check("429 comes after the allowed burst", 201 in codes and codes.index(429) >= 1)
        r = c.post(
            "/api/v1/sessions",
            json={"robot_id": "e", "robot_name": "e", "sdp_offer": "v=0"},
            headers={"X-Robot-API-Key": "bob"},
        )
        check("429 carries Retry-After", r.status_code != 429 or "retry-after" in r.headers)
    finally:
        main.rate_limiter.enforce = False

print(f"\n{'ALL PASS' if PASS[0] == 0 else str(PASS[0]) + ' FAILED'}")
os.unlink(_db)
raise SystemExit(PASS[0])
