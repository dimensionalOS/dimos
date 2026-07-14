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

"""E2E for tenant isolation. Run: cd app && python test_tenant_isolation.py

No framework (matches repo). Fakes auth via headers and stubs Cloudflare so
it exercises the real endpoints + DB filtering without network/Cognito.
"""

import os
import tempfile

os.environ["ENVIRONMENT"] = "dev"  # relax prod config validators
_db = tempfile.NamedTemporaryFile(suffix=".db", delete=False).name
os.environ["DATABASE_URL"] = f"sqlite+aiosqlite:///{_db}"

from fastapi import Header, HTTPException
from fastapi.testclient import TestClient
from main import app
from services.auth import get_current_user, get_robot_owner
from services.cloudflare import cf_client


# Robot identity = the X-Robot-API-Key header value (stands in for key owner).
async def _owner(x_robot_api_key: str = Header(None)) -> str:
    if not x_robot_api_key:
        raise HTTPException(401, "no key")
    return x_robot_api_key


# Operator identity = bearer token "email" or "email:admin".
async def _user(authorization: str = Header(None)) -> dict:
    tok = (authorization or "").removeprefix("Bearer ").strip()
    if not tok:
        raise HTTPException(401, "no token")
    sub, _, role = tok.partition(":")
    return {"sub": sub, "role": role or "operator"}


async def _fake_cf(_sdp: str) -> dict:
    return {"cf_session_id": "cf-fake", "sdp_answer": "v=0"}


app.dependency_overrides[get_robot_owner] = _owner
app.dependency_overrides[get_current_user] = _user
cf_client.create_session = _fake_cf  # type: ignore[assignment]

PASS = [0]


def check(name: str, cond: bool) -> None:
    print(("PASS" if cond else "FAIL"), name)
    PASS[0] += not cond


B = "/api/v1/sessions"
with TestClient(app) as c:

    def create(owner: str, robot_id: str) -> str:
        r = c.post(
            B,
            json={"robot_id": robot_id, "robot_name": robot_id, "sdp_offer": "v=0"},
            headers={"X-Robot-API-Key": owner},
        )
        assert r.status_code == 201, r.text
        return r.json()["session_id"]

    def names(owner: str) -> set[str]:
        r = c.get(B, headers={"Authorization": f"Bearer {owner}"})
        assert r.status_code == 200, r.text
        return {s["robot_name"] for s in r.json()}

    create("alice", "r1")
    create("alice", "r2")
    s_bob = create("bob", "r3")

    check("alice sees only her two robots", names("alice") == {"r1", "r2"})
    check("bob sees only his robot", names("bob") == {"r3"})
    check(
        "one key, multiple robots both visible", "r1" in names("alice") and "r2" in names("alice")
    )

    rs = c.get(f"{B}/{s_bob}/status", headers={"Authorization": "Bearer alice"})
    check("alice cannot status bob's session (404)", rs.status_code == 404)
    rj = c.post(
        f"{B}/{s_bob}/join",
        json={"role": "operator", "sdp_offer": "v=0"},
        headers={"Authorization": "Bearer alice"},
    )
    check("alice cannot join bob's session (404)", rj.status_code == 404)

    check("admin sees all robots", names("admin@x:admin") == {"r1", "r2", "r3"})

    create("alice", "r1")  # reconnect same robot_id
    check("reconnect dedups (no duplicate r1)", sorted(names("alice")) == ["r1", "r2"])

print(f"\n{'ALL PASS' if PASS[0] == 0 else str(PASS[0]) + ' FAILED'}")
os.unlink(_db)
raise SystemExit(PASS[0])
