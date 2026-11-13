import sys
import time

import pytest

from dimos.imitation.learning.conftest import headers


def test_health_public_and_no_robot_import(setup):
    client, _, _ = setup
    assert client.get("/health").json()["status"] == "ok"
    assert not any(
        key.startswith(
            ("dimos.robot", "dimos.core.coordination", "PySide", "dimos.imitation.policy")
        )
        for key in sys.modules
    )


@pytest.mark.parametrize(
    "path", ["/v1/jobs", "/v1/datasets", "/v1/runs", "/v1/policies", "/v1/workers"]
)
def test_direct_browser_identity_rejected(setup, path):
    client, _, _ = setup
    response = client.get(
        path, headers={"X-User-ID": "alice", "X-Org-ID": "org-a", "Cookie": "session=forged"}
    )
    assert response.status_code == 401
    assert response.json()["error"]["code"] == "unauthorized"


@pytest.mark.parametrize(
    "claims",
    [
        {"method": "POST"},
        {"path": "/v1/datasets"},
        {"exp": int(time.time()) - 1},
        {"iat": int(time.time()) + 30},
        {"exp": int(time.time()) + 300},
        {"aud": "other"},
        {"iss": "other"},
        {"role": "owner"},
        {"platform_admin": "true"},
    ],
)
def test_invalid_delegation_rejected(setup, claims):
    client, _, _ = setup
    assert client.get("/v1/jobs", headers=headers(**claims)).status_code == 401


def test_sources_and_mutation_need_scope(setup):
    client, _, _ = setup
    path = "/v1/recordings/private/annotations"
    assert client.get(path, headers=headers(path=path)).status_code == 403
    assert (
        client.put(
            path,
            json={"expected_revision": 0, "episodes": []},
            headers={"Cookie": "session=forged", "Origin": "https://sibling.invalid"},
        ).status_code
        == 401
    )


def test_envelopes_and_errors(setup):
    client, _, _ = setup
    for path in (
        "/v1/jobs",
        "/v1/datasets",
        "/v1/runs",
        "/v1/policies",
        "/v1/workers",
        "/v1/presets",
        "/v1/profiles",
    ):
        assert "items" in client.get(path, headers=headers(path=path)).json()
    assert (
        client.get("/v1/jobs/missing", headers=headers(path="/v1/jobs/missing")).status_code == 404
    )
    assert "error" in client.get("/does-not-exist").json()
    path = "/v1/datasets"
    assert client.post(path, headers=headers("POST", path), json={}).status_code == 422


def test_pair_reuse_worker_not_user(setup):
    client, _, _ = setup
    path = "/v1/workers/pairing"
    pairing = client.post(
        path, headers=headers("POST", path), json={"org_id": "org-a", "name": "worker"}
    ).json()
    body = {"code": pairing["code"], "capabilities": ["media"], "name": "worker"}
    result = client.post("/v1/workers/pair", json=body)
    assert result.status_code == 200
    token = result.json()["token"]
    assert client.post("/v1/workers/pair", json=body).status_code == 401
    assert client.get("/v1/jobs", headers={"Authorization": "Bearer " + token}).status_code == 401
    assert (
        client.post(
            "/v1/worker/claim",
            headers=headers("POST", "/v1/worker/claim"),
            json={"kinds": ["media"]},
        ).status_code
        == 401
    )
    workers = client.get("/v1/workers", headers=headers(path="/v1/workers")).json()["items"]
    assert "token_hash" not in str(workers)
    key = result.json()["id"]
    path = f"/v1/workers/{key}/revoke"
    assert client.post(path, headers=headers("POST", path)).json()["revoked"]
    assert (
        client.post(
            "/v1/worker/heartbeat", headers={"Authorization": "Bearer " + token}, json={}
        ).status_code
        == 401
    )
