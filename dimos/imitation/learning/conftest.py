"""Isolated API/service fixtures; no robot graph imports."""

from __future__ import annotations

import copy
import time

from fastapi.testclient import TestClient
import jwt
import pytest

from dimos.imitation.learning.app import create_app
from dimos.imitation.learning.store import Store

SECRET = "test-only-dedicated-secret-abcdefghijklmnopqrstuvwxyz"


@pytest.fixture
def setup(tmp_path, monkeypatch):
    monkeypatch.setenv("DIMOS_LEARNING_DELEGATION_SECRET", SECRET)
    store = Store(tmp_path / "state.sqlite3")
    store.migrate()
    app = create_app(store)
    with TestClient(app) as client:
        yield client, app.state.service, store


def headers(method="GET", path="/v1/jobs", **changes):
    now = int(time.time())
    claims = {
        "iss": "dimos-cloud",
        "aud": "dimos-learning",
        "sub": "alice",
        "org_id": "org-a",
        "role": "member",
        "platform_admin": False,
        "iat": now,
        "exp": now + 45,
        "method": method,
        "path": path,
    }
    claims.update(changes)
    return {"Authorization": "Bearer " + jwt.encode(claims, SECRET, algorithm="HS256")}


def who(**changes):
    result = {
        "sub": "alice",
        "org_id": "org-a",
        "role": "member",
        "platform_admin": False,
        "sources": [
            {
                "id": "source",
                "can_read": True,
                "can_write": True,
                "complete": True,
                "verified": True,
                "quarantined": False,
            }
        ],
    }
    result.update(changes)
    return result


def indexed(service, monkeypatch):
    info = {
        "id": "source",
        "profile_id": "profile",
        "duration_s": 10.0,
        "origin_s": 1000.0,
        "cameras": [],
        "capture_truth": {"gripperCommandRecorded": False},
        "schema": {
            "metadata": {"gripperSemantics": {"gripperCommandRecorded": False}},
            "action": {"action": {}},
            "observation": {"state": {}},
        },
        "episodes": [
            {
                "id": "stable-episode",
                "start_s": 1.0,
                "end_s": 8.0,
                "instruction": "move cup",
                "outcome": "success",
                "capture_event": "save",
            }
        ],
    }
    monkeypatch.setattr(service, "recording", lambda *args: copy.deepcopy(info))
    return info


def pair(service, identity=None, capabilities=None):
    pairing = service.pairing(
        identity or who(), {"org_id": (identity or who())["org_id"], "name": "test"}
    )
    return service.pair(
        {
            "code": pairing["code"],
            "name": "test",
            "capabilities": capabilities or ["media", "dataset", "train", "load-check"],
        }
    )
