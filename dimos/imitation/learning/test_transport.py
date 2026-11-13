"""Cross-service transport regressions: signed body instead of oversized JWTs."""

import base64
import copy
import hashlib
import json

import pytest

from dimos.imitation.learning.conftest import headers, indexed, who
from dimos.imitation.learning.envelope import sources_digest
from dimos.imitation.learning.media import materialize
from dimos.imitation.learning.protocol import LearningError


def test_large_envelope_roundtrip_and_tamper(setup):
    client, _, _ = setup
    sources = [{"id": "one", "can_read": True, "metadata": {"manifest": {"notes": "x" * 50000}}}]
    path = "/v1/recordings"
    auth = {**headers(path=path, source_digest=sources_digest(sources)), "X-Learning-Envelope": "1"}
    value = {"sources": sources, "body_base64": ""}
    response = client.request("GET", path, json=value, headers=auth)
    assert response.status_code == 200
    assert response.json()["items"][0]["id"] == "one"
    assert len(auth["Authorization"]) < 1500
    bad = copy.deepcopy(value)
    bad["sources"][0]["id"] = "forged"
    assert client.request("GET", path, json=bad, headers=auth).status_code == 401
    assert client.get(path, headers=auth).status_code == 422
    assert (
        client.get(
            path, headers=headers(path=path, source_digest=sources_digest(sources))
        ).status_code
        == 401
    )
    assert (
        client.request("GET", path, json=value, headers={"X-Learning-Envelope": "1"}).status_code
        == 401
    )


def test_envelope_restores_original_post_body(setup):
    client, _, _ = setup
    path = "/v1/workers/pairing"
    raw = json.dumps({"org_id": "org-a", "name": "envelope-pairing"}).encode()
    response = client.post(
        path,
        json={"sources": [], "body_base64": base64.b64encode(raw).decode()},
        headers={
            **headers("POST", path, source_digest=sources_digest([])),
            "X-Learning-Envelope": "1",
        },
    )
    assert response.status_code == 200
    assert response.json()["code"]


def test_inline_companion_download_checksum_and_stable_cache(tmp_path, monkeypatch):
    src = tmp_path / "original.mcap"
    src.write_bytes(b"read-only source bytes")
    expected = hashlib.sha256(src.read_bytes()).hexdigest()
    monkeypatch.setenv("DIMOS_LEARNING_SOURCE_ROOTS", str(tmp_path))
    descriptor = {
        "id": "source",
        "org_id": "org-a",
        "download_url": src.as_uri(),
        "source_sha256": expected,
        "schema": {"payload": "recording.mcap", "name": "profile"},
        "manifest": {"capture": "untouched"},
    }
    state = tmp_path / "state"
    cached = materialize(descriptor, state)
    assert json.loads((cached / "schema.json").read_text())["name"] == "profile"
    assert json.loads((cached / "manifest.json").read_text()) == descriptor["manifest"]
    changed_url = {
        **descriptor,
        "download_url": "file:///new-presigned-url-is-not-content-identity",
    }
    assert materialize(changed_url, state) == cached
    assert src.read_bytes() == b"read-only source bytes"
    with pytest.raises(LearningError, match="") as error:
        materialize({**descriptor, "source_sha256": "a" * 64}, state)
    assert error.value.code == "source_checksum_mismatch"
    assert not list(state.glob("sources/*.partial-*"))


def test_pair_command_quotes_worker_name(setup, monkeypatch):
    import shlex

    _, service, _ = setup
    monkeypatch.setenv("DIMOS_LEARNING_PUBLIC_API_URL", "https://worker.example.invalid")
    name = "worker; not-a-shell-command"
    paired = service.pairing(who(), {"org_id": "org-a", "name": name})
    args = shlex.split(paired["command"])
    assert args[-1] == name
    assert args[args.index("--api") + 1] == "https://worker.example.invalid"


def test_queued_cancel_updates_resource(setup, monkeypatch):
    _, service, _ = setup
    indexed(service, monkeypatch)
    dataset = service.dataset(
        who(),
        {
            "org_id": "org-a",
            "name": "cancel",
            "profile_id": "profile",
            "sources": [{"recording_id": "source", "revision": 0}],
        },
    )
    assert service.cancel(who(), dataset["job_id"])["status"] == "cancelled"
    assert service.detail(who(), "dataset", dataset["id"])["status"] == "cancelled"


def test_unsupported_payload_and_missing_checksum(tmp_path):
    with pytest.raises(LearningError) as error:
        materialize({"download_url": "https://example.invalid/source", "schema": {}}, tmp_path)
    assert error.value.code == "source_checksum_missing"
    with pytest.raises(LearningError) as error:
        materialize(
            {
                "download_url": "https://example.invalid/source",
                "source_sha256": "a" * 64,
                "schema": {"payload": "recording.db"},
            },
            tmp_path,
        )
    assert error.value.code == "source_format_unsupported"
