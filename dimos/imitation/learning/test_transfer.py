"""Real object transfer/report/download across isolated API and worker state dirs."""

import copy
import hashlib
import json
from pathlib import Path
import shutil
from uuid import uuid4

from fastapi.testclient import TestClient
from moto.server import ThreadedMotoServer
import pytest

from dimos.imitation.learning import worker
from dimos.imitation.learning.app import create_app
from dimos.imitation.learning.artifacts import Artifacts, seal, verify
from dimos.imitation.learning.conftest import headers, pair, who
from dimos.imitation.learning.media import sha256
from dimos.imitation.learning.protocol import LearningError


@pytest.fixture
def storage(monkeypatch):
    monkeypatch.setenv("AWS_ACCESS_KEY_ID", "testing")
    monkeypatch.setenv("AWS_SECRET_ACCESS_KEY", "testing")
    server = ThreadedMotoServer(ip_address="127.0.0.1", port=0, verbose=False)
    server.start()
    host, port = server.get_host_and_port()
    monkeypatch.setenv("DIMOS_LEARNING_S3_BUCKET", "learning-transfer-" + uuid4().hex)
    monkeypatch.setenv("DIMOS_LEARNING_S3_ENDPOINT", f"http://{host}:{port}")
    try:
        artifacts = Artifacts()
        artifacts.client.create_bucket(Bucket=artifacts.bucket)
        yield artifacts
    finally:
        server.stop()


def test_rejected_reseal_preserves_every_original_byte(tmp_path):
    (tmp_path / "weights").write_bytes(b"fixture weights")
    first = seal(tmp_path, {"revision": 1}, "policy")
    before = {p.name: p.read_bytes() for p in tmp_path.iterdir()}
    with pytest.raises(LearningError, match="already sealed"):
        seal(tmp_path, {"revision": 2}, "policy")
    assert {p.name: p.read_bytes() for p in tmp_path.iterdir()} == before
    assert verify(tmp_path) == first


def make_job(service, store, kind):
    payload = {"recording_id": "source", "source": {"id": "source"}}
    with store.transaction() as db:
        if kind == "train":
            store.put(
                db,
                "run",
                "run",
                {
                    "id": "run",
                    "org_id": "org-a",
                    "owner_id": "alice",
                    "name": "fixture",
                    "dataset_id": "dataset",
                    "status": "queued",
                },
            )
        service.new_job(db, who(), kind, payload, **({"run_id": "run"} if kind == "train" else {}))
    machine = pair(service)
    return machine, service.claim(machine["token"], {"kinds": [kind]})["job"]


def report(service, machine, job, result):
    return service.report(
        machine["token"],
        job["id"],
        {"lease_token": job["lease_token"], "sequence": 1, "status": "succeeded", "result": result},
    )


def media_result(job, state, monkeypatch):
    monkeypatch.setattr(worker, "materialize", lambda *args: state)

    def prepare(directory, output, emit):
        output.mkdir(parents=True)
        path = output / "head.mp4"
        path.write_bytes(b"synthetic prepared camera bytes")
        return {
            "status": "ready",
            "duration_s": 5.0,
            "cameras": [{"id": "head", "offset_s": 2.0, "sha256": sha256(path)}],
        }

    monkeypatch.setattr(worker, "prepare", prepare)
    return worker.execute(job, state)


def train_result(job, state, monkeypatch, storage):
    dataset = state / "dataset"
    dataset.mkdir(parents=True)
    (dataset / "weights").write_bytes(b"fixture dataset, no training")
    seal(dataset, {}, "fixture")
    publication = storage.publish(dataset, "org-a", "dataset")
    job["payload"] = {
        "run": {"id": "run"},
        "dataset": {"id": "dataset", "path": str(dataset), "publication": publication},
    }

    def native(request):
        root = Path(request["output"])
        root.mkdir()
        (root / "weights").write_bytes(b"synthetic policy, no model training")
        return {"load_ready": False, "rollout_ready": False}

    monkeypatch.setattr(worker, "native", native)
    return worker.execute(job, state)


@pytest.mark.parametrize("kind", ["media", "train"])
def test_separate_state_report_and_authenticated_download(
    setup, tmp_path_factory, monkeypatch, storage, kind
):
    client, service, store = setup
    state = tmp_path_factory.mktemp("worker-only")
    assert not state.is_relative_to(service.state) and not service.state.is_relative_to(state)
    machine, job = make_job(service, store, kind)
    result = (
        media_result(job, state, monkeypatch)
        if kind == "media"
        else train_result(job, state, monkeypatch, storage)
    )
    expected = (
        b"synthetic prepared camera bytes"
        if kind == "media"
        else Path(result["package_path"]).read_bytes()
    )
    # Real worker disk disappears BEFORE API verification: object store is the only bridge.
    shutil.rmtree(state)
    if kind == "train":
        result["package_path"] = (
            "/etc/passwd"  # ignored: only the verified object is authoritative.
        )
    original = copy.deepcopy(result)
    assert report(service, machine, job, result)["status"] == "succeeded"
    assert report(service, machine, job, result)["status"] == "succeeded"  # identical retry
    assert result == original  # API normalization must not mutate the caller's retry payload.
    if kind == "media":
        path = "/v1/recordings/source/media/head"
        status = service.media_status(who(), "source")
        assert status["cameras"][0]["offset_s"] == 2.0
        local = service.state / status["cameras"][0]["path"]
    else:
        run = service.detail(who(), "run", "run")
        path = f"/v1/policies/{run['policy_id']}/download"
        local = Path(service.detail(who(), "policy", run["policy_id"])["package_path"])
    response = client.get(path, headers=headers(path=path, sources=who()["sources"]))
    assert response.status_code == 200, response.text
    assert response.content == expected
    assert local.is_relative_to(service.state)
    assert client.get(path, headers=headers(path=path, org_id="org-b", sources=[])).status_code in (
        403,
        404,
    )
    assert client.get(path).status_code == 401
    partial = client.get(
        path, headers={**headers(path=path, sources=who()["sources"]), "Range": "bytes=0-7"}
    )
    assert partial.status_code == 206 and partial.content == expected[:8]
    with TestClient(create_app(store)) as reopened:
        restored = reopened.get(path, headers=headers(path=path, sources=who()["sources"]))
        assert restored.status_code == 200 and restored.content == expected
    local.write_bytes(b"cache corruption")
    assert client.get(path, headers=headers(path=path, sources=who()["sources"])).status_code == 409


@pytest.mark.parametrize("kind", ["media", "train"])
@pytest.mark.parametrize(
    "corruption", ["prefix", "target", "object", "digest", "path", "lineage", "kind"]
)
def test_transfer_rejects_unverified_or_wrong_scope_without_terminalizing(
    setup, tmp_path_factory, monkeypatch, storage, kind, corruption
):
    _, service, store = setup
    state = tmp_path_factory.mktemp("worker-only")
    machine, job = make_job(service, store, kind)
    result = (
        media_result(job, state, monkeypatch)
        if kind == "media"
        else train_result(job, state, monkeypatch, storage)
    )
    publication = result["publication" if kind == "media" else "package_publication"]
    if corruption == "prefix":
        publication["prefix"] = "learning/other-org/other-job/"
    elif corruption == "target":
        publication["bucket"] = "wrong-bucket"
    elif corruption == "object":
        name = "head.mp4" if kind == "media" else "policy.tar.gz"
        storage.client.put_object(
            Bucket=storage.bucket, Key=publication["prefix"] + name, Body=b"tamper"
        )
    elif corruption in ("lineage", "kind"):
        key = publication["prefix"] + "manifest.json"
        manifest = json.loads(
            storage.client.get_object(Bucket=storage.bucket, Key=key)["Body"].read()
        )
        if corruption == "lineage":
            manifest["lineage"] = {"different": "job or policy"}
        else:
            manifest["kind"] = "wrong-kind"
        raw = json.dumps(manifest).encode()
        storage.client.put_object(Bucket=storage.bucket, Key=key, Body=raw)
        publication["manifest_sha256"] = hashlib.sha256(raw).hexdigest()
    elif corruption == "digest":
        if kind == "media":
            result["cameras"][0]["sha256"] = "0" * 64
        else:
            result["package_sha256"] = "0" * 64
    elif kind == "media":
        result["cameras"][0]["path"] = "../../secret"
    else:
        result["package_path"] = "/etc/passwd"
        result.pop("package_publication")  # local-only legacy paths must fail closed.
    with pytest.raises(LearningError):
        report(service, machine, job, result)
    assert service.detail(who(), "job", job["id"])["status"] == "running"
