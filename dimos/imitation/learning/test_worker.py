import time

import pytest

from dimos.imitation.learning.conftest import pair, who
from dimos.imitation.learning.protocol import LearningError


def queued(service, store, kind="media", identity=None):
    with store.transaction() as db:
        return service.new_job(db, identity or who(), kind, {"recording_id": "source"})


def test_native_empty_split_error_is_blocked(monkeypatch):
    import io
    import sys
    from types import SimpleNamespace

    from dimos.imitation.learning.worker import native

    monkeypatch.setenv("DIMOS_LEARNING_NATIVE_PYTHON", sys.executable)
    process = SimpleNamespace(
        stdin=io.StringIO(),
        stdout=io.StringIO(
            '{"message":"Native split blocked","error":{"code":"empty_split","message":"0 training episodes"}}\n'
        ),
        wait=lambda: 0,
    )
    monkeypatch.setattr(
        "dimos.imitation.learning.worker.subprocess.Popen", lambda *a, **kw: process
    )
    with pytest.raises(LearningError) as error:
        native({"command": "train"})
    assert error.value.code == "empty_split" and error.value.status == 422


def test_concurrent_claim_is_single_winner(setup):
    from concurrent.futures import ThreadPoolExecutor

    _, service, store = setup
    job = queued(service, store)
    workers = [pair(service), pair(service)]
    with ThreadPoolExecutor(max_workers=2) as pool:
        claims = list(
            pool.map(lambda w: service.claim(w["token"], {"kinds": ["media"]})["job"], workers)
        )
    winners = [claim for claim in claims if claim is not None]
    assert len(winners) == 1 and winners[0]["id"] == job["id"]
    with store.read() as db:
        saved = store.get(db, "job", job["id"])
        assert saved["attempt"] == 1
        assert saved["lease_token"] == winners[0]["lease_token"]


def test_claim_scope_capability_and_single_active(setup):
    _, service, store = setup
    foreign = queued(service, store, identity=who(org_id="org-b"))
    train = queued(service, store, "train")
    media = queued(service, store)
    worker = pair(service, capabilities=["media"])
    claim = service.claim(worker["token"], {"kinds": ["media", "train"]})["job"]
    assert claim["id"] == media["id"]
    assert claim["attempt"] == 1 and claim["lease_expires_at"] > time.time()
    assert service.claim(worker["token"], {"kinds": ["media"]})["job"] is None
    with store.read() as db:
        assert store.get(db, "job", train["id"])["status"] == "queued"
        assert store.get(db, "job", foreign["id"])["status"] == "queued"


def test_reports_idempotent_logs_and_stale_fenced(setup):
    _, service, store = setup
    job = queued(service, store)
    worker = pair(service)
    claim = service.claim(worker["token"], {"kinds": ["media"]})["job"]
    report = {
        "lease_token": claim["lease_token"],
        "sequence": 1,
        "stage": "decode",
        "status": "running",
        "progress": {"completed": 3, "total": 10, "unit": "frames"},
        "logs": [{"message": "real counter"}],
    }
    first = service.report(worker["token"], job["id"], report)
    assert service.report(worker["token"], job["id"], report) == first
    assert first["logs"][0]["sequence"] == 1
    with pytest.raises(LearningError):
        service.report(worker["token"], job["id"], {**report, "stage": "different"})
    with store.transaction() as db:
        stored = store.get(db, "job", job["id"])
        stored["lease_expires_at"] = 0
        store.put(db, "job", job["id"], stored)
    next_claim = service.claim(worker["token"], {"kinds": ["media"]})["job"]
    assert next_claim["attempt"] == 2 and next_claim["lease_token"] != claim["lease_token"]
    with pytest.raises(LearningError) as error:
        service.report(worker["token"], job["id"], report)
    assert error.value.code == "stale_lease"


def test_cancel_propagates_and_never_accepts_success(setup):
    _, service, store = setup
    job = queued(service, store)
    worker = pair(service)
    claim = service.claim(worker["token"], {"kinds": ["media"]})["job"]
    assert service.cancel(who(), job["id"])["status"] == "cancelling"
    assert job["id"] in service.heartbeat(worker["token"], {})["cancelled_job_ids"]
    result = service.report(
        worker["token"],
        job["id"],
        {"lease_token": claim["lease_token"], "sequence": 1, "status": "succeeded", "result": {}},
    )
    assert result["status"] == "cancelled"
    assert service.media_status(who(), "source")["status"] == "cancelled"


def test_incomplete_success_and_identity_rejected(setup):
    _, service, store = setup
    job = queued(service, store)
    worker = pair(service)
    claim = service.claim(worker["token"], {"kinds": ["media"]})["job"]
    for result in ({}, {"org_id": "other"}):
        with pytest.raises(LearningError):
            service.report(
                worker["token"],
                job["id"],
                {
                    "lease_token": claim["lease_token"],
                    "sequence": 1,
                    "status": "succeeded",
                    "result": result,
                },
            )
    assert service.detail(who(), "job", job["id"])["status"] == "running"


def test_queued_cancel_no_claim(setup):
    _, service, store = setup
    job = queued(service, store)
    assert service.cancel(who(), job["id"])["status"] == "cancelled"
    worker = pair(service)
    assert service.claim(worker["token"], {"kinds": ["media"]})["job"] is None


def test_revoked_and_other_workers_cannot_report(setup):
    _, service, store = setup
    job = queued(service, store)
    worker = pair(service)
    other = pair(service)
    claim = service.claim(worker["token"], {"kinds": ["media"]})["job"]
    report = {"lease_token": claim["lease_token"], "sequence": 1, "status": "running"}
    with pytest.raises(LearningError):
        service.report(other["token"], job["id"], report)
    with store.transaction() as db:
        item = store.get(db, "worker", worker["id"])
        item["revoked"] = True
        store.put(db, "worker", worker["id"], item)
    with pytest.raises(LearningError) as error:
        service.report(worker["token"], job["id"], report)
    assert error.value.status == 401
