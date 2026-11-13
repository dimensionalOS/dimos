"""Regressions for trusted enrollment and atomic lease terminal projection."""

from concurrent.futures import ThreadPoolExecutor

import pytest

from dimos.imitation.learning.conftest import headers, indexed, pair, who
from dimos.imitation.learning.protocol import LearningError, digest
from dimos.imitation.learning.service import Service


def test_private_share_cannot_be_claimed_by_self_enrolled_member(setup, monkeypatch):
    client, service, store = setup
    indexed(service, monkeypatch)
    private = {
        **who()["sources"][0],
        "can_write": False,
        "org_id": "outside-org",
        "owner_id": "outside-owner",
        "download_url": "https://example.invalid/synthetic-private-presigned-token",
    }
    victim = who(sub="victim", sources=[private])
    dataset = service.dataset(
        victim,
        {
            "org_id": "org-a",
            "name": "private share",
            "profile_id": "profile",
            "sources": [{"recording_id": "source", "revision": 0}],
        },
    )
    path = "/v1/workers/pairing"
    body = {"org_id": "org-a", "name": "attacker", "role": "admin", "platform_admin": True}
    response = client.post(path, json=body, headers=headers("POST", path, sub="attacker"))
    assert response.status_code == 403
    assert "synthetic-private" not in response.text
    for read_path in (
        "/v1/jobs",
        f"/v1/jobs/{dataset['job_id']}",
        "/v1/datasets",
        f"/v1/datasets/{dataset['id']}",
    ):
        public = client.get(read_path, headers=headers("GET", read_path, sub="attacker"))
        assert public.status_code == 200 and "synthetic-private" not in public.text
    with pytest.raises(LearningError) as error:
        service.pairing(who(sub="attacker", sources=[]), body)
    assert error.value.status == 403
    with store.read() as db:
        assert store.all(db, "pairing") == []
        assert store.all(db, "worker") == []
        assert store.get(db, "job", dataset["job_id"])["status"] == "queued"
    # An administrator explicitly entrusts an org-wide worker; its source list
    # need not include each job owner's private share. No new source RBAC.
    response = client.post(
        path,
        json={"org_id": "org-a"},
        headers=headers("POST", path, role="admin", sub="trusted-admin"),
    )
    assert response.status_code == 200
    paired = client.post(
        "/v1/workers/pair", json={"code": response.json()["code"], "capabilities": ["dataset"]}
    )
    assert paired.status_code == 200
    claim = client.post(
        "/v1/worker/claim",
        json={"kinds": ["dataset"]},
        headers={"Authorization": "Bearer " + paired.json()["token"]},
    )
    assert claim.status_code == 200
    job = claim.json()["job"]
    assert job["id"] == dataset["job_id"] and job["owner_id"] == "victim"
    assert job["payload"]["snapshots"][0]["source"] == private


def test_untrusted_legacy_pairings_and_credentials_fail_closed(setup):
    _, service, store = setup
    trusted = pair(service, who(role="admin"))
    with store.transaction() as db:
        worker = store.get(db, "worker", trusted["id"])
        worker.pop("trusted_org_worker", None)
        store.put(db, "worker", worker["id"], worker)
    with pytest.raises(LearningError) as error:
        service.claim(trusted["token"], {"kinds": ["dataset"]})
    assert error.value.status == 401
    code = service.pairing(who(role="admin"), {"org_id": "org-a"})["code"]

    with store.transaction() as db:
        invitation = store.get(db, "pairing", digest(code))
        invitation.pop("trusted_org_worker", None)
        store.put(db, "pairing", digest(code), invitation)
    with pytest.raises(LearningError) as error:
        service.pair({"code": code, "capabilities": ["dataset"]})
    assert error.value.status == 401


def test_executor_unexpected_failure_persists_result_and_logs_traceback(
    tmp_path, monkeypatch, caplog
):
    import json
    import sys

    from dimos.imitation.learning import worker

    request = tmp_path / "job.json"
    result = tmp_path / "result.json"
    request.write_text("{}")
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "worker",
            "--execute-file",
            str(request),
            "--result-file",
            str(result),
            "--state",
            str(tmp_path),
        ],
    )

    def broken_executor(*args):
        raise RuntimeError("synthetic executor failure")

    monkeypatch.setattr(worker, "execute", broken_executor)
    worker.main()
    assert json.loads(result.read_text()) == {
        "status": "failed",
        "error": {"code": "execution_failed", "message": "synthetic executor failure"},
    }
    assert any(record.exc_info and "executor failed" in record.message for record in caplog.records)


def linked_job(service, store, kind):
    resource_kind = {"dataset": "dataset", "train": "run", "media": "media"}[kind]
    resource_id = "org-a:source" if kind == "media" else "resource"
    with store.transaction() as db:
        job = service.new_job(
            db,
            who(),
            kind,
            {"recording_id": "source"},
            **({resource_kind + "_id": resource_id} if kind != "media" else {}),
        )
        store.put(
            db,
            resource_kind,
            resource_id,
            {
                "id": resource_id,
                "org_id": "org-a",
                "owner_id": "alice",
                "status": "queued",
                "job_id": job["id"],
            },
        )
    return job, resource_kind, resource_id


@pytest.mark.parametrize("kind", ["dataset", "train", "media"])
@pytest.mark.parametrize("cancel", [True, False])
def test_expired_terminal_projects_resource_and_fences_late_report(setup, kind, cancel):
    _, service, store = setup
    job, resource_kind, resource_id = linked_job(service, store, kind)
    worker = pair(service, who(role="admin"))
    claim = service.claim(worker["token"], {"kinds": [kind]})["job"]
    if cancel:
        assert service.cancel(who(), job["id"])["status"] == "cancelling"
    with store.transaction() as db:
        current = store.get(db, "job", job["id"])
        current.update(lease_expires_at=0, attempt=3)
        store.put(db, "job", job["id"], current)
    assert service.claim(worker["token"], {"kinds": [kind]})["job"] is None
    expected = "cancelled" if cancel else "failed"
    reopened = Service(store)
    assert reopened.detail(who(), "job", job["id"])["status"] == expected
    with store.read() as db:
        resource = store.get(db, resource_kind, resource_id)
        assert resource["status"] == expected
    with pytest.raises(LearningError) as error:
        service.report(
            worker["token"],
            job["id"],
            {
                "lease_token": claim["lease_token"],
                "sequence": 1,
                "status": "succeeded",
                "result": {},
            },
        )
    assert error.value.code == "stale_lease"


@pytest.mark.parametrize("kind", ["dataset", "train", "media"])
def test_cancellation_wins_terminal_report_and_discards_success_payload(setup, kind):
    _, service, store = setup
    job, resource_kind, resource_id = linked_job(service, store, kind)
    worker = pair(service, who(role="admin"))
    claim = service.claim(worker["token"], {"kinds": [kind]})["job"]
    service.cancel(who(), job["id"])
    result = (
        {"status": "ready", "cameras": [{"id": "head"}]}
        if kind == "media"
        else {"publication": {"prefix": "ignored"}}
    )
    report = {
        "lease_token": claim["lease_token"],
        "sequence": 1,
        "status": "succeeded",
        "result": result,
    }
    assert service.report(worker["token"], job["id"], report)["status"] == "cancelled"
    assert service.report(worker["token"], job["id"], report)["status"] == "cancelled"
    with store.read() as db:
        resource = store.get(db, resource_kind, resource_id)
        assert resource["status"] == "cancelled"
        assert not resource.get("cameras") and "publication" not in resource
        assert not store.all(db, "policy")


def test_cancel_and_terminal_report_race_agree(setup):
    _, service, store = setup
    job, kind, key = linked_job(service, store, "dataset")
    worker = pair(service, who(role="admin"))
    claim = service.claim(worker["token"], {"kinds": ["dataset"]})["job"]
    with ThreadPoolExecutor(max_workers=2) as pool:
        cancel = pool.submit(service.cancel, who(), job["id"])
        report = pool.submit(
            service.report,
            worker["token"],
            job["id"],
            {"lease_token": claim["lease_token"], "sequence": 1, "status": "failed"},
        )
        cancel.result()
        report.result()
    with store.read() as db:
        assert store.get(db, "job", job["id"])["status"] == store.get(db, kind, key)["status"]


def test_projection_failure_rolls_back_job_and_resource(setup, monkeypatch):
    _, service, store = setup
    job, kind, key = linked_job(service, store, "dataset")
    worker = pair(service, who(role="admin"))
    service.claim(worker["token"], {"kinds": ["dataset"]})
    service.cancel(who(), job["id"])
    with store.transaction() as db:
        current = store.get(db, "job", job["id"])
        current["lease_expires_at"] = 0
        store.put(db, "job", job["id"], current)
    put = store.put

    def fail_projection(db, resource_kind, resource_id, resource):
        put(db, resource_kind, resource_id, resource)
        if resource_kind == kind:
            raise RuntimeError("synthetic projection write failure")

    monkeypatch.setattr(store, "put", fail_projection)
    with pytest.raises(RuntimeError, match="synthetic projection"):
        service.claim(worker["token"], {"kinds": ["dataset"]})
    with store.read() as db:
        assert store.get(db, "job", job["id"])["status"] == "cancelling"
        assert store.get(db, kind, key)["status"] == "queued"
