import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from dimos.imitation.learning import worker as worker_module
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


def process_state(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0]
    except FileNotFoundError:
        return None


@pytest.mark.skipif(
    not sys.platform.startswith("linux"), reason="Linux /proc death-state regression"
)
@pytest.mark.parametrize(
    ("mode", "stubborn_leader"),
    [
        ("cancel", False),
        ("cancel", True),
        ("heartbeat_error", False),
        ("report_error", True),
        ("success", False),
        ("failure", False),
        ("invalid_result", False),
    ],
)
def test_once_real_executor_group_cleanup(tmp_path, monkeypatch, mode, stubborn_leader):
    """Real OS descendants must be stopped BEFORE reporting, not just leader.wait()."""
    real_popen = subprocess.Popen
    processes = []
    descendant_file = tmp_path / "descendant.pid"
    leaf = (
        "import os, signal, time; from pathlib import Path; "
        "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
        f"Path({str(descendant_file)!r}).write_text(str(os.getpid())); time.sleep(60)"
    )

    def spawn(argv, **kwargs):
        assert kwargs["start_new_session"] is True
        output = argv[argv.index("--result-file") + 1]
        if mode in ("success", "failure", "invalid_result"):
            result = {"status": "succeeded", "result": {"preserved": True}}
            if mode == "failure":
                result = {"status": "failed", "error": {"code": "expected_failure"}}
            content = "not json" if mode == "invalid_result" else json.dumps(result)
            script = f"from pathlib import Path; Path({output!r}).write_text({content!r})"
        else:
            script = (
                "import signal, subprocess, sys, time; "
                + ("signal.signal(signal.SIGTERM, signal.SIG_IGN); " if stubborn_leader else "")
                + f"subprocess.Popen([sys.executable, '-c', {leaf!r}]); time.sleep(60)"
            )
        process = real_popen([sys.executable, "-c", script], **kwargs)
        processes.append(process)
        deadline = time.monotonic() + 5
        while not descendant_file.exists() and mode not in ("success", "failure", "invalid_result"):
            assert time.monotonic() < deadline, "Disposable descendant did not start"
            time.sleep(0.01)
        return process

    def assert_stopped():
        assert processes[0].returncode is not None  # leader was actually reaped
        if descendant_file.exists():
            # Orphan zombies cannot run; only init can reap them, not this worker.
            assert process_state(int(descendant_file.read_text())) in (None, "Z", "X")

    reports = []

    def post(route, body):
        if route == "worker/claim":
            return {"job": {"id": "disposable", "attempt": 1, "lease_token": "synthetic"}}
        if route == "worker/heartbeat":
            if body.get("active_job_id") and mode == "heartbeat_error":
                raise RuntimeError("synthetic heartbeat failure")
            return {"cancelled_job_ids": ["disposable"] if mode == "cancel" else []}
        if mode == "report_error":
            raise RuntimeError("synthetic report failure")
        if body["status"] != "running":
            assert_stopped()
            reports.append(body)
        return {}

    monkeypatch.setattr(worker_module.subprocess, "Popen", spawn)
    monkeypatch.setattr(worker_module, "EXECUTOR_STOP_GRACE", 0.2)
    worker = worker_module.Worker("http://unused.invalid", "synthetic", tmp_path)
    monkeypatch.setattr(worker, "post", post)
    started = time.monotonic()
    try:
        if mode in ("heartbeat_error", "report_error", "invalid_result"):
            with pytest.raises((RuntimeError, json.JSONDecodeError)):
                worker.once()
            assert reports == []
        else:
            assert worker.once() is True
            expected = {"cancel": "cancelled", "success": "succeeded", "failure": "failed"}
            assert reports[0]["status"] == expected[mode]
            if mode == "success":
                assert reports[0]["result"] == {"preserved": True}
            if mode == "failure":
                assert reports[0]["error"] == {"code": "expected_failure"}
        assert_stopped()
        assert time.monotonic() - started < 5
        assert os.getpgrp() != processes[0].pid
    finally:
        # Test-only fail-safe: signal ONLY the disposable executor group, never services.
        for process in processes:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5)
