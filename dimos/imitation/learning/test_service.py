from concurrent.futures import ThreadPoolExecutor
import copy

import pytest

from dimos.imitation.learning.annotation_adapter import preflight
from dimos.imitation.learning.conftest import indexed, who
from dimos.imitation.learning.protocol import LearningError
from dimos.imitation.learning.service import Service


def test_cas_concurrency_and_reopen(setup, monkeypatch):
    _, service, store = setup
    indexed(service, monkeypatch)
    initial = service.annotations(who(), "source")
    body = {"expected_revision": 0, "episodes": initial["episodes"]}

    def save():
        try:
            return service.save_annotations(who(), "source", body)["revision"]
        except LearningError as error:
            return error.code

    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(lambda _: save(), range(2)))
    assert sorted(map(str, results)) == ["1", "revision_conflict"]
    reopened = Service(store)
    indexed(reopened, monkeypatch)
    assert reopened.annotations(who(), "source")["revision"] == 1
    with store.read() as db:
        assert (
            store.get(db, "revision", "org-a:source:1")["capture_truth"] == initial["capture_truth"]
        )


@pytest.mark.parametrize(
    "mutation", ["outside", "unknown", "duplicate", "missing", "nan", "included", "review"]
)
def test_invalid_annotations(setup, monkeypatch, mutation):
    _, service, _ = setup
    indexed(service, monkeypatch)
    body = {"expected_revision": 0, "episodes": service.annotations(who(), "source")["episodes"]}
    ep = body["episodes"][0]
    if mutation == "outside":
        ep["start_s"] = 0
    if mutation == "unknown":
        ep["episode_id"] = "invented"
    if mutation == "duplicate":
        body["episodes"].append(copy.deepcopy(ep))
    if mutation == "missing":
        body["episodes"] = []
    if mutation == "nan":
        ep["start_s"] = float("nan")
    if mutation == "included":
        ep["included"] = "yes"
    if mutation == "review":
        ep["review_status"] = "anything"
    with pytest.raises(LearningError) as error:
        service.save_annotations(who(), "source", body)
    assert error.value.status == 422


def test_read_only_and_org_denial(setup, monkeypatch):
    _, service, store = setup
    indexed(service, monkeypatch)
    identity = who()
    identity["sources"][0]["can_write"] = False
    assert service.annotations(identity, "source")["revision"] == 0
    with pytest.raises(LearningError):
        service.save_annotations(identity, "source", {"expected_revision": 0, "episodes": []})
    with store.transaction() as db:
        store.put(
            db, "dataset", "private", {"id": "private", "org_id": "other", "owner_id": "alice"}
        )
    with pytest.raises(LearningError) as error:
        service.detail(who(), "dataset", "private")
    assert error.value.status == 403


def test_snapshot_exact_revision_and_block(setup, monkeypatch):
    _, service, store = setup
    indexed(service, monkeypatch)
    initial = service.annotations(who(), "source")
    saved = service.save_annotations(
        who(), "source", {"expected_revision": 0, "episodes": initial["episodes"]}
    )
    dataset = service.dataset(
        who(),
        {
            "org_id": "org-a",
            "name": "snapshot",
            "profile_id": "profile",
            "sources": [{"recording_id": "source", "revision": 1}],
        },
    )
    initial["episodes"][0]["notes"] = "later edit"
    service.save_annotations(
        who(), "source", {"expected_revision": 1, "episodes": initial["episodes"]}
    )
    with store.read() as db:
        job = store.get(db, "job", dataset["job_id"])
    assert job["payload"]["snapshots"][0]["annotation"] == saved
    with pytest.raises(LearningError) as error:
        preflight(job["payload"]["snapshots"])
    assert error.value.code == "gripper_command_missing"
    with pytest.raises(LearningError):
        service.dataset(
            who(sources=[]),
            {
                "org_id": "org-a",
                "name": "bad",
                "profile_id": "profile",
                "sources": [{"recording_id": "source", "revision": 1}],
            },
        )


@pytest.mark.parametrize("split,expected", [(0, []), (0.1, [9]), (0.2, [8, 9])])
def test_explicit_run_splits(setup, split, expected):
    _, service, store = setup
    with store.transaction() as db:
        store.put(
            db,
            "dataset",
            "d",
            {"id": "d", "org_id": "org-a", "owner_id": "alice", "status": "ready", "episodes": 10},
        )
    run = service.run(
        who(),
        {
            "org_id": "org-a",
            "dataset_id": "d",
            "name": "train",
            "preset": "act",
            "eval_split": split,
            "steps": 1,
        },
    )
    # Effective indices are deliberately unresolved until native training reads tasks.
    assert run["eval_indices"] is None
    assert run["train_indices"] is None
    assert run["eval_split"] == split
