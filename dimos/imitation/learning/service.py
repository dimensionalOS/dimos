"""Learning business rules and transactional worker leases."""

from __future__ import annotations

import copy
import json
import math
import os
from pathlib import Path
import secrets
import shlex
import time
from typing import Any
from uuid import uuid4

from dimos.imitation.learning.artifacts import Artifacts
from dimos.imitation.learning.media import capture, materialize, sha256
from dimos.imitation.learning.protocol import LearningError, allowed, digest, source
from dimos.imitation.learning.store import Store

TERMINAL = {"succeeded", "failed", "blocked", "cancelled"}
KINDS = {"media", "dataset", "train", "load-check"}
PRESETS = [
    {
        "id": "act",
        "name": "ACT",
        "policy_type": "act",
        "device": "cpu",
        "eval_split": 0,
        "steps": 1000,
    }
]


def uid() -> str:
    return str(uuid4())


class Service:
    def __init__(self, store: Store):
        self.store = store
        self.state = store.path.parent

    def recording(self, who: dict[str, Any], key: str) -> dict[str, Any]:
        descriptor = source(who, key)
        directory = materialize(descriptor, self.state)
        index_path = directory / "index-v2.json"
        if index_path.exists():
            info = json.loads(index_path.read_text())
        else:
            info = capture(directory)
            temporary = index_path.with_suffix(".partial-" + uid())
            temporary.write_text(json.dumps(info))
            temporary.replace(index_path)
        return {
            **info,
            "source_checksums": json.loads((directory / "complete.json").read_text()),
            "id": key,
            "org_id": who["org_id"],
            "can_write": bool(descriptor.get("can_write")),
            "media_status": self.media_status(who, key)["status"],
        }

    def annotations(self, who: dict[str, Any], key: str, db: Any = None) -> dict[str, Any]:
        info = self.recording(who, key)
        if db is None:
            with self.store.transaction() as connection:
                return self.annotations(who, key, connection)
        identity = f"{who['org_id']}:{key}"
        current = self.store.get(db, "annotation", identity) or self.store.get(
            db, "revision", identity + ":0"
        )
        if current:
            if current.get("source_checksums", {}) != info.get("source_checksums", {}):
                raise LearningError(
                    409, "source_changed", "Source bytes differ from the saved annotation lineage"
                )
            return current
        baseline = {
            "recording_id": key,
            "revision": 0,
            "source_checksums": info.get("source_checksums", {}),
            "capture_truth": info["capture_truth"],
            "episodes": [
                {
                    "episode_id": ep["id"],
                    "start_s": ep["start_s"],
                    "end_s": ep["end_s"],
                    "included": ep.get("capture_event") == "save",
                    "instruction": ep["instruction"],
                    "outcome": ep["outcome"],
                    "notes": "",
                    "review_status": "unreviewed",
                }
                for ep in info["episodes"]
            ],
        }
        self.store.put(db, "revision", identity + ":0", baseline)
        return baseline

    def save_annotations(
        self, who: dict[str, Any], key: str, body: dict[str, Any]
    ) -> dict[str, Any]:
        source(who, key, write=True)
        info = self.recording(who, key)
        episodes = body.get("episodes")
        if not isinstance(episodes, list) or type(body.get("expected_revision")) is not int:
            raise LearningError(
                422, "invalid_annotations", "expected_revision and episodes required"
            )
        original = {ep["id"]: ep for ep in info["episodes"]}
        seen = set()
        for ep in episodes:
            key_ep = ep.get("episode_id")
            if key_ep not in original or key_ep in seen:
                raise LearningError(
                    422, "invalid_episode", "Episodes must have unique original source IDs"
                )
            seen.add(key_ep)
            start, end = ep.get("start_s"), ep.get("end_s")
            if (
                any(type(x) not in (float, int) or not math.isfinite(x) for x in (start, end))
                or not original[key_ep]["start_s"] <= start < end <= original[key_ep]["end_s"]
            ):
                raise LearningError(
                    422, "invalid_trim", "Trim must lie within the original episode"
                )
            if (
                type(ep.get("included")) is not bool
                or not isinstance(ep.get("instruction"), (str, type(None)))
                or not isinstance(ep.get("notes", ""), str)
            ):
                raise LearningError(422, "invalid_episode", "Invalid annotation field types")
            if ep.get("review_status") not in (
                "unreviewed",
                "pending",
                "approved",
                "rejected",
                "reviewed",
            ) or ep.get("outcome") not in (
                "success",
                "failure",
                "failed",
                "discarded",
                "unknown",
                None,
            ):
                raise LearningError(422, "invalid_episode", "Invalid review status or outcome")
        if seen != set(original):
            raise LearningError(
                422, "missing_episode", "Keep all original episodes; exclude using included=false"
            )
        with self.store.transaction() as db:
            current = self.annotations(who, key, db)
            if body["expected_revision"] != current["revision"]:
                raise LearningError(
                    409,
                    "revision_conflict",
                    "Annotations changed since this revision",
                    current_revision=current["revision"],
                )
            saved = {
                "recording_id": key,
                "source_checksums": info.get("source_checksums", {}),
                "revision": current["revision"] + 1,
                "episodes": copy.deepcopy(episodes),
                "capture_truth": info["capture_truth"],
            }
            identity = f"{who['org_id']}:{key}"
            self.store.put(db, "revision", f"{identity}:{saved['revision']}", saved)
            self.store.put(db, "annotation", identity, saved)
            return saved

    def media_status(self, who: dict[str, Any], key: str) -> dict[str, Any]:
        source(who, key)
        with self.store.read() as db:
            item = self.store.get(db, "media", f"{who['org_id']}:{key}")
        if not item:
            return {"status": "not_prepared", "cameras": [], "duration_s": None}
        result = copy.deepcopy(item)
        for camera in result.get("cameras", []):
            camera["url"] = f"/console/learning/api/v1/recordings/{key}/media/{camera['id']}"
        return result

    def queue_media(self, who: dict[str, Any], key: str) -> dict[str, Any]:
        descriptor = source(who, key)
        with self.store.transaction() as db:
            existing = [
                j
                for j in self.store.all(db, "job")
                if j["org_id"] == who["org_id"]
                and j["kind"] == "media"
                and j["payload"].get("recording_id") == key
                and j["status"] not in TERMINAL
            ]
            if existing:
                return self.public_job(existing[0])
            job = self.new_job(db, who, "media", {"source": descriptor, "recording_id": key})
            self.store.put(
                db,
                "media",
                f"{who['org_id']}:{key}",
                {"status": "queued", "cameras": [], "duration_s": None, "job_id": job["id"]},
            )
            return self.public_job(job)

    def new_job(
        self, db: Any, who: dict[str, Any], kind: str, payload: dict[str, Any], **links: Any
    ) -> dict[str, Any]:
        job = {
            "id": uid(),
            "org_id": who["org_id"],
            "owner_id": who["sub"],
            "kind": kind,
            "payload": payload,
            "status": "queued",
            "stage": "queued",
            "progress": {"completed": 0, "total": None, "unit": "items"},
            "attempt": 0,
            "logs": [],
            "last_sequence": 0,
            "created_at": time.time(),
            **links,
        }
        self.store.put(db, "job", job["id"], job)
        return job

    @staticmethod
    def public_job(job: dict[str, Any]) -> dict[str, Any]:
        return {
            k: v
            for k, v in job.items()
            if k not in ("payload", "lease_token", "worker_id", "last_report_hash")
        }

    def list(self, who: dict[str, Any], kind: str) -> list[dict[str, Any]]:
        with self.store.transaction() as db:
            items = [x for x in self.store.all(db, kind) if x.get("org_id") == who["org_id"]]
        if kind == "job":
            return [self.public_job(x) for x in items]
        if kind == "worker":
            return [
                {
                    **{k: v for k, v in x.items() if k != "token_hash"},
                    "status": "revoked"
                    if x["revoked"]
                    else "online"
                    if time.time() - x.get("last_seen", 0) <= 30
                    else "offline",
                }
                for x in items
            ]
        return items

    def detail(self, who: dict[str, Any], kind: str, key: str) -> dict[str, Any]:
        with self.store.transaction() as db:
            item = allowed(self.store.get(db, kind, key), who)
        return self.public_job(item) if kind == "job" else item

    def dataset(self, who: dict[str, Any], body: dict[str, Any]) -> dict[str, Any]:
        if body.get("org_id") != who["org_id"] or not body.get("sources") or not body.get("name"):
            raise LearningError(
                422, "invalid_dataset", "Explicit matching org, name and sources required"
            )
        snapshots = []
        for selection in body["sources"]:
            key, revision = selection.get("recording_id"), selection.get("revision")
            if type(revision) is not int or revision < 0:
                raise LearningError(422, "invalid_revision", "Exact nonnegative revision required")
            descriptor = source(who, key)
            info = self.recording(who, key)
            if body.get("profile_id") != info["profile_id"]:
                raise LearningError(
                    422,
                    "profile_mismatch",
                    "Selected recordings must match the requested schema profile",
                )
            with self.store.transaction() as db:
                annotation = self.store.get(db, "revision", f"{who['org_id']}:{key}:{revision}")
                if annotation is None and revision == 0:
                    annotation = self.annotations(who, key, db)
            if annotation is None or annotation["revision"] != revision:
                raise LearningError(
                    409, "revision_missing", "Requested saved revision does not exist"
                )
            if annotation.get("source_checksums", {}) != info.get("source_checksums", {}):
                raise LearningError(
                    409, "source_changed", "Dataset revision is bound to different source bytes"
                )
            snapshots.append({"source": descriptor, "annotation": annotation, "capture": info})
        with self.store.transaction() as db:
            key = uid()
            item = {
                "id": key,
                "org_id": who["org_id"],
                "owner_id": who["sub"],
                "name": body["name"],
                "profile_id": body["profile_id"],
                "sources": copy.deepcopy(body["sources"]),
                "status": "queued",
                "created_at": time.time(),
            }
            job = self.new_job(
                db, who, "dataset", {"snapshots": snapshots, "dataset_id": key}, dataset_id=key
            )
            item["job_id"] = job["id"]
            self.store.put(db, "dataset", key, item)
            return item

    def run(self, who: dict[str, Any], body: dict[str, Any]) -> dict[str, Any]:
        dataset = self.detail(who, "dataset", body.get("dataset_id", ""))
        if (
            body.get("org_id") != who["org_id"]
            or dataset["status"] != "ready"
            or body.get("preset") != "act"
            or body.get("eval_split", 0) not in (0, 0.1, 0.2)
        ):
            raise LearningError(
                422,
                "run_preflight",
                "Ready compatible dataset, act preset and valid split required",
            )
        steps = body.get("steps", 1000)
        if type(steps) is not int or not 1 <= steps <= 1000000:
            raise LearningError(422, "invalid_steps", "steps must be a positive bounded integer")
        with self.store.transaction() as db:
            key = uid()
            # Only native metadata knows task membership. Resolve at training time
            # with pinned LeRobot's per-task last-ceil split, never global slicing.
            train_indices, eval_indices = None, None
            item = {
                "id": key,
                "org_id": who["org_id"],
                "owner_id": who["sub"],
                "name": body.get("name", "ACT"),
                "dataset_id": dataset["id"],
                "preset": "act",
                "steps": steps,
                "eval_split": body.get("eval_split", 0),
                "train_indices": train_indices,
                "eval_indices": eval_indices,
                "status": "queued",
                "archived": False,
            }
            job = self.new_job(db, who, "train", {"dataset": dataset, "run": item}, run_id=key)
            item["job_id"] = job["id"]
            self.store.put(db, "run", key, item)
            return item

    def project_terminal(self, db: Any, job: dict[str, Any]) -> None:
        """Project unsuccessful terminal state inside the caller's job transaction.

        Cancellation never publishes a late executor result. Lease recovery and
        reports use the same projection; transaction serialization fences races.
        Successful publications retain their separately validated result path.
        """
        status = job["status"]
        if status not in TERMINAL - {"succeeded"}:
            raise ValueError("Unsuccessful terminal job required")
        kind = {"dataset": "dataset", "train": "run"}.get(job["kind"])
        if kind:
            resource = self.store.get(db, kind, job[kind + "_id"])
            if resource is not None:
                resource["status"] = status
                self.store.put(db, kind, resource["id"], resource)
        elif job["kind"] == "media":
            self.store.put(
                db,
                "media",
                f"{job['org_id']}:{job['payload']['recording_id']}",
                {"status": status, "cameras": [], "duration_s": None, "error": job.get("error")},
            )
        elif job["kind"] == "load-check":
            policy = self.store.get(db, "policy", job["payload"]["policy"]["id"])
            if policy is not None:
                policy["load_check"] = {"status": status, "error": job.get("error")}
                self.store.put(db, "policy", policy["id"], policy)

    def cancel(self, who: dict[str, Any], key: str) -> dict[str, Any]:
        with self.store.transaction() as db:
            job = allowed(self.store.get(db, "job", key), who, True)
            if job["status"] not in TERMINAL:
                job["status"] = "cancelling" if job["status"] == "running" else "cancelled"
                if job["status"] == "cancelled":
                    self.project_terminal(db, job)
                self.store.put(db, "job", key, job)
            return self.public_job(job)

    def pairing(self, who: dict[str, Any], body: dict[str, Any]) -> dict[str, Any]:
        if body.get("org_id") != who["org_id"]:
            raise LearningError(403, "forbidden", "Organization mismatch")
        if who.get("role") != "admin" and who.get("platform_admin") is not True:
            raise LearningError(
                403, "forbidden", "Organization worker enrollment requires administrator authority"
            )
        code = secrets.token_urlsafe(18)
        item = {
            "org_id": who["org_id"],
            "owner_id": who["sub"],
            "trusted_org_worker": True,
            "name": body.get("name", "worker"),
            "expires_at": time.time() + 300,
        }
        with self.store.transaction() as db:
            self.store.put(db, "pairing", digest(code), item)
        return {
            "code": code,
            "expires_at": item["expires_at"],
            "command": shlex.join(
                [
                    "python",
                    "-m",
                    "dimos.imitation.learning.worker",
                    "--api",
                    os.environ.get(
                        "DIMOS_LEARNING_PUBLIC_API_URL",
                        os.environ.get("DIMOS_LEARNING_API_URL", "http://127.0.0.1:8768"),
                    ),
                    "--pair",
                    code,
                    "--name",
                    str(item["name"]),
                ]
            ),
        }

    def pair(self, body: dict[str, Any]) -> dict[str, Any]:
        with self.store.transaction() as db:
            hashed = digest(str(body.get("code", "")))
            item = self.store.get(db, "pairing", hashed)
            if (
                not item
                or item["expires_at"] < time.time()
                or item.get("used")
                or item.get("trusted_org_worker") is not True
            ):
                raise LearningError(401, "pairing_invalid", "Pairing code expired or already used")
            capabilities = body.get("capabilities", [])
            if not isinstance(capabilities, list) or not set(capabilities) <= KINDS:
                raise LearningError(422, "capabilities_invalid", "Unsupported capabilities")
            token, key = secrets.token_urlsafe(40), uid()
            worker = {
                **item,
                "id": key,
                "name": body.get("name", item["name"]),
                "capabilities": capabilities,
                "token_hash": digest(token),
                "revoked": False,
                "last_seen": time.time(),
            }
            self.store.put(db, "worker", key, worker)
            item["used"] = True
            self.store.put(db, "pairing", hashed, item)
            return {"id": key, "token": token}

    def worker(self, db: Any, token: str) -> dict[str, Any]:
        worker = next(
            (
                x
                for x in self.store.all(db, "worker")
                if secrets.compare_digest(x["token_hash"], digest(token))
                and not x["revoked"]
                and x.get("trusted_org_worker") is True
            ),
            None,
        )
        if worker is None:
            raise LearningError(
                401, "worker_unauthorized", "Valid non-revoked machine token required"
            )
        return worker

    def heartbeat(self, token: str, body: dict[str, Any]) -> dict[str, Any]:
        with self.store.transaction() as db:
            worker = self.worker(db, token)
            worker["last_seen"] = time.time()
            self.store.put(db, "worker", worker["id"], worker)
            cancelled = []
            for job in self.store.all(db, "job"):
                if job.get("worker_id") == worker["id"] and job["status"] in (
                    "running",
                    "cancelling",
                ):
                    if job["status"] == "cancelling":
                        cancelled.append(job["id"])
                    elif job["lease_expires_at"] >= time.time():
                        job["lease_expires_at"] = time.time() + 90
                        self.store.put(db, "job", job["id"], job)
            return {"id": worker["id"], "cancelled_job_ids": cancelled}

    def claim(self, token: str, body: dict[str, Any]) -> dict[str, Any]:
        with self.store.transaction() as db:
            worker = self.worker(db, token)
            kinds = set(body.get("kinds", [])) & set(worker["capabilities"])
            if worker.get("active_job_id"):
                active = self.store.get(db, "job", worker["active_job_id"])
                if (
                    active
                    and active.get("worker_id") == worker["id"]
                    and active["status"] in ("running", "cancelling")
                    and active.get("lease_expires_at", 0) >= time.time()
                ):
                    return {"job": None}
                worker.pop("active_job_id", None)
                self.store.put(db, "worker", worker["id"], worker)
            jobs = self.store.all(db, "job")
            now = time.time()
            for job in jobs:
                if job["org_id"] != worker["org_id"]:
                    continue
                if job["status"] == "cancelling" and job.get("lease_expires_at", 0) < now:
                    job["status"] = "cancelled"
                    self.project_terminal(db, job)
                    self.store.put(db, "job", job["id"], job)
                if job["status"] == "running" and job.get("lease_expires_at", 0) < now:
                    job["status"] = "queued" if job["attempt"] < 3 else "failed"
                    job["error"] = {"code": "lease_expired", "message": "Worker lease expired"}
                    if job["status"] in TERMINAL:
                        self.project_terminal(db, job)
                    self.store.put(db, "job", job["id"], job)
                if (
                    job.get("worker_id") == worker["id"]
                    and job["status"] in ("running", "cancelling")
                    and job.get("lease_expires_at", 0) >= now
                ):
                    return {"job": None}
            for job in jobs:
                if (
                    job["org_id"] != worker["org_id"]
                    or job["kind"] not in kinds
                    or job["status"] != "queued"
                ):
                    continue
                job.update(
                    status="running",
                    stage="claimed",
                    worker_id=worker["id"],
                    lease_token=secrets.token_urlsafe(32),
                    lease_expires_at=now + 90,
                    attempt=job["attempt"] + 1,
                    last_sequence=0,
                )
                job.pop("last_report_hash", None)
                self.store.put(db, "job", job["id"], job)
                worker["active_job_id"] = job["id"]
                self.store.put(db, "worker", worker["id"], worker)
                return {"job": job}
            return {"job": None}

    def validate_result(self, job: dict[str, Any], result: dict[str, Any]) -> None:
        if job["kind"] == "media":
            if result.get("status") != "ready" or not result.get("cameras"):
                raise LearningError(422, "incomplete_media", "Prepared cameras required")
            for camera in result["cameras"]:
                path = (self.state / camera.get("path", "")).resolve()
                if (
                    not path.is_relative_to(self.state.resolve())
                    or not path.is_file()
                    or sha256(path) != camera.get("sha256")
                ):
                    raise LearningError(
                        409, "media_checksum", "Prepared camera failed server verification"
                    )
        elif job["kind"] in ("dataset", "train"):
            publication = result.get("publication")
            resource_id = job.get("dataset_id") if job["kind"] == "dataset" else job.get("run_id")
            if (
                not isinstance(publication, dict)
                or publication.get("prefix") != f"learning/{job['org_id']}/{resource_id}/"
            ):
                raise LearningError(
                    422, "incomplete_publication", "Sealed immutable resource publication required"
                )
            manifest = Artifacts().verify_publication(publication)
            if job["kind"] == "dataset":
                paths = [entry["path"] for entry in manifest["files"]]
                if (
                    any(
                        not any(path.startswith(directory + "/") for path in paths)
                        for directory in ("meta", "data", "videos")
                    )
                    or not result.get("episodes")
                    or not result.get("frames")
                ):
                    raise LearningError(
                        422,
                        "incomplete_dataset",
                        "Complete native dataset with real episode and frame counts required",
                    )
            else:
                package = Path(result.get("package_path", "")).resolve()
                if (
                    not package.is_relative_to(self.state.resolve())
                    or not package.is_file()
                    or sha256(package) != result.get("package_sha256")
                ):
                    raise LearningError(
                        409, "package_checksum", "Policy package failed server verification"
                    )
        elif job["kind"] == "load-check" and (
            result.get("no_action_executed") is not True or result.get("load_ready") is not True
        ):
            raise LearningError(422, "invalid_load_check", "No-action CPU loader result required")

    def report(self, token: str, key: str, body: dict[str, Any]) -> dict[str, Any]:
        with self.store.transaction() as db:
            worker = self.worker(db, token)
            job = self.store.get(db, "job", key)
            if (
                not job
                or job.get("worker_id") != worker["id"]
                or body.get("lease_token") != job.get("lease_token")
                or job.get("lease_expires_at", 0) < time.time()
            ):
                raise LearningError(
                    409, "stale_lease", "Worker report does not hold the current lease"
                )
            hashed = digest(json.dumps(body, sort_keys=True))
            if body.get("sequence") == job["last_sequence"] and hashed == job.get(
                "last_report_hash"
            ):
                return self.public_job(job)
            if (
                job["status"] in TERMINAL
                or type(body.get("sequence")) is not int
                or body["sequence"] != job["last_sequence"] + 1
            ):
                raise LearningError(
                    409,
                    "report_sequence",
                    "Reports require a monotonic sequence; retries must be identical",
                )
            status = body.get("status", "running")
            if status not in {"running", "succeeded", "failed", "blocked", "cancelled"}:
                raise LearningError(422, "invalid_status", "Invalid worker transition")
            if job["status"] == "cancelling":
                status = "cancelled" if status in TERMINAL else "cancelling"
            progress = body.get("progress", job["progress"])
            if (
                not isinstance(progress, dict)
                or type(progress.get("completed")) is not int
                or progress["completed"] < 0
                or (
                    progress.get("total") is not None
                    and (
                        type(progress["total"]) is not int
                        or progress["total"] < progress["completed"]
                    )
                )
            ):
                raise LearningError(
                    422, "invalid_progress", "Progress must be real nonnegative counters"
                )
            job.update(
                status=status,
                stage=body.get("stage", job["stage"]),
                progress=progress,
                last_sequence=body["sequence"],
                last_report_hash=hashed,
                lease_expires_at=time.time() + 90,
            )
            for log in body.get("logs", []):
                job["logs"].append(
                    {
                        "sequence": len(job["logs"]) + 1,
                        "message": str(log.get("message", log) if isinstance(log, dict) else log)[
                            :16000
                        ],
                        "created_at": time.time(),
                    }
                )
            if body.get("error"):
                job["error"] = body["error"]
            if status in TERMINAL:
                result = body.get("result", {}) if status == "succeeded" else {}
                if (
                    not isinstance(result, dict)
                    or set(result) & {"id", "org_id", "owner_id", "job_id", "dataset_id", "run_id"}
                    or (job["kind"] != "media" and "status" in result)
                ):
                    raise LearningError(
                        422,
                        "invalid_result",
                        "Worker cannot replace resource identity or ownership",
                    )
                if status == "succeeded":
                    self.validate_result(job, result)
                else:
                    self.project_terminal(db, job)
                if status == "succeeded" and job["kind"] == "media":
                    self.store.put(
                        db,
                        "media",
                        f"{job['org_id']}:{job['payload']['recording_id']}",
                        result,
                    )
                resource_kind = {"dataset": "dataset", "train": "run"}.get(job["kind"])
                if status == "succeeded" and resource_kind:
                    resource = self.store.get(db, resource_kind, job[resource_kind + "_id"])
                    resource.update(status="ready", **result)
                    self.store.put(db, resource_kind, resource["id"], resource)
                    if job["kind"] == "train":
                        policy = {
                            **result,
                            "id": uid(),
                            "org_id": job["org_id"],
                            "owner_id": job["owner_id"],
                            "run_id": resource["id"],
                            "dataset_id": resource["dataset_id"],
                            "name": resource["name"],
                            "status": "ready",
                            "rollout_ready": False,
                        }
                        self.store.put(db, "policy", policy["id"], policy)
                        resource["policy_id"] = policy["id"]
                        self.store.put(db, resource_kind, resource["id"], resource)
                if status == "succeeded" and job["kind"] == "load-check":
                    checked_policy = self.store.get(db, "policy", job["payload"]["policy"]["id"])
                    if checked_policy is None:
                        raise LearningError(
                            409, "policy_missing", "Load check references a missing policy"
                        )
                    checked_policy["load_check"] = {
                        "status": status,
                        **result,
                        "error": job.get("error"),
                    }
                    self.store.put(db, "policy", checked_policy["id"], checked_policy)
            self.store.put(db, "job", key, job)
            return self.public_job(job)
