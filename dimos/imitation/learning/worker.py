"""Separately runnable, scoped Learning worker; executes only offline CPU jobs."""

from __future__ import annotations

import argparse
import json
import logging
import os
from pathlib import Path
import signal
import subprocess
import sys
import tarfile
import time
from typing import Any

import requests

from dimos.imitation.learning.annotation_adapter import preflight
from dimos.imitation.learning.artifacts import Artifacts, seal, verify
from dimos.imitation.learning.media import materialize, prepare, sha256
from dimos.imitation.learning.protocol import LearningError

logger = logging.getLogger(__name__)


def emit(message: str, **fields: Any) -> None:
    print(json.dumps({"message": message, **fields}), flush=True)


def native(request: dict[str, Any]) -> dict[str, Any]:
    project = Path(__file__).resolve().parents[3] / "native/python/lerobot"
    executable = os.environ.get("DIMOS_LEARNING_NATIVE_PYTHON", str(project / ".venv/bin/python"))
    if not Path(executable).is_file():
        raise LearningError(
            503,
            "native_unavailable",
            "Install the locked native LeRobot environment or set DIMOS_LEARNING_NATIVE_PYTHON",
        )
    env = os.environ.copy()
    env["CUDA_VISIBLE_DEVICES"] = ""
    env["HF_HUB_OFFLINE"] = "1"
    env["WANDB_MODE"] = "disabled"
    env["PYTHONPATH"] = os.pathsep.join(
        [
            str(project),
            str(project.parents[2]),
            env.get("DIMOS_LEARNING_NATIVE_EXTRA_PATH", ""),
            env.get("PYTHONPATH", ""),
        ]
    )
    process = subprocess.Popen(
        [executable, "-m", "dimos_lerobot.learning_train"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=None,
        text=True,
        env=env,
    )
    assert process.stdin is not None and process.stdout is not None
    process.stdin.write(json.dumps(request))
    process.stdin.close()
    result = None
    native_error = None
    for line in process.stdout:
        try:
            item = json.loads(line)
            if "result" in item:
                result = item["result"]
            elif "error" in item:
                native_error = item["error"]
            else:
                print(line.strip(), flush=True)
        except ValueError:
            emit(line.strip())
    returncode = process.wait()
    if native_error and native_error.get("code") == "empty_split":
        raise LearningError(422, "empty_split", native_error["message"])
    if returncode or result is None:
        raise LearningError(503, "native_failed", "Native LeRobot adapter failed; see worker logs")
    return result


def execute(job: dict[str, Any], state: Path) -> dict[str, Any]:
    payload = job["payload"]
    work = state / "work" / job["id"] / str(job["attempt"])
    work.mkdir(parents=True, exist_ok=False)
    if job["kind"] == "media":
        directory = materialize(payload["source"], state)
        result = prepare(directory, work / "media", emit)
        for camera in result["cameras"]:
            camera["path"] = str((work / "media" / f"{camera['id']}.mp4").relative_to(state))
        return result
    if job["kind"] == "dataset":
        snapshots = payload["snapshots"]
        preflight(snapshots)
        for snapshot in snapshots:
            directory = materialize(snapshot["source"], state)
            snapshot["directory"] = str(directory)
            snapshot["source_checksums"] = json.loads((directory / "complete.json").read_text())
            expected = snapshot["annotation"].get("source_checksums", {})
            if snapshot["source_checksums"] != expected or any(
                sha256(directory / name) != checksum for name, checksum in expected.items()
            ):
                raise LearningError(
                    409, "source_changed", "Frozen source checksum verification failed"
                )
        root = work / "dataset"
        info = native({"command": "convert", "snapshots": snapshots, "output": str(root)})
        lineage = {
            "dataset_id": payload["dataset_id"],
            "job_id": job["id"],
            "attempt": job["attempt"],
            "sources": [
                {
                    "recording_id": s["source"]["id"],
                    "annotation": s["annotation"],
                    "source_checksums": s["source_checksums"],
                }
                for s in snapshots
            ],
        }
        seal(root, lineage)
        publication = Artifacts().publish(root, job["org_id"], payload["dataset_id"])
        return {
            "path": str(root),
            "publication": publication,
            "episodes": info["episodes"],
            "frames": info["frames"],
            "features": {"observation": info["observation"], "action": info["action"]},
            "lineage": lineage,
            "ready": True,
        }
    if job["kind"] == "train":
        dataset = payload["dataset"]
        dataset_path = Path(dataset.get("path", ""))
        if dataset_path.is_dir():
            verify(dataset_path)
        else:
            dataset_path = Artifacts().restore(dataset["publication"], work / "dataset")
        root = work / "policy"
        result = native(
            {
                "command": "train",
                "dataset_path": str(dataset_path),
                "run": payload["run"],
                "output": str(root),
            }
        )
        seal(
            root,
            {
                "run": payload["run"],
                "dataset_id": dataset["id"],
                "dataset_manifest_sha256": dataset["publication"]["manifest_sha256"],
            },
            "policy",
        )
        publication = Artifacts().publish(root, job["org_id"], payload["run"]["id"])
        package = work / "policy.tar.gz"
        with tarfile.open(package, "w:gz") as archive:
            archive.add(root, arcname="policy")
        return {
            **result,
            "publication": publication,
            "package_path": str(package),
            "package_sha256": sha256(package),
            "checkpoint_path": str(root),
            "dataset_manifest_sha256": dataset["publication"]["manifest_sha256"],
        }
    if job["kind"] == "load-check":
        policy = payload["policy"]
        root = Path(policy.get("checkpoint_path", ""))
        if root.is_dir():
            verify(root)
        else:
            root = Artifacts().restore(policy["publication"], work / "policy")
        return native({"command": "load-check", "path": str(root)})
    raise LearningError(422, "unsupported_job", "Unknown worker job kind")


EXECUTOR_STOP_GRACE = 5.0


def _executor_group_running(pgid: int) -> bool:
    """Linux can distinguish live members from zombies awaiting their parent's reap."""
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    if not sys.platform.startswith("linux"):
        return True  # Other POSIX systems rely on normal orphan reaping.
    for entry in Path("/proc").iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            # comm can contain spaces/parentheses; fields after its final ')' are stable.
            fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            # Disappearing processes and other users' private /proc entries are not ours.
            continue
        if int(fields[2]) == pgid and fields[0] not in ("Z", "X"):
            return True
    return False


def _stop_executor_group(child: subprocess.Popen[Any]) -> None:
    """Bounded cleanup of the dedicated start_new_session executor group.

    Leader exit is NOT descendant exit. Reap the leader even on cleanup failure;
    orphan zombies belong to init, and cannot execute or publish.
    """
    pgid = child.pid
    if pgid <= 1 or pgid == os.getpgrp():
        raise RuntimeError("Refusing to signal the worker/shared process group")
    try:
        for sig in (signal.SIGTERM, signal.SIGKILL):
            if not _executor_group_running(pgid):
                break
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + EXECUTOR_STOP_GRACE
            while _executor_group_running(pgid):
                if not sys.platform.startswith("linux"):
                    child.poll()  # Reap our own zombie before checking POSIX group disappearance.
                if time.monotonic() >= deadline:
                    if sig == signal.SIGKILL:
                        raise TimeoutError("Executor group did not stop after SIGKILL")
                    break
                time.sleep(0.05)
    finally:
        child.wait(timeout=EXECUTOR_STOP_GRACE)


class Worker:
    def __init__(self, api: str, token: str, state: Path):
        self.api = api.rstrip("/")
        self.token = token
        self.state = state.resolve()
        self.state.mkdir(parents=True, exist_ok=True)

    def post(self, route: str, body: dict[str, Any]) -> dict[str, Any]:
        response = requests.post(
            self.api + "/v1/" + route,
            json=body,
            headers={"Authorization": "Bearer " + self.token},
            timeout=(10, 40),
        )
        response.raise_for_status()
        return response.json()

    def once(self) -> bool:
        self.post("worker/heartbeat", {"capabilities": ["media", "dataset", "train", "load-check"]})
        job = self.post("worker/claim", {"kinds": ["media", "dataset", "train", "load-check"]})[
            "job"
        ]
        if job is None:
            return False
        input_path = self.state / f"job-{job['id']}-{job['attempt']}.json"
        output_path = input_path.with_suffix(".result.json")
        log_path = input_path.with_suffix(".log")
        input_path.write_text(json.dumps(job))
        input_path.chmod(0o600)
        sequence = 0
        cancelled = False
        with log_path.open("w+") as log:
            child = subprocess.Popen(
                [
                    sys.executable,
                    "-m",
                    "dimos.imitation.learning.worker",
                    "--execute-file",
                    str(input_path),
                    "--result-file",
                    str(output_path),
                    "--state",
                    str(self.state),
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            last_heartbeat = 0.0
            last_report = 0.0
            position = 0
            progress = {"completed": 0, "total": None, "unit": "items"}
            try:
                while child.poll() is None:
                    if time.monotonic() - last_heartbeat >= 10:
                        heartbeat = self.post("worker/heartbeat", {"active_job_id": job["id"]})
                        last_heartbeat = time.monotonic()
                        if job["id"] in heartbeat["cancelled_job_ids"]:
                            cancelled = True
                            break
                    if time.monotonic() - last_report >= 2:
                        log.seek(position)
                        lines = log.readlines()
                        position = log.tell()
                        logs = []
                        for line in lines:
                            try:
                                item = json.loads(line)
                                logs.append({"message": item.get("message", line.strip())})
                                if "completed" in item:
                                    progress = {k: item[k] for k in ("completed", "total", "unit")}
                            except ValueError:
                                logs.append({"message": line.strip()})
                        sequence += 1
                        self.post(
                            f"worker/jobs/{job['id']}/report",
                            {
                                "lease_token": job["lease_token"],
                                "sequence": sequence,
                                "stage": "processing",
                                "status": "running",
                                "progress": progress,
                                "logs": logs,
                            },
                        )
                        last_report = time.monotonic()
                    time.sleep(0.1)
            finally:
                _stop_executor_group(child)
            log.seek(position)
            remaining = [{"message": line.strip()} for line in log.readlines()]
        result = (
            json.loads(output_path.read_text())
            if output_path.exists()
            else {
                "status": "failed",
                "error": {
                    "code": "worker_exit",
                    "message": f"Offline executor exited {child.returncode}; see persisted logs",
                },
            }
        )
        if cancelled:
            result = {"status": "cancelled"}
        sequence += 1
        self.post(
            f"worker/jobs/{job['id']}/report",
            {
                "lease_token": job["lease_token"],
                "sequence": sequence,
                "stage": result["status"],
                "progress": progress,
                "logs": remaining,
                **result,
            },
        )
        return True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--api", default=os.environ.get("DIMOS_LEARNING_API", "http://127.0.0.1:8768")
    )
    parser.add_argument("--pair")
    parser.add_argument("--name", default="local-cpu")
    parser.add_argument("--credentials", type=Path, default=Path(".learning-worker.json"))
    parser.add_argument(
        "--state",
        type=Path,
        default=Path(os.environ.get("DIMOS_LEARNING_STATE_DIR", ".learning-state")),
    )
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--execute-file", type=Path)
    parser.add_argument("--result-file", type=Path)
    args = parser.parse_args()
    if args.execute_file:
        try:
            result = {
                "status": "succeeded",
                "result": execute(json.loads(args.execute_file.read_text()), args.state.resolve()),
            }
        except LearningError as error:
            emit(error.message)
            result = {
                "status": "blocked" if error.status in (422, 503) else "failed",
                "error": {"code": error.code, "message": error.message},
            }
        except Exception as error:
            logger.exception("Offline Learning executor failed")
            result = {
                "status": "failed",
                "error": {"code": "execution_failed", "message": str(error)},
            }
        args.result_file.write_text(json.dumps(result))
        return
    if args.pair:
        if args.credentials.exists():
            raise SystemExit("Credentials already exist; do not overwrite a paired identity")
        response = requests.post(
            args.api.rstrip("/") + "/v1/workers/pair",
            json={
                "code": args.pair,
                "name": args.name,
                "capabilities": ["media", "dataset", "train", "load-check"],
            },
            timeout=30,
        )
        response.raise_for_status()
        fd = os.open(args.credentials, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w") as stream:
            json.dump(response.json(), stream)
    credentials = json.loads(args.credentials.read_text())
    worker = Worker(args.api, credentials["token"], args.state)
    while True:
        worked = worker.once()
        if args.once:
            break
        if not worked:
            time.sleep(2)


if __name__ == "__main__":
    main()
