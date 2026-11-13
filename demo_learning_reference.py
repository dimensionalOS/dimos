"""Bounded positive reference conversion / native ACT / emulator S3 acceptance."""

from pathlib import Path
import json
import os
import socket
import subprocess
import sys
import time
import uuid
import boto3
import jwt
from moto.server import ThreadedMotoServer
import requests
from dimos.imitation.learning.media import sha256
from dimos.imitation.learning.worker import Worker

root = Path(__file__).resolve().parent
state = root / ".learning-evidence" / ("reference-" + uuid.uuid4().hex[:8])
state.mkdir(parents=True)
secret = "reference-isolated-only-123456789012345678901234567890"
sources_root = Path("/home/cc/hermes/dimos-annotation-data/reference")
os.environ.update(
    DIMOS_LEARNING_STATE_DIR=str(state),
    DIMOS_LEARNING_DELEGATION_SECRET=secret,
    DIMOS_LEARNING_SOURCE_ROOTS=str(sources_root),
    AWS_ACCESS_KEY_ID="local-test",
    AWS_SECRET_ACCESS_KEY="local-test",
    AWS_DEFAULT_REGION="us-east-1",
    DIMOS_LEARNING_S3_BUCKET="learning-acceptance",
)
emulator = ThreadedMotoServer(ip_address="127.0.0.1", port=0, verbose=False)
emulator.start()
host, port = emulator.get_host_and_port()
os.environ["DIMOS_LEARNING_S3_ENDPOINT"] = f"http://{host}:{port}"
boto3.client("s3", endpoint_url=os.environ["DIMOS_LEARNING_S3_ENDPOINT"]).create_bucket(
    Bucket="learning-acceptance"
)
sources = []
hashes = {}
for name in ("session_19ep_v1", "session_fix_v1"):
    path = sources_root / name
    hashes[name] = {f: sha256(path / f) for f in ("recording.mcap", "schema.json")}
    sources.append(
        {
            "id": name,
            "org_id": "test-org",
            "owner_id": "test-user",
            "can_read": True,
            "can_write": True,
            "complete": True,
            "verified": True,
            "quarantined": False,
            "filename": "recording.mcap",
            "download_url": (path / "recording.mcap").as_uri(),
            "schema_url": (path / "schema.json").as_uri(),
            "metadata": {},
        }
    )
with socket.socket() as sock:
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
api = f"http://127.0.0.1:{port}"
subprocess.run([sys.executable, "-m", "dimos.imitation.learning.store"], check=True)
log = (state / "api.log").open("w")
server = subprocess.Popen(
    [
        sys.executable,
        "-m",
        "uvicorn",
        "dimos.imitation.learning.app:app",
        "--host",
        "127.0.0.1",
        "--port",
        str(port),
    ],
    stdout=log,
    stderr=subprocess.STDOUT,
)


def call(method, path, body=None, raw=False):
    now = int(time.time())
    token = jwt.encode(
        {
            "iss": "dimos-cloud",
            "aud": "dimos-learning",
            "sub": "test-user",
            "org_id": "test-org",
            "role": "admin",
            "platform_admin": False,
            "iat": now,
            "exp": now + 45,
            "method": method,
            "path": path,
            "sources": sources,
        },
        secret,
        algorithm="HS256",
    )
    response = requests.request(
        method, api + path, json=body, headers={"Authorization": "Bearer " + token}, timeout=120
    )
    response.raise_for_status()
    return response if raw else response.json()


try:
    for _ in range(100):
        try:
            if requests.get(api + "/health", timeout=1).ok:
                break
        except requests.RequestException:
            pass
        time.sleep(0.1)
    selections = []
    for descriptor in sources:
        key = descriptor["id"]
        annotations = call("GET", f"/v1/recordings/{key}/annotations")
        selected = False
        for ep in annotations["episodes"]:
            ep["included"] = not selected and ep["included"]
            if ep["included"]:
                ep["end_s"] = min(ep["end_s"], ep["start_s"] + 1.2)
                ep["review_status"] = "approved"
                selected = True
        saved = call(
            "PUT",
            f"/v1/recordings/{key}/annotations",
            {"expected_revision": 0, "episodes": annotations["episodes"]},
        )
        selections.append({"recording_id": key, "revision": saved["revision"]})
    dataset = call(
        "POST",
        "/v1/datasets",
        {
            "org_id": "test-org",
            "name": "two real single-arm references",
            "profile_id": "xarm7-sim-pickplace",
            "sources": selections,
        },
    )
    pairing = call("POST", "/v1/workers/pairing", {"org_id": "test-org", "name": "reference-cpu"})
    response = requests.post(
        api + "/v1/workers/pair",
        json={
            "code": pairing["code"],
            "name": "reference-cpu",
            "capabilities": ["dataset", "train", "load-check"],
        },
        timeout=20,
    )
    response.raise_for_status()
    worker = Worker(api, response.json()["token"], state)
    worker.once()
    dataset_job = call("GET", "/v1/jobs/" + dataset["job_id"])
    (state / "dataset-job.json").write_text(json.dumps(dataset_job, indent=2))
    assert dataset_job["status"] == "succeeded", dataset_job
    dataset = call("GET", "/v1/datasets/" + dataset["id"])
    run = call(
        "POST",
        "/v1/runs",
        {
            "org_id": "test-org",
            "dataset_id": dataset["id"],
            "name": "bounded CPU ACT one step",
            "preset": "act",
            "eval_split": 0,
            "steps": 1,
        },
    )
    worker.once()
    train_job = call("GET", "/v1/jobs/" + run["job_id"])
    (state / "train-job.json").write_text(json.dumps(train_job, indent=2))
    assert train_job["status"] == "succeeded", train_job
    run = call("GET", "/v1/runs/" + run["id"])
    policy = call("GET", "/v1/policies/" + run["policy_id"])
    package = call("GET", "/v1/policies/" + policy["id"] + "/download", raw=True)
    download = state / "downloaded-policy.tar.gz"
    download.write_bytes(package.content)
    assert sha256(download) == policy["package_sha256"]
    load_job = call("POST", "/v1/policies/" + policy["id"] + "/load-check")
    worker.once()
    load_result = call("GET", "/v1/jobs/" + load_job["id"])
    assert load_result["status"] == "succeeded", load_result
    for name, files in hashes.items():
        assert all(sha256(sources_root / name / f) == expected for f, expected in files.items())
    evidence = {
        "source_hashes": hashes,
        "dataset": dataset,
        "dataset_job": dataset_job,
        "run": run,
        "train_job": train_job,
        "policy": policy,
        "load_check": load_result,
        "s3_emulator": os.environ["DIMOS_LEARNING_S3_ENDPOINT"],
        "download_sha256": sha256(download),
        "scope": "Two single-arm references; NOT three-camera training acceptance",
    }
    (state / "acceptance.json").write_text(json.dumps(evidence, indent=2))
    print(
        json.dumps(
            {
                "evidence": str(state / "acceptance.json"),
                "dataset_episodes": dataset["episodes"],
                "dataset_frames": dataset["frames"],
                "run_steps": run["steps"],
                "losses": run["losses"],
                "package_sha256": policy["package_sha256"],
                "load_check": load_result["status"],
            },
            indent=2,
        )
    )
finally:
    server.terminate()
    server.wait(timeout=10)
    log.close()
    emulator.stop()
    print("Evidence directory:", state)
