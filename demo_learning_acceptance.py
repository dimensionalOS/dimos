"""Opt-in real MCAP acceptance; creates isolated outputs, never alters sources."""

from pathlib import Path
import json
import os
import subprocess
import sys
import time
import socket
import jwt
import requests
from dimos.imitation.learning.worker import Worker

ROOT = Path(__file__).resolve().parent
STATE = ROOT / ".learning-evidence" / ("three-camera-" + __import__("uuid").uuid4().hex[:8])
STATE.mkdir(parents=True, exist_ok=True)
secret = "isolated-local-acceptance-only-1234567890123456789"
os.environ["DIMOS_LEARNING_STATE_DIR"] = str(STATE)
os.environ["DIMOS_LEARNING_DELEGATION_SECRET"] = secret
os.environ["DIMOS_LEARNING_SOURCE_ROOTS"] = (
    "/home/cc/work/dimos-bimanual-annotation-artifacts/session_v1"
)
source = Path(os.environ["DIMOS_LEARNING_SOURCE_ROOTS"])
descriptor = {
    "id": "three-camera",
    "org_id": "test-org",
    "owner_id": "test-user",
    "can_read": True,
    "can_write": True,
    "complete": True,
    "verified": True,
    "quarantined": False,
    "filename": "recording.mcap",
    "download_url": (source / "recording.mcap").as_uri(),
    "schema_url": (source / "schema.json").as_uri(),
    "manifest_url": (source / "manifest.json").as_uri(),
    "metadata": {},
}
with socket.socket() as sock:
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
api = f"http://127.0.0.1:{port}"
subprocess.run([sys.executable, "-m", "dimos.imitation.learning.store"], check=True)
log = (STATE / "api.log").open("w")
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


def call(method, path, body=None):
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
            "sources": [descriptor],
        },
        secret,
        algorithm="HS256",
    )
    response = requests.request(
        method, api + path, json=body, headers={"Authorization": "Bearer " + token}, timeout=120
    )
    response.raise_for_status()
    return response.json()


try:
    for _ in range(100):
        try:
            if requests.get(api + "/health", timeout=1).ok:
                break
        except requests.RequestException:
            pass
        time.sleep(0.1)
    recording = call("GET", "/v1/recordings/three-camera")
    annotation = call("GET", "/v1/recordings/three-camera/annotations")
    annotation["episodes"][0]["notes"] = "real source durable acceptance"
    saved = call(
        "PUT",
        "/v1/recordings/three-camera/annotations",
        {"expected_revision": annotation["revision"], "episodes": annotation["episodes"]},
    )
    assert call("GET", "/v1/recordings/three-camera/annotations") == saved
    job = call("POST", "/v1/recordings/three-camera/media")
    pair = call("POST", "/v1/workers/pairing", {"org_id": "test-org", "name": "real-cpu"})
    response = requests.post(
        api + "/v1/workers/pair",
        json={"code": pair["code"], "name": "real-cpu", "capabilities": ["media", "dataset"]},
        timeout=10,
    )
    response.raise_for_status()
    worker = Worker(api, response.json()["token"], STATE)
    worker.once()
    completed = call("GET", "/v1/jobs/" + job["id"])
    assert completed["status"] == "succeeded", completed
    media = call("GET", "/v1/recordings/three-camera/media")
    probes = []
    for camera in media["cameras"]:
        response = requests.get(
            api + f"/v1/recordings/three-camera/media/{camera['id']}",
            headers={
                "Authorization": "Bearer "
                + jwt.encode(
                    {
                        "iss": "dimos-cloud",
                        "aud": "dimos-learning",
                        "sub": "test-user",
                        "org_id": "test-org",
                        "role": "admin",
                        "platform_admin": False,
                        "iat": int(time.time()),
                        "exp": int(time.time()) + 45,
                        "method": "GET",
                        "path": f"/v1/recordings/three-camera/media/{camera['id']}",
                        "sources": [descriptor],
                    },
                    secret,
                    algorithm="HS256",
                ),
                "Range": "bytes=0-1023",
            },
            timeout=20,
        )
        assert response.status_code == 206, (response.status_code, response.text)
        assert len(response.content) == 1024
        file = STATE / camera["path"]
        probe = subprocess.check_output(
            ["ffprobe", "-v", "error", "-show_streams", "-show_format", "-of", "json", str(file)],
            text=True,
        )
        probes.append(
            {"camera": camera, "ffprobe": json.loads(probe), "range_status": response.status_code}
        )
    dataset = call(
        "POST",
        "/v1/datasets",
        {
            "org_id": "test-org",
            "name": "must block missing command",
            "profile_id": recording["profile_id"],
            "sources": [{"recording_id": "three-camera", "revision": saved["revision"]}],
        },
    )
    worker.once()
    blocked = call("GET", "/v1/jobs/" + dataset["job_id"])
    assert (
        blocked["status"] == "blocked" and blocked["error"]["code"] == "gripper_command_missing"
    ), blocked
    evidence = {
        "recording": recording,
        "saved_revision": saved["revision"],
        "media_job": completed,
        "probes": probes,
        "blocked_dataset": blocked,
    }
    (STATE / "acceptance.json").write_text(json.dumps(evidence, indent=2))
    print(
        json.dumps(
            {
                "evidence": str(STATE / "acceptance.json"),
                "cameras": [
                    {k: p["camera"][k] for k in ("id", "frames", "sha256", "offset_s")}
                    for p in probes
                ],
                "blocked": blocked["error"],
            },
            indent=2,
        )
    )
finally:
    server.terminate()
    server.wait(timeout=10)
    log.close()
