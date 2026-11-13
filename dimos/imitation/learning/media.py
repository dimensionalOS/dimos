"""Read-only MCAP source materialization and timestamp-preserving camera media."""

from __future__ import annotations

from collections.abc import Callable
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
from typing import Any
from urllib.parse import unquote, urlparse

from mcap.reader import make_reader
import requests

from dimos.imitation.learning.protocol import LearningError


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def fetch(url: str, target: Path) -> None:
    parsed = urlparse(url)
    if parsed.scheme == "file":
        path = Path(unquote(parsed.path)).resolve()
        roots = [
            Path(x).resolve()
            for x in os.environ.get("DIMOS_LEARNING_SOURCE_ROOTS", "").split(os.pathsep)
            if x
        ]
        if not any(path.is_relative_to(root) for root in roots) or not path.is_file():
            raise LearningError(
                403, "source_path_denied", "Local fixture is not explicitly authorized"
            )
        shutil.copyfile(path, target)
    elif parsed.scheme in ("http", "https"):
        hosts = os.environ.get("DIMOS_LEARNING_SOURCE_HOSTS", "").split(",")
        if parsed.netloc not in hosts:
            raise LearningError(403, "source_host_denied", "Source host is not configured")
        with requests.get(url, stream=True, timeout=(10, 120), allow_redirects=False) as response:
            response.raise_for_status()
            with target.open("wb") as stream:
                for block in response.iter_content(1024 * 1024):
                    stream.write(block)
    else:
        raise LearningError(422, "source_url_invalid", "Authorized HTTP download URL required")


def materialize(descriptor: dict[str, Any], state: Path) -> Path:
    # Freeze stable content identity, not an expiring presigned download URL.
    expected = descriptor.get("source_sha256") or descriptor.get("metadata", {}).get("sha256")
    if expected is not None and (
        not isinstance(expected, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", expected)
    ):
        raise LearningError(
            422, "source_checksum_invalid", "Source SHA256 must be a declared hexadecimal digest"
        )
    if not expected and urlparse(descriptor.get("download_url", "")).scheme != "file":
        raise LearningError(422, "source_checksum_missing", "Platform source SHA256 is required")
    identity = {
        k: descriptor.get(k)
        for k in ("id", "org_id", "schema", "manifest", "schema_url", "manifest_url")
    }
    identity["source_sha256"] = expected
    if not expected:
        identity["download_url"] = descriptor.get("download_url")
    key = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    destination = state / "sources" / key
    if (destination / "complete.json").exists():
        return destination
    schema = descriptor.get("schema")
    if schema is not None and (
        not isinstance(schema, dict) or schema.get("payload", "recording.mcap") != "recording.mcap"
    ):
        raise LearningError(
            422,
            "source_format_unsupported",
            "Learning media currently requires an MCAP RecordingSchema",
        )
    encoding = descriptor.get("metadata", {}).get("content_encoding")
    if encoding:
        raise LearningError(
            422,
            "source_encoding_unsupported",
            "Supply an uncompressed MCAP source; compressed database recordings are not MCAP",
        )
    staging = destination.with_name(key + ".partial-" + os.urandom(8).hex())
    staging.mkdir(parents=True)
    try:
        url = descriptor.get("download_url")
        if not url:
            raise LearningError(422, "source_url_invalid", "Authorized recording URL required")
        fetch(url, staging / "recording.mcap")
        actual = sha256(staging / "recording.mcap")
        if expected and actual != expected.lower():
            raise LearningError(
                409,
                "source_checksum_mismatch",
                "Downloaded recording differs from upload-declared SHA256",
            )
        if schema is not None:
            (staging / "schema.json").write_text(json.dumps(schema, allow_nan=False))
        elif descriptor.get("schema_url"):
            fetch(descriptor["schema_url"], staging / "schema.json")
        else:
            raise LearningError(
                422, "source_schema_missing", "An authorized companion RecordingSchema is required"
            )
        if descriptor.get("manifest") is not None:
            manifest = descriptor["manifest"]
            if not isinstance(manifest, dict):
                raise LearningError(
                    422, "source_manifest_invalid", "Source manifest must be an object"
                )
            # The inline schema is already frozen separately; retain capture metadata unchanged.
            (staging / "manifest.json").write_text(json.dumps(manifest, allow_nan=False))
        elif descriptor.get("manifest_url"):
            fetch(descriptor["manifest_url"], staging / "manifest.json")
        hashes = {p.name: sha256(p) for p in staging.iterdir() if p.is_file()}
        (staging / "complete.json").write_text(json.dumps(hashes))
        destination.parent.mkdir(parents=True, exist_ok=True)
        try:
            staging.rename(destination)
        except OSError:
            if not (destination / "complete.json").exists():
                raise
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    return destination


def capture(directory: Path) -> dict[str, Any]:
    schema = json.loads((directory / "schema.json").read_text())
    camera_defs = schema.get("metadata", {}).get("cameras", {})
    cameras = [
        {"id": key, "label": key.replace("_", " ").title(), "topic": value["topic"]}
        for key, value in camera_defs.items()
    ]
    if not cameras:
        cameras = [
            {"id": "head" if key == "camera" else key, "label": key, "topic": value["stream"]}
            for key, value in schema.get("observation", {}).items()
            if value.get("dtype") == "video"
        ]
    if len({camera["id"] for camera in cameras}) != len(cameras) or any(
        not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", camera["id"]) for camera in cameras
    ):
        raise LearningError(422, "camera_id_invalid", "Unique safe camera identifiers required")
    episodes: list[dict[str, Any]] = []
    start = None
    origin = None
    end = None
    with (directory / "recording.mcap").open("rb") as stream:
        for _, channel, message in make_reader(stream).iter_messages():
            stamp = message.log_time / 1e9
            origin = stamp if origin is None else min(origin, stamp)
            end = stamp if end is None else max(end, stamp)
            if channel.topic == schema.get("episodes", {}).get("status_stream", "status"):
                status = json.loads(message.data)
                if status.get("last_event") == "start":
                    start = status
                elif status.get("last_event") in ("save", "discard") and start is not None:
                    # Identity binds source start time, not list position.
                    episodes.append(
                        {
                            "id": f"ep-{round(start['ts'] * 1e9)}",
                            "start_ts": start["ts"],
                            "end_ts": status["ts"],
                            "instruction": start.get("task_label"),
                            "capture_event": status["last_event"],
                            "outcome": "unknown" if status["last_event"] == "save" else "discarded",
                        }
                    )
                    start = None
    if origin is None or end is None:
        raise LearningError(422, "empty_recording", "Recording contains no messages")
    manifest_path = directory / "manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    ground_truth = {}
    for episode in episodes:
        segment = next(
            (
                item
                for item in manifest.get("segments", [])
                if abs(item.get("startStatusTs", -1) - episode["start_ts"]) < 1e-6
            ),
            None,
        )
        if segment:
            episode["outcome"] = segment.get("outcome", "unknown")
            ground_truth[episode["id"]] = segment
        episode["start_s"] = episode.pop("start_ts") - origin
        episode["end_s"] = episode.pop("end_ts") - origin
    semantics = schema.get("metadata", {}).get("gripperSemantics")
    truth = {
        "gripper_semantics": semantics,
        "gripperCommandRecorded": (semantics or {}).get("gripperCommandRecorded"),
        "source_kind": "simulator"
        if "sim" in schema.get("name", "") or schema.get("robot_type") == "dual_xarm7"
        else None,
        "episodes": ground_truth,
    }
    return {
        "profile_id": schema.get("name"),
        "duration_s": end - origin,
        "origin_s": origin,
        "cameras": cameras,
        "episodes": episodes,
        "capture_truth": truth,
        "schema": schema,
    }


def prepare(directory: Path, destination: Path, report: Callable[[str], None]) -> dict[str, Any]:
    info = capture(directory)
    destination.mkdir(parents=True, exist_ok=True)
    frames: dict[str, list[tuple[float, Path]]] = {
        camera["topic"]: [] for camera in info["cameras"]
    }
    with (directory / "recording.mcap").open("rb") as stream:
        for _, channel, message in make_reader(stream).iter_messages():
            if channel.topic not in frames:
                continue
            # Native lcm_jpeg envelope embeds a complete JPEG; validate markers.
            begin, end = message.data.find(b"\xff\xd8"), message.data.rfind(b"\xff\xd9")
            if begin < 0 or end < begin:
                raise LearningError(
                    422, "unsupported_camera_encoding", f"{channel.topic}: JPEG payload required"
                )
            bucket = frames[channel.topic]
            folder = destination / (
                "frames-" + hashlib.sha256(channel.topic.encode()).hexdigest()[:16]
            )
            folder.mkdir(exist_ok=True)
            path = folder / f"{len(bucket):07d}.jpg"
            path.write_bytes(message.data[begin : end + 2])
            bucket.append((message.log_time / 1e9, path))
    for camera in info["cameras"]:
        bucket = frames[camera["topic"]]
        if len(bucket) < 2:
            raise LearningError(422, "camera_empty", f"No usable frames: {camera['id']}")
        listing = destination / f"{camera['id']}.concat"
        lines = []
        for index, (stamp, path) in enumerate(bucket):
            duration = (
                bucket[index + 1][0] - stamp
                if index + 1 < len(bucket)
                else stamp - bucket[index - 1][0]
            )
            if duration <= 0:
                raise LearningError(
                    422, "camera_timestamp", "Camera timestamps must be strictly increasing"
                )
            lines.extend([f"file '{path.resolve()}'", f"duration {duration:.9f}"])
        lines.append(f"file '{bucket[-1][1].resolve()}'")
        listing.write_text("\n".join(lines) + "\n")
        target = destination / f"{camera['id']}.mp4"
        subprocess.run(
            [
                "ffmpeg",
                "-nostdin",
                "-v",
                "error",
                "-f",
                "concat",
                "-safe",
                "0",
                "-i",
                str(listing),
                "-fps_mode",
                "vfr",
                "-c:v",
                "libx264",
                "-preset",
                "veryfast",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                "-y",
                str(target),
            ],
            check=True,
            timeout=180,
        )
        camera.update(
            offset_s=bucket[0][0] - info["origin_s"], frames=len(bucket), sha256=sha256(target)
        )
        report(f"Decoded {camera['id']}: {len(bucket)} source frames")
        shutil.rmtree(bucket[0][1].parent)
        listing.unlink()
    result = {"status": "ready", "cameras": info["cameras"], "duration_s": info["duration_s"]}
    (destination / "media.json").write_text(json.dumps(result))
    return result
