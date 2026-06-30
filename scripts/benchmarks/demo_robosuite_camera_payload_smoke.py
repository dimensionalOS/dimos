#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Smoke-test Robosuite camera payload transmission without Rerun.

This verifies the suspect seam directly:

Robosuite observation image in sidecar -> `.npy` payload store -> HTTP fetch ->
client `np.load` decode.

The sidecar records source image hashes/statistics in the `ObservationFrame`
metadata before storing the payload. This script fetches the payload and verifies
the decoded array matches those metadata exactly.
"""

from __future__ import annotations

import argparse
import hashlib
from io import BytesIO
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
ROBOSUITE_SIDECAR_SRC = REPO_ROOT / "packages" / "dimos-robosuite-sidecar" / "src"

for package_src in (PROTOCOL_SRC, ROBOSUITE_SIDECAR_SRC):
    sys.path.insert(0, str(package_src))

from dimos_runtime_protocol import (
    EpisodeResetRequest,
    MotorActionFrame,
    ObservationKind,
    StepRequest,
)

from dimos.benchmark.runtime.artifacts import write_json
from dimos.benchmark.runtime.config import (
    BenchmarkEpisodeConfig,
    resolve_runtime_plan,
)
from dimos.simulation.runtime_client.http_client import RuntimeSidecarClient


def _load_config(path: Path) -> BenchmarkEpisodeConfig:
    return BenchmarkEpisodeConfig.model_validate_json(path.read_text())


def _sidecar_env() -> dict[str, str]:
    env = dict(os.environ)
    existing = env.get("PYTHONPATH", "")
    paths = [str(PROTOCOL_SRC), str(ROBOSUITE_SIDECAR_SRC)]
    if existing:
        paths.append(existing)
    env["PYTHONPATH"] = os.pathsep.join(paths)
    return env


def _start_sidecar(config: BenchmarkEpisodeConfig) -> subprocess.Popen[str]:
    command = [
        sys.executable,
        "-m",
        "dimos_robosuite_sidecar.server",
        "--host",
        config.runtime_host,
        "--port",
        str(config.runtime_port),
        "--env-name",
        config.env_name,
        "--robot-id",
        config.robot_id,
        "--robot-model",
        config.robot_model,
        "--controller",
        config.controller,
        "--control-freq",
        str(config.control_step_hz),
        "--horizon",
        str(config.horizon),
        "--camera-name",
        config.camera_name,
        "--seed",
        str(config.seed) if config.seed is not None else "0",
    ]
    return subprocess.Popen(
        command,
        cwd=REPO_ROOT,
        env=_sidecar_env(),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def _wait_healthy(sidecar: subprocess.Popen[str], client: RuntimeSidecarClient) -> object:
    deadline = time.monotonic() + 20.0
    last_error = ""
    while time.monotonic() < deadline:
        if sidecar.poll() is not None:
            raise RuntimeError("Robosuite sidecar exited before becoming healthy")
        try:
            return client.health()
        except Exception as exc:
            last_error = str(exc)
            time.sleep(0.1)
    raise RuntimeError(f"Robosuite sidecar did not become healthy: {last_error}")


def _array_sha256(array: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()


def _payload_sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _pixel_summary(array: np.ndarray) -> dict[str, object]:
    return {
        "shape": [int(item) for item in array.shape],
        "dtype": str(array.dtype),
        "min": float(array.min()) if array.size else 0.0,
        "max": float(array.max()) if array.size else 0.0,
        "mean": float(array.mean()) if array.size else 0.0,
        "top_left": [int(item) for item in array[0, 0].tolist()] if array.ndim == 3 else [],
        "center": [int(item) for item in array[array.shape[0] // 2, array.shape[1] // 2].tolist()]
        if array.ndim == 3
        else [],
        "bottom_left": [int(item) for item in array[-1, 0].tolist()] if array.ndim == 3 else [],
    }


def _write_jpeg(path: Path, rgb: np.ndarray) -> None:
    import cv2

    path.parent.mkdir(parents=True, exist_ok=True)
    if rgb.ndim != 3 or rgb.shape[2] < 3:
        raise ValueError(f"expected HxWx3 RGB image, got shape {rgb.shape}")
    bgr = cv2.cvtColor(rgb[:, :, :3], cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(str(path), bgr):
        raise RuntimeError(f"failed to write JPEG {path}")


def _validate_image_payload(
    client: RuntimeSidecarClient,
    frame: object,
    *,
    phase: str,
    image_dir: Path,
) -> dict[str, object]:
    data_ref = getattr(frame, "data_ref", None)
    if not isinstance(data_ref, str):
        raise AssertionError("image frame did not include data_ref")
    metadata = getattr(frame, "metadata", {})
    if not isinstance(metadata, dict):
        raise AssertionError("image frame metadata is not a dict")
    payload = client.payload(data_ref)
    payload_second_fetch = client.payload(data_ref)
    if payload != payload_second_fetch:
        raise AssertionError("re-fetching the same payload returned different bytes")
    decoded = np.load(BytesIO(payload), allow_pickle=False)
    decoded_second = np.load(BytesIO(payload_second_fetch), allow_pickle=False)
    if not np.array_equal(decoded, decoded_second):
        raise AssertionError("re-fetching the same payload decoded to different arrays")

    expected_shape = getattr(frame, "shape", None)
    expected_dtype = getattr(frame, "dtype", None)
    if expected_shape != [int(item) for item in decoded.shape]:
        raise AssertionError(f"shape mismatch: metadata={expected_shape} decoded={decoded.shape}")
    if expected_dtype != str(decoded.dtype):
        raise AssertionError(f"dtype mismatch: metadata={expected_dtype} decoded={decoded.dtype}")
    if metadata.get("array_sha256") != _array_sha256(decoded):
        raise AssertionError("decoded array hash does not match sidecar source hash")
    if metadata.get("payload_sha256") != _payload_sha256(payload):
        raise AssertionError("payload hash does not match sidecar source payload hash")

    image_basename = f"{phase}_{getattr(frame, 'stream', 'camera')}"
    raw_jpeg = image_dir / f"{image_basename}_raw.jpg"
    _write_jpeg(raw_jpeg, decoded)
    display_jpeg: Path | None = None
    if metadata.get("image_convention") == "opengl":
        display_jpeg = image_dir / f"{image_basename}_display_flipud.jpg"
        _write_jpeg(display_jpeg, np.flipud(decoded))

    return {
        "stream": getattr(frame, "stream", ""),
        "data_ref": data_ref,
        "encoding": getattr(frame, "encoding", ""),
        "same_payload_refetch_matches": True,
        "raw_jpeg": str(raw_jpeg),
        "display_jpeg": str(display_jpeg) if display_jpeg is not None else None,
        "metadata": metadata,
        "decoded": _pixel_summary(decoded),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPO_ROOT
        / "dimos"
        / "benchmark"
        / "runtime"
        / "configs"
        / "robosuite_panda_lift.json",
    )
    parser.add_argument("--ticks", type=int, default=3)
    parser.add_argument("--camera-name", default=None)
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        default=REPO_ROOT / "artifacts" / "benchmark" / "robosuite-camera-payload-smoke",
    )
    args = parser.parse_args()

    config = _load_config(args.config)
    updates: dict[str, object] = {
        "ticks": args.ticks,
        "horizon": max(config.horizon, args.ticks + 1),
    }
    if args.camera_name is not None:
        updates["camera_name"] = args.camera_name
    config = BenchmarkEpisodeConfig.model_validate({**config.model_dump(), **updates})

    sidecar = _start_sidecar(config)
    client = RuntimeSidecarClient(f"http://{config.runtime_host}:{config.runtime_port}")
    sidecar_output = ""
    checks: list[dict[str, object]] = []
    image_dir = args.artifact_dir / "images"
    try:
        health = _wait_healthy(sidecar, client)
        description = client.describe()
        plan = resolve_runtime_plan(config, description)
        reset = client.reset(
            EpisodeResetRequest(episode_id=plan.episode_id, task_id=plan.task_id, seed=config.seed)
        )

        for frame in reset.observations:
            if getattr(frame, "kind", None) == ObservationKind.IMAGE:
                phase = "reset"
                checks.append(
                    {
                        "phase": phase,
                        **_validate_image_payload(client, frame, phase=phase, image_dir=image_dir),
                    }
                )

        zero_action = MotorActionFrame(
            robot_id=plan.robot_id, names=plan.motor_names, q=[0.0] * len(plan.motor_names)
        )
        for tick in range(plan.ticks):
            response = client.step(
                StepRequest(episode_id=plan.episode_id, tick_id=tick, action=zero_action)
            )
            for frame in response.observations:
                if getattr(frame, "kind", None) == ObservationKind.IMAGE:
                    phase = f"step_{tick}"
                    checks.append(
                        {
                            "phase": phase,
                            **_validate_image_payload(
                                client, frame, phase=phase, image_dir=image_dir
                            ),
                        }
                    )

        summary = {
            "ok": True,
            "camera_name": config.camera_name,
            "ticks": plan.ticks,
            "checked_payloads": len(checks),
            "image_dir": str(image_dir),
            "checks": checks,
            "health": getattr(health, "model_dump", lambda **_: health)(mode="json"),
            "runtime_description": description.model_dump(mode="json"),
        }
        write_json(args.artifact_dir / "camera_payload_smoke_summary.json", summary)
        print(
            json.dumps(
                {
                    "ok": True,
                    "artifact_dir": str(args.artifact_dir),
                    "checked_payloads": len(checks),
                },
                indent=2,
            )
        )
    finally:
        sidecar.terminate()
        try:
            sidecar_output, _ = sidecar.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            sidecar.kill()
            sidecar_output, _ = sidecar.communicate(timeout=2.0)
        args.artifact_dir.mkdir(parents=True, exist_ok=True)
        (args.artifact_dir / "robosuite_sidecar.log").write_text(sidecar_output)


if __name__ == "__main__":
    main()
