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

"""Development sensor-frame evidence, not a sandbox or task-truth observation."""

from collections.abc import Mapping, Sequence
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import re
import shlex
from typing import Any
from uuid import uuid4

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.tf2_msgs.TFMessage import TFMessage

CAMERA_STREAMS = {
    "head": ("color_image", "depth_image", "camera_info", "camera_optical"),
    "left_wrist": (
        "left_wrist_image",
        "left_wrist_depth",
        "left_wrist_camera_info",
        "left_wrist_optical",
    ),
}


def make_sensor_snapshot(messages: Mapping[str, Any], episode: str, step: int) -> dict[str, Any]:
    """Tag one native capture; allow only RGB/depth/K and camera-to-base TF.

    Called on the simulator owner thread for one messages() result, never by
    independently polling streams or querying task truth from a policy.
    """
    keys = {key for streams in CAMERA_STREAMS.values() for key in streams[:3]}
    sensors = {key: copy.deepcopy(messages[key]) for key in keys}
    sensors["tf"] = TFMessage(
        *[
            copy.deepcopy(t)
            for t in messages["tf"].transforms
            if t.frame_id == "base_link"
            and t.child_frame_id in {s[3] for s in CAMERA_STREAMS.values()}
        ]
    )
    return {
        "capture_id": uuid4().hex,
        "episode": episode,
        "step": step,
        "captured_at": messages["color_image"].ts,
        "sensors": sensors,
    }


def observation_metadata(observation: Mapping[str, Any]) -> dict[str, Any]:
    transform = observation["camera_to_base"]
    calibration = observation["calibration"]
    return {
        "observation_id": observation["id"],
        "camera": observation["camera"],
        "capture": observation["capture"],
        "rgb": {
            "ts": observation["rgb"].ts,
            "frame": observation["rgb"].frame_id,
            "format": observation["rgb"].format.name,
            "dtype": str(observation["rgb"].data.dtype),
            "shape": list(observation["rgb"].data.shape),
        },
        "depth": {
            "ts": observation["depth"].ts,
            "frame": observation["depth"].frame_id,
            "format": observation["depth"].format.name,
            "semantics": "optical_Z_m",
            "dtype": str(observation["depth"].data.dtype),
            "shape": list(observation["depth"].data.shape),
        },
        "intrinsics": {
            **calibration.to_dict(),
            "ts": calibration.ts,
            "frame": calibration.frame_id,
            "width": calibration.width,
            "height": calibration.height,
            "K": list(calibration.K),
        },
        "camera_to_base": {
            "ts": transform.ts,
            "frame": transform.frame_id,
            "child_frame": transform.child_frame_id,
            "translation": list(transform.translation.to_tuple()),
            "orientation": list(transform.rotation.to_tuple()),
        },
    }


def observation_fingerprint(observation: Mapping[str, Any]) -> str:
    digest = hashlib.sha256(
        json.dumps(
            observation_metadata(observation),
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode()
    )
    for key in ("rgb", "depth"):
        digest.update(np.ascontiguousarray(observation[key].data).tobytes())
    return digest.hexdigest()


def project_base_point(observation: Mapping[str, Any], position: Sequence[float]) -> list[float]:
    """Independent projection using the retained exact K and camera TF."""
    transform = observation["camera_to_base"]
    point = (
        Rotation.from_quat(transform.rotation.to_tuple())
        .inv()
        .apply(np.asarray(position) - np.asarray(transform.translation.to_tuple()))
    )
    if not np.isfinite(point).all() or point[2] <= 0:
        raise ValueError("Point must be finite and in front of the camera")
    pixel = np.asarray(observation["calibration"].K).reshape(3, 3) @ point
    return [float(v) for v in pixel[:2] / pixel[2]]


def persist_observation(directory: Path, observation: Mapping[str, Any]) -> Path:
    """Immutable, pickle-free evaluator bundle of allowed sensor data only."""
    capture = observation["capture"]
    if capture["capture_id"] is None or capture["episode"] is None or capture["step"] is None:
        raise ValueError("Evidence requires an atomic native capture with episode/step")
    metadata = observation_metadata(observation)
    fingerprint = observation_fingerprint(observation)
    if observation["fingerprint"] != fingerprint:
        raise ValueError("Observation content no longer matches its fingerprint")
    metadata["fingerprint"] = fingerprint
    directory.mkdir(parents=True, exist_ok=True)
    # Never interpret an agent-supplied observation ID as a path.
    stem = hashlib.sha256(observation["id"].encode()).hexdigest()
    path = directory / f"{stem}.npz"
    if path.exists():
        with np.load(path, allow_pickle=False) as existing:
            previous = json.loads(existing["metadata"].tobytes())
            matches = all(
                existing[key].dtype == observation[key].data.dtype
                and existing[key].shape == observation[key].data.shape
                and np.ascontiguousarray(existing[key]).tobytes()
                == np.ascontiguousarray(observation[key].data).tobytes()
                for key in ("rgb", "depth")
            )
        if previous != metadata or not matches:
            raise ValueError("Existing evidence conflicts with the observation ID")
        return path
    buffer = io.BytesIO()
    np.savez_compressed(
        buffer,
        rgb=observation["rgb"].data,
        depth=observation["depth"].data,
        metadata=np.frombuffer(json.dumps(metadata, allow_nan=False).encode(), dtype=np.uint8),
    )
    with path.open("xb") as stream:
        stream.write(buffer.getvalue())
    return path


def persist_grounding(
    directory: Path, observation: Mapping[str, Any], result: Mapping[str, Any]
) -> Path:
    bundle = persist_observation(directory / "observations", observation)
    if (
        result["observation_id"] != observation["id"]
        or result["fingerprint"] != observation["fingerprint"]
        or result["capture"] != observation["capture"]
        or result["frame"] != "base_link"
    ):
        raise ValueError("Grounding result does not match the retained sensor snapshot")
    pixel = result["pixel"]
    height, width = observation["depth"].data.shape
    if (
        len(pixel) != 2
        or any(type(v) is not int for v in pixel)
        or not 1 <= pixel[0] < width - 1
        or not 1 <= pixel[1] < height - 1
    ):
        raise ValueError("Grounding result has an invalid pixel request")
    projected = project_base_point(observation, result["position"])
    transform = observation["camera_to_base"]
    optical = (
        Rotation.from_quat(transform.rotation.to_tuple())
        .inv()
        .apply(np.asarray(result["position"]) - transform.translation.to_tuple())
    )
    u, v = pixel
    window = observation["depth"].data[v - 1 : v + 2, u - 1 : u + 2]
    valid = window[np.isfinite(window) & (window > 0)]
    if (
        valid.size < 5
        or not np.allclose(projected, pixel, atol=1e-6, rtol=0)
        or not math.isclose(float(optical[2]), result["depth"], abs_tol=1e-8)
        or not math.isclose(float(np.median(valid)), result["depth"], abs_tol=1e-8)
    ):
        raise ValueError("Grounding point/depth does not match the selected pixel")
    record = {"result": result, "sensor_bundle": bundle.name, "projection_check_pixel": projected}
    path = directory / f"grounding-{uuid4().hex}.json"
    with path.open("x") as stream:
        json.dump(record, stream, allow_nan=False, indent=2)
    return path


def reconcile_grounding_claim(
    events: Sequence[Mapping[str, Any]], claimed_executed: bool
) -> dict[str, Any]:
    """Compare an explicit summary claim with successful tool/result records.

    This checks recorded commands and feedback, not arbitrary Python side effects
    or semantic natural-language truth. The caller supplies the interpreted claim.
    """
    starts = {e.get("toolCallId"): e for e in events if e.get("type") == "tool_execution_start"}
    completed = []
    for event in events:
        if event.get("type") != "tool_execution_end" or event.get("isError") is not False:
            continue
        start = starts.get(event.get("toolCallId"))
        if start is None or start.get("toolName") != "bash":
            continue
        try:
            command = shlex.split(start["args"]["command"])
        except (KeyError, ValueError):
            continue
        if (
            len(command) < 2
            or command[-1] != "ground.py"
            or re.fullmatch(r"python(?:[23](?:\.\d+)?)?", Path(command[-2]).name) is None
        ):
            continue
        for block in event.get("result", {}).get("content", []):
            if block.get("type") != "text":
                continue
            for line in block.get("text", "").splitlines():
                try:
                    value = json.loads(line)
                    if (
                        isinstance(value, dict)
                        and value.get("frame") == "base_link"
                        and isinstance(value.get("observation_id"), str)
                        and len(value.get("position", [])) == 3
                        and all(math.isfinite(v) for v in value["position"])
                    ):
                        completed.append(
                            {"tool_call_id": event.get("toolCallId"), "feedback": value}
                        )
                except (ValueError, TypeError):
                    continue
    return {
        "claimed_executed": claimed_executed,
        "recorded_grounding_completed": bool(completed),
        "contradiction": claimed_executed != bool(completed),
        "completed": completed,
    }
