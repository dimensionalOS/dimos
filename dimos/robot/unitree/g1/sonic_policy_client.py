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

"""Bounded client for LeRobot's g1_dex3_training/sonic_policy_server.py.

Run inference in a worker, never in the session's 50 Hz control loop. Results
retain the caller's observation timestamp. The session owns freshness, action
bounds, hand ordering and resampling; the server does not negotiate these.
RTC is intentionally excluded: its server-side state is not scoped to a task.
"""

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
import time
from typing import Any

import cv2
import msgpack  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray
import zmq

_MAX_REPLY_BYTES = 4 * 1024 * 1024
_MAX_REQUEST_BYTES = 16 * 1024 * 1024


@dataclass(frozen=True)
class SonicPolicyInfo:
    n_obs: int
    image_shapes: Mapping[str, tuple[int, int, int]]


@dataclass(frozen=True)
class SonicPolicyChunk:
    task_id: str
    sequence: int
    observed_at: float
    actions: NDArray[np.float32]
    server_latency_seconds: float
    roundtrip_seconds: float


class SonicPolicyClient:
    def __init__(self, endpoint: str, *, timeout_seconds: float = 1.5) -> None:
        if not endpoint.startswith(("tcp://", "ipc://")):
            raise ValueError("explicit tcp:// or ipc:// policy endpoint required")
        if not math.isfinite(timeout_seconds) or not 0 < timeout_seconds <= 30:
            raise ValueError("timeout_seconds must be in (0, 30]")
        self.endpoint = endpoint
        self.timeout_seconds = timeout_seconds

    def _request(self, request: dict[str, Any]) -> dict[str, Any]:
        payload = msgpack.packb(request, use_bin_type=True)
        if len(payload) > _MAX_REQUEST_BYTES:
            raise ValueError("policy request too large")
        # Each request owns its socket, so timeouts/late replies cannot poison the
        # next REQ exchange and no ZMQ socket crosses worker-thread boundaries.
        with zmq.Context() as context, context.socket(zmq.REQ) as sock:
            sock.setsockopt(zmq.LINGER, 0)
            sock.setsockopt(zmq.MAXMSGSIZE, _MAX_REPLY_BYTES)
            sock.setsockopt(zmq.SNDTIMEO, math.ceil(self.timeout_seconds * 1000))
            sock.connect(self.endpoint)
            deadline = time.monotonic() + self.timeout_seconds
            try:
                sock.send(payload)
            except zmq.Again as exc:
                raise TimeoutError("policy request send timed out") from exc
            remaining_ms = max(0, math.ceil((deadline - time.monotonic()) * 1000))
            if not sock.poll(remaining_ms):
                raise TimeoutError("policy response timed out")
            raw = sock.recv()
        try:
            reply = msgpack.unpackb(raw, raw=False, strict_map_key=True)
        except (ValueError, msgpack.UnpackException) as exc:
            raise ValueError("invalid policy MessagePack response") from exc
        if not isinstance(reply, dict):
            raise ValueError("policy response must be a map")
        if "error" in reply:
            raise RuntimeError(f"policy server: {str(reply['error'])[:500]}")
        return dict(reply)

    def get_info(self) -> SonicPolicyInfo:
        reply = self._request({"op": "info"})
        keys, n_obs, shapes = (reply.get(k) for k in ("image_keys", "n_obs", "shapes"))
        if type(n_obs) is not int or not 1 <= n_obs <= 16:
            raise ValueError("policy n_obs must be an integer in [1, 16]")
        if (
            not isinstance(keys, list)
            or not 1 <= len(keys) <= 8
            or any(not isinstance(k, str) or not k for k in keys)
            or len(set(keys)) != len(keys)
            or not isinstance(shapes, dict)
            or set(shapes) != set(keys)
        ):
            raise ValueError("invalid policy image keys/shapes")
        result = {}
        for key in keys:
            shape = shapes[key]
            if (
                not isinstance(shape, list)
                or len(shape) != 3
                or any(type(x) is not int or not 1 <= x <= 4096 for x in shape)
                or shape[0] != 3
            ):
                raise ValueError("policy requires RGB [3, height, width] image shapes")
            result[key] = (shape[0], shape[1], shape[2])
        return SonicPolicyInfo(n_obs, result)

    def infer(
        self,
        *,
        info: SonicPolicyInfo,
        task_id: str,
        sequence: int,
        observed_at: float,
        states: NDArray[Any],
        images: Sequence[Mapping[str, NDArray[np.uint8]]],
        task: str,
    ) -> SonicPolicyChunk:
        if not task_id or not isinstance(task, str) or not task.strip():
            raise ValueError("task_id and task instruction required")
        if type(sequence) is not int or sequence < 0 or not math.isfinite(observed_at):
            raise ValueError("invalid sequence or observation time")
        state = np.asarray(states)
        if (
            state.shape != (info.n_obs, 28)
            or state.dtype.kind not in "fiu"
            or not np.isfinite(state).all()
            or np.max(np.abs(state)) > np.finfo(np.float32).max
        ):
            raise ValueError("states must be finite float32-compatible [n_obs, 28]")
        if len(images) != info.n_obs:
            raise ValueError("image history must match n_obs")
        started = time.monotonic()
        jpegs = []
        for frame in images:
            if set(frame) != set(info.image_shapes):
                raise ValueError("image keys must match policy")
            encoded = {}
            for key, (_, height, width) in info.image_shapes.items():
                rgb = np.asarray(frame[key])
                if rgb.dtype != np.uint8 or rgb.shape != (height, width, 3):
                    raise ValueError("images must be RGB uint8 at the policy's size")
                # LeRobot's paired codec deliberately passes RGB through OpenCV
                # unchanged at both ends; adding BGR conversion here swaps colors.
                ok, jpeg = cv2.imencode(".jpg", rgb, [cv2.IMWRITE_JPEG_QUALITY, 90])
                if not ok:
                    raise ValueError("JPEG encoding failed")
                encoded[key] = jpeg.tobytes()
            jpegs.append(encoded)
        reply = self._request(
            {
                "op": "chunk",
                "states": state.astype(np.float32).tolist(),
                "images": jpegs,
                "task": task,
            }
        )
        shape, raw, latency = (reply.get(k) for k in ("shape", "chunk", "latency_s"))
        if (
            not isinstance(shape, list)
            or len(shape) != 2
            or any(type(n) is not int for n in shape)
            or not 1 <= shape[0] <= 1000
            or shape[1] != 78
            or not isinstance(raw, bytes)
            or len(raw) != shape[0] * 78 * 4
        ):
            raise ValueError("policy must return float32 bytes of shape [1..1000, 78]")
        if (
            not isinstance(latency, (int, float))
            or isinstance(latency, bool)
            or not math.isfinite(latency)
            or latency < 0
        ):
            raise ValueError("invalid policy latency")
        actions = np.frombuffer(raw, dtype="<f4").reshape(shape).copy()
        if not np.isfinite(actions).all():
            raise ValueError("policy actions must be finite")
        return SonicPolicyChunk(
            task_id, sequence, observed_at, actions, float(latency), time.monotonic() - started
        )
