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

"""Authoritative VLN-CE result validation and atomic publication."""

import hashlib
import json
import math
import os
import tempfile

SCHEMA_VERSION = "vlnce-result.v1"
TERMINAL_REASONS = frozenset(("submitted", "timeout"))
OFFICIAL_METRICS = {
    "distance_to_goal": "DISTANCE_TO_GOAL",
    "success": "SUCCESS",
    "spl": "SPL",
    "ndtw": "NDTW",
    "path_length": "PATH_LENGTH",
    "oracle_success": "ORACLE_SUCCESS",
    "steps_taken": "STEPS_TAKEN",
}


class ResultError(RuntimeError):
    """A terminal result is incomplete, inconsistent, or already published."""


def build_result(private_case, terminal_reason, trajectory, native_metrics, runtime, duration):
    """Create a strict result without recomputing any official metric."""

    if terminal_reason not in TERMINAL_REASONS:
        raise ResultError("only submitted and healthy timeout episodes are scoreable")
    if not trajectory:
        raise ResultError("terminal result requires a non-empty trajectory")
    if frozenset(native_metrics) != frozenset(OFFICIAL_METRICS):
        raise ResultError("official metric set does not match the configured VLN-CE task")
    metrics = {}
    for native_name, result_name in OFFICIAL_METRICS.items():
        value = native_metrics[native_name]
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            raise ResultError(f"official metric {native_name!r} must be finite")
        metrics[result_name] = float(value)
    if metrics["SUCCESS"] not in (0.0, 1.0):
        raise ResultError("official SUCCESS must be zero or one")
    if not isinstance(duration, (int, float)) or not math.isfinite(duration) or duration < 0:
        raise ResultError("terminal duration must be finite and non-negative")
    trajectory_bytes = _canonical_json(trajectory)
    return {
        "schema_version": SCHEMA_VERSION,
        "attempt_id": private_case["attempt_id"],
        "case_id": private_case["case_id"],
        "case_fingerprint": private_case["case_fingerprint"],
        "benchmark": private_case["benchmark"],
        "dataset_revision": private_case["dataset_revision"],
        "split": private_case["split"],
        "episode_id": private_case["episode_id"],
        "scene_id": private_case["scene_id"],
        "upstream_revision": private_case["upstream_revision"],
        "runtime_image_digest": private_case["runtime_image_digest"],
        "protocol_revision": private_case["protocol_revision"],
        "result_schema_revision": private_case["result_schema_revision"],
        "condition_label": private_case["condition_label"],
        "terminal_reason": terminal_reason,
        "duration_seconds": float(duration),
        "trajectory": {
            "sha256": hashlib.sha256(trajectory_bytes).hexdigest(),
            "points": len(trajectory),
        },
        "metrics": metrics,
        "runtime": runtime,
    }


def publish_result(path, result):
    """Atomically publish exactly one validated terminal result."""

    _validate_result_shape(result)
    path = str(path)
    directory = os.path.dirname(path)
    if not directory or not os.path.isdir(directory):
        raise ResultError("terminal result directory does not exist")
    if os.path.exists(path):
        raise ResultError("terminal result was already published")
    descriptor, temporary = tempfile.mkstemp(prefix=".vlnce-result-", dir=directory)
    try:
        payload = _canonical_json(result) + b"\n"
        with os.fdopen(descriptor, "wb") as handle:
            descriptor = None
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        if os.path.exists(path):
            raise ResultError("terminal result was already published")
        os.replace(temporary, path)
        temporary = None
        directory_descriptor = os.open(directory, os.O_RDONLY)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if temporary is not None and os.path.exists(temporary):
            os.unlink(temporary)


def _validate_result_shape(result):
    required = {
        "schema_version",
        "attempt_id",
        "case_id",
        "case_fingerprint",
        "benchmark",
        "dataset_revision",
        "split",
        "episode_id",
        "scene_id",
        "upstream_revision",
        "runtime_image_digest",
        "protocol_revision",
        "result_schema_revision",
        "condition_label",
        "terminal_reason",
        "duration_seconds",
        "trajectory",
        "metrics",
        "runtime",
    }
    if not isinstance(result, dict) or frozenset(result) != frozenset(required):
        raise ResultError("terminal result fields do not match the v1 schema")
    if result["schema_version"] != SCHEMA_VERSION:
        raise ResultError("terminal result schema revision is invalid")
    if result["terminal_reason"] not in TERMINAL_REASONS:
        raise ResultError("terminal result reason is not scoreable")
    if frozenset(result["metrics"]) != frozenset(OFFICIAL_METRICS.values()):
        raise ResultError("terminal result official metrics are incomplete")


def _canonical_json(value):
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
