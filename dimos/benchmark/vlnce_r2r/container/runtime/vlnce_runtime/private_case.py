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

"""Strict private episode binding loaded before Habitat is initialized."""

import gzip
import hashlib
import json
import re

SCHEMA_VERSION = "vlnce-private-case.v1"
BENCHMARK = "vlnce_r2r"
IDENTITY_FIELDS = (
    "attempt_id",
    "case_id",
    "case_fingerprint",
    "upstream_revision",
    "dataset_revision",
    "split",
    "episode_id",
    "episode_sha256",
    "scene_id",
    "instruction",
    "instruction_sha256",
    "runtime_image_digest",
    "protocol_revision",
    "result_schema_revision",
    "condition_label",
)
REQUIRED_FIELDS = frozenset(("schema_version", "benchmark", "timeout_seconds", *IDENTITY_FIELDS))
SHA256_FIELDS = frozenset(
    (
        "case_fingerprint",
        "episode_sha256",
        "instruction_sha256",
        "runtime_image_digest",
    )
)
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


class PrivateCaseError(RuntimeError):
    """The private case or mounted episode is inconsistent with the attempt."""


def load_private_case(path, episode_dataset_path, expected_identity):
    """Load and verify every private identity before creating a simulator."""

    document = _load_json(path, "private case")
    if not isinstance(document, dict) or frozenset(document) != REQUIRED_FIELDS:
        raise PrivateCaseError("private case fields do not match the v1 contract")
    if document["schema_version"] != SCHEMA_VERSION:
        raise PrivateCaseError("unsupported private case schema")
    if document["benchmark"] != BENCHMARK:
        raise PrivateCaseError("private case benchmark does not match the runtime")

    for field in IDENTITY_FIELDS:
        value = document[field]
        if not isinstance(value, str) or not value:
            raise PrivateCaseError(f"private case field {field!r} must be a non-empty string")
        if field in SHA256_FIELDS and SHA256_PATTERN.match(value) is None:
            raise PrivateCaseError(f"private case field {field!r} must be a SHA-256 digest")
        if expected_identity.get(field) != value:
            raise PrivateCaseError(f"private case field {field!r} does not match the attempt")

    timeout_seconds = document["timeout_seconds"]
    if (
        isinstance(timeout_seconds, bool)
        or not isinstance(timeout_seconds, (int, float))
        or timeout_seconds <= 0
    ):
        raise PrivateCaseError("private case timeout_seconds must be positive")

    instruction_digest = hashlib.sha256(document["instruction"].encode("utf-8")).hexdigest()
    if instruction_digest != document["instruction_sha256"]:
        raise PrivateCaseError("private case instruction digest does not match its text")

    episode = _select_episode(episode_dataset_path, document["episode_id"])
    if episode.get("scene_id") != document["scene_id"]:
        raise PrivateCaseError("mounted episode scene does not match the private case")
    instruction = episode.get("instruction", {}).get("instruction_text")
    if instruction != document["instruction"]:
        raise PrivateCaseError("mounted episode instruction does not match the private case")
    episode_digest = hashlib.sha256(_canonical_json(episode)).hexdigest()
    if episode_digest != document["episode_sha256"]:
        raise PrivateCaseError("mounted episode payload does not match the private case")
    return document, episode


def _select_episode(path, episode_id):
    try:
        with gzip.open(str(path), "rt", encoding="utf-8") as handle:
            dataset = json.load(handle)
    except (OSError, ValueError) as error:
        raise PrivateCaseError("could not read the mounted episode dataset") from error
    episodes = [
        episode
        for episode in dataset.get("episodes", [])
        if str(episode.get("episode_id")) == episode_id
    ]
    if len(episodes) != 1:
        raise PrivateCaseError("mounted dataset must contain exactly one selected episode")
    return episodes[0]


def _load_json(path, label):
    try:
        with open(str(path), encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError) as error:
        raise PrivateCaseError(f"could not read {label}") from error


def _canonical_json(value):
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
