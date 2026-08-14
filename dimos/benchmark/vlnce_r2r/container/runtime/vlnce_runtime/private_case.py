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

SCHEMA_VERSION = "vlnce-private-case.v1"
REQUIRED_FIELDS = frozenset(
    (
        "schema_version",
        "attempt_id",
        "case_id",
        "episode_id",
        "episode_sha256",
        "scene_id",
        "split",
        "instruction",
        "timeout_seconds",
    )
)


class PrivateCaseError(RuntimeError):
    """The private case or mounted episode is inconsistent with the attempt."""


def load_private_case(path, episode_dataset_path):
    """Load the attempt binding and verify it against the mounted episode."""

    document = _load_json(path, "private case")
    if not isinstance(document, dict) or frozenset(document) != REQUIRED_FIELDS:
        raise PrivateCaseError("private case fields do not match the v1 contract")
    if document["schema_version"] != SCHEMA_VERSION:
        raise PrivateCaseError("unsupported private case schema")
    for field in ("attempt_id", "case_id", "episode_id", "scene_id", "split", "instruction"):
        value = document[field]
        if not isinstance(value, str) or not value:
            raise PrivateCaseError(f"private case field {field!r} must be a non-empty string")
    if not isinstance(document["episode_sha256"], str) or len(document["episode_sha256"]) != 64:
        raise PrivateCaseError("private case episode_sha256 must be a SHA-256 digest")

    timeout_seconds = document["timeout_seconds"]
    if (
        isinstance(timeout_seconds, bool)
        or not isinstance(timeout_seconds, (int, float))
        or timeout_seconds <= 0
    ):
        raise PrivateCaseError("private case timeout_seconds must be positive")

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
