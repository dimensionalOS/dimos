# Copyright 2025-2026 Dimensional Inc.
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

"""Canonical JSONL persistence for private teacher observations."""

from collections.abc import Iterable
import json
from pathlib import Path
from typing import Any

from pydantic import ValidationError

from dimos.benchmark.spatiotemporal.models import ObjectObservation
from dimos.benchmark.spatiotemporal.utilities import canonical_model_json


def _reject_nonfinite_json(constant: str) -> None:
    raise ValueError(f"invalid JSON numeric constant: {constant}")


def _reject_duplicate_json_fields(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"duplicate JSON field: {key}")
        value[key] = item
    return value


def _identity(observation: ObjectObservation) -> tuple[str, int, str]:
    return observation.episode_id, observation.frame_id, observation.object_id


def _validated_observations(
    observations: Iterable[ObjectObservation],
) -> tuple[ObjectObservation, ...]:
    by_identity: dict[tuple[str, int, str], ObjectObservation] = {}
    for observation in observations:
        try:
            observation = ObjectObservation.model_validate(observation.model_dump())
        except ValidationError as error:
            raise ValueError(f"invalid observation: {error}") from error
        if observation.timestamp_s < 0.0:
            raise ValueError("observation timestamp must be non-negative")

        identity = _identity(observation)
        previous = by_identity.get(identity)
        if previous is not None:
            qualifier = "duplicate" if previous == observation else "conflicting"
            raise ValueError(f"{qualifier} observation identity: {identity}")
        by_identity[identity] = observation

    return tuple(sorted(by_identity.values(), key=_identity))


def write_observations(path: Path, observations: Iterable[ObjectObservation]) -> None:
    """Write observations as deterministic, identity-ordered canonical JSONL."""
    ordered = _validated_observations(observations)
    document = "".join(f"{canonical_model_json(observation)}\n" for observation in ordered)
    path.write_bytes(document.encode("utf-8"))


def read_observations(path: Path) -> tuple[ObjectObservation, ...]:
    """Read and strictly validate canonical teacher observations from JSONL."""
    try:
        document = path.read_bytes().decode("utf-8")
    except UnicodeDecodeError as error:
        raise ValueError("observation JSONL must be valid UTF-8") from error
    if document and not document.endswith("\n"):
        raise ValueError("observation JSONL must end with a canonical newline")

    lines = document[:-1].split("\n") if document else []
    observations: list[ObjectObservation] = []
    for line_number, line in enumerate(lines, start=1):
        try:
            value = json.loads(
                line,
                object_pairs_hook=_reject_duplicate_json_fields,
                parse_constant=_reject_nonfinite_json,
            )
        except (json.JSONDecodeError, ValueError) as error:
            raise ValueError(f"invalid JSON at line {line_number}: {error}") from error
        try:
            observation = ObjectObservation.model_validate(value)
        except ValidationError as error:
            raise ValueError(f"invalid observation at line {line_number}: {error}") from error
        if line != canonical_model_json(observation):
            raise ValueError(f"non-canonical observation JSON at line {line_number}")
        observations.append(observation)

    ordered = _validated_observations(observations)
    if tuple(observations) != ordered:
        raise ValueError("observation JSONL records are not in canonical identity order")
    return ordered
