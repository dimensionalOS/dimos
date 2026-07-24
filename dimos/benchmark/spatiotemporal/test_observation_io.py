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

"""Tests for canonical teacher-observation persistence."""

from pathlib import Path

import pytest

from dimos.benchmark.spatiotemporal.models import BoundingBox2D, ObjectObservation
from dimos.benchmark.spatiotemporal.observation_io import read_observations, write_observations


def _observation(object_id: str, label: str) -> ObjectObservation:
    return ObjectObservation(
        episode_id="episode_1",
        frame_id=12,
        timestamp_s=0.5,
        object_id=object_id,
        label=label,
        box=BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.3, y_max=0.5),
        confidence=0.9,
    )


def test_observation_jsonl_round_trip_is_canonical_and_strict(tmp_path: Path) -> None:
    path = tmp_path / "observations.jsonl"
    mug = _observation("mug_1", "mug")
    laptop = _observation("laptop_1", "laptop")

    write_observations(path, (mug, laptop))

    expected = (
        b'{"box":{"x_max":0.3,"x_min":0.1,"y_max":0.5,"y_min":0.2},'
        b'"confidence":0.9,"episode_id":"episode_1","frame_id":12,'
        b'"label":"laptop","object_id":"laptop_1","timestamp_s":0.5}\n'
        b'{"box":{"x_max":0.3,"x_min":0.1,"y_max":0.5,"y_min":0.2},'
        b'"confidence":0.9,"episode_id":"episode_1","frame_id":12,'
        b'"label":"mug","object_id":"mug_1","timestamp_s":0.5}\n'
    )
    assert path.read_bytes() == expected
    assert read_observations(path) == (laptop, mug)
    write_observations(path, read_observations(path))
    assert path.read_bytes() == expected

    invalid_documents = (
        (expected.splitlines(keepends=True)[0] * 2, "duplicate observation identity"),
        (
            expected.splitlines(keepends=True)[0]
            + expected.splitlines(keepends=True)[0].replace(
                b'"label":"laptop"', b'"label":"computer"'
            ),
            "conflicting observation identity",
        ),
        (expected.replace(b'"confidence":0.9', b'"confidence":NaN', 1), "invalid JSON"),
        (
            expected.replace(b'"timestamp_s":0.5', b'"timestamp_s":1e999', 1),
            "invalid observation",
        ),
        (expected.replace(b'"x_max":0.3', b'"x_max":1e999', 1), "invalid observation"),
        (expected.replace(b'"x_min":0.1', b'"x_min":0.4', 1), "invalid observation"),
        (expected.replace(b'"timestamp_s":0.5', b'"timestamp_s":-0.5', 1), "timestamp"),
        (expected.replace(b'"label":"laptop"', '"label":"cafe\u0301"'.encode(), 1), "NFC"),
    )
    for document, message in invalid_documents:
        path.write_bytes(document)
        with pytest.raises(ValueError, match=message):
            read_observations(path)

    with pytest.raises(ValueError, match="duplicate observation identity"):
        write_observations(path, (laptop, laptop))
    conflicting_laptop = laptop.model_copy(update={"label": "computer"})
    with pytest.raises(ValueError, match="conflicting observation identity"):
        write_observations(path, (laptop, conflicting_laptop))


def test_read_observations_rejects_duplicate_json_field_names(tmp_path: Path) -> None:
    path = tmp_path / "observations.jsonl"
    path.write_bytes(
        b'{"box":{"x_max":0.3,"x_min":0.1,"y_max":0.5,"y_min":0.2},'
        b'"confidence":0.9,"episode_id":"episode_1","frame_id":12,'
        b'"label":"laptop","object_id":"mug_1","object_id":"laptop_1",'
        b'"timestamp_s":0.5}\n'
    )

    with pytest.raises(ValueError, match="duplicate JSON field: object_id"):
        read_observations(path)


@pytest.mark.parametrize("mutation", ("whitespace", "crlf", "no-newline", "out-of-order"))
def test_read_observations_rejects_noncanonical_jsonl(tmp_path: Path, mutation: str) -> None:
    path = tmp_path / "observations.jsonl"
    write_observations(
        path,
        (_observation("mug_1", "mug"), _observation("laptop_1", "laptop")),
    )
    canonical = path.read_bytes()
    lines = canonical.splitlines(keepends=True)
    mutations = {
        "whitespace": canonical.replace(b'"confidence":0.9', b'"confidence": 0.9', 1),
        "crlf": canonical.replace(b"\n", b"\r\n"),
        "no-newline": canonical.rstrip(b"\n"),
        "out-of-order": b"".join(reversed(lines)),
    }
    path.write_bytes(mutations[mutation])

    with pytest.raises(ValueError, match="canonical"):
        read_observations(path)


@pytest.mark.parametrize("invalid_field", ("timestamp", "box", "label"))
def test_write_observations_revalidates_model_instances(tmp_path: Path, invalid_field: str) -> None:
    path = tmp_path / "observations.jsonl"
    observation = _observation("laptop_1", "laptop")
    invalid_observations = {
        "timestamp": observation.model_copy(update={"timestamp_s": float("inf")}),
        "box": observation.model_copy(
            update={"box": observation.box.model_copy(update={"x_min": 0.4})}
        ),
        "label": observation.model_copy(update={"label": "cafe\u0301"}),
    }

    with pytest.raises(ValueError):
        write_observations(path, (invalid_observations[invalid_field],))
