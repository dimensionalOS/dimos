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

import json

import pytest

from dimos.imitation.collection.episode import EpisodeStatus


@pytest.mark.parametrize("task_label", ["pick", None, "", "拿起积木 🦾"])
def test_episode_status_json_roundtrip_preserves_status_update(
    task_label: str | None,
) -> None:
    expected = EpisodeStatus(
        ts=12.25,
        state="recording",
        episodes_saved=2,
        episodes_discarded=1,
        last_event="start",
        task_label=task_label,
    )

    actual = EpisodeStatus.from_json(expected.to_json())

    assert actual == expected


def test_episode_status_describes_the_json_recording_document() -> None:
    status = EpisodeStatus(
        ts=1790796295.1234567,
        state="idle",
        episodes_saved=2**31 - 1,
        episodes_discarded=0,
        task_label="拿起积木 🦾",
    )
    payload = json.loads(status.to_json())
    assert payload == {"schema_version": 1, **status.model_dump()}
    assert "schema_version" not in status.model_dump()
    assert payload["schema_version"] == 1
    assert EpisodeStatus.from_json(status.to_json()) == status


@pytest.mark.parametrize(
    "updates",
    [
        {"schema_version": 2},
        {"schema_version": True},
        {"schema_version": 1.0},
        {"ts": float("nan")},
        {"state": "unknown"},
    ],
)
def test_episode_status_rejects_invalid_wire_payload(updates) -> None:
    payload = {
        "schema_version": 1,
        "ts": 1.0,
        "state": "idle",
        "episodes_saved": 0,
        "episodes_discarded": 0,
    }
    payload.update(updates)
    with pytest.raises(ValueError):
        EpisodeStatus.from_json(json.dumps(payload))


@pytest.mark.parametrize("payload", ["not json", "null", "[]", '{"schema_version":2}'])
def test_episode_status_rejects_malformed_json_or_wrong_shape(payload) -> None:
    with pytest.raises(ValueError):
        EpisodeStatus.from_json(payload)


def test_episode_status_requires_explicit_wire_version() -> None:
    payload = '{"ts":1.0,"state":"idle","episodes_saved":0,"episodes_discarded":0}'
    with pytest.raises(ValueError, match="requires schema_version"):
        EpisodeStatus.from_json(payload)
