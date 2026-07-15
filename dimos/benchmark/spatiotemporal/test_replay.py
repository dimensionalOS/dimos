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

"""Behavioral tests for deterministic observation replay."""

from pathlib import Path

import pytest

from dimos.benchmark.spatiotemporal import replay
from dimos.benchmark.spatiotemporal.bundles import load_bundle
from dimos.benchmark.spatiotemporal.models import BoundingBox2D, ObjectObservation
from dimos.benchmark.spatiotemporal.observation_io import read_observations, write_observations
from dimos.benchmark.spatiotemporal.ports import (
    ReplayInsufficiencyCode,
    ReplayInsufficiencyError,
)
from dimos.benchmark.spatiotemporal.replay import (
    DeterministicObservationBundleGenerator,
    replay_observations,
)


def _observation(
    *,
    frame_id: int,
    object_id: str,
    box: BoundingBox2D,
    episode_id: str = "episode_1",
) -> ObjectObservation:
    return ObjectObservation(
        episode_id=episode_id,
        frame_id=frame_id,
        timestamp_s=frame_id / 10,
        object_id=object_id,
        label=object_id,
        box=box,
        confidence=0.9,
    )


def test_replays_saved_observations_to_root_independent_bundles(tmp_path: Path) -> None:
    observations = (
        _observation(
            frame_id=1,
            object_id="red",
            box=BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.2, y_max=0.4),
        ),
        _observation(
            frame_id=1,
            object_id="blue",
            box=BoundingBox2D(x_min=0.7, y_min=0.2, x_max=0.8, y_max=0.4),
        ),
        _observation(
            frame_id=3,
            object_id="green",
            box=BoundingBox2D(x_min=0.2, y_min=0.1, x_max=0.4, y_max=0.2),
        ),
        _observation(
            frame_id=3,
            object_id="yellow",
            box=BoundingBox2D(x_min=0.2, y_min=0.7, x_max=0.4, y_max=0.8),
        ),
    )
    observations_path = tmp_path / "observations.jsonl"
    write_observations(observations_path, observations)

    results = tuple(
        replay_observations(observations_path, root, "1" * 64)
        for root in (tmp_path / "first", tmp_path / "second")
    )

    assert results[0].logical_sha256 == results[1].logical_sha256
    for root, result in zip((tmp_path / "first", tmp_path / "second"), results, strict=True):
        bundle = load_bundle(root)
        assert bundle.public_manifest == result.public_manifest
        assert bundle.oracle_manifest == result.oracle_manifest
        assert bundle.observations == read_observations(observations_path)
        assert bundle.relation_facts
        assert bundle.relation_intervals
        assert bundle.questions
        assert len(bundle.answers) == len(bundle.questions)


def test_replay_coalesces_relations_across_consecutive_sparse_samples(tmp_path: Path) -> None:
    left = BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.2, y_max=0.4)
    right = BoundingBox2D(x_min=0.7, y_min=0.2, x_max=0.8, y_max=0.4)
    observations = tuple(
        _observation(frame_id=frame_id, object_id=object_id, box=box)
        for frame_id in (0, 150)
        for object_id, box in (("left", left), ("right", right))
    )

    DeterministicObservationBundleGenerator(sample_schedule=((0, 0.0), (150, 15.0))).generate(
        observations, tmp_path / "bundle", "1" * 64
    )
    intervals = load_bundle(tmp_path / "bundle").relation_intervals

    assert len(intervals) == 2
    assert all(interval.start_frame_id == 0 for interval in intervals)
    assert all(interval.end_frame_id == 150 for interval in intervals)
    assert all(interval.evidence_frame_ids == (0, 150) for interval in intervals)


def test_replay_does_not_bridge_an_empty_intervening_sample(tmp_path: Path) -> None:
    left = BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.2, y_max=0.4)
    right = BoundingBox2D(x_min=0.7, y_min=0.2, x_max=0.8, y_max=0.4)
    observations = tuple(
        _observation(frame_id=frame_id, object_id=object_id, box=box)
        for frame_id in (0, 300)
        for object_id, box in (("left", left), ("right", right))
    )

    DeterministicObservationBundleGenerator(
        sample_schedule=((0, 0.0), (150, 15.0), (300, 30.0))
    ).generate(observations, tmp_path / "bundle", "1" * 64)
    intervals = load_bundle(tmp_path / "bundle").relation_intervals

    assert len(intervals) == 4
    assert all(interval.start_frame_id == interval.end_frame_id for interval in intervals)


def test_replay_observations_accepts_complete_sample_schedule(tmp_path: Path) -> None:
    left = BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.2, y_max=0.4)
    right = BoundingBox2D(x_min=0.7, y_min=0.2, x_max=0.8, y_max=0.4)
    observations = tuple(
        _observation(frame_id=frame_id, object_id=object_id, box=box)
        for frame_id in (0, 150)
        for object_id, box in (("left", left), ("right", right))
    )
    observations_path = tmp_path / "observations.jsonl"
    write_observations(observations_path, observations)

    replay_observations(
        observations_path,
        tmp_path / "bundle",
        "1" * 64,
        sample_schedule=((0, 0.0), (150, 15.0)),
    )

    assert len(load_bundle(tmp_path / "bundle").relation_intervals) == 2


@pytest.mark.parametrize(
    ("observations", "expected_code", "expected_guidance"),
    [
        ((), ReplayInsufficiencyCode.EMPTY_OBSERVATIONS, "capture at least one sampled frame"),
        (
            (
                _observation(
                    frame_id=1,
                    object_id="red",
                    box=BoundingBox2D(x_min=0.1, y_min=0.1, x_max=0.2, y_max=0.2),
                ),
                _observation(
                    frame_id=2,
                    object_id="blue",
                    box=BoundingBox2D(x_min=0.7, y_min=0.7, x_max=0.8, y_max=0.8),
                    episode_id="episode_2",
                ),
            ),
            ReplayInsufficiencyCode.MIXED_EPISODES,
            "replay one episode at a time",
        ),
        (
            (
                _observation(
                    frame_id=1,
                    object_id="red",
                    box=BoundingBox2D(x_min=0.1, y_min=0.1, x_max=0.2, y_max=0.2),
                ),
            ),
            ReplayInsufficiencyCode.NO_RELATIONS,
            "spatially separated object pairs",
        ),
    ],
)
def test_reports_actionable_stable_insufficiency_reasons(
    tmp_path: Path,
    observations: tuple[ObjectObservation, ...],
    expected_code: ReplayInsufficiencyCode,
    expected_guidance: str,
) -> None:
    generator = DeterministicObservationBundleGenerator()

    with pytest.raises(ReplayInsufficiencyError) as raised:
        generator.generate(observations, tmp_path / "bundle", "1" * 64)

    assert raised.value.code is expected_code
    assert expected_guidance in str(raised.value)


def test_reports_when_accepted_relations_produce_no_questions(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    observations = (
        _observation(
            frame_id=1,
            object_id="red",
            box=BoundingBox2D(x_min=0.1, y_min=0.1, x_max=0.2, y_max=0.2),
        ),
        _observation(
            frame_id=1,
            object_id="blue",
            box=BoundingBox2D(x_min=0.7, y_min=0.1, x_max=0.8, y_max=0.2),
        ),
    )
    monkeypatch.setattr(replay, "generate_spatial_questions", lambda facts: ())

    with pytest.raises(ReplayInsufficiencyError) as raised:
        DeterministicObservationBundleGenerator().generate(
            observations, tmp_path / "bundle", "1" * 64
        )

    assert raised.value.code is ReplayInsufficiencyCode.NO_QUESTIONS
    assert "evaluation questions" in str(raised.value)
