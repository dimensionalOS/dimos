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

"""Unit tests for the pure DataPrep helpers in `core.py`.

No I/O: a tiny in-memory fake stands in for `SqliteStore`, exposing only the
surface the helpers touch (`stream(name)` → iterable of `.ts`/`.data` records,
with `.time_range(t0, t1)`). Keeps these fast and dependency-free.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.imitation.dataprep.build import _write_dimos_meta, inspect_dataset, run_dataprep
from dimos.imitation.dataprep.core import (
    DataPrepConfig,
    DatasetSchema,
    Episode,
    EpisodeExtractor,
    EpisodeQualityReport,
    FeatureSpec,
    OutputConfig,
    QualityConfig,
    Sample,
    SyncConfig,
    extract_episodes,
    inspect_episode_quality,
    inspect_episodes,
    is_image_array,
    iter_episode_samples,
    resolve_field,
    summarize_lengths,
)
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.JointState import JointState


@pytest.mark.parametrize(
    ("arr", "expected"),
    [
        (np.zeros(8, np.float32), False),  # 1D proprio vector
        (np.zeros((48, 64), np.uint8), True),  # 2D grayscale image
        (np.zeros((48, 64, 3), np.uint8), True),  # RGB image
        (np.zeros((4, 4), np.float64), False),  # SE(3) pose matrix — not an image
        (np.zeros((3, 3), np.float32), False),  # rotation matrix
        (np.zeros((6, 1), np.float32), False),  # stacked force-torque
        (np.zeros((2, 7), np.float64), False),  # jacobian slice
    ],
)
def test_is_image_array_disambiguates_2d_by_dtype(arr: np.ndarray, expected: bool) -> None:
    # 2D float matrices stay low-dim; 2D integer frames are grayscale images.
    assert is_image_array(arr) is expected


def test_inspect_empty_recording_without_status_stream(tmp_path: Path) -> None:
    db_path = tmp_path / "empty.db"
    with SqliteStore(path=str(db_path)):
        pass

    info = inspect_dataset(db_path)

    assert info["streams"] == {}
    assert info["status_stream"] is None
    assert info["episodes"] == 0
    assert info["incomplete_episodes"] == []


def test_inspect_rejects_unknown_format(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="Cannot detect data format"):
        inspect_dataset(tmp_path / "unknown")


# ── fakes ────────────────────────────────────────────────────────────────────


@dataclass
class _Obs:
    ts: float
    data: Any


class _FakeStream:
    def __init__(self, obs: list[_Obs]) -> None:
        self._obs = sorted(obs, key=lambda o: o.ts)

    def __iter__(self):
        return iter(self._obs)

    def time_range(self, t0: float, t1: float) -> _FakeStream:
        return _FakeStream([o for o in self._obs if t0 <= o.ts <= t1])


class _FakeStore:
    def __init__(self, streams: dict[str, list[_Obs]]) -> None:
        self._streams = {k: _FakeStream(v) for k, v in streams.items()}

    def stream(self, name: str) -> _FakeStream:
        return self._streams.get(name, _FakeStream([]))

    def list_streams(self) -> list[str]:
        return list(self._streams)


@dataclass
class _Status:
    """Mimics EpisodeStatus fields the extractor reads via getattr."""

    last_event: str
    task_label: str | None = None


def _status(events: list[tuple[float, str, str | None]]) -> list[_Obs]:
    """events = [(ts, last_event, label), ...]"""
    return [_Obs(ts=ts, data=_Status(last_event=ev, task_label=lbl)) for ts, ev, lbl in events]


def _feature(stream: str, field: str | None = "position") -> FeatureSpec:
    return FeatureSpec(stream=stream, field=field, dtype="float32", shape=(1,), names=["joint"])


# ── resolve_field ────────────────────────────────────────────────────────────


def test_resolve_field_attribute() -> None:
    @dataclass
    class Msg:
        position: list[float]

    arr = resolve_field(
        Msg(position=[1.0, 2.0, 3.0]),
        FeatureSpec(
            stream="x", field="position", dtype="float32", shape=(3,), names=["a", "b", "c"]
        ),
    )
    assert isinstance(arr, np.ndarray)
    np.testing.assert_array_equal(arr, np.array([1.0, 2.0, 3.0]))


def test_resolve_field_orders_joint_state_by_feature_names() -> None:
    arr = resolve_field(
        JointState(name=["joint_b", "joint_a"], position=[2.0, 1.0]),
        FeatureSpec(
            stream="state",
            field="position",
            dtype="float32",
            shape=(2,),
            names=["joint_a", "joint_b"],
        ),
    )

    np.testing.assert_array_equal(arr, np.array([1.0, 2.0]))


@pytest.mark.parametrize(
    ("field", "expected"),
    [("velocity", [1.0, 2.0]), ("effort", [3.0, 4.0])],
)
def test_resolve_field_orders_all_joint_state_vectors(field: str, expected: list[float]) -> None:
    state = JointState(
        name=["joint_b", "joint_a"],
        velocity=[2.0, 1.0],
        effort=[4.0, 3.0],
    )
    spec = FeatureSpec(
        stream="state",
        field=field,
        dtype="float32",
        shape=(2,),
        names=["joint_a", "joint_b"],
    )

    np.testing.assert_array_equal(resolve_field(state, spec), expected)


def test_resolve_field_rejects_duplicate_joint_names() -> None:
    with pytest.raises(ValueError, match="duplicate joint names"):
        resolve_field(
            JointState(name=["joint_a", "joint_a"], position=[1.0, 2.0]),
            FeatureSpec(
                stream="state",
                field="position",
                dtype="float32",
                shape=(1,),
                names=["joint_a"],
            ),
        )


def test_resolve_field_rejects_missing_configured_joint() -> None:
    with pytest.raises(ValueError, match="missing configured joints.*joint_b"):
        resolve_field(
            JointState(name=["joint_a"], position=[1.0]),
            FeatureSpec(
                stream="state",
                field="position",
                dtype="float32",
                shape=(2,),
                names=["joint_a", "joint_b"],
            ),
        )


def test_resolve_field_dict_payload() -> None:
    arr = resolve_field(
        {"q": [4, 5]},
        FeatureSpec(stream="x", field="q", dtype="float32", shape=(2,), names=["a", "b"]),
    )
    np.testing.assert_array_equal(arr, np.array([4, 5]))


def test_resolve_field_none_passthrough_ndarray() -> None:
    src = np.arange(6).reshape(2, 3)
    out = resolve_field(
        src, FeatureSpec(stream="x", dtype="float32", shape=(2, 3), names=["row", "column"])
    )
    assert out is src  # ndarray passes straight through


def test_resolve_field_none_unwraps_data_attr() -> None:
    @dataclass
    class Image:
        data: np.ndarray

    img = Image(data=np.ones((2, 2)))
    out = resolve_field(
        img, FeatureSpec(stream="x", dtype="float32", shape=(2, 2), names=["row", "column"])
    )
    np.testing.assert_array_equal(out, np.ones((2, 2)))


# ── extract_episodes: episode_status ─────────────────────────────────────────


def test_extract_start_save() -> None:
    store = _FakeStore({"status": _status([(1.0, "start", "pick"), (5.0, "save", None)])})
    eps = extract_episodes(store, EpisodeExtractor(status_stream="status"))
    assert len(eps) == 1
    assert eps[0].start_ts == 1.0 and eps[0].end_ts == 5.0
    assert eps[0].success is True
    assert eps[0].task_label == "pick"


def test_extract_discard_marks_failure() -> None:
    store = _FakeStore({"status": _status([(1.0, "start", None), (3.0, "discard", None)])})
    eps = extract_episodes(store, EpisodeExtractor(status_stream="status"))
    assert len(eps) == 1
    assert eps[0].success is False


def test_extract_auto_commit_on_restart() -> None:
    # start, then another start without save → first auto-commits (success=True)
    store = _FakeStore(
        {
            "status": _status(
                [
                    (1.0, "start", None),
                    (4.0, "start", None),
                    (8.0, "save", None),
                ]
            )
        }
    )
    eps = extract_episodes(store, EpisodeExtractor(status_stream="status"))
    assert len(eps) == 2
    assert eps[0].start_ts == 1.0 and eps[0].end_ts == 4.0 and eps[0].success is True
    assert eps[1].start_ts == 4.0 and eps[1].end_ts == 8.0


def test_extract_pending_at_eof_dropped() -> None:
    store = _FakeStore({"status": _status([(1.0, "start", None)])})
    eps = extract_episodes(store, EpisodeExtractor(status_stream="status"))
    assert eps == []


def test_inspect_pending_at_eof_reports_incomplete() -> None:
    store = _FakeStore({"status": _status([(1.0, "start", "unfinished")])})

    report = inspect_episodes(store, EpisodeExtractor(status_stream="status"))

    assert report.episodes == []
    assert len(report.incomplete) == 1
    assert report.incomplete[0].start_ts == 1.0
    assert report.incomplete[0].task_label == "unfinished"


def test_extract_init_and_unknown_are_noops() -> None:
    store = _FakeStore(
        {"status": _status([(0.5, "init", None), (1.0, "start", None), (5.0, "save", None)])}
    )
    eps = extract_episodes(store, EpisodeExtractor(status_stream="status"))
    assert len(eps) == 1


def test_extract_save_without_start_emits_nothing() -> None:
    store = _FakeStore({"status": _status([(2.0, "save", None)])})
    assert extract_episodes(store, EpisodeExtractor(status_stream="status")) == []


# ── extract_episodes: ranges ─────────────────────────────────────────────────


def test_extract_ranges() -> None:
    cfg = EpisodeExtractor(extractor="ranges", ranges=[(0.0, 1.0), (2.0, 3.0)])
    eps = extract_episodes(_FakeStore({}), cfg)
    assert [(e.start_ts, e.end_ts) for e in eps] == [(0.0, 1.0), (2.0, 3.0)]


def test_extract_ranges_empty() -> None:
    cfg = EpisodeExtractor(extractor="ranges", ranges=None)
    assert extract_episodes(_FakeStore({}), cfg) == []


# ── iter_episode_samples ─────────────────────────────────────────────────────


def _scalar_stream(values: list[tuple[float, float]]) -> list[_Obs]:
    """values = [(ts, scalar), ...] → messages with a `.position` 1-vector."""

    @dataclass
    class S:
        position: list[float]

    return [_Obs(ts=ts, data=S(position=[v])) for ts, v in values]


def test_sync_uses_same_frame_observation_and_applied_action() -> None:
    store = _FakeStore(
        {
            "js": _scalar_stream([(0.0, 10.0), (1.0, 11.0), (2.0, 12.0)]),
        }
    )
    ep = Episode(id="ep_0", start_ts=0.0, end_ts=2.0)
    streams = {
        "state": _feature("js"),
        "act": _feature("js"),
    }
    sync = SyncConfig(anchor="state", rate_hz=1.0, tolerance_ms=100.0)
    samples = list(
        iter_episode_samples(
            store,
            ep,
            streams,
            sync,
            QualityConfig(),
            obs_keys={"state"},
            action_keys={"act"},
        )
    )
    assert len(samples) == 3
    # action equals state at the same frame
    np.testing.assert_array_equal(samples[0].observation["state"], samples[0].action["act"])


def test_fill_preserves_grid_and_marks_held_frame() -> None:
    # anchor ticks every 1s, but the second stream has a big gap around t=1
    store = _FakeStore(
        {
            "anchor": _scalar_stream([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]),
            "other": _scalar_stream([(0.0, 5.0), (2.0, 7.0)]),  # nothing near t=1
        }
    )
    ep = Episode(id="ep_0", start_ts=0.0, end_ts=2.0)
    streams = {
        "anchor": _feature("anchor"),
        "other": _feature("other"),
    }
    sync = SyncConfig(anchor="anchor", rate_hz=1.0, tolerance_ms=100.0)
    samples = list(
        iter_episode_samples(
            store,
            ep,
            streams,
            sync,
            QualityConfig(mode="fill"),
            obs_keys={"anchor", "other"},
        )
    )
    assert [round(s.ts) for s in samples] == [0, 1, 2]
    assert [bool(s.complementary_info["is_filled"][0]) for s in samples] == [False, True, False]
    np.testing.assert_array_equal(samples[1].observation["other"], [5.0])


def test_strict_quality_rejects_missing_fixed_rate_slot() -> None:
    store = _FakeStore(
        {
            "anchor": _scalar_stream([(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]),
            "other": _scalar_stream([(0.0, 5.0), (2.0, 7.0)]),
        }
    )
    episode = Episode(id="ep_0", start_ts=0.0, end_ts=2.0)
    streams = {"anchor": _feature("anchor"), "other": _feature("other")}

    report = inspect_episode_quality(
        store,
        episode,
        streams,
        SyncConfig(anchor="anchor", rate_hz=1.0, tolerance_ms=20.0),
        QualityConfig(mode="strict"),
    )

    assert report.valid is False
    assert report.expected_frames == 3
    assert report.emitted_frames == 2
    assert "no complete aligned sample" in report.rejection_reasons[-1]

    samples = list(
        iter_episode_samples(
            store,
            episode,
            streams,
            SyncConfig(anchor="anchor", rate_hz=1.0, tolerance_ms=20.0),
            QualityConfig(mode="strict"),
        )
    )
    assert len(samples) == report.emitted_frames


def test_fill_quality_accepts_gap_and_reports_filled_slot() -> None:
    store = _FakeStore(
        {
            "anchor": _scalar_stream([(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]),
            "other": _scalar_stream([(0.0, 5.0), (2.0, 7.0)]),
        }
    )
    episode = Episode(id="ep_0", start_ts=0.0, end_ts=2.0)
    streams = {"anchor": _feature("anchor"), "other": _feature("other")}

    report = inspect_episode_quality(
        store,
        episode,
        streams,
        SyncConfig(anchor="anchor", rate_hz=1.0, tolerance_ms=20.0),
        QualityConfig(mode="fill"),
    )

    assert report.valid is True
    assert report.expected_frames == 3
    assert report.emitted_frames == 3
    assert report.filled_frames == 1

    samples = list(
        iter_episode_samples(
            store,
            episode,
            streams,
            SyncConfig(anchor="anchor", rate_hz=1.0, tolerance_ms=20.0),
            QualityConfig(mode="fill"),
        )
    )
    assert len(samples) == report.emitted_frames
    assert sum(bool(sample.complementary_info["is_filled"][0]) for sample in samples) == (
        report.filled_frames
    )


def test_sync_missing_anchor_raises() -> None:
    ep = Episode(id="ep_0", start_ts=0.0, end_ts=1.0)
    streams = {"x": _feature("x")}
    sync = SyncConfig(anchor="not_there", rate_hz=1.0, tolerance_ms=10.0)
    with pytest.raises(ValueError, match="anchor"):
        list(iter_episode_samples(_FakeStore({}), ep, streams, sync, QualityConfig()))


def test_sync_empty_anchor_yields_nothing() -> None:
    store = _FakeStore({"a": []})
    ep = Episode(id="ep_0", start_ts=0.0, end_ts=1.0)
    streams = {"a": _feature("a")}
    sync = SyncConfig(anchor="a", rate_hz=1.0, tolerance_ms=10.0)
    assert list(iter_episode_samples(store, ep, streams, sync, QualityConfig())) == []


def _held_action():
    return FeatureSpec(
        stream="commands",
        field="position",
        dtype="float32",
        shape=(3,),
        names=["left", "right", "gripper"],
        source_kind="joint_position_updates",
    )


@pytest.mark.parametrize("sampling", ["nearest", "joint_position_hold"])
def test_obsolete_sampling_option_is_rejected(sampling):
    values = _held_action().model_dump()
    values["sampling"] = sampling
    with pytest.raises(
        ValueError,
        match="Extra inputs are not permitted",
    ):
        FeatureSpec(**values)


def test_saved_schema_rejects_conflicting_source_kinds():
    updates = _held_action()
    snapshot = FeatureSpec(**(updates.model_dump() | {"source_kind": "snapshot"}))
    with pytest.raises(ValueError, match="conflicting source kinds"):
        DatasetSchema.model_validate_json(
            json.dumps(
                {
                    "observation": {"state": snapshot.model_dump()},
                    "action": {"action": updates.model_dump()},
                }
            )
        )


def test_snapshots_can_align_forward_but_targets_remain_causal():
    store = _FakeStore(
        {
            "anchor": _scalar_stream([(10, 0), (11, 0)]),
            "measured": _scalar_stream([(10.01, 3), (10.99, 4)]),
            "commands": [
                _Obs(1, JointState(name=["left", "right", "gripper"], position=[1, 2, 0])),
                _Obs(10.001, JointState(name=["left"], position=[5])),
            ],
        }
    )
    streams = {
        "anchor": _feature("anchor"),
        "measured": _feature("measured"),
        "target": _held_action(),
    }
    episode = Episode(id="episode", start_ts=10, end_ts=11)
    sync = SyncConfig(anchor="anchor", rate_hz=1, tolerance_ms=20)

    report = inspect_episode_quality(store, episode, streams, sync, QualityConfig())
    samples = list(
        iter_episode_samples(
            store, episode, streams, sync, QualityConfig(), action_keys={"measured"}
        )
    )

    assert report.valid, report.rejection_reasons
    assert report.emitted_frames == len(samples) == 2
    assert report.max_alignment_error_ms == pytest.approx(10)
    np.testing.assert_array_equal([sample.action["measured"] for sample in samples], [[3], [4]])
    np.testing.assert_array_equal(
        [sample.observation["target"] for sample in samples], [[1, 2, 0], [5, 2, 0]]
    )


def test_shared_update_source_is_read_once_for_multiple_projections(mocker):
    store = _FakeStore(
        {
            "anchor": _scalar_stream([(10, 0)]),
            "commands": [
                _Obs(1, JointState(name=["left", "right", "gripper"], position=[1, 2, 0]))
            ],
        }
    )
    read = mocker.spy(store, "stream")
    streams = {
        "anchor": _feature("anchor"),
        "all": _held_action(),
        "gripper": FeatureSpec(
            **(_held_action().model_dump() | {"names": ["gripper"], "shape": (1,)})
        ),
    }
    samples = list(
        iter_episode_samples(
            store,
            Episode(id="episode", start_ts=10, end_ts=10),
            streams,
            SyncConfig(anchor="anchor", rate_hz=1, tolerance_ms=20),
            QualityConfig(),
        )
    )
    assert read.call_args_list == [mocker.call("anchor"), mocker.call("commands")]
    np.testing.assert_array_equal(samples[0].observation["all"], [1, 2, 0])
    np.testing.assert_array_equal(samples[0].observation["gripper"], [0])


def test_held_action_uses_causal_recording_history_across_episodes():
    store = _FakeStore(
        {
            "anchor": _scalar_stream([(10.0, 0.0), (11.0, 0.0), (12.0, 0.0), (20.0, 0.0)]),
            "commands": [
                _Obs(1.0, JointState(name=["right", "left"], position=[2.0, 1.0])),
                _Obs(2.0, JointState(name=["gripper"], position=[0.0])),
                _Obs(10.0, JointState(name=["gripper"], position=[1.0])),
                _Obs(10.01, JointState(name=["left"], position=[3.0])),
                _Obs(12.0, JointState(name=["gripper", "right", "left"], position=[0.0, 5.0, 4.0])),
            ],
        }
    )
    streams = {"anchor": _feature("anchor"), "action": _held_action()}
    sync = SyncConfig(anchor="anchor", rate_hz=1.0, tolerance_ms=20)
    for start, end, expected in [
        (10.0, 12.0, [[1, 2, 1], [3, 2, 1], [4, 5, 0]]),
        (20.0, 20.0, [[4, 5, 0]]),
    ]:
        episode = Episode(id="episode", start_ts=start, end_ts=end)
        report = inspect_episode_quality(store, episode, streams, sync, QualityConfig())
        samples = list(
            iter_episode_samples(
                store,
                episode,
                streams,
                sync,
                QualityConfig(),
                action_keys={"action"},
            )
        )
        assert report.valid, report.rejection_reasons
        assert report.emitted_frames == len(samples) == len(expected)
        assert report.filled_frames == 0
        assert report.max_alignment_error_ms == 0
        np.testing.assert_array_equal([sample.action["action"] for sample in samples], expected)


def test_held_action_does_not_initialize_from_future_commands():
    store = _FakeStore(
        {
            "anchor": _scalar_stream([(10.0, 0.0), (11.0, 0.0)]),
            "commands": [
                _Obs(1.0, JointState(name=["left", "right"], position=[1.0, 2.0])),
                _Obs(10.001, JointState(name=["gripper"], position=[1.0])),
            ],
        }
    )
    streams = {"anchor": _feature("anchor"), "action": _held_action()}
    sync = SyncConfig(anchor="anchor", rate_hz=1, tolerance_ms=20)
    episode = Episode(id="episode", start_ts=10, end_ts=11)
    report = inspect_episode_quality(store, episode, streams, sync, QualityConfig())
    samples = list(iter_episode_samples(store, episode, streams, sync, QualityConfig()))
    assert not report.valid
    assert report.emitted_frames == len(samples) == 1
    assert "gripper" in " ".join(report.rejection_reasons)
    assert "10.0" in " ".join(report.rejection_reasons)
    assert samples[0].ts == 11


@pytest.mark.parametrize(
    "message, reason",
    [
        (JointState(name=["left", "left"], position=[1, 2]), "duplicate"),
        (JointState(name=["left"], position=[]), "names but"),
        (JointState(name=["left"], position=[float("nan")]), "non-finite"),
        ({"position": [1]}, "JointState"),
    ],
)
def test_held_action_rejects_malformed_updates_even_between_samples(message, reason):
    store = _FakeStore(
        {
            "anchor": _scalar_stream([(10.0, 0.0)]),
            "commands": [
                _Obs(1, message),
                _Obs(2, JointState(name=["left", "right", "gripper"], position=[1, 2, 0])),
            ],
        }
    )
    streams = {"anchor": _feature("anchor"), "action": _held_action()}
    episode = Episode(id="episode", start_ts=10, end_ts=10)
    sync = SyncConfig(anchor="anchor", rate_hz=1, tolerance_ms=20)
    report = inspect_episode_quality(store, episode, streams, sync, QualityConfig())
    assert not report.valid
    assert reason in " ".join(report.rejection_reasons)
    with pytest.raises(ValueError, match=reason):
        list(iter_episode_samples(store, episode, streams, sync, QualityConfig()))


def test_nearest_validates_selected_samples_not_unused_partial_messages():
    store = _FakeStore(
        {
            "anchor": _scalar_stream([(10.0, 0.0), (11.0, 0.0)]),
            "commands": [
                _Obs(10, JointState(name=["joint"], position=[1])),
                _Obs(10.5, JointState(name=[], position=[])),
                _Obs(11, JointState(name=["joint"], position=[2])),
            ],
        }
    )
    streams = {"anchor": _feature("anchor"), "action": _feature("commands")}
    episode = Episode(id="episode", start_ts=10, end_ts=11)
    sync = SyncConfig(anchor="anchor", rate_hz=1, tolerance_ms=20)
    report = inspect_episode_quality(store, episode, streams, sync, QualityConfig())
    samples = list(iter_episode_samples(store, episode, streams, sync, QualityConfig()))
    assert report.valid, report.rejection_reasons
    assert report.emitted_frames == len(samples) == 2


def test_held_action_without_history_reports_missing_joints_and_emits_nothing():
    store = _FakeStore({"anchor": _scalar_stream([(10.0, 0.0)])})
    streams = {"anchor": _feature("anchor"), "action": _held_action()}
    episode = Episode(id="episode", start_ts=10, end_ts=10)
    sync = SyncConfig(anchor="anchor", rate_hz=1, tolerance_ms=20)

    report = inspect_episode_quality(store, episode, streams, sync, QualityConfig())

    assert not report.valid
    assert report.expected_frames == 1
    assert report.emitted_frames == 0
    assert "['left', 'right', 'gripper']" in report.rejection_reasons[0]
    assert list(iter_episode_samples(store, episode, streams, sync, QualityConfig())) == []


def test_held_actions_do_not_relax_camera_rate_or_gap_checks():
    image = {"data": np.zeros((8, 8, 3), dtype=np.uint8)}
    store = _FakeStore(
        {
            "camera": [_Obs(10, image), _Obs(10.1, image), _Obs(10.3, image)],
            "commands": [
                _Obs(
                    1,
                    JointState(
                        name=["left", "right", "gripper"],
                        position=[1, 2, 0],
                    ),
                )
            ],
        }
    )
    streams = {
        "camera": FeatureSpec(
            stream="camera",
            field="data",
            dtype="video",
            shape=(8, 8, 3),
            names=["height", "width", "channels"],
        ),
        "action": _held_action(),
    }
    episode = Episode(id="episode", start_ts=10, end_ts=10.3)
    sync = SyncConfig(anchor="camera", rate_hz=30, tolerance_ms=20)

    report = inspect_episode_quality(store, episode, streams, sync, QualityConfig())
    samples = list(iter_episode_samples(store, episode, streams, sync, QualityConfig()))

    assert not report.valid
    assert report.source_rates_hz["camera"] == pytest.approx(2 / 0.3)
    assert report.max_gaps_ms["camera"] == pytest.approx(200)
    reasons = " ".join(report.rejection_reasons)
    assert "source rate" in reasons
    assert "maximum source gap" in reasons
    assert report.emitted_frames == len(samples)


@pytest.mark.parametrize(
    "changes",
    [
        {"field": "velocity"},
        {"field": None},
        {"dtype": "video", "shape": (2, 2, 3)},
        {"names": ["left", "left", "gripper"]},
    ],
)
def test_held_action_requires_named_position_vector(changes):
    values = dict(
        stream="commands",
        field="position",
        dtype="float32",
        shape=(3,),
        names=["left", "right", "gripper"],
        source_kind="joint_position_updates",
    )
    with pytest.raises(ValueError):
        FeatureSpec(**(values | changes))


# ── summarize_lengths ────────────────────────────────────────────────────────


def test_summarize_lengths_uniform() -> None:
    assert summarize_lengths([5, 5, 5]) == {"min": 5, "max": 5, "mean": 5.0, "uniform": True}


def test_summarize_lengths_varied() -> None:
    s = summarize_lengths([2, 4, 6])
    assert s == {"min": 2, "max": 6, "mean": 4.0, "uniform": False}


def test_summarize_lengths_empty() -> None:
    assert summarize_lengths([]) == {"min": 0, "max": 0, "mean": 0.0, "uniform": True}


# ── dimos_meta sidecar ───────────────────────────────────────────────────────


def test_dimos_meta_records_sync_and_quality(tmp_path: Path) -> None:
    cfg = DataPrepConfig(
        source="s.db",
        observation={"state": _feature("js")},
        action={"action": _feature("js")},
        sync=SyncConfig(anchor="state", rate_hz=14.0, tolerance_ms=80.0),
        output=OutputConfig(format="lerobot", path=tmp_path, metadata={"fps": 14}),
    )
    _write_dimos_meta(tmp_path, cfg, episodes=[], quality_reports=[])

    meta = json.loads((tmp_path / "dimos_meta.json").read_text())
    assert meta["quality"]["mode"] == "strict"
    assert meta["source"] == "s.db"


def test_dimos_meta_beside_file_for_hdf5(tmp_path: Path) -> None:
    """hdf5 writer returns a FILE path; the sidecar must land beside it, not
    inside it (which would treat the .hdf5 file as a directory and crash)."""
    ds_file = tmp_path / "session.hdf5"
    ds_file.write_bytes(b"\x89HDF\r\n")  # stand-in for a real .hdf5
    cfg = DataPrepConfig(source="s.db", output=OutputConfig(format="hdf5", path=ds_file))

    _write_dimos_meta(ds_file, cfg, episodes=[], quality_reports=[])

    sidecar = tmp_path / "session.dimos_meta.json"
    assert sidecar.exists()  # beside the file, not session.hdf5/dimos_meta.json
    assert json.loads(sidecar.read_text())["format"] == "hdf5"


def test_run_dataprep_rejects_shared_obs_action_key() -> None:
    """A name in both obs and action would silently drop the obs feature when the
    two maps merge; run_dataprep must reject it before opening the store."""
    cfg = DataPrepConfig(
        source="nonexistent.db",  # never reached — the check runs first
        observation={"joints": _feature("joint_state")},
        action={"joints": _feature("joint_state")},
    )
    with pytest.raises(ValueError, match="share feature name"):
        run_dataprep(cfg)


def test_run_dataprep_rejects_empty_recorded_stream_before_writer(mocker, tmp_path: Path) -> None:
    store = mocker.MagicMock()
    store.list_streams.return_value = ["color_image", "joint_state", "status"]
    stream_counts = {"color_image": 0, "joint_state": 20}
    store.stream.side_effect = lambda name: mocker.MagicMock(
        count=mocker.Mock(return_value=stream_counts[name])
    )
    mocker.patch("dimos.imitation.dataprep.build._open_recording", return_value=store)
    mocker.patch(
        "dimos.imitation.dataprep.build.extract_episodes",
        return_value=[Episode(id="ep_0", start_ts=1.0, end_ts=2.0)],
    )
    writer = mocker.Mock(return_value=tmp_path)
    cfg = DataPrepConfig(
        source="recording.db",
        observation={
            "wrist": FeatureSpec(
                stream="color_image",
                field="data",
                dtype="video",
                shape=(2, 2, 3),
                names=["height", "width", "channels"],
            ),
            "state": _feature("joint_state"),
        },
        output=OutputConfig(format="hdf5", path=tmp_path / "dataset.hdf5"),
        sync=SyncConfig(anchor="wrist", rate_hz=30.0, tolerance_ms=20.0),
    )

    with pytest.raises(RuntimeError, match="stream 'color_image' has no episode data"):
        run_dataprep(cfg, writer=writer)

    writer.assert_not_called()
    store.stop.assert_called_once_with()


def test_run_dataprep_excludes_only_invalid_episode(mocker, tmp_path: Path) -> None:
    store = mocker.MagicMock()
    store.list_streams.return_value = ["joint_state", "status"]
    mocker.patch("dimos.imitation.dataprep.build._open_recording", return_value=store)
    episodes = [
        Episode(id="bad", start_ts=0.0, end_ts=1.0, task_label="pick"),
        Episode(id="good", start_ts=2.0, end_ts=3.0, task_label="pick"),
    ]
    mocker.patch("dimos.imitation.dataprep.build.extract_episodes", return_value=episodes)
    mocker.patch(
        "dimos.imitation.dataprep.build.inspect_episode_quality",
        side_effect=[
            EpisodeQualityReport(
                episode_id="bad",
                valid=False,
                mode="strict",
                rejection_reasons=["missing frame"],
            ),
            EpisodeQualityReport(episode_id="good", valid=True, mode="strict"),
        ],
    )
    sample = Sample(
        ts=2.0,
        episode_id="good",
        observation={"state": np.asarray([1.0], dtype=np.float32)},
        action={},
        task_label="pick",
        complementary_info={"is_filled": np.asarray([False])},
    )
    aligned = mocker.patch(
        "dimos.imitation.dataprep.build.iter_episode_samples", return_value=iter([sample])
    )
    received: list[Sample] = []

    def writer(samples, _output):
        received.extend(samples)
        return tmp_path

    config = DataPrepConfig(
        source="recording.db",
        observation={"state": _feature("joint_state")},
        sync=SyncConfig(anchor="state", rate_hz=1.0, tolerance_ms=20.0),
        output=OutputConfig(format="hdf5", path=tmp_path / "dataset.hdf5"),
    )

    run_dataprep(config, writer=writer)

    assert [value.episode_id for value in received] == ["good"]
    assert aligned.call_args.kwargs["episode"].id == "good"
    reports = json.loads((tmp_path / "dimos_meta.json").read_text())["quality_reports"]
    assert [(report["episode_id"], report["valid"]) for report in reports] == [
        ("bad", False),
        ("good", True),
    ]
