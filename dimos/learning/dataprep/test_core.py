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

from dimos.learning.dataprep.build import _write_dimos_meta, run_dataprep
from dimos.learning.dataprep.core import (
    DataPrepConfig,
    Episode,
    EpisodeExtractor,
    OutputConfig,
    StreamField,
    SyncConfig,
    extract_episodes,
    is_image_array,
    iter_episode_samples,
    resolve_field,
    summarize_lengths,
)


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


# ── resolve_field ────────────────────────────────────────────────────────────


def test_resolve_field_attribute() -> None:
    @dataclass
    class Msg:
        position: list[float]

    arr = resolve_field(Msg(position=[1.0, 2.0, 3.0]), StreamField(stream="x", field="position"))
    assert isinstance(arr, np.ndarray)
    np.testing.assert_array_equal(arr, np.array([1.0, 2.0, 3.0]))


def test_resolve_field_dict_payload() -> None:
    arr = resolve_field({"q": [4, 5]}, StreamField(stream="x", field="q"))
    np.testing.assert_array_equal(arr, np.array([4, 5]))


def test_resolve_field_none_passthrough_ndarray() -> None:
    src = np.arange(6).reshape(2, 3)
    out = resolve_field(src, StreamField(stream="x", field=None))
    assert out is src  # ndarray passes straight through


def test_resolve_field_none_unwraps_data_attr() -> None:
    @dataclass
    class Image:
        data: np.ndarray

    img = Image(data=np.ones((2, 2)))
    out = resolve_field(img, StreamField(stream="x", field=None))
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


def test_sync_basic_no_shift() -> None:
    # obs == action, shift disabled → one sample per anchor target
    store = _FakeStore(
        {
            "js": _scalar_stream([(0.0, 10.0), (1.0, 11.0), (2.0, 12.0)]),
        }
    )
    ep = Episode(id="ep_0", start_ts=0.0, end_ts=2.0)
    streams = {
        "state": StreamField(stream="js", field="position"),
        "act": StreamField(stream="js", field="position"),
    }
    sync = SyncConfig(anchor="state", rate_hz=1.0, tolerance_ms=100.0, action_shift=0)
    samples = list(
        iter_episode_samples(store, ep, streams, sync, obs_keys={"state"}, action_keys={"act"})
    )
    assert len(samples) == 3
    # action equals state at the same frame
    np.testing.assert_array_equal(samples[0].observation["state"], samples[0].action["act"])


def test_sync_action_shift_next_state() -> None:
    store = _FakeStore({"js": _scalar_stream([(0.0, 10.0), (1.0, 11.0), (2.0, 12.0)])})
    ep = Episode(id="ep_0", start_ts=0.0, end_ts=2.0)
    streams = {
        "state": StreamField(stream="js", field="position"),
        "act": StreamField(stream="js", field="position"),
    }
    sync = SyncConfig(anchor="state", rate_hz=1.0, tolerance_ms=100.0, action_shift=1)
    samples = list(
        iter_episode_samples(store, ep, streams, sync, obs_keys={"state"}, action_keys={"act"})
    )
    # 3 frames, shift 1 → 2 emitted; trailing frame dropped
    assert len(samples) == 2
    # frame 0: obs is state@0 (10), action is state@1 (11)
    np.testing.assert_array_equal(samples[0].observation["state"], [10.0])
    np.testing.assert_array_equal(samples[0].action["act"], [11.0])
    np.testing.assert_array_equal(samples[1].observation["state"], [11.0])
    np.testing.assert_array_equal(samples[1].action["act"], [12.0])


def test_sync_tolerance_skips_unmatched_frame() -> None:
    # anchor ticks every 1s, but the second stream has a big gap around t=1
    store = _FakeStore(
        {
            "anchor": _scalar_stream([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]),
            "other": _scalar_stream([(0.0, 5.0), (2.0, 7.0)]),  # nothing near t=1
        }
    )
    ep = Episode(id="ep_0", start_ts=0.0, end_ts=2.0)
    streams = {
        "anchor": StreamField(stream="anchor", field="position"),
        "other": StreamField(stream="other", field="position"),
    }
    sync = SyncConfig(anchor="anchor", rate_hz=1.0, tolerance_ms=100.0, action_shift=0)
    samples = list(iter_episode_samples(store, ep, streams, sync, obs_keys={"anchor", "other"}))
    # t=1 dropped (no `other` within 100ms) → only t=0 and t=2 survive
    assert [round(s.ts) for s in samples] == [0, 2]


def test_sync_missing_anchor_raises() -> None:
    ep = Episode(id="ep_0", start_ts=0.0, end_ts=1.0)
    streams = {"x": StreamField(stream="x", field="position")}
    sync = SyncConfig(anchor="not_there", rate_hz=1.0, tolerance_ms=10.0)
    with pytest.raises(ValueError, match="anchor"):
        list(iter_episode_samples(_FakeStore({}), ep, streams, sync))


def test_sync_empty_anchor_yields_nothing() -> None:
    store = _FakeStore({"a": []})
    ep = Episode(id="ep_0", start_ts=0.0, end_ts=1.0)
    streams = {"a": StreamField(stream="a", field="position")}
    sync = SyncConfig(anchor="a", rate_hz=1.0, tolerance_ms=10.0)
    assert list(iter_episode_samples(store, ep, streams, sync)) == []


# ── summarize_lengths ────────────────────────────────────────────────────────


def test_summarize_lengths_uniform() -> None:
    assert summarize_lengths([5, 5, 5]) == {"min": 5, "max": 5, "mean": 5.0, "uniform": True}


def test_summarize_lengths_varied() -> None:
    s = summarize_lengths([2, 4, 6])
    assert s == {"min": 2, "max": 6, "mean": 4.0, "uniform": False}


def test_summarize_lengths_empty() -> None:
    assert summarize_lengths([]) == {"min": 0, "max": 0, "mean": 0.0, "uniform": True}


# ── dimos_meta sidecar ───────────────────────────────────────────────────────


def test_dimos_meta_records_sync_and_action_shift(tmp_path: Path) -> None:
    cfg = DataPrepConfig(
        source="s.db",
        observation={"state": StreamField(stream="js", field="position")},
        action={"action": StreamField(stream="js", field="position")},
        sync=SyncConfig(anchor="state", rate_hz=14.0, tolerance_ms=80.0, action_shift=0),
        output=OutputConfig(format="lerobot", path=tmp_path, metadata={"fps": 14}),
    )
    _write_dimos_meta(tmp_path, cfg, episodes=[])

    meta = json.loads((tmp_path / "dimos_meta.json").read_text())
    assert meta["sync"]["action_shift"] == 0
    assert meta["source"] == "s.db"


def test_dimos_meta_beside_file_for_hdf5(tmp_path: Path) -> None:
    """hdf5 writer returns a FILE path; the sidecar must land beside it, not
    inside it (which would treat the .hdf5 file as a directory and crash)."""
    ds_file = tmp_path / "session.hdf5"
    ds_file.write_bytes(b"\x89HDF\r\n")  # stand-in for a real .hdf5
    cfg = DataPrepConfig(source="s.db", output=OutputConfig(format="hdf5", path=ds_file))

    _write_dimos_meta(ds_file, cfg, episodes=[])

    sidecar = tmp_path / "session.dimos_meta.json"
    assert sidecar.exists()  # beside the file, not session.hdf5/dimos_meta.json
    assert json.loads(sidecar.read_text())["format"] == "hdf5"


def test_run_dataprep_rejects_shared_obs_action_key() -> None:
    """A name in both obs and action would silently drop the obs feature when the
    two maps merge; run_dataprep must reject it before opening the store."""
    cfg = DataPrepConfig(
        source="nonexistent.db",  # never reached — the check runs first
        observation={"joints": StreamField(stream="joint_state", field="position")},
        action={"joints": StreamField(stream="joint_state", field="position")},
    )
    with pytest.raises(ValueError, match="share feature name"):
        run_dataprep(cfg)
