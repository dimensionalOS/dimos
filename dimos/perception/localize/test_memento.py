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

"""Memento's three clauses in use, on a scripted desk read at 1 Hz; no recording needed.

1. A window is answered only from what was seen inside it.
2. A window read before is answered from memory, with no detector pass.
3. A window overlapping earlier ones detects only its unread part.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from pathlib import Path
import random
import threading

import numpy as np
import pytest

from dimos.perception.localize.localize import LocalizeTrace, answer_window, memory_key
from dimos.perception.localize.localize_params import XARM_PARAMS
from dimos.perception.localize.memento import Gap, Memento, Sighting
from dimos.perception.localize.types import Localization, LocalizeParams

LABELS = ["book", "pen", "red marker", "yellow sticky notes"]
A, B = (0.60, -0.15, 0.10), (0.70, 0.25, 0.10)  # the book before and after it moves
PEN = (0.60, 0.05, 0.10)
NOTE_1, NOTE_2 = (0.80, 0.03, 0.10), (0.81, 0.23, 0.10)  # identical twins
M1, M2 = (0.78, -0.08, 0.15), (0.65, 0.11, 0.15)  # the marker before removal and on return


@dataclass(frozen=True)
class Stay:
    """One object of a label resting at one place over a closed span of seconds."""

    label: str
    at: tuple[float, float, float]
    start: float
    end: float


DESK = [
    Stay("book", A, 0, 50),
    Stay("book", B, 55, 100),
    Stay("pen", PEN, 0, 100),
    Stay("yellow sticky notes", NOTE_1, 0, 100),
    Stay("yellow sticky notes", NOTE_2, 30, 100),
    Stay("red marker", M1, 0, 20),
    Stay("red marker", M2, 60, 100),
]


class Desk:
    """A camera circling the desk, one frame a second up to ``recorded``.

    ``head`` is the newest frame a caller saw when it asked; frames up to
    ``recorded`` may arrive while it reads. ``read`` is what localize's
    detector side does with a gap: every frame inside it is detected and
    every object resting there is lifted.
    """

    def __init__(self, stays: list[Stay], head: float, recorded: float | None = None) -> None:
        self.stays = stays
        self.head = head
        self.recorded = head if recorded is None else recorded
        self.reads: list[dict[str, tuple[tuple[float, float], ...]]] = []
        self.detected: list[tuple[str, float]] = []

    def read(self, gaps: dict[str, Gap]) -> dict[str, list[Sighting]]:
        self.reads.append({label: gap.spans for label, gap in gaps.items()})
        found: dict[str, list[Sighting]] = {label: [] for label in gaps}
        for ts in (float(t) for t in range(math.floor(self.recorded) + 1)):
            for label, gap in gaps.items():
                if ts not in gap:
                    continue
                self.detected.append((label, ts))
                found[label] += [
                    _sighting(stay, ts)
                    for stay in self.stays
                    if stay.label == label and stay.start <= ts <= stay.end
                ]
        return found


def _sighting(stay: Stay, ts: float) -> Sighting:
    grid = np.stack(np.meshgrid([-0.01, 0, 0.01], [-0.01, 0, 0.01], [-0.005, 0.005]), -1)
    return Sighting(
        ts=ts,
        camera="camera",
        score=0.6,
        points=np.asarray(stay.at) + grid.reshape(-1, 3),
        viewpoint=(math.cos(0.3 * ts), math.sin(0.3 * ts), 1.0),
        detection=(stay.label, stay.at, ts),
    )


def ask(
    desk: Desk,
    memento: Memento,
    labels: list[str],
    window: tuple[float, float],
    params: LocalizeParams = XARM_PARAMS,
    traces: list[LocalizeTrace] | None = None,
) -> dict[str, list[Localization]]:
    answers = answer_window(
        labels,
        memento=memento,
        key=memory_key(params),
        window=window,
        head=desk.head,
        read=desk.read,
        params=params,
        world_frame="world",
        traces=list(traces) if traces is not None else [None] * len(labels),
    )
    return dict(zip(labels, answers, strict=True))


def places(answers: list[Localization]) -> list[tuple[float, ...]]:
    return [tuple(round(v, 3) for v in hit.position_world_xyz) for hit in answers]


def spans(answers: list[Localization]) -> list[tuple[float, float]]:
    return [(hit.first_seen_timestamp, hit.last_seen_timestamp) for hit in answers]


def evidence(answers: list[Localization]) -> list[tuple[object, ...]]:
    """Everything an answer reports except its id."""
    return [
        (
            tuple(round(v, 6) for v in hit.position_world_xyz),
            hit.first_seen_timestamp,
            hit.last_seen_timestamp,
            hit.semantic_score,
            hit.ambiguity_margin,
            hit.n_views,
            len(hit.point_cloud),
        )
        for hit in answers
    ]


@pytest.fixture
def desk() -> Desk:
    return Desk(DESK, head=100)


# Clause 1: a window is answered only from what was seen inside it


def test_a_window_answers_only_what_was_seen_inside_it(desk: Desk) -> None:
    memento = Memento()
    assert places(ask(desk, memento, ["book"], (60, 100))["book"]) == [B]
    assert places(ask(desk, memento, ["book"], (0, 50))["book"]) == [A]
    assert spans(ask(desk, memento, ["book"], (60, 100))["book"]) == [(60, 100)]


def test_a_long_window_earlier_never_leaks_into_a_later_recent_one(desk: Desk) -> None:
    memento = Memento()
    ask(desk, memento, LABELS, (0, 100))
    recent = ask(desk, memento, LABELS, (95, 100))
    assert places(recent["book"]) == [B]
    assert places(recent["red marker"]) == [M2]
    assert places(recent["pen"]) == [PEN]
    assert sorted(places(recent["yellow sticky notes"])) == sorted([NOTE_1, NOTE_2])
    assert all(hit.first_seen_timestamp >= 95 for hits in recent.values() for hit in hits)


def test_a_window_reports_nothing_seen_after_it_from_a_later_call(desk: Desk) -> None:
    memento = Memento()
    ask(desk, memento, ["book"], (90, 100))
    earlier = ask(desk, memento, ["book"], (0, 80))["book"]
    assert spans(earlier) == [(55, 80), (0, 50)]
    assert evidence(earlier) == evidence(ask(desk, Memento(), ["book"], (0, 80))["book"])


def test_an_object_moved_inside_a_window_is_one_answer_per_place(desk: Desk) -> None:
    book = ask(desk, Memento(), ["book"], (0, 100))["book"]
    assert places(book) == [B, A]
    assert spans(book) == [(55, 100), (0, 50)]


def test_two_copies_seen_together_are_two_answers(desk: Desk) -> None:
    notes = ask(desk, Memento(), ["yellow sticky notes"], (40, 60))["yellow sticky notes"]
    assert sorted(places(notes)) == sorted([NOTE_1, NOTE_2])
    assert all(hit.ambiguity_margin <= 0 for hit in notes)


def test_a_removed_object_is_gone_from_the_next_recent_window(desk: Desk) -> None:
    memento = Memento()
    assert spans(ask(desk, memento, ["red marker"], (15, 25))["red marker"]) == [(15, 20)]
    assert ask(desk, memento, ["red marker"], (25, 50))["red marker"] == []


def test_recent_windows_count_what_is_there_now() -> None:
    books = [Stay("book", (0.5 + 0.2 * k, 0.0, 0.1), 10, 20) for k in range(3)]
    desk, memento = Desk(books, head=40), Memento()
    assert ask(desk, memento, ["book"], (5, 10))["book"] == []
    assert len(ask(desk, memento, ["book"], (15, 20))["book"]) == 3
    assert ask(desk, memento, ["book"], (25, 30))["book"] == []


def test_same_place_across_windows_when_objects_swap_presence_and_absence() -> None:
    # UC2: at place A, O2 is present and O1 is absent in (0, 20), and in (60, 80) O1 is
    # present at A and O2 is absent.
    stays = [Stay("pen", A, 0, 20), Stay("book", A, 60, 80)]
    desk, memento = Desk(stays, head=100), Memento()

    visit_1 = ask(desk, memento, ["book", "pen"], (0, 20))
    assert visit_1["book"] == []
    assert places(visit_1["pen"]) == [A]

    visit_2 = ask(desk, memento, ["book", "pen"], (60, 80))
    assert places(visit_2["book"]) == [A]
    assert visit_2["pen"] == []

    # Re-querying either window comes entirely from memory with each window's own state.
    assert evidence(ask(desk, memento, ["book", "pen"], (0, 20))["pen"]) == evidence(visit_1["pen"])
    assert ask(desk, memento, ["book", "pen"], (0, 20))["book"] == []
    assert evidence(ask(desk, memento, ["book", "pen"], (60, 80))["book"]) == evidence(
        visit_2["book"]
    )
    assert ask(desk, memento, ["book", "pen"], (60, 80))["pen"] == []
    assert len(desk.reads) == 2


# Clause 2: a window read before is answered from memory


def test_asking_a_window_again_reads_nothing_and_answers_the_same(desk: Desk) -> None:
    memento = Memento()
    first_traces = [LocalizeTrace() for _ in LABELS]
    again_traces = [LocalizeTrace() for _ in LABELS]
    first = ask(desk, memento, LABELS, (0, 100), traces=first_traces)
    again = ask(desk, memento, LABELS, (0, 100), traces=again_traces)
    assert len(desk.reads) == 1
    for label, first_trace, again_trace in zip(LABELS, first_traces, again_traces, strict=True):
        assert evidence(again[label]) == evidence(first[label])
        assert [hit.instance_id for hit in again[label]] == [
            hit.instance_id for hit in first[label]
        ]
        assert again_trace.answers == first_trace.answers
        assert again_trace.spans == first_trace.spans
        for a, b in zip(again[label], first[label], strict=True):
            assert np.array_equal(a.point_cloud.points_f32(), b.point_cloud.points_f32())


def test_changing_only_what_builds_the_answer_reads_nothing(desk: Desk) -> None:
    memento = Memento()
    ask(desk, memento, ["book"], (0, 100))
    loose = replace(
        XARM_PARAMS, accept_score=0.1, min_views=1, cluster_radius_m=0.5, fuse_voxel_m=0.02
    )
    assert len(ask(desk, memento, ["book"], (0, 100), loose)["book"]) == 1  # A and B joined
    assert len(desk.reads) == 1


def test_changing_a_read_param_reads_the_window_again(desk: Desk) -> None:
    memento = Memento()
    ask(desk, memento, ["book"], (0, 100))
    ask(desk, memento, ["book"], (0, 100), replace(XARM_PARAMS, candidate_floor=0.3))
    assert len(desk.reads) == 2


def test_a_label_asked_later_reads_only_that_label(desk: Desk) -> None:
    memento = Memento()
    ask(desk, memento, ["book"], (0, 100))
    answers = ask(desk, memento, ["book", "pen"], (0, 100))
    assert desk.reads[1] == {"pen": ((0, 100),)}
    assert places(answers["book"]) == [B, A]
    assert places(answers["pen"]) == [PEN]
    again = ask(desk, memento, ["book", "pen"], (0, 100))
    assert len(desk.reads) == 2
    assert evidence(again["book"]) == evidence(answers["book"])
    assert evidence(again["pen"]) == evidence(answers["pen"])


def test_an_absent_label_already_searched_is_not_read_again_when_a_new_label_joins(
    desk: Desk,
) -> None:
    memento = Memento()
    assert ask(desk, memento, ["red marker"], (25, 50))["red marker"] == []
    combined = ask(desk, memento, ["red marker", "pen"], (25, 50))
    assert desk.reads[1] == {"pen": ((25, 50),)}
    assert combined["red marker"] == []
    assert places(combined["pen"]) == [PEN]
    again = ask(desk, memento, ["red marker", "pen"], (25, 50))
    assert len(desk.reads) == 2
    assert again["red marker"] == []
    assert evidence(again["pen"]) == evidence(combined["pen"])


def test_memory_survives_a_dump_and_load(desk: Desk, tmp_path: Path) -> None:
    memento = Memento()
    first = ask(desk, memento, LABELS, (0, 100))
    db_path = tmp_path / "dir#1" / "memento.db"
    db_path.parent.mkdir()
    memento.dump(db_path)
    loaded = Memento.load(db_path)
    again = ask(desk, loaded, LABELS, (0, 100))
    assert len(desk.reads) == 1
    for label in LABELS:
        assert evidence(again[label]) == evidence(first[label])
        assert [hit.instance_id for hit in again[label]] == [
            hit.instance_id for hit in first[label]
        ]


# Clause 3: an overlapping window detects only its unread part


def test_an_overlapping_window_reads_only_its_new_part(desk: Desk) -> None:
    memento = Memento()
    ask(desk, memento, ["book"], (10, 35))
    desk.detected.clear()
    trace = LocalizeTrace()
    book = ask(desk, memento, ["book"], (20, 50), traces=[trace])["book"]
    assert desk.reads[1] == {"book": ((35, 50),)}
    assert [ts for _, ts in desk.detected] == [float(t) for t in range(36, 51)]
    assert spans(book) == [(20, 50)]  # 20 to 35 from memory, 36 to 50 just read
    fresh_trace = LocalizeTrace()
    ask(Desk(DESK, head=100), Memento(), ["book"], (20, 50), traces=[fresh_trace])
    assert trace.answers == fresh_trace.answers


def test_an_overlapping_window_with_a_new_label_reads_each_labels_own_gap(desk: Desk) -> None:
    memento = Memento()
    ask(desk, memento, ["book"], (10, 35))
    desk.detected.clear()
    answers = ask(desk, memento, ["book", "pen"], (20, 50))
    assert desk.reads[1] == {"book": ((35, 50),), "pen": ((20, 50),)}
    assert [ts for label, ts in desk.detected if label == "book"] == [
        float(t) for t in range(36, 51)
    ]
    assert [ts for label, ts in desk.detected if label == "pen"] == [
        float(t) for t in range(20, 51)
    ]
    fresh = ask(Desk(DESK, head=100), Memento(), ["book", "pen"], (20, 50))
    assert evidence(answers["book"]) == evidence(fresh["book"])
    assert evidence(answers["pen"]) == evidence(fresh["pen"])
    again = ask(desk, memento, ["book", "pen"], (20, 50))
    assert len(desk.reads) == 2
    assert evidence(again["book"]) == evidence(answers["book"])
    assert evidence(again["pen"]) == evidence(answers["pen"])


def test_a_window_inside_what_was_read_reads_nothing(desk: Desk) -> None:
    memento = Memento()
    ask(desk, memento, ["book"], (0, 100))
    assert spans(ask(desk, memento, ["book"], (30, 40))["book"]) == [(30, 40)]
    assert len(desk.reads) == 1


def test_a_window_across_two_read_spans_reads_only_the_hole_between(desk: Desk) -> None:
    memento = Memento()
    ask(desk, memento, ["book"], (0, 20))
    ask(desk, memento, ["book"], (40, 60))
    desk.detected.clear()
    ask(desk, memento, ["book"], (10, 50))
    assert desk.reads[2] == {"book": ((20, 40),)}
    assert [ts for _, ts in desk.detected] == [float(t) for t in range(21, 40)]


def test_a_window_across_three_read_spans_reads_only_both_holes_and_coalesces(
    desk: Desk,
) -> None:
    memento = Memento()
    ask(desk, memento, ["book"], (0, 15))
    ask(desk, memento, ["book"], (30, 45))
    ask(desk, memento, ["book"], (60, 75))
    desk.detected.clear()
    book = ask(desk, memento, ["book"], (10, 70))["book"]
    assert desk.reads[3] == {"book": ((15, 30), (45, 60))}
    assert [ts for _, ts in desk.detected] == [
        *[float(t) for t in range(16, 30)],
        *[float(t) for t in range(46, 60)],
    ]
    assert spans(book) == [(55, 70), (10, 50)]
    assert len(desk.reads) == 4
    ask(desk, memento, ["book"], (0, 75))
    assert len(desk.reads) == 4


def test_no_frame_is_detected_twice_and_memory_answers_like_a_fresh_read(desk: Desk) -> None:
    memento, rng = Memento(), random.Random(7)
    for _ in range(40):
        start = float(rng.randint(0, 95))
        window = (start, min(100.0, start + rng.randint(1, 30)))
        labels = rng.sample(LABELS, rng.randint(1, len(LABELS)))
        remembered = ask(desk, memento, labels, window)
        fresh = ask(Desk(DESK, head=100), Memento(), labels, window)
        for label in labels:
            assert evidence(remembered[label]) == evidence(fresh[label])
    assert len(desk.detected) == len(set(desk.detected))


def test_a_window_past_the_newest_frame_reads_the_rest_once_it_arrives() -> None:
    desk, memento = Desk(DESK, head=50, recorded=90), Memento()
    assert ask(desk, memento, ["book"], (60, 90))["book"] == []
    assert desk.reads == []
    desk.head = 70
    assert spans(ask(desk, memento, ["book"], (60, 90))["book"]) == [(60, 70)]
    desk.head = 90
    assert spans(ask(desk, memento, ["book"], (60, 90))["book"]) == [(60, 90)]
    assert [ts for _, ts in desk.detected] == [float(t) for t in range(60, 91)]


def test_concurrent_recalls_serialize_and_never_detect_a_frame_twice(desk: Desk) -> None:
    memento = Memento()
    windows = [(0.0, 60.0), (20.0, 80.0), (40.0, 100.0), (0.0, 100.0)]
    results: dict[tuple[float, float], dict[str, list[Localization]]] = {}
    barrier = threading.Barrier(len(windows))

    def worker(w: tuple[float, float]) -> None:
        barrier.wait()
        results[w] = ask(desk, memento, ["book", "pen"], w)

    threads = [threading.Thread(target=worker, args=(w,)) for w in windows]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert len(desk.detected) == len(set(desk.detected))
    for w in windows:
        fresh = ask(Desk(DESK, head=100), Memento(), ["book", "pen"], w)
        assert evidence(results[w]["book"]) == evidence(fresh["book"])
        assert evidence(results[w]["pen"]) == evidence(fresh["pen"])


def test_numpy_scalar_timestamps_and_scores_round_trip_and_compare_as_floats(
    tmp_path: Path,
) -> None:
    memento, key = Memento(), memory_key(XARM_PARAMS)

    def read_np(gaps: dict[str, Gap]) -> dict[str, list[Sighting]]:
        return {
            "book": [
                Sighting(
                    ts=float(np.float32(t)),
                    camera="camera",
                    score=float(np.float32(0.75)),
                    points=np.asarray(A) + np.zeros((18, 3), dtype=np.float32),
                    viewpoint=(
                        float(np.float32(math.cos(t))),
                        float(np.float32(math.sin(t))),
                        float(np.float32(1.0)),
                    ),
                )
                for t in (10.0, 20.0, 80.0, 90.0)
                if np.float32(t) in gaps["book"]
            ]
        }

    # Construct Sighting with raw numpy.float32 scalars at runtime as callers might.
    s_early = _sighting(DESK[0], 10.0)
    s_late = _sighting(DESK[1], 80.0)
    s_early.ts = np.float32(10.0)  # type: ignore[assignment]
    s_early.score = np.float32(0.7)  # type: ignore[assignment]
    s_early.__post_init__()
    s_late.ts = np.float32(80.0)  # type: ignore[assignment]
    s_late.score = np.float32(0.8)  # type: ignore[assignment]
    s_late.viewpoint = (np.float32(0.0), np.float32(1.0), np.float32(1.0))  # type: ignore[assignment]
    s_late.__post_init__()

    memento.recall(
        ["book"],
        key,
        (float(np.float32(0.0)), float(np.float32(100.0))),
        float(np.float32(100.0)),
        lambda _: {"book": [s_early, s_late]},
    )
    memento.forget(np.float32(50.0))  # type: ignore[arg-type]
    memento.dump(tmp_path / "np.db")
    loaded = Memento.load(tmp_path / "np.db")
    found, _ = loaded.recall(["book"], key, (50.0, 100.0), 100.0, read_np)
    assert [s.ts for s in found["book"]] == [80.0]
    assert isinstance(found["book"][0].score, float)
    assert all(isinstance(v, float) for v in found["book"][0].viewpoint)


def test_a_failed_read_leaves_nothing_behind(desk: Desk) -> None:
    memento = Memento()

    def broken(gaps: dict[str, Gap]) -> dict[str, list[Sighting]]:
        desk.read(gaps)
        raise RuntimeError("detector died")

    with pytest.raises(RuntimeError):
        answer_window(
            ["book"],
            memento=memento,
            key=memory_key(XARM_PARAMS),
            window=(0, 50),
            head=desk.head,
            read=broken,
            params=XARM_PARAMS,
            world_frame="world",
            traces=[None],
        )
    assert places(ask(desk, memento, ["book"], (0, 50))["book"]) == [A]
    assert desk.reads[1] == {"book": ((0, 50),)}


def test_a_read_rejected_after_storing_a_sighting_leaves_nothing_behind(desk: Desk) -> None:
    memento, key = Memento(), memory_key(XARM_PARAMS)

    def stray(gaps: dict[str, Gap]) -> dict[str, list[Sighting]]:
        # the first is stored before the second, outside the gap, is refused
        return {"book": [_sighting(DESK[0], 10.0), _sighting(DESK[0], 80.0)]}

    with pytest.raises(ValueError):
        memento.recall(["book"], key, (0, 50), desk.head, stray)

    with pytest.raises(ValueError):
        memento.recall(["book"], key, (0, 50), desk.head, lambda _: {"pen": []})

    retried: list[dict[str, Gap]] = []

    def nothing(gaps: dict[str, Gap]) -> dict[str, list[Sighting]]:
        retried.append(gaps)
        return {}

    found, _ = memento.recall(["book"], key, (0, 50), desk.head, nothing)
    assert found == {"book": []}
    assert [gaps["book"].spans for gaps in retried] == [((0, 50),)]


def test_forgotten_time_is_read_again_and_only_it(desk: Desk) -> None:
    memento = Memento()
    ask(desk, memento, ["book"], (0, 100))
    memento.forget(50)
    book = ask(desk, memento, ["book"], (0, 100))["book"]
    assert desk.reads[1] == {"book": ((0, 50),)}
    assert spans(book) == [(55, 100), (0, 50)]
