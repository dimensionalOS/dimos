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

from __future__ import annotations

import numpy as np
import pytest

from dimos.evals.vqa.contracts import InsufficientEvidenceError
from dimos.evals.vqa.primitives.cross_view_identity import (
    SightingView,
    answer_cross_view_same_object,
    bbox_framing_ok,
    collect_query_instances,
    filter_instance_views,
    instance_pool_ok,
    pair_case_id,
    propose_identity_pairs,
    views_from_localize,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def _image(width: int = 200, height: int = 200, ts: float = 1.0) -> Image:
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    rgb[:, :] = (40, 40, 40)
    return Image.from_numpy(rgb, format=ImageFormat.RGB, frame_id="camera_optical", ts=ts)


def _view(
    *,
    instance_id: str,
    bbox: tuple[float, float, float, float],
    ts: float,
    confidence: float = 0.6,
    n_views: int = 5,
    score: float = 0.6,
    query: str = "storage box",
) -> SightingView:
    return SightingView(
        query=query,
        instance_id=instance_id,
        image=_image(ts=ts),
        bbox=bbox,
        confidence=confidence,
        timestamp=ts,
        n_views=n_views,
        semantic_score=score,
    )


def test_bbox_framing_rejects_corner_and_tiny() -> None:
    assert bbox_framing_ok((70, 70, 130, 130), 200, 200)
    assert not bbox_framing_ok((0, 0, 20, 20), 200, 200)  # corner + tiny
    assert not bbox_framing_ok((5, 70, 40, 130), 200, 200)  # far off-center


def test_instance_pool_requires_min_views_only() -> None:
    assert instance_pool_ok(n_views=3)
    assert instance_pool_ok(n_views=5)
    assert not instance_pool_ok(n_views=2)
    assert not instance_pool_ok(n_views=2, min_views=3)
    assert instance_pool_ok(n_views=2, min_views=2)


def test_filter_and_propose_same_and_diff_pairs() -> None:
    centered = (70.0, 70.0, 130.0, 130.0)
    # Same-instance revisits ≥ MIN_SAME_PAIR_TIME_GAP_S (8s); diff only needs >1s.
    a = (
        _view(instance_id="a", bbox=centered, ts=1.0),
        _view(instance_id="a", bbox=centered, ts=10.0),
        _view(instance_id="a", bbox=centered, ts=20.0),
    )
    b = (
        _view(instance_id="b", bbox=centered, ts=3.0, score=0.62),
        _view(instance_id="b", bbox=centered, ts=5.5, score=0.62),
    )
    # Corner-only instance should vanish from same-pair pool.
    corner = (_view(instance_id="c", bbox=(0.0, 0.0, 25.0, 25.0), ts=4.0, score=0.9),)

    pairs = propose_identity_pairs({"a": a, "b": b, "c": corner})
    assert any(p.same_object and p.left.instance_id == "a" for p in pairs)
    assert any(not p.same_object for p in pairs)
    # Corner-only instance c is framing-rejected, so it is never a pair endpoint.
    assert all("c" not in {p.left.instance_id, p.right.instance_id} for p in pairs)
    assert not any(p.same_object and p.left.instance_id == "c" for p in pairs)


def test_answer_cross_view_same_object() -> None:
    centered = (70.0, 70.0, 130.0, 130.0)
    left = _view(instance_id="a", bbox=centered, ts=1.0)
    right = _view(instance_id="a", bbox=centered, ts=10.0)
    from dimos.evals.vqa.primitives.cross_view_identity import IdentityPair

    same = IdentityPair(query="storage box", left=left, right=right, same_object=True)
    answered = answer_cross_view_same_object(
        same, first_clip_duration_s=0.5, second_clip_duration_s=0.8
    )
    assert answered.answer == "yes"
    assert "first_clip" in answered.question and "t=0.0s" in answered.question

    other = _view(instance_id="b", bbox=centered, ts=12.0)
    diff = IdentityPair(query="storage box", left=left, right=other, same_object=False)
    assert (
        answer_cross_view_same_object(
            diff, first_clip_duration_s=0.5, second_clip_duration_s=0.8
        ).answer
        == "no"
    )


def test_answer_rejects_bad_framing() -> None:
    from dimos.evals.vqa.primitives.cross_view_identity import IdentityPair

    bad = _view(instance_id="a", bbox=(0.0, 0.0, 20.0, 20.0), ts=1.0)
    good = _view(instance_id="a", bbox=(70.0, 70.0, 130.0, 130.0), ts=10.0)
    pair = IdentityPair(query="storage box", left=bad, right=good, same_object=True)
    with pytest.raises(InsufficientEvidenceError):
        answer_cross_view_same_object(pair, first_clip_duration_s=0.5, second_clip_duration_s=0.8)


def test_filter_instance_views_drops_weak_pool() -> None:
    centered = (70.0, 70.0, 130.0, 130.0)
    weak = (
        _view(instance_id="a", bbox=centered, ts=1.0, n_views=1, score=0.9),
        _view(instance_id="a", bbox=centered, ts=2.0, n_views=1, score=0.9),
    )
    assert filter_instance_views(weak) == ()


def test_filter_instance_views_honors_min_views() -> None:
    centered = (70.0, 70.0, 130.0, 130.0)
    two = (
        _view(instance_id="a", bbox=centered, ts=1.0, n_views=2, score=0.9),
        _view(instance_id="a", bbox=centered, ts=4.0, n_views=2, score=0.9),
    )
    assert filter_instance_views(two, min_views=3) == ()
    assert len(filter_instance_views(two, min_views=2)) == 2


def test_diff_pairs_skip_same_timestamp_frames() -> None:
    centered = (70.0, 70.0, 130.0, 130.0)
    # Two instances co-visible in one frame only → must not emit a "no" pair.
    a = (_view(instance_id="a", bbox=centered, ts=1.0, n_views=3, score=0.9),)
    b = (_view(instance_id="b", bbox=centered, ts=1.0, n_views=3, score=0.8),)
    pairs = propose_identity_pairs({"a": a, "b": b})
    assert pairs == ()


def test_pair_case_id_sanitizes_slashes() -> None:
    from dimos.evals.vqa.primitives.cross_view_identity import IdentityPair

    centered = (70.0, 70.0, 130.0, 130.0)
    pair = IdentityPair(
        query="cup/mug",
        left=_view(instance_id="a/1", bbox=centered, ts=1.0),
        right=_view(instance_id="a/1", bbox=centered, ts=10.0),
        same_object=True,
    )
    case_id = pair_case_id(pair, 0)
    assert "/" not in case_id
    assert " " not in case_id
    assert case_id.startswith("xview-cup-mug-same-")


def test_views_from_localize_and_collect_query_instances() -> None:
    from types import SimpleNamespace

    centered = (70.0, 70.0, 130.0, 130.0)
    hit = SimpleNamespace(instance_id="inst-1", n_views=4, semantic_score=0.8)
    sighting = SimpleNamespace(image=_image(ts=3.0), bbox=centered, confidence=0.55)
    views = views_from_localize("chair", hit, [sighting])
    assert len(views) == 1
    assert views[0].instance_id == "inst-1"
    assert views[0].query == "chair"

    # Bad bbox length is skipped.
    bad = SimpleNamespace(image=_image(ts=4.0), bbox=(1.0, 2.0), confidence=0.9)
    assert views_from_localize("chair", hit, [bad]) == ()

    by_id = collect_query_instances("chair", [hit], [[sighting]])
    assert list(by_id) == ["inst-1"]

    with pytest.raises(ValueError, match="trace length mismatch"):
        collect_query_instances("chair", [hit], [])


def test_pair_case_id_and_answer_mismatched_ids() -> None:
    from dimos.evals.vqa.primitives.cross_view_identity import IdentityPair

    centered = (70.0, 70.0, 130.0, 130.0)
    left = _view(instance_id="a", bbox=centered, ts=1.0)
    right = _view(instance_id="b", bbox=centered, ts=10.0)
    bad_same = IdentityPair(query="box", left=left, right=right, same_object=True)
    with pytest.raises(InsufficientEvidenceError, match="mismatched instance"):
        answer_cross_view_same_object(
            bad_same, first_clip_duration_s=0.5, second_clip_duration_s=0.8
        )

    twin = _view(instance_id="a", bbox=centered, ts=12.0)
    bad_diff = IdentityPair(query="box", left=left, right=twin, same_object=False)
    with pytest.raises(InsufficientEvidenceError, match="identical instance"):
        answer_cross_view_same_object(
            bad_diff, first_clip_duration_s=0.5, second_clip_duration_s=0.8
        )

    ok = IdentityPair(query="box", left=left, right=twin, same_object=True)
    assert pair_case_id(ok, 0).startswith("xview-box-same-")

    mismatched_query = IdentityPair(
        query="box",
        left=left,
        right=_view(instance_id="a", bbox=centered, ts=6.0, query="table"),
        same_object=True,
    )
    with pytest.raises(InsufficientEvidenceError, match="same category"):
        answer_cross_view_same_object(
            mismatched_query, first_clip_duration_s=0.5, second_clip_duration_s=0.8
        )
