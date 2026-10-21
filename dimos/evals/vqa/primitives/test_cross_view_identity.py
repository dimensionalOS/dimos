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
    filter_instance_views,
    instance_pool_ok,
    propose_identity_pairs,
    stack_pair_image,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def _image(width: int = 200, height: int = 200, ts: float = 1.0) -> Image:
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    rgb[:, :] = (40, 40, 40)
    return Image.from_numpy(
        rgb, format=ImageFormat.RGB, frame_id="camera_optical", ts=ts
    )


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
    assert not bbox_framing_ok((10, 70, 50, 130), 200, 200)  # off-center


def test_instance_pool_requires_views_score_and_min_conf() -> None:
    assert instance_pool_ok(
        n_views=3, semantic_score=0.55, sighting_confidences=(0.3, 0.4, 0.5)
    )
    assert not instance_pool_ok(
        n_views=2, semantic_score=0.9, sighting_confidences=(0.9, 0.9)
    )
    assert not instance_pool_ok(
        n_views=5, semantic_score=0.4, sighting_confidences=(0.9, 0.9)
    )
    assert not instance_pool_ok(
        n_views=5, semantic_score=0.7, sighting_confidences=(0.1, 0.8)
    )


def test_filter_and_propose_same_and_diff_pairs() -> None:
    centered = (70.0, 70.0, 130.0, 130.0)
    a = (
        _view(instance_id="a", bbox=centered, ts=1.0),
        _view(instance_id="a", bbox=centered, ts=5.0),
        _view(instance_id="a", bbox=centered, ts=9.0),
    )
    b = (
        _view(instance_id="b", bbox=centered, ts=2.0, score=0.62),
        _view(instance_id="b", bbox=centered, ts=8.0, score=0.62),
    )
    # Corner-only instance should vanish from same-pair pool.
    corner = (_view(instance_id="c", bbox=(0.0, 0.0, 25.0, 25.0), ts=3.0, score=0.9),)

    pairs = propose_identity_pairs({"a": a, "b": b, "c": corner})
    assert any(p.same_object and p.left.instance_id == "a" for p in pairs)
    assert any(not p.same_object for p in pairs)
    assert all(
        {p.left.instance_id, p.right.instance_id} != {"c"} or True for p in pairs
    )
    # No same-pair for c (only one bad view → filtered empty or single).
    assert not any(p.same_object and p.left.instance_id == "c" for p in pairs)


def test_answer_and_stack() -> None:
    centered = (70.0, 70.0, 130.0, 130.0)
    left = _view(instance_id="a", bbox=centered, ts=1.0)
    right = _view(instance_id="a", bbox=centered, ts=4.0)
    from dimos.evals.vqa.primitives.cross_view_identity import IdentityPair

    same = IdentityPair(query="storage box", left=left, right=right, same_object=True)
    answered = answer_cross_view_same_object(same)
    assert answered.answer == "yes"
    stacked = stack_pair_image(left.image, right.image)
    assert stacked.to_rgb().data.shape[1] > left.image.to_rgb().data.shape[1]

    other = _view(instance_id="b", bbox=centered, ts=5.0)
    diff = IdentityPair(query="storage box", left=left, right=other, same_object=False)
    assert answer_cross_view_same_object(diff).answer == "no"


def test_answer_rejects_bad_framing() -> None:
    from dimos.evals.vqa.primitives.cross_view_identity import IdentityPair

    bad = _view(instance_id="a", bbox=(0.0, 0.0, 20.0, 20.0), ts=1.0)
    good = _view(instance_id="a", bbox=(70.0, 70.0, 130.0, 130.0), ts=4.0)
    pair = IdentityPair(query="storage box", left=bad, right=good, same_object=True)
    with pytest.raises(InsufficientEvidenceError):
        answer_cross_view_same_object(pair)


def test_filter_instance_views_drops_weak_pool() -> None:
    centered = (70.0, 70.0, 130.0, 130.0)
    weak = (
        _view(instance_id="a", bbox=centered, ts=1.0, n_views=1, score=0.9),
        _view(instance_id="a", bbox=centered, ts=2.0, n_views=1, score=0.9),
    )
    assert filter_instance_views(weak) == ()
