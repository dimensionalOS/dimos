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

from types import SimpleNamespace

import numpy as np
import pytest

from dimos.evals.vqa.localize_evidence import (
    InstanceEvidence,
    LocalizeEvidenceDocument,
    SightingEvidence,
    assert_evidence_matches_window,
    evidence_from_window_result,
    hydrate_views_and_hits,
    load_localize_evidence,
    write_localize_evidence,
)
from dimos.evals.vqa.primitives.cross_view_identity import SightingView
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.localize.types import Localization


def _image(ts: float) -> Image:
    rgb = np.full((40, 40, 3), 30, dtype=np.uint8)
    return Image.from_numpy(rgb, format=ImageFormat.RGB, frame_id="camera_optical", ts=ts)


def _hit(*, instance_id: str = "aaaa1111") -> Localization:
    cloud = PointCloud2.from_numpy(
        np.zeros((1, 3), dtype=np.float32), frame_id="world", timestamp=12.0
    )
    return Localization(
        instance_id=instance_id,
        semantic_score=0.7,
        ambiguity_margin=0.1,
        position_world_xyz=(1.0, 2.0, 0.5),
        frame_id="world",
        last_seen_timestamp=12.0,
        point_cloud=cloud,
        n_views=4,
    )


def test_evidence_roundtrip_includes_name_coords_and_sightings(tmp_path) -> None:
    view = SightingView(
        query="storage box",
        instance_id="aaaa1111",
        image=_image(10.0),
        bbox=(5.0, 5.0, 20.0, 20.0),
        confidence=0.7,
        timestamp=10.0,
        n_views=4,
        semantic_score=0.7,
    )
    result = SimpleNamespace(
        named_hits=(("storage box", _hit()),),
        queries=("storage box",),
        first_seen_by_instance={"aaaa1111": 10.0},
        views_by_query={"storage box": {"aaaa1111": (view,)}},
    )
    doc = evidence_from_window_result(
        result,
        dataset="recording.db",
        after=0.0,
        before=20.0,  # type: ignore[arg-type]
    )
    assert doc.instances[0].object_name == "storage box"
    assert doc.instances[0].position_world_xyz == (1.0, 2.0, 0.5)
    assert doc.instances[0].first_seen == 10.0
    assert doc.instances[0].sightings == (
        SightingEvidence(timestamp=10.0, bbox=(5.0, 5.0, 20.0, 20.0)),
    )
    path = tmp_path / "localize_evidence.json"
    write_localize_evidence(path, doc)
    loaded = load_localize_evidence(path)
    assert loaded == doc


def test_assert_evidence_rejects_window_mismatch() -> None:
    doc = LocalizeEvidenceDocument(
        dataset="recording.db",
        after=0.0,
        before=10.0,
        queries=("chair",),
        instances=(),
    )
    with pytest.raises(ValueError, match="window"):
        assert_evidence_matches_window(doc, dataset="recording.db", after=0.0, before=11.0)


def test_hydrate_rebuilds_views_from_color_frames() -> None:
    doc = LocalizeEvidenceDocument(
        dataset="recording.db",
        after=0.0,
        before=20.0,
        queries=("chair",),
        instances=(
            InstanceEvidence(
                object_name="chair",
                instance_id="bbbb2222",
                position_world_xyz=(0.5, 0.0, 0.4),
                first_seen=5.0,
                n_views=3,
                semantic_score=0.8,
                sightings=(
                    SightingEvidence(timestamp=5.0, bbox=(1.0, 1.0, 10.0, 10.0)),
                    SightingEvidence(timestamp=15.0, bbox=(2.0, 2.0, 12.0, 12.0)),
                ),
            ),
        ),
    )
    color_frames = tuple((float(t), _image(float(t))) for t in range(0, 20))
    named, first_seen, views = hydrate_views_and_hits(doc, color_frames)
    assert len(named) == 1
    assert named[0][0] == "chair"
    assert named[0][1].position_world_xyz == (0.5, 0.0, 0.4)
    assert first_seen == {"bbbb2222": 5.0}
    assert len(views["chair"]["bbbb2222"]) == 2
    assert views["chair"]["bbbb2222"][0].bbox == (1.0, 1.0, 10.0, 10.0)
