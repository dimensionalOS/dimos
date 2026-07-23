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

"""Marker-map fusion for `dimos map global --markers-out`.

Constructed known-truth detections: three marker_ids, each seen across three
separate tracks, with two far outlier glimpses injected for one id. The verified
robust estimator must fuse each id to ONE canonical map_T_tag pose that lands on
truth and down-weights the outliers a naive mean cannot.
"""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest
import rerun as rr

from dimos.mapping.utils.cli import map as map_cli
from dimos.mapping.utils.cli.map import _fuse_marker_map, _write_marker_map
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3

# marker_id -> (translation_m, rotation_xyzw) known truth we fuse back toward.
TRUTH: dict[int, tuple[tuple[float, float, float], tuple[float, float, float, float]]] = {
    2: ((1.0, 2.0, 0.5), (0.0, 0.0, 0.0, 1.0)),
    3: ((-4.0, 0.0, 0.3), (0.1010, 0.2020, 0.0, 0.9740)),
    4: ((5.0, -3.0, 0.2), (0.0, 0.3030, 0.1010, 0.9476)),
}
OUTLIER_ID = 2  # this id's two outlier glimpses must be down-weighted, not dropped
OUTLIER_OFFSET_M = np.array([2.0, -1.5, 0.0])  # ~2.5 m away — far past huber_delta (0.05 m)


def _normalize(q: np.ndarray) -> np.ndarray:
    unit: np.ndarray = q / np.linalg.norm(q)
    return unit


def _make_detection(
    marker_id: int, ts: float, translation: np.ndarray, rotation: np.ndarray
) -> Any:
    """A minimal stand-in for one Observation[Detection3DMarker] carrying only the
    fields `_fuse_marker_map` reads (real Vector3/Quaternion pose, ts, marker_id)."""
    return SimpleNamespace(
        ts=float(ts),
        data=SimpleNamespace(
            marker_id=marker_id,
            center=Vector3(float(translation[0]), float(translation[1]), float(translation[2])),
            orientation=Quaternion(
                float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3])
            ),
        ),
    )


def _build_detections(seed: int) -> list[Any]:
    """Per id: 3 tracks x 5 tightly-noised glimpses of truth; for OUTLIER_ID, two of
    those are replaced with a far, mirror-flipped pose (the outliers to reject)."""
    rng = np.random.default_rng(seed)
    dets: list[Any] = []
    for marker_id, (trans, quat) in TRUTH.items():
        trans_v = np.asarray(trans, dtype=np.float64)
        quat_v = np.asarray(quat, dtype=np.float64)
        glimpses: list[tuple[float, np.ndarray, np.ndarray]] = []
        for track in range(3):
            ts = track * 10.0  # tracks are >> cluster gap apart in time
            for _ in range(5):
                ts += 0.2
                glimpses.append(
                    (
                        ts,
                        trans_v + rng.normal(0.0, 0.01, 3),
                        _normalize(quat_v + rng.normal(0.0, 0.005, 4)),
                    )
                )
        if marker_id == OUTLIER_ID:
            flipped = _normalize(np.array([quat_v[3], quat_v[0], -quat_v[1], -quat_v[2]]))
            for k in range(2):
                glimpses[k] = (glimpses[k][0], trans_v + OUTLIER_OFFSET_M, flipped)
        dets.extend(_make_detection(marker_id, ts, t, q) for (ts, t, q) in glimpses)
    return dets


def test_fuse_one_canonical_pose_per_id_downweights_outlier() -> None:
    """Each marker_id fuses to exactly ONE pose on truth; the outlier id's robust
    translation beats its naive (outlier-poisoned) mean while keeping every glimpse."""
    dets = _build_detections(seed=7)

    fused = _fuse_marker_map(dets, graph=None)

    assert set(fused) == set(TRUTH)  # one canonical entry per id, no per-track duplicates
    for marker_id, (trans_true, _quat) in TRUTH.items():
        pose, n = fused[marker_id]
        assert len(pose) == 7
        assert n == 15  # all glimpses fused (3 tracks x 5), outliers down-weighted not dropped
        err = float(np.linalg.norm(np.array(pose[:3]) - np.array(trans_true)))
        assert err < 0.05, f"id {marker_id}: fused translation off truth by {err:.3f} m"

    outlier_truth = np.array(TRUTH[OUTLIER_ID][0])
    id_trans = np.array(
        [
            [d.data.center.x, d.data.center.y, d.data.center.z]
            for d in dets
            if d.data.marker_id == OUTLIER_ID
        ]
    )
    naive_err = float(np.linalg.norm(id_trans.mean(axis=0) - outlier_truth))
    robust_err = float(np.linalg.norm(np.array(fused[OUTLIER_ID][0][:3]) - outlier_truth))
    assert naive_err > 0.2  # two far glimpses drag the naive mean well off truth
    assert robust_err < 0.05  # the robust estimate stays on truth
    assert robust_err < 0.25 * naive_err  # and is far better than the naive mean


def test_marker_map_round_trips_through_reloc_loader(tmp_path: Path) -> None:
    """The written JSON is the map_T_tag schema load_marker_map reads back to one
    map_T_marker Transform per id, translations matching the fused poses exactly."""
    from dimos.perception.fiducial.fiducial_relocalization import load_marker_map

    dets = _build_detections(seed=7)
    fused = _fuse_marker_map(dets, graph=None)
    path = tmp_path / "site.marker_map.json"

    _write_marker_map(path, fused, source="unit_test.db")

    doc = json.loads(path.read_text())
    assert doc["meta"]["schema"] == "map_T_tag"
    assert doc["meta"]["source_recording"] == "unit_test.db"
    assert doc["meta"]["n_detections_fused"] == {str(m): 15 for m in TRUTH}
    assert set(doc["markers"]) == {str(m) for m in TRUTH}
    for entry in doc["markers"].values():
        assert len(entry["translation"]) == 3
        assert len(entry["rotation"]) == 4

    loaded = load_marker_map(path)  # the real fiducial-prior loader
    assert set(loaded) == set(TRUTH)
    for marker_id, transform in loaded.items():
        pose = fused[marker_id][0]
        assert (transform.translation.x, transform.translation.y, transform.translation.z) == pose[
            :3
        ]


def test_canonical_markers_drawn_as_distinct_entity(monkeypatch: pytest.MonkeyPatch) -> None:
    """_log_reconstruction draws the fused poses as world/pgo_map/markers_canonical
    — one box per marker_id, id= labels — additively (per-track boxes still drawn)."""
    calls: list[dict[str, Any]] = []

    def _spy(
        prefix: str,
        centers: Any,
        quats: Any,
        *,
        fill_half: Any,
        outline_half: Any,
        colors: Any,
        labels: Any,
    ) -> None:
        calls.append({"prefix": prefix, "centers": centers, "labels": labels})

    monkeypatch.setattr(rr, "send_blueprint", lambda *a, **k: None)  # map.py's `rr` is this module
    monkeypatch.setattr(map_cli, "_log_markers", _spy)

    dets = _build_detections(seed=7)
    # Per-track draw reads track_id; give the stand-ins one alongside the real pose.
    marker_dets: list[Any] = [
        SimpleNamespace(
            ts=d.ts,
            data=SimpleNamespace(
                marker_id=d.data.marker_id,
                track_id=0,
                center=d.data.center,
                orientation=d.data.orientation,
            ),
        )
        for d in dets
    ]
    canonical = {mid: pose for mid, (pose, _n) in _fuse_marker_map(dets, graph=None).items()}

    map_cli._log_reconstruction(
        voxel=0.05,
        global_map=None,
        path=[],
        pgo_map=None,
        full_pgo_map=None,
        pgo_path=[],
        graph=None,
        marker_dets=marker_dets,
        marker_size=0.1,
        canonical_markers=canonical,
    )

    prefixes = [c["prefix"] for c in calls]
    assert "world/raw_map/markers" in prefixes  # per-track draw stays (additive)
    canonical_calls = [c for c in calls if c["prefix"] == "world/pgo_map/markers_canonical"]
    assert len(canonical_calls) == 1
    drawn = canonical_calls[0]
    assert len(drawn["centers"]) == len(TRUTH)  # exactly one box per marker_id
    assert drawn["labels"] == [f"id={m}" for m in sorted(TRUTH)]
