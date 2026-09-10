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

"""The flat-plane gate and segment bookkeeping, on synthetic depth. No model."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from dimos.mapping.hyperspace import patches as hs, segmenter as seg
from dimos.mapping.hyperspace.segments import SEGMENT_STREAM, SegmentIngestConfig, SegmentIngestor
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image

WIDTH, HEIGHT = 96, 72
CAMERA = hs.Intrinsics(width=WIDTH, height=HEIGHT, fx=80.0, fy=80.0, cx=48.0, cy=36.0)
WALL, CHAIR = 0, 19  # ADE20K ids


def wall_depth(distance: float = 2.0) -> np.ndarray:
    """A wall square-on to the camera: every pixel at the same depth."""
    return np.full((HEIGHT, WIDTH), distance, dtype=np.float32)


def tilted_floor() -> np.ndarray:
    """A plane the camera looks down at: depth grows with the row, still flat in 3D."""
    rows = np.arange(HEIGHT)[:, None]
    # z = h / (y/fy) for a floor h below the camera, clipped to sane depths.
    y = (rows - CAMERA.cy) / CAMERA.fy
    z = np.where(y > 0.05, 1.2 / np.maximum(y, 0.05), 0.0)
    return np.broadcast_to(z, (HEIGHT, WIDTH)).astype(np.float32)


def test_flat_wall_and_tilted_floor_are_flat() -> None:
    config = seg.FlatnessConfig()
    flat, judged = seg.flatness_mask(wall_depth(), CAMERA, config)
    assert flat.mean() > 0.95 and judged.all()
    flat, judged = seg.flatness_mask(tilted_floor(), CAMERA, config)
    seen = tilted_floor() > 0
    assert flat[seen].mean() > 0.9
    assert not judged[~seen].any()


def test_far_noisy_wall_is_still_flat_but_a_near_one_is_not() -> None:
    """The limits grow with depth squared, like stereo noise does."""
    rng = np.random.default_rng(1)
    noise = rng.normal(0.0, 0.025, (HEIGHT, WIDTH)).astype(np.float32)
    far, _ = seg.flatness_mask(wall_depth(4.0) + noise, CAMERA, seg.FlatnessConfig())
    near, _ = seg.flatness_mask(wall_depth(1.0) + noise, CAMERA, seg.FlatnessConfig())
    assert far.mean() > 0.8
    assert near.mean() < 0.2


def test_a_bump_breaks_flatness_only_around_it() -> None:
    depth = wall_depth()
    depth[30:42, 40:56] -= 0.10  # a 10 cm box on the wall
    flat, _ = seg.flatness_mask(depth, CAMERA, seg.FlatnessConfig())
    assert not flat[36, 48]  # inside the box edge's reach
    assert not flat[30, 40]  # the box edge
    assert flat[5, 5] and flat[HEIGHT - 5, WIDTH - 5]  # far from it


def test_gate_demotes_only_structural_labels_off_flat() -> None:
    labels = np.full((HEIGHT, WIDTH), WALL, dtype=np.int16)
    labels[:, : WIDTH // 2] = CHAIR
    flat = np.zeros((HEIGHT, WIDTH), dtype=bool)
    flat[: HEIGHT // 2] = True
    gated = seg.apply_flatness_gate(labels, flat, {WALL})
    assert (gated[: HEIGHT // 2, WIDTH // 2 :] == WALL).all()
    assert (gated[HEIGHT // 2 :, WIDTH // 2 :] == seg.UNSURE).all()
    assert (gated[:, : WIDTH // 2] == CHAIR).all()  # chairs never gated
    judged = np.zeros((HEIGHT, WIDTH), dtype=bool)
    judged[:, : WIDTH * 3 // 4] = True
    gated = seg.apply_flatness_gate(labels, flat, {WALL}, judged)
    assert (gated[HEIGHT // 2 :, WIDTH * 3 // 4 :] == WALL).all()  # no depth: label stands
    assert (gated[HEIGHT // 2 :, WIDTH // 2 : WIDTH * 3 // 4] == seg.UNSURE).all()


def test_rle_round_trips() -> None:
    rng = np.random.default_rng(0)
    for mask in (rng.random((7, 11)) > 0.5, np.ones((3, 4), bool), np.zeros((3, 4), bool)):
        assert (seg.rle_decode(seg.rle_encode(mask), mask.shape) == mask).all()
        assert sum(seg.rle_encode(mask)) == mask.size


def test_segments_are_components_with_flat_fraction() -> None:
    labels = np.full((HEIGHT, WIDTH), WALL, dtype=np.int16)
    labels[10:30, 10:30] = CHAIR
    labels[40:60, 60:90] = CHAIR
    confidence = np.full((HEIGHT, WIDTH), 0.8, dtype=np.float32)
    flat = np.ones((HEIGHT, WIDTH), dtype=bool)
    flat[10:30, 10:30] = False
    judged = np.ones((HEIGHT, WIDTH), dtype=bool)
    judged[40:60, 60:75] = False
    segments = seg.segments_from_labels(
        labels, confidence, flat, judged, {WALL: "wall", CHAIR: "chair"}
    )
    names = [s.name for s in segments]
    assert names == ["wall", "chair", "chair"]  # largest first
    chairs = [s for s in segments if s.name == "chair"]
    assert {c.flat_fraction for c in chairs} == {0.0, 1.0}
    assert chairs[0].bbox == (60, 40, 90, 60)
    assert chairs[0].depth_fraction == 0.5
    assert seg.rle_decode(chairs[0].rle, labels.shape).sum() == 600


class StubSegmenter:
    """Labels the left half chair and the right half wall, at constant confidence."""

    names = {WALL: "wall", CHAIR: "chair"}
    structural_ids = {WALL}
    colors = seg.palette(150)
    config = seg.SegmenterConfig()

    def segment(self, rgb: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        labels = np.full(rgb.shape[:2], WALL, dtype=np.int16)
        labels[:, : rgb.shape[1] // 2] = CHAIR
        return labels, np.full(rgb.shape[:2], 0.9, dtype=np.float32)


@pytest.fixture
def store(tmp_path: Path) -> SqliteStore:
    memory = SqliteStore(path=str(tmp_path / "hyperspace.db"))
    memory.start()
    yield memory
    memory.stop()


def test_ingestor_writes_one_record_per_segment_with_embedding(store: SqliteStore) -> None:
    info = CameraInfo(width=WIDTH, height=HEIGHT, distortion_model="plumb_bob", frame_id="cam")
    info.set_K_matrix(np.array([[80.0, 0, 48.0], [0, 80.0, 36.0], [0, 0, 1.0]]))
    embeds: list[str] = []

    def embed_text(name: str) -> np.ndarray:
        embeds.append(name)
        return np.full(8, 0.5, dtype=np.float32)

    ingestor = SegmentIngestor(
        store,
        StubSegmenter(),
        SegmentIngestConfig(min_frame_interval_s=0.0),
        embed_text,  # type: ignore[arg-type]
    )
    ingestor.add_camera_info(info)
    depth = wall_depth()
    depth[30:42, 60:76] -= 0.10  # a bump on the wall half
    for ts in (10.0, 11.0):
        ingestor.add_depth(
            Image.from_numpy((depth * 1000).astype(np.uint16), frame_id="cam", ts=ts)
        )
        frame = ingestor.add_image(
            Image.from_numpy(np.full((HEIGHT, WIDTH, 3), 128, np.uint8), frame_id="cam", ts=ts)
        )
        assert frame is not None
        assert 0 < frame.result.demoted_fraction < 0.5
    records = store.stream(SEGMENT_STREAM, dict).order_by("ts").to_list()
    assert len(records) == 4  # wall + chair, twice
    assert {r.data["name"] for r in records} == {"wall", "chair"}
    assert embeds == ["chair", "wall"] or embeds == ["wall", "chair"]  # cached per label
    wall = next(r.data for r in records if r.data["name"] == "wall")
    assert wall["flat_fraction"] == 1.0  # demoted pixels are no longer wall
    assert seg.rle_decode(wall["rle"], (HEIGHT, WIDTH)).sum() == wall["area"]
