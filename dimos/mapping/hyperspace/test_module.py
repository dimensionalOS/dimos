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

"""Ingest and query through a real memory store, with a stub model: frames
taken from known poses around a planted object go in as keyframes + patch
vectors, and asking for the object lights up its voxel."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from dimos.mapping.hyperspace import patches as hs, segmenter as seg
from dimos.mapping.hyperspace.ingest import (
    KEYFRAME_STREAM,
    PATCH_STREAM,
    IngestConfig,
    PatchIngestor,
)
from dimos.mapping.hyperspace.query import HyperspaceQuery
from dimos.mapping.hyperspace.segments import SEGMENT_STREAM
from dimos.memory.store.sqlite import SqliteStore
from dimos.models.embedding.base import Embedding
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

SIDE = 8  # 8x8 patch grid: small, but the same code path as 24x24
DIM = 8
WIDTH, HEIGHT = 64, 48
CAMERA = "camera_optical"
WORLD = "odom"
OBJECT = np.array([3.0, 2.0, 0.5])


class StubModel:
    """A 'model' whose patch under the object points along axis 0, everything
    else along axis 1, and whose text for "object" is axis 0."""

    patches_per_side = SIDE
    dim = DIM

    def __init__(self) -> None:
        self.pixel: tuple[float, float] | None = None

    def embed_patches(self, image: Image) -> list[np.ndarray]:
        grid = np.zeros((SIDE * SIDE, DIM), dtype=np.float32)
        grid[:, 1] = 1.0
        assert self.pixel is not None
        u, v = self.pixel
        col, row = int(u * SIDE / WIDTH), int(v * SIDE / HEIGHT)
        grid[row * SIDE + col] = 0.0
        grid[row * SIDE + col, 0] = 1.0
        return [grid]

    @staticmethod
    def embed_text(text: str) -> np.ndarray:
        vector = np.zeros(DIM, dtype=np.float32)
        vector[0 if text == "object" else 1] = 1.0
        return vector


def look_at(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    forward = target - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack([right, down, forward])
    pose[:3, 3] = position
    return pose


def quaternion_of(matrix: np.ndarray) -> tuple[float, float, float, float]:
    m = matrix
    w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    return x, y, z, w


def camera_info() -> CameraInfo:
    info = CameraInfo(width=WIDTH, height=HEIGHT, distortion_model="plumb_bob", frame_id=CAMERA)
    info.set_K_matrix(np.array([[48.0, 0.0, 32.0], [0.0, 48.0, 24.0], [0.0, 0.0, 1.0]]))
    return info


@pytest.fixture
def store(tmp_path: Path) -> SqliteStore:
    memory = SqliteStore(path=str(tmp_path / "hyperspace.db"))
    memory.start()
    yield memory
    memory.stop()


def fill(store: SqliteStore, poses: list[np.ndarray], *, with_depth: bool = True) -> PatchIngestor:
    model = StubModel()
    config = IngestConfig(
        gate=hs.KeyframeGateConfig(
            buffer_len=1, max_angular_velocity=None, max_dark_fraction=None, min_interval=None
        ),
        min_frame_interval_s=0.0,
    )
    ingestor = PatchIngestor(store, model, config)  # type: ignore[arg-type]
    ingestor.add_camera_info(camera_info())
    for index, pose in enumerate(poses):
        ts = 10.0 + index
        x, y, z, w = quaternion_of(pose[:3, :3])
        transform = Transform(
            translation=Vector3(*pose[:3, 3]),
            rotation=Quaternion(x, y, z, w),
            frame_id=WORLD,
            child_frame_id=CAMERA,
            ts=ts,
        )
        ingestor.add_tf(TFMessage(transform), ts=ts)
        local = np.linalg.inv(pose) @ np.append(OBJECT, 1.0)
        depth = float(local[2])
        model.pixel = (local[0] / local[2] * 48.0 + 32.0, local[1] / local[2] * 48.0 + 24.0)
        if with_depth:
            ingestor.add_depth(
                Image.from_numpy(
                    np.full((HEIGHT, WIDTH), int(depth * 1000), dtype=np.uint16),
                    frame_id=CAMERA,
                    ts=ts,
                )
            )
        ingestor.add_image(
            Image.from_numpy(
                np.full((HEIGHT, WIDTH, 3), 128, dtype=np.uint8), frame_id=CAMERA, ts=ts
            )
        )
    ingestor.flush()
    return ingestor


def ring(count: int, radius: float) -> list[np.ndarray]:
    return [
        look_at(OBJECT + np.array([radius * math.cos(a), radius * math.sin(a), 0.3]), OBJECT)
        for a in (i / count * math.tau for i in range(count))
    ]


def voxel_of(point: np.ndarray, size: float) -> tuple[int, ...]:
    return tuple(int(v) for v in np.floor(point / size))


def test_ingest_writes_keyframes_and_one_vector_per_patch(store: SqliteStore) -> None:
    ingestor = fill(store, ring(3, 2.5))
    assert ingestor.stats["kept"] == 3
    assert store.stream(KEYFRAME_STREAM, dict).count() == 3
    assert store.stream(PATCH_STREAM, dict).count() == 3 * SIDE * SIDE
    first = store.stream(KEYFRAME_STREAM, dict).order_by("ts").first()
    assert first.data["camera_frame"] == CAMERA
    assert first.data["grid"].shape == (SIDE * SIDE, DIM)
    assert np.isfinite(first.data["patch_depth"]).all()


def test_query_lights_up_the_object_voxel(store: SqliteStore) -> None:
    fill(store, ring(3, 2.5))
    engine = HyperspaceQuery(
        store,
        StubModel.embed_text,
        hs.QueryConfig(hot_threshold=0.3, background_prompts=["background"]),
        world_frame=WORLD,
        voxel_size=0.1,
    )
    answer = engine.answer("object", 1)
    assert answer["voxels"] > 0, answer["stats"]
    assert answer["stats"]["keyframes_placed"] == 3
    best = np.asarray(answer["best"][0]["xyz"])
    # An 8 px patch at 2.5 m is ~0.4 m; the peak sits within that of the point.
    assert np.abs(best - OBJECT).max() <= 0.5, best
    # Only the object's patch is hot in each frame: one hot patch per keyframe.
    assert answer["stats"]["hot_patches"] == 3


@pytest.mark.xfail(
    strict=True,
    reason="loop closure is parked (Jeff, 2026-09-11): the transform buffer keeps both "
    "the stale and the corrected transform for a stamp, and the lookup breaks ties on "
    "timestamp rather than write order. Fixing it belongs in TBuffer, in core.",
)
def test_rewriting_tf_moves_the_answer(store: SqliteStore) -> None:
    poses = ring(3, 2.5)
    ingestor = fill(store, poses)
    engine = HyperspaceQuery(
        store,
        StubModel.embed_text,
        hs.QueryConfig(hot_threshold=0.3, background_prompts=["background"]),
        WORLD,
        0.1,
    )
    before = np.asarray(engine.answer("object", 1)["best"][0]["xyz"])
    # Loop closure: every camera was really 1 m further along x. Re-publish tf.
    for index, pose in enumerate(poses):
        ts = 10.0 + index
        x, y, z, w = quaternion_of(pose[:3, :3])
        ingestor.add_tf(
            TFMessage(
                Transform(
                    translation=Vector3(pose[0, 3] + 1.0, pose[1, 3], pose[2, 3]),
                    rotation=Quaternion(x, y, z, w),
                    frame_id=WORLD,
                    child_frame_id=CAMERA,
                    ts=ts,
                )
            ),
            ts=ts,
        )
    after = np.asarray(engine.answer("object", 2)["best"][0]["xyz"])
    assert abs((after - before)[0] - 1.0) < 0.15, (before, after)


class CountingStream:
    """Forwards to a real stream, tallying the observations pulled from it."""

    def __init__(self, inner: object, tally: list[int]) -> None:
        self.inner, self.tally = inner, tally

    def __getattr__(self, name: str) -> object:
        attribute = getattr(self.inner, name)
        if not callable(attribute):
            return attribute

        def call(*args: object, **kwargs: object) -> object:
            result = attribute(*args, **kwargs)
            return CountingStream(result, self.tally) if hasattr(result, "__iter__") else result

        return call

    def __iter__(self):  # type: ignore[no-untyped-def]
        for observation in self.inner:
            self.tally[0] += 1
            yield observation


def test_tf_is_read_once_not_on_every_query(store: SqliteStore) -> None:
    ingestor = fill(store, ring(3, 2.5))
    engine = HyperspaceQuery(store, StubModel.embed_text, hs.QueryConfig(), WORLD, 0.1)
    engine.read_tf()
    tally = [0]
    original = store.stream
    store.stream = lambda *a, **k: CountingStream(original(*a, **k), tally)  # type: ignore[method-assign]
    try:
        engine.read_tf()
        assert tally[0] <= 1, "an unchanged tf stream should not be re-scanned"
        ingestor.add_tf(
            TFMessage(
                Transform(
                    translation=Vector3(9.0, 0.0, 0.0),
                    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                    frame_id=WORLD,
                    child_frame_id="extra",
                    ts=99.0,
                )
            ),
            ts=99.0,
        )
        tally[0] = 0
        engine.read_tf()
        # The one new observation is read (plus the row the scan stops on).
        assert tally[0] <= 2, tally[0]
        assert engine.tf.get(WORLD, "extra", 99.0, warn=False) is not None
    finally:
        store.stream = original  # type: ignore[method-assign]


def test_live_transforms_land_in_the_buffer_the_answers_read(store: SqliteStore) -> None:
    fill(store, ring(3, 2.5))
    engine = HyperspaceQuery(store, StubModel.embed_text, hs.QueryConfig(), WORLD, 0.1)
    engine.placer(WORLD)  # reads what the store holds
    # What Hyperspace.handle_tf does with a transform published while running.
    engine.tf.receive_tfmessage(
        TFMessage(
            Transform(
                translation=Vector3(9.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id=WORLD,
                child_frame_id="extra",
                ts=99.0,
            )
        )
    )
    assert engine.tf.get(WORLD, "extra", 99.0, warn=False) is not None


def test_scene_voxels_come_from_depth_thumbnails(store: SqliteStore) -> None:
    fill(store, ring(3, 2.5))
    engine = HyperspaceQuery(store, StubModel.embed_text, hs.QueryConfig(), WORLD, 0.1)
    scene = engine.scene_voxels(min_samples=1)
    assert scene
    centres = (np.asarray([i for i, _ in scene], dtype=float) + 0.5) * 0.1
    # A flat depth wall at the object's range: its voxels sit around the object.
    assert np.linalg.norm(centres.mean(axis=0) - OBJECT) < 1.5


def test_without_depth_nothing_is_placed(store: SqliteStore) -> None:
    fill(store, ring(3, 2.5), with_depth=False)
    engine = HyperspaceQuery(
        store,
        StubModel.embed_text,
        hs.QueryConfig(hot_threshold=0.3, background_prompts=["background"]),
        WORLD,
        0.1,
    )
    answer = engine.answer("object", 1)
    assert answer["voxels"] == 0
    assert answer["stats"]["hot_patches_without_depth"] == answer["stats"]["hot_patches"] == 3


def add_segment(store: SqliteStore, pose: np.ndarray, ts: float, name: str, cell: int) -> None:
    """One segment record covering ``cell`` of the grid at the object's depth."""
    depth = float((np.linalg.inv(pose) @ np.append(OBJECT, 1.0))[2])
    segment = seg.Segment(
        label=1,
        name=name,
        confidence=0.8,
        area=48,
        bbox=(0, 0, 8, 6),
        depth_fraction=1.0,
        flat_fraction=0.0,
        rle=[],
    )
    camera = hs.Intrinsics(width=WIDTH, height=HEIGHT, fx=48.0, fy=48.0, cx=32.0, cy=24.0)
    store.stream(SEGMENT_STREAM, dict).append(
        seg.segment_record(
            segment,
            camera_frame=CAMERA,
            ts=ts,
            width=WIDTH,
            height=HEIGHT,
            cells=[[cell, 1.0, depth]],
            grid=(SIDE, SIDE),
            intrinsics=camera.__dict__,
        ),
        ts=ts,
        tags={"camera_frame": CAMERA, "name": name},
        embedding=Embedding(vector=StubModel.embed_text(name)),
    )


def test_segments_add_a_red_channel_on_top_of_the_patches(store: SqliteStore) -> None:
    poses = ring(3, 2.5)
    ingestor = fill(store, poses)
    for index, pose in enumerate(poses):
        local = np.linalg.inv(pose) @ np.append(OBJECT, 1.0)
        u, v = local[0] / local[2] * 48.0 + 32.0, local[1] / local[2] * 48.0 + 24.0
        cell = int(v * SIDE / HEIGHT) * SIDE + int(u * SIDE / WIDTH)
        add_segment(store, pose, 10.0 + index, "object", cell)  # the object's own cell
        add_segment(store, pose, 10.0 + index, "other", (cell + 3) % (SIDE * SIDE))
    # One stray "object" segment on a cell no patch lit: a segment-only region.
    add_segment(store, poses[0], 10.0, "object", (cell + 1) % (SIDE * SIDE))
    # Two labels only, so z-scores are +-1: the floor has to sit below that.
    config = hs.QueryConfig(hot_threshold=0.3, background_prompts=["background"], segment_min_z=0.5)
    engine = HyperspaceQuery(store, StubModel.embed_text, config, world_frame=WORLD)
    answer = engine.answer("object", 1)
    stats = answer["stats"]
    # "other" embeds along axis 1: cosine 0 with the query, so only the four
    # "object" segments (one cell each) are read and become hot.
    assert stats["segment_labels"] == ["object"]
    assert stats["segment_hot_patches"] == 4
    assert stats["segment_read"] == 4
    assert stats["voxels_in_both"] > 0
    result: hs.Heatmap = answer["heatmap"]
    best_index, best_score = result.voxels[0]
    assert best_score == 1.0
    assert np.abs(np.asarray(answer["best"][0]["xyz"]) - OBJECT).max() <= 0.5
    # The winner is lit by both channels; the stray segment's voxels rank below it.
    patch_score, segment_score = result.channels[best_index]
    assert patch_score > 0 and segment_score > 0
    single = [s for e, s in result.channels.values() if e == 0]
    assert single and max(single) < best_score
    # Turning the channel off gives the patch-only answer back.
    off = HyperspaceQuery(
        store,
        StubModel.embed_text,
        hs.QueryConfig(hot_threshold=0.3, background_prompts=["background"], segment_weight=0),
        world_frame=WORLD,
    ).answer("object", 2)
    assert "segment_hot_patches" not in off["stats"]
    assert not off["heatmap"].channels
    del ingestor


class StubEnsemble:
    """Two stub members on different grids. Member A (8x8) sees the object;
    member B (4x4) sees it too but also hallucinates a hot cell in the top-left
    corner that A does not have, so a per-cell minimum should drop it."""

    specs = ["stub-a", "stub-b@16"]
    tags = ["stub-a", "stub-b-16"]

    def __init__(self) -> None:
        self.pixel: tuple[float, float] | None = None

    def _grid(self, side: int, hallucinate: bool) -> np.ndarray:
        grid = np.zeros((side * side, DIM), dtype=np.float32)
        grid[:, 1] = 1.0
        assert self.pixel is not None
        u, v = self.pixel
        col, row = int(u * side / WIDTH), int(v * side / HEIGHT)
        grid[row * side + col] = 0.0
        grid[row * side + col, 0] = 1.0
        if hallucinate:
            grid[0] = 0.0
            grid[0, 0] = 1.0
        return grid

    def embed_grids(self, image: Image) -> list[tuple[np.ndarray, tuple[int, int]]]:
        return [(self._grid(8, False), (8, 8)), (self._grid(4, True), (4, 4))]

    @staticmethod
    def embed_text(text: str) -> list[np.ndarray]:
        return [StubModel.embed_text(text), StubModel.embed_text(text)]


def fill_ensemble(store: SqliteStore, poses: list[np.ndarray]) -> PatchIngestor:
    model = StubEnsemble()
    config = IngestConfig(
        gate=hs.KeyframeGateConfig(
            buffer_len=1, max_angular_velocity=None, max_dark_fraction=None, min_interval=None
        ),
        min_frame_interval_s=0.0,
    )
    ingestor = PatchIngestor(store, model, config)  # type: ignore[arg-type]
    ingestor.add_camera_info(camera_info())
    for index, pose in enumerate(poses):
        ts = 10.0 + index
        x, y, z, w = quaternion_of(pose[:3, :3])
        transform = Transform(
            translation=Vector3(*pose[:3, 3]),
            rotation=Quaternion(x, y, z, w),
            frame_id=WORLD,
            child_frame_id=CAMERA,
            ts=ts,
        )
        ingestor.add_tf(TFMessage(transform), ts=ts)
        local = np.linalg.inv(pose) @ np.append(OBJECT, 1.0)
        model.pixel = (local[0] / local[2] * 48.0 + 32.0, local[1] / local[2] * 48.0 + 24.0)
        ingestor.add_depth(
            Image.from_numpy(
                np.full((HEIGHT, WIDTH), int(local[2] * 1000), dtype=np.uint16),
                frame_id=CAMERA,
                ts=ts,
            )
        )
        ingestor.add_image(
            Image.from_numpy(
                np.full((HEIGHT, WIDTH, 3), 128, dtype=np.uint8), frame_id=CAMERA, ts=ts
            )
        )
    ingestor.flush()
    return ingestor


def test_cell_matrix_spreads_and_averages_exactly() -> None:
    identity = hs.cell_matrix((8, 8), (8, 8))
    assert np.allclose(identity, np.eye(64))
    up = hs.cell_matrix((2, 2), (4, 4))  # each source cell covers a 2x2 block of targets
    assert up.shape == (16, 4)
    assert np.allclose(up.sum(axis=1), 1.0)
    assert up[0, 0] == 1.0 and up[15, 3] == 1.0
    down = hs.cell_matrix((4, 4), (2, 2))  # each target averages a 2x2 block of sources
    assert np.allclose(down[0, [0, 1, 4, 5]], 0.25)
    odd = hs.cell_matrix((14, 14), (24, 24))  # grids that do not divide each other
    assert odd.shape == (576, 196)
    assert np.allclose(odd.sum(axis=1), 1.0)


def test_pool_cells() -> None:
    a = np.array([0.5, 0.1, 0.0], dtype=np.float32)
    b = np.array([0.4, 0.3, 0.2], dtype=np.float32)
    c = np.array([0.6, 0.0, 0.1], dtype=np.float32)
    assert np.allclose(hs.pool_cells([a, b, c], "min"), [0.4, 0.0, 0.0])
    assert np.allclose(hs.pool_cells([a, b, c], "2nd"), [0.5, 0.1, 0.1])
    assert np.allclose(hs.pool_cells([a, b, c], "mean"), [0.5, 0.4 / 3, 0.1])
    assert np.allclose(hs.pool_cells([a], "2nd"), a)  # one member: nothing to pool
    with pytest.raises(ValueError):
        hs.pool_cells([a, b], "median")


def test_ensemble_keyframes_carry_every_member_and_pool_with_a_minimum(store: SqliteStore) -> None:
    ingestor = fill_ensemble(store, ring(3, 2.5))
    assert ingestor.stats["kept"] == 3
    first = next(iter(store.stream(KEYFRAME_STREAM, dict).order_by("ts"))).data
    assert first["members"] == ["stub-a", "stub-b-16"]
    assert first["member_specs"] == ["stub-a", "stub-b@16"]
    assert first["grid_shapes"] == [[8, 8], [4, 4]]
    assert (first["rows"], first["cols"]) == (24, 24)
    assert len(first["patch_depth"]) == 24 * 24
    # the vector index holds the primary member only, on its own 8x8 grid
    assert store.stream(PATCH_STREAM, dict).count() == 3 * 64

    config = hs.QueryConfig(
        structural_gate=False, segment_weight=0.0, pool="min", pooled_hot_threshold=0.005
    )
    engine = HyperspaceQuery(store, StubEnsemble.embed_text, config, world_frame=WORLD)
    assert engine.members() == ["stub-a", "stub-b-16"]
    result = engine.heatmap("object")
    assert result.voxels, "the object every member sees must light up"
    best = (np.asarray(result.voxels[0][0]) + 0.5) * result.voxel_size
    assert np.linalg.norm(best - OBJECT) < 0.35
    # member B's corner hallucination is not in member A, so the minimum has no
    # hot cell there: nothing is placed along the top-left rays
    hot, _ = engine.hot_patches(StubEnsemble.embed_text("object"))
    assert hot and all(h.patch != 0 for h in hot)

    # "2nd" of two members is the maximum: the hallucination comes back
    engine.config.pool = "2nd"
    hot, _ = engine.hot_patches(StubEnsemble.embed_text("object"))
    assert any(h.patch == 0 for h in hot)

    # a query side with the wrong number of text towers is refused
    with pytest.raises(ValueError):
        engine.hot_patches([StubModel.embed_text("object")])


def test_member_specs_and_tags() -> None:
    from dimos.mapping.hyperspace.embedder import member_tag, parse_member

    assert parse_member("google/siglip2-base-patch16-naflex@576") == (
        "google/siglip2-base-patch16-naflex",
        576,
    )
    assert parse_member("/models/siglip2-so400m-patch16-384") == (
        "/models/siglip2-so400m-patch16-384",
        None,
    )
    assert member_tag("google/siglip2-base-patch16-naflex@576") == "base-patch16-naflex-576"
    assert member_tag("/models/siglip2-so400m-patch16-384") == "so400m-patch16-384"
