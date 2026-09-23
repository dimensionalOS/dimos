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

import itertools
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch
import typer

from dimos.mapping.hyperspace import cli, embedder, patches as hs, segmenter as seg
from dimos.mapping.hyperspace.ingest import (
    COMPLETE_STREAM,
    KEYFRAME_STREAM,
    PATCH_STREAM,
    IngestConfig,
    PatchIngestor,
    info_stream_for,
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
from dimos.msgs.std_msgs.String import String
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
    # A real model always names itself; the stream of its vectors is named after it.
    specs = ["stub"]
    tags = ["stub"]

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


def fill(
    store: SqliteStore,
    poses: list[np.ndarray],
    *,
    with_depth: bool = True,
    copy_tf: bool = True,
    flat: bool = False,
    start_ts: float = 10.0,
) -> PatchIngestor:
    model = StubModel()
    config = IngestConfig(
        gate=hs.KeyframeGateConfig(
            lookahead=0,
            # Every distinct pose is a keyframe here: these tests are about what an
            # ingest writes, not about the tuned novelty of a real recording.
            novelty_threshold=0.0,
            patch_novelty_threshold=None,
            max_angular_velocity=None,
            max_dark_fraction=None,
            min_interval=None,
        ),
        min_frame_interval_s=0.0,
        flat=flat,
    )
    ingestor = PatchIngestor(store, model, config, copy_tf=copy_tf)  # type: ignore[arg-type]
    ingestor.add_camera_info(camera_info())
    for index, pose in enumerate(poses):
        ts = start_ts + index
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
    assert store.stream(cli.patch_stream_for("", "stub"), dict).count() == 3 * SIDE * SIDE
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
            lookahead=0,
            # Every distinct pose is a keyframe here: these tests are about what an
            # ingest writes, not about the tuned novelty of a real recording.
            novelty_threshold=0.0,
            patch_novelty_threshold=None,
            max_angular_velocity=None,
            max_dark_fraction=None,
            min_interval=None,
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
    # One searchable index PER MODEL, each holding that model's own cells.
    assert cli.patch_stream_for("", "stub-a") == f"{PATCH_STREAM}__m_stub_a"
    assert store.stream(f"{PATCH_STREAM}__m_stub_a", dict).count() == 3 * 8 * 8
    assert store.stream(f"{PATCH_STREAM}__m_stub_b_16", dict).count() == 3 * 4 * 4

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
        None,
    )
    assert parse_member("/models/siglip2-so400m-patch16-384") == (
        "/models/siglip2-so400m-patch16-384",
        None,
        None,
    )
    assert parse_member("google/siglip2-base-patch16-224#2x3") == (
        "google/siglip2-base-patch16-224",
        None,
        (2, 3),
    )
    assert member_tag("google/siglip2-base-patch16-naflex@576") == "base-patch16-naflex-576"
    assert member_tag("/models/siglip2-so400m-patch16-384") == "so400m-patch16-384"
    assert member_tag("google/siglip2-base-patch16-224#2x3") == "base-patch16-224-2x3"


def test_naflex_on_metal_gets_its_missing_resize() -> None:
    """A NaFlex member resizes its position grid with an antialiased bilinear, which
    torch has no Metal kernel for, so before this the forward pass raised mid-run and
    NaFlex was unusable on a Mac. `PYTORCH_ENABLE_MPS_FALLBACK=1` cannot fix it from
    inside the process -- torch reads that when it is imported."""
    installed = len(embedder._mps_antialias_fallback)
    embedder.ensure_mps_antialias("cpu")
    embedder.ensure_mps_antialias("cuda")
    assert len(embedder._mps_antialias_fallback) == installed, "only Metal is missing the kernel"

    if not torch.backends.mps.is_available():
        pytest.skip("no Metal device")

    embedder.ensure_mps_antialias("mps")
    on_metal = torch.nn.functional.interpolate(
        torch.arange(64, dtype=torch.float32, device="mps").reshape(1, 1, 8, 8),
        size=(4, 4),
        mode="bilinear",
        antialias=True,
    )
    on_cpu = torch.nn.functional.interpolate(
        torch.arange(64, dtype=torch.float32).reshape(1, 1, 8, 8),
        size=(4, 4),
        mode="bilinear",
        antialias=True,
    )
    assert on_metal.device.type == "mps", "the answer comes back where the caller left it"
    assert on_metal.cpu().numpy() == pytest.approx(on_cpu.numpy())


def test_every_module_can_say_no_to_metal_in_its_config() -> None:
    """`allow_mps=False` is the way out for a stack whose parent process touches Metal.

    There is no probe for that case -- see `pick_device` -- so the way out has to be a
    setting, and it has to be one a blueprint can set: all three modules carry it, on by
    default, and each hands it to the same chooser.
    """
    import torch

    from dimos.mapping.hyperspace.module import (
        HyperspaceConfig,
        HyperspacePatchesConfig,
        pick_device,
    )

    for config in [HyperspaceConfig, HyperspacePatchesConfig]:
        assert config.model_fields["allow_mps"].default is True, config.__name__

    if not torch.backends.mps.is_available() or torch.cuda.is_available():
        pytest.skip("the rest only says anything on an Apple machine with no CUDA")
    assert pick_device("auto") == "mps"
    assert pick_device("auto", allow_mps=False) == "cpu"
    assert pick_device("mps", allow_mps=False) == "mps", "a named device still wins"


def test_the_live_query_disposes_the_way_the_module_registers_it() -> None:
    """`Hyperspace.start()` hands the LiveQuery to `register_disposable`, and a
    `CompositeDisposable` calls `dispose` on what it holds. While the method was named
    `close`, every stop raised `'LiveQuery' object has no attribute 'dispose'` and the
    towers were never released -- invisible until a failed start made a stop happen."""
    from dimos.core.resource import CompositeDisposable
    from dimos.mapping.hyperspace.live import LiveQuery

    closed = []

    class Towers:
        def close(self) -> None:
            closed.append(True)

    live = LiveQuery.__new__(LiveQuery)
    live.towers = Towers()

    held = CompositeDisposable()
    held.add(live)
    held.dispose()

    assert closed == [True], "the composite must reach the towers through dispose()"


def test_tiles_cover_the_frame_exactly_and_stitch_back() -> None:
    """A tiled member must rebuild the frame's geometry with no patch dropped,
    duplicated or averaged -- the stitched grid is the tiles laid side by side."""
    import numpy as np
    from PIL import Image as PILImage

    from dimos.mapping.hyperspace.embedder import stitch_tiles, tile_image

    frame = PILImage.fromarray(np.arange(480 * 848 * 3, dtype=np.uint8).reshape(480, 848, 3))
    crops = tile_image(frame, 2, 3)
    assert len(crops) == 6
    assert [c.size for c in crops] == [(282, 240), (283, 240), (283, 240)] * 2
    assert sum(w * h for w, h in (c.size for c in crops)) == 848 * 480  # exact cover

    side, dim = 14, 4
    # Each tile carries its own index, so a mis-stitch shows up as a wrong block.
    grids = [np.full((side * side, dim), i, np.float32) for i in range(6)]
    stitched = stitch_tiles(grids, 2, 3, side)
    assert stitched.shape == (2 * side * 3 * side, dim)
    block = stitched.reshape(2 * side, 3 * side, dim)
    for index in range(6):
        r, c = divmod(index, 3)
        tile = block[r * side : (r + 1) * side, c * side : (c + 1) * side]
        assert (tile == index).all(), f"tile {index} landed in the wrong place"


def test_ensemble_cell_grid_follows_the_finest_member() -> None:
    """A tiled member's resolution must survive the common cell grid: pooling
    two members onto a fixed 24x24 would throw the tiling away."""
    import numpy as np

    from dimos.mapping.hyperspace.ingest import IngestConfig, PatchIngestor

    ingestor = PatchIngestor.__new__(PatchIngestor)
    ingestor.config = IngestConfig(gate=hs.KeyframeGateConfig())

    def grids(*shapes: tuple[int, int]) -> list:
        return [(np.zeros((r * c, 4), np.float32), (r, c)) for r, c in shapes]

    assert ingestor.cell_grid(grids((14, 14))) == (14, 14)  # single member keeps its own
    assert ingestor.cell_grid(grids((14, 14), (16, 16))) == (24, 24)  # untiled pair unchanged
    assert ingestor.cell_grid(grids((14, 14), (28, 42))) == (28, 42)  # tiling survives
    ingestor.config = IngestConfig(gate=hs.KeyframeGateConfig(), cell_grid=(24, 24))
    assert ingestor.cell_grid(grids((14, 14), (28, 42))) == (24, 24)  # explicit still wins


# --- one recording, one file ---------------------------------------------------------
# The keyframes and patches belong in the recording they describe. Only an .mcap, which
# cannot be written to, gets a companion db beside it.


def test_a_db_indexes_itself_and_only_an_mcap_gets_a_companion() -> None:
    assert cli.memory_db_for(Path("/data/grocery.db")) == Path("/data/grocery.db")
    assert cli.memory_db_for(Path("/data/grocery.mcap")) == Path("/data/grocery.hyperspace.db")


def test_keyframes_alone_are_reusable_and_a_legacy_marker_is_dropped_with_them(
    store: SqliteStore,
) -> None:
    """No completeness marker (Jeff, 2026-09-12): keyframes being there is the whole test.

    The cost, stated so a later reader does not think it an oversight: a run killed
    mid-ingest leaves keyframes that read as a finished index, and only --no-reuse
    replaces them.
    """
    assert not cli.index_is_finished(store)
    fill(store, ring(3, 2.5))
    assert store.stream(KEYFRAME_STREAM, dict).count() == 3
    assert cli.index_is_finished(store)

    # Dbs indexed before the marker went away still carry one; dropping the index must
    # take it too, or it would vouch for keyframes that are no longer there.
    store.stream(COMPLETE_STREAM, String).append(String("3 keyframes"), ts=1.0)
    cli.drop_index(store)
    assert not cli.index_is_finished(store)
    for name in (KEYFRAME_STREAM, PATCH_STREAM, COMPLETE_STREAM):
        assert name not in store.list_streams(), name


def test_an_empty_source_stream_is_refused_before_the_old_index_is_touched(
    store: SqliteStore,
) -> None:
    """The pre-flight is the whole point: refusing after the drop costs the index."""
    fill(store, ring(3, 2.5))
    store.stream("empty_camera_info", CameraInfo)  # named, never written to

    with pytest.raises(typer.BadParameter, match="nothing was changed"):
        cli.refuse_unless_readable(store, (KEYFRAME_STREAM, "empty_camera_info"))

    # The refusal cost nothing: the index that was there is still there.
    assert cli.index_is_finished(store)
    assert store.stream(KEYFRAME_STREAM, dict).count() == 3


def test_reingesting_in_place_replaces_the_keyframes_rather_than_appending(
    store: SqliteStore,
) -> None:
    """Writing into the recording means a rerun would otherwise double every keyframe."""
    fill(store, ring(3, 2.5))
    patches_after_one_run = store.stream(PATCH_STREAM, dict).count()

    cli.drop_index(store)  # what the ingest does before it re-embeds
    fill(store, ring(3, 2.5))

    assert store.stream(KEYFRAME_STREAM, dict).count() == 3
    assert store.stream(PATCH_STREAM, dict).count() == patches_after_one_run


# --- several models in one recording ---------------------------------------------------
# A recording can carry more than one index so two checkpoints can be compared on the
# same question. Each keyframe and patch says which model wrote it.


def test_every_keyframe_and_patch_records_the_model_that_wrote_it(store: SqliteStore) -> None:
    fill(store, ring(2, 2.5))
    first = store.stream(KEYFRAME_STREAM, dict).order_by("ts").first()
    assert first.data["model"], "a keyframe with no model is a keyframe nobody can place"
    assert "member_specs" in first.data and "members" in first.data
    assert first.tags["model"] == first.data["model"]
    patch = store.stream(cli.patch_stream_for("", "stub"), dict).order_by("ts").first()
    assert patch.data["model"] == first.data["model"]


def test_a_second_model_gets_its_own_index_instead_of_overwriting_the_first(
    store: SqliteStore,
) -> None:
    """The point of keeping both is comparison; clobbering one would defeat it."""
    one = ["google/siglip2-base-patch16-224"]
    two = ["google/siglip2-base-patch16-384"]

    assert cli.pick_index(store, one) == "", "the first model in takes the canonical streams"
    fill(store, ring(2, 2.5))  # writes the canonical pair

    # Same checkpoints again: the same index, so a re-ingest replaces rather than forks.
    specs = store.stream(KEYFRAME_STREAM, dict).order_by("ts").first().data["member_specs"]
    assert cli.pick_index(store, specs) == ""

    # A different checkpoint: its own streams, named after it.
    slug = cli.pick_index(store, two)
    assert slug == cli.index_slug(two) != ""
    keyframes, patches = cli.stream_names(slug)
    assert keyframes == f"{KEYFRAME_STREAM}__{slug}"
    assert patches == f"{PATCH_STREAM}__{slug}"


def test_dropping_one_index_leaves_the_others_alone(store: SqliteStore) -> None:
    fill(store, ring(2, 2.5))
    other = cli.index_slug(["google/siglip2-base-patch16-384"])
    other_keyframes, other_patches = cli.stream_names(other)
    store.stream(other_keyframes, dict).append({"model": other}, ts=1.0)
    store.stream(other_patches, dict).append({"model": other}, ts=1.0)

    cli.drop_index(store, other)
    assert other_keyframes not in store.list_streams()
    assert cli.index_is_finished(store), "the canonical index must survive its neighbour"

    cli.drop_index(store)
    assert not cli.index_is_finished(store)


def test_every_model_gets_its_own_searchable_index(store: SqliteStore, tmp_path: Path) -> None:
    """A query asks each model its own nearest-neighbour question.

    It cannot do that against one table holding only the primary model, and reading a
    patch out of a keyframe's `grids` blob means unpickling the whole keyframe -- which
    is why a query used to load every one of them into memory.
    """
    # One model keeps the bare name, so stores written before ensembles still read.
    fill(store, ring(2, 2.5))
    assert store.stream(cli.patch_stream_for("", "stub"), dict).count() == 2 * SIDE * SIDE

    other = SqliteStore(path=str(tmp_path / "ensemble.db"))
    other.start()
    try:
        fill_ensemble(other, ring(2, 2.5))
        names = set(other.list_streams())
        assert f"{PATCH_STREAM}__m_stub_a" in names, sorted(names)
        assert f"{PATCH_STREAM}__m_stub_b_16" in names, sorted(names)
        # Each index holds ITS OWN model's cells, not a copy of the primary's.
        assert other.stream(f"{PATCH_STREAM}__m_stub_a", dict).count() == 2 * 8 * 8
        assert other.stream(f"{PATCH_STREAM}__m_stub_b_16", dict).count() == 2 * 4 * 4
        # A re-ingest must replace them, not append a second copy of every vector.
        cli.drop_index(other)
        assert f"{PATCH_STREAM}__m_stub_a" not in set(other.list_streams())
    finally:
        other.stop()


def test_indexing_a_recording_in_place_does_not_copy_its_tf_into_itself(
    store: SqliteStore, tmp_path: Path
) -> None:
    """The index used to live in a separate file, which needed its own copy of tf.

    In place, the transforms are already there, and copying them appends a second set of
    the recording's own tf to itself. grocery.db reached 85,302 rows over 16,048 distinct
    stamps -- 5.3x duplicated -- across a handful of ingests before this was noticed.
    """
    tf = store.stream("tf", TFMessage)
    for index in range(4):
        transform = Transform(translation=Vector3(0.0, 0.0, 0.0), rotation=Quaternion(0, 0, 0, 1))
        transform.frame_id, transform.child_frame_id = WORLD, CAMERA
        tf.append(TFMessage(transform), ts=float(index))
    before = tf.count()

    ingestor = fill(store, ring(2, 2.5), copy_tf=False)  # store IS the recording
    assert ingestor.stats["kept"] == 2
    assert store.stream("tf", TFMessage).count() == before, "the ingest duplicated tf"


def test_the_flat_layout_writes_patches_that_place_themselves(store: SqliteStore) -> None:
    """A patch row carries its own camera, time, ray and depth.

    That is what lets a query turn a search hit into a point in the world without
    opening a per-frame row -- the thing that made it load every frame into memory.
    """
    fill(store, ring(2, 2.5), flat=True)

    assert KEYFRAME_STREAM not in store.list_streams(), "the flat layout has no keyframe blob"

    patches = store.stream(cli.patch_stream_for("", "stub"), dict)
    assert patches.count() == 2 * SIDE * SIDE
    row = patches.order_by("ts").first().data
    assert row["camera_frame"] == CAMERA
    assert row["grid"] == [SIDE, SIDE]
    assert len(row["ray"]) == 2
    assert np.isfinite(row["depth"]) and row["depth"] > 0
    assert set(row) >= {"camera_frame", "ts", "cell", "grid", "ray", "depth", "member"}

    # the thumbnail is a point cloud in the CAMERA's frame, not a depth picture
    thumbnails = store.stream(cli.thumbnail_stream_for(""), dict)
    assert thumbnails.count() == 2
    cloud = np.asarray(thumbnails.order_by("ts").first().data["points_mm"])
    assert cloud.ndim == 2 and cloud.shape[1] == 3
    assert cloud.dtype == np.int16
    # z forward and positive: these are camera-frame points, so no pose is baked in
    assert (cloud[:, 2] > 0).all()


def test_dropping_a_flat_index_takes_its_thumbnails_too(store: SqliteStore) -> None:
    fill(store, ring(2, 2.5), flat=True)
    assert cli.index_is_finished(store), "patch rows alone make an index answerable"
    cli.drop_index(store)
    assert not cli.index_is_finished(store)
    assert cli.thumbnail_stream_for("") not in store.list_streams()


def test_hot_frames_reads_the_flat_layout_and_ranks_the_frames(store: SqliteStore) -> None:
    """The shared first step, against a real vec0 index rather than a fixture.

    The stub's grid has exactly one patch on the object and everything else on the
    background axis, so a correct read of the flat layout finds one hit per keyframe --
    and each hit has to carry its own frame, stamp, ray and depth, because in this
    layout there is no keyframe row to look them up in.
    """
    from dimos.mapping.hyperspace.frames import (
        BACKGROUND_PROMPTS,
        episodes,
        hot_frames,
        member_streams,
    )

    class StubTowers:
        """`TextTowers`' interface, without a checkpoint to download."""

        def query(self, spec: str, text: str) -> np.ndarray:
            del spec
            return StubModel.embed_text(text)

        def background(self, spec: str) -> np.ndarray:
            del spec
            return np.stack([StubModel.embed_text(prompt) for prompt in BACKGROUND_PROMPTS])

        def close(self) -> None:
            pass

    ingestor = fill(store, ring(4, 2.5), flat=True)
    assert [tag for tag, _ in member_streams(store)] == ["stub"]

    frames = hot_frames(store, "object", towers=StubTowers())
    assert len(frames) == ingestor.stats["kept"], "every kept frame saw the object"
    assert all(len(frame.hits) == 1 for frame in frames), "one patch per frame is on it"
    assert [frame.ts for frame in frames] == sorted(frame.ts for frame in frames)
    first = frames[0]
    assert first.frame == CAMERA
    assert first.members == {"stub"}
    assert first.best > 0.5
    assert np.isfinite(first.hits[0].depth)

    # The frames are whole seconds apart, so every gap wider than one splits them all.
    stamps = [frame.ts for frame in frames]
    gaps = [b - a for a, b in itertools.pairwise(stamps)]
    assert len(episodes(frames, gap_s=0.5)) == len(frames)
    assert len(episodes(frames, gap_s=max(gaps))) == 1


def test_filled_depth_is_what_a_box_is_placed_off(store: SqliteStore) -> None:
    """A recording carrying filled depth is placed off that, without touching stereo.

    The point of writing it once -- live by the depth2depth module, or afterwards by
    `fill_depth` -- is that answering a query never pays for a model. So the read side
    has to prefer it, and has to find it by stamp like any other frame.
    """
    from dimos.mapping.hyperspace.detect import DetectConfig, RecordingFrames
    from dimos.mapping.hyperspace.ingest import filled_stream_for

    fill(store, ring(4, 2.5), flat=True)
    config = DetectConfig(world_frame=WORLD)
    assert RecordingFrames(store, config=config).depth(CAMERA, 10.0) is None, (
        "nothing to place off yet: the fixture writes no depth stream"
    )

    store.stream(filled_stream_for(""), dict).append(
        {
            "camera_frame": CAMERA,
            "ts": 10.0,
            "depth_mm": np.full((HEIGHT, WIDTH), 2500, dtype=np.uint16),
        },
        ts=10.0,
        tags={"camera_frame": CAMERA},
    )

    frames = RecordingFrames(store, config=config)
    got = frames.depth(CAMERA, 10.0)
    assert got is not None and np.allclose(got, 2.5), "metres, from the filled stream"
    assert frames.depth(CAMERA, 30.0) is None, "and only where a filled frame exists"


def test_the_fill_pass_only_visits_the_frames_that_were_embedded(store: SqliteStore) -> None:
    """Filling costs fifty milliseconds a frame, so it must not visit every frame.

    A box is only ever placed off a frame that was embedded -- grocery embedded 3462 of
    its 24110 colour frames -- and walking the rest is twenty minutes spent on frames
    nothing will ever ask about.
    """
    from dimos.mapping.hyperspace.ingest import _colour_at, embedded_stamps

    ingestor = fill(store, ring(4, 2.5), flat=True)
    stamps = embedded_stamps(store)
    assert len(stamps) == ingestor.stats["kept"], "one stamp per embedded frame"
    assert stamps == sorted(stamps)

    class Colours:
        """Stands in for the colour stream: one frame every tenth of a second."""

        def __init__(self) -> None:
            self.asked: list[float] = []

        def at(self, ts: float, tolerance: float) -> Colours:
            self.asked.append(ts)
            self.window = (ts, tolerance)
            return self

        def to_list(self) -> list[SimpleNamespace]:
            ts, tolerance = self.window
            every = [10.0 + tenth / 10.0 for tenth in range(41)]
            return [SimpleNamespace(ts=at) for at in every if abs(at - ts) <= tolerance]

    colours = Colours()
    got = [float(found.ts) for found in _colour_at(colours, stamps)]
    assert got == stamps, "the embedded frame itself, not a neighbour"
    assert colours.asked == stamps, "and nothing else was read"


def test_the_resident_index_carries_everything_a_patch_is_placed_by(
    store: SqliteStore,
) -> None:
    """A patch's stamp, ray, grid and depth survive the move out of sqlite.

    Those are what place it in the world later, and the vectors are held at half
    precision, so this is also the check that halving them did not cost a hit.
    """
    from dimos.mapping.hyperspace.frames import BACKGROUND_PROMPTS, hot_frames, member_streams
    from dimos.mapping.hyperspace.resident import ResidentIndex

    class StubTowers:
        def query(self, spec: str, text: str) -> np.ndarray:
            del spec
            return StubModel.embed_text(text)

        def background(self, spec: str) -> np.ndarray:
            del spec
            return np.stack([StubModel.embed_text(prompt) for prompt in BACKGROUND_PROMPTS])

        def close(self) -> None:
            pass

    ingestor = fill(store, ring(4, 2.5), flat=True)
    held = ResidentIndex()
    held.warm(store, list(member_streams(store)))
    found = hot_frames(store, "object", towers=StubTowers(), resident=held)

    assert len(found) == ingestor.stats["kept"], "every kept frame saw the object"
    assert all(len(frame.hits) == 1 for frame in found), "one patch per frame is on it"
    assert [frame.ts for frame in found] == sorted(frame.ts for frame in found)
    first = found[0].hits[0]
    assert first.grid == (8, 8) and 0 <= first.cell < 64
    assert np.isfinite(first.depth) and first.depth > 0
    assert first.score > 0.5, "half precision still separates the object from the room"


def test_the_index_grows_while_the_ingest_is_still_writing(store: SqliteStore) -> None:
    """A live query has to see what the robot drove past a second ago.

    On a robot the db starts empty and fills while questions are being asked, so
    "load the index once at startup" answers off a map that stops at boot. The index
    picks up what has landed since it last looked, without re-reading what it holds.
    """
    from dimos.mapping.hyperspace.frames import BACKGROUND_PROMPTS, hot_frames
    from dimos.mapping.hyperspace.resident import ResidentIndex

    class StubTowers:
        def query(self, spec: str, text: str) -> np.ndarray:
            del spec
            return StubModel.embed_text(text)

        def background(self, spec: str) -> np.ndarray:
            del spec
            return np.stack([StubModel.embed_text(prompt) for prompt in BACKGROUND_PROMPTS])

        def close(self) -> None:
            pass

    held = ResidentIndex()
    members = [("stub", cli.patch_stream_for("", "stub"))]

    # Nothing written yet: an empty db is the normal state at boot, not an error.
    assert held.grow(store, members) == 0

    first = fill(store, ring(2, 2.5), flat=True)
    assert held.grow(store, members) == first.stats["kept"] * SIDE * SIDE
    patches = held.get(members[0][1])
    assert patches is not None
    before = patches.rows
    seen = len(hot_frames(store, "object", towers=StubTowers(), resident=held))
    assert seen == first.stats["kept"]

    # The ingest keeps going. Later frames, from poses the first read never saw.
    second = fill(store, ring(4, 3.0), flat=True, start_ts=100.0)
    assert second.stats["kept"], "the second pass kept something to find"
    added = held.grow(store, members)
    assert added == second.stats["kept"] * SIDE * SIDE, "only the new rows were read"
    assert patches is held.get(members[0][1]), "grown in place, not reloaded"
    assert patches.rows == before + added
    assert patches.capacity >= patches.rows
    assert len(hot_frames(store, "object", towers=StubTowers(), resident=held)) == (
        first.stats["kept"] + second.stats["kept"]
    ), "the frames written after the index was warmed answer too"

    # And a poll with nothing new is free rather than a reload.
    assert held.grow(store, members) == 0
    assert patches.rows == before + added


def test_the_chunk_read_and_the_row_read_agree(store: SqliteStore) -> None:
    """The fast read is sqlite-vec's own storage, so it has to be checked against SQL.

    Fifty times faster is worth nothing if it returns different numbers, and the whole
    reason it is allowed is that a mismatch can be caught here rather than in an answer.
    """
    from dimos.mapping.hyperspace.frames import member_streams
    from dimos.mapping.hyperspace.resident import _from_chunks, _vectors_of

    fill(store, ring(4, 2.5), flat=True)
    _, stream = next(iter(member_streams(store)))
    conn = store._registry_conn
    rows = int(conn.execute(f'SELECT COUNT(*) FROM "{stream}"').fetchone()[0])
    probe = conn.execute(f'SELECT embedding FROM "{stream}_vec" LIMIT 1').fetchone()
    width = len(np.frombuffer(probe[0], dtype=np.float32))

    quick = _from_chunks(conn, stream, width, rows)
    assert quick is not None, "this store does have chunk storage"
    slow = _vectors_of(conn, stream, width, rows)
    assert quick.shape == slow.shape == (rows, width)
    assert np.array_equal(quick, slow), "same vectors, same order"


def test_a_store_without_chunk_storage_still_loads(store: SqliteStore) -> None:
    """Refuse rather than lie: no chunk tables means the slow read, not a wrong answer."""
    from dimos.mapping.hyperspace.frames import member_streams
    from dimos.mapping.hyperspace.resident import _from_chunks

    fill(store, ring(4, 2.5), flat=True)
    _, stream = next(iter(member_streams(store)))
    assert _from_chunks(store._registry_conn, "not_a_stream", 8, 4) is None
    assert _from_chunks(store._registry_conn, stream, 8, 999999) is None, (
        "a row count that does not match the layout is a refusal too"
    )


def test_scoring_a_block_at_a_time_is_the_same_as_scoring_it_whole(
    store: SqliteStore,
) -> None:
    """The blocks exist for speed, so they must not change the answer at a boundary."""
    from dimos.mapping.hyperspace import resident as resident_module
    from dimos.mapping.hyperspace.frames import BACKGROUND_PROMPTS, member_streams
    from dimos.mapping.hyperspace.resident import ResidentIndex

    fill(store, ring(4, 2.5), flat=True)
    held = ResidentIndex()
    tag, stream = next(iter(member_streams(store)))
    patches = held.of(store, tag, stream)
    query = StubModel.embed_text("object")
    background = np.stack([StubModel.embed_text(prompt) for prompt in BACKGROUND_PROMPTS])

    whole = patches.scores(query, background)
    was = resident_module.SCORE_CHUNK
    try:
        # A block size that lands mid-index, so a boundary is actually crossed.
        resident_module.SCORE_CHUNK = max(1, patches.rows // 3)
        in_blocks = patches.scores(query, background)
    finally:
        resident_module.SCORE_CHUNK = was
    assert np.array_equal(whole, in_blocks)


def test_the_resident_index_is_loaded_once_and_reused(store: SqliteStore) -> None:
    """Paying the read twice would defeat the whole point of holding it."""
    from dimos.mapping.hyperspace.frames import member_streams
    from dimos.mapping.hyperspace.resident import ResidentIndex

    fill(store, ring(4, 2.5), flat=True)
    members = list(member_streams(store))
    held = ResidentIndex()
    held.warm(store, members)
    first = held.of(store, *members[0])
    again = held.of(store, *members[0])
    assert again is first, "the second ask must not re-read the index"
    assert first.rows == first.vectors.shape[0] == len(first.ts) == len(first.depth)


def test_resident_hot_rows_are_ranked_and_capped(store: SqliteStore) -> None:
    """`hot` is the whole filter: above the threshold, best first, no more than asked."""
    from dimos.mapping.hyperspace.frames import BACKGROUND_PROMPTS, member_streams
    from dimos.mapping.hyperspace.resident import ResidentIndex

    fill(store, ring(4, 2.5), flat=True)
    held = ResidentIndex()
    members = list(member_streams(store))
    held.warm(store, members)
    patches = held.of(store, *members[0])

    query = StubModel.embed_text("object")
    background = np.stack([StubModel.embed_text(prompt) for prompt in BACKGROUND_PROMPTS])
    picked, scored = patches.hot(query, background, threshold=0.0)
    assert len(picked) and np.all(scored > 0.0)
    assert list(scored) == sorted(scored, reverse=True), "strongest first"

    capped, _ = patches.hot(query, background, threshold=0.0, limit=1)
    assert len(capped) == 1 and capped[0] == picked[0], "the cap keeps the best, not the first"
    none, _ = patches.hot(query, background, threshold=10.0)
    assert len(none) == 0, "nothing clears an impossible threshold"


def test_hot_frames_finds_nothing_for_words_the_recording_does_not_contain(
    store: SqliteStore,
) -> None:
    """A query that matches nothing returns nothing, rather than the least-bad patch."""
    from dimos.mapping.hyperspace.frames import BACKGROUND_PROMPTS, hot_frames

    class StubTowers:
        def query(self, spec: str, text: str) -> np.ndarray:
            del spec
            return StubModel.embed_text(text)

        def background(self, spec: str) -> np.ndarray:
            del spec
            return np.stack([StubModel.embed_text(prompt) for prompt in BACKGROUND_PROMPTS])

        def close(self) -> None:
            pass

    fill(store, ring(4, 2.5), flat=True)
    assert hot_frames(store, "something else entirely", towers=StubTowers()) == []


def test_one_answer_per_place_carries_its_own_frame() -> None:
    """A second look at a cone is a better box for that cone, not another cone.

    The RPC's answer is what a caller navigates by, so two rows for one thing would
    have it drive to the same place twice; and an answer without the picture it was
    made from cannot be checked by anyone.
    """
    from dimos.mapping.hyperspace.detect import Box3D, Detection
    from dimos.mapping.hyperspace.live import objects_of

    def looked(
        rank: int, place: int, score: float, centre: tuple[float, float, float]
    ) -> Detection:
        found = Detection(
            query="a cone",
            rank=rank,
            ts=100.0 + rank,
            camera_frame=CAMERA,
            episode_frames=4,
            episode_span=1.0,
            episode_score=1.0,
            models=["stub"],
            attempts=1,
            score=score,
            box2d=(1.0, 2.0, 3.0, 4.0),
            box3d=Box3D(frame=WORLD, centre=centre, extent=(0.4, 0.4, 0.4), pixels=10, depth_m=2.0),
        )
        found.place_id = place
        found.image = Image.from_numpy(
            np.full((4, 4, 3), 7, dtype=np.uint8), frame_id=CAMERA, ts=found.ts
        )
        return found

    answers = [
        looked(1, 1, 0.6, (1.0, 0.0, 0.0)),
        looked(2, 1, 0.9, (1.1, 0.0, 0.0)),  # the same cone, seen better
        looked(3, 2, 0.7, (9.0, 0.0, 0.0)),  # a different cone
        Detection(
            query="a cone",
            rank=4,
            ts=104.0,
            camera_frame=CAMERA,
            episode_frames=1,
            episode_span=0.0,
            episode_score=0.1,
            models=["stub"],
            attempts=3,
        ),
    ]

    objects = objects_of(answers, top=0)
    assert [round(found.confidence, 2) for found in objects] == [0.9, 0.7], "strongest first"
    assert [found.place_id for found in objects] == [1, 2], "one row per place"
    assert objects[0].views == 2, "and it says how many looks agreed"
    assert objects[0].image is not None and objects[0].camera_frame == CAMERA
    assert objects[0].stamp == 102.0, "the stamp of the look that was reported"
    assert objects[0].box2d == (1.0, 2.0, 3.0, 4.0)
    assert objects_of(answers, top=1) == objects[:1]


def test_a_stream_says_which_family_of_model_wrote_it() -> None:
    """A stream of vectors is unusable without knowing the checkpoint behind it.

    Two families now write patch streams, and their text towers are not
    interchangeable: encoding "a traffic cone" with SigLIP and scoring it against
    Perception Encoder vectors would compare two different spaces and answer
    confidently with noise. The tag has to round-trip, both ways, for both.
    """
    from dimos.mapping.hyperspace.embedder import member_tag
    from dimos.mapping.hyperspace.frames import spec_of
    from dimos.mapping.hyperspace.ingest import sql_safe

    for spec in (
        "pe:PE-Core-B-16",
        "pe:PE-Core-L-14-336",
        "google/siglip2-base-patch16-224",
        "google/siglip2-so400m-patch16-naflex@1024",
    ):
        assert spec_of(sql_safe(member_tag(spec))) == spec, spec


def test_a_kept_frame_comes_back_the_colour_it_went_in(store: SqliteStore) -> None:
    """The picture the detector is shown has to be the picture the camera took.

    `Image.from_numpy` defaults to BGR and the ingest holds RGB, so the default labels
    the channels backwards and `to_rgb()` then swaps them for real. Nothing raises: the
    frames look plausible until you notice brake lights are blue, and OWLv2 asked for
    "a car" on a street full of cars refuses every one of them.
    """
    from dimos.mapping.hyperspace.detect import DetectConfig, RecordingFrames
    from dimos.mapping.hyperspace.ingest import frame_stream_for

    ingestor = fill(store, ring(2, 2.5), flat=True)
    ingestor.config.keep_frames = True
    # One frame whose channels are all different, so a swap cannot hide.
    painted = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    painted[..., 0], painted[..., 1], painted[..., 2] = 200, 120, 40
    ingestor._keep_frame(CAMERA, 50.0, painted)

    kept = store.streams[frame_stream_for("")].at(50.0, tolerance=0.01).to_list()
    assert kept, "the frame was written"
    frames = RecordingFrames(store, config=DetectConfig(world_frame=WORLD))
    shown = frames.color(50.0)
    assert shown is not None
    back = np.asarray(shown.to_rgb().data)
    # The store's codec is JPEG, so a couple of levels of drift is the compression,
    # not the channels. A swap would land ~160 levels away on two of the three.
    assert np.allclose(back[0, 0], (200, 120, 40), atol=8), (
        f"channels came back as {tuple(int(v) for v in back[0, 0])}, wanted (200, 120, 40)"
    )


def test_the_text_towers_stay_off_the_card_the_detector_needs(store: SqliteStore) -> None:
    """Three text towers plus OWLv2 does not fit in 8 GB, so the towers go on the CPU.

    Measured on an RTX 5070 (2026-09-14): the towers held 6.0 GiB of 7.5 and OWLv2's
    warm-up then asked for 594 MiB against 217 MiB free, so every query died before it
    reached a frame. The towers encode one short string per question -- about a second
    of CPU -- and the detector is what actually needs the card. A `device="cuda"` that
    silently took the towers with it is the bug this pins down.
    """
    from dimos.mapping.hyperspace.detect import DetectConfig
    from dimos.mapping.hyperspace.live import LiveConfig, LiveQuery

    assert LiveConfig.tower_device == "cpu", "the default has to keep the card free"

    live = LiveQuery(store, LiveConfig(detect=DetectConfig(device="cuda")))
    assert live.towers.device == "cpu", (
        f"the towers followed the detector onto {live.towers.device!r}"
    )

    # And an explicit choice still wins, for a card with room to spare.
    shared = LiveQuery(store, LiveConfig(detect=DetectConfig(device="cuda"), tower_device="cuda"))
    assert shared.towers.device == "cuda"


def test_the_camera_is_written_down_even_when_the_frames_are_not(store: SqliteStore) -> None:
    """`keep_frames` is about colour FRAMES; the camera is what places a box.

    An offline index is built with `keep_frames` off, because the recording already holds
    its own colour -- and that used to take the intrinsics with it. The query side then
    looked for `camera_info` or `depth_camera_info` by those exact names, found neither on
    a recording that calls them something else, and every answer died as "no camera_info
    for <frame>": measured on roscon_jpeg.db as "0 place(s), 6 refused, 282 ms", which
    reads as a detector declining rather than as a query that never looked.
    """
    fill(store, [look_at(np.array([0.0, -2.0, 0.0]), OBJECT)], flat=True)
    assert info_stream_for("") in store.list_streams(), "the camera was not written down"
    held = store.stream(info_stream_for(""), CameraInfo).to_list()
    assert [observation.data.frame_id for observation in held] == [CAMERA]


def test_an_answer_too_big_for_the_transport_is_published_at_a_smaller_scale() -> None:
    """Seven evidence frames do not fit, and nothing reports that they did not.

    MEASURED on the pickled-LCM transport: 16 MB arrives, 19 MB does not -- the
    receiver's fragment buffer drops it and `publish` still returns success. An item
    answer carries one 1280x720 frame per place at 2.77 MB, so three places fit and
    seven do not; roscon answered "7 fire extinguishers" in words over a viewer that
    drew nothing.
    """
    import pickle

    from dimos.mapping.hyperspace.module import PUBLISHED_FRAME_BUDGET_BYTES, _fits_the_transport
    from dimos.mapping.hyperspace.msgs import FoundObject, FoundObjects
    from dimos.msgs.sensor_msgs.Image import Image

    def frame() -> Image:
        return Image(data=np.zeros((720, 1280, 3), dtype=np.uint8), frame_id="cam", ts=1.0)

    answer = FoundObjects(
        query="a fire extinguisher",
        objects=[FoundObject(image=frame(), centre=(1.0, 2.0, 0.5)) for _ in range(7)],
    )
    published = _fits_the_transport(answer)

    assert len(pickle.dumps(answer)) > 16_000_000, "the case under test is not big enough"
    assert len(pickle.dumps(published)) < PUBLISHED_FRAME_BUDGET_BYTES + 1_000_000
    assert published.objects[0].image is not None, "the evidence went away entirely"
    # The caller kept the full frame: only what goes on the wire is shrunk.
    assert answer.objects[0].image is not None
    assert answer.objects[0].image.data.shape == (720, 1280, 3)


def test_an_answer_that_already_fits_is_published_unchanged() -> None:
    """A heatmap carries no frames, and three places fit: neither pays for the cap."""
    from dimos.mapping.hyperspace.module import _fits_the_transport
    from dimos.mapping.hyperspace.msgs import FoundObject, FoundObjects
    from dimos.msgs.sensor_msgs.Image import Image

    heat = FoundObjects(query="crowded", kind="heatmap", objects=[FoundObject() for _ in range(48)])
    assert _fits_the_transport(heat) is heat

    small = FoundObjects(
        query="a basket",
        objects=[
            FoundObject(image=Image(data=np.zeros((720, 1280, 3), dtype=np.uint8)))
            for _ in range(3)
        ],
    )
    assert _fits_the_transport(small) is small


def test_a_counting_question_spends_more_looks_and_gives_the_budget_back() -> None:
    """`episodes` raises the budget for one question, and nothing inherits it.

    One LiveQuery serves every question, so a counting question that left its budget
    behind would quietly make every later question four times slower -- and the way that
    shows up is a demo that gets worse the longer it runs, which is the hardest kind of
    slowness to attribute.
    """
    from dimos.mapping.hyperspace.module import Hyperspace
    from dimos.mapping.hyperspace.queries import Query

    spent: list[int] = []

    def ask(text: str, background_prompts: Any = None) -> SimpleNamespace:
        spent.append(live.config.detect.max_episodes)
        return SimpleNamespace(kind="", refused=0, timings={}, objects=[])

    live = SimpleNamespace(
        config=SimpleNamespace(top=1, detect=SimpleNamespace(max_episodes=30)), ask=ask
    )
    module = SimpleNamespace(live=live, found=SimpleNamespace(publish=lambda result: None))

    Hyperspace._fill_from_detector(
        module, Query(query_id="q1", text="a fire extinguisher", kind="item")
    )
    Hyperspace._fill_from_detector(
        module, Query(query_id="q2", text="a fire extinguisher", kind="item"), None, 60
    )
    Hyperspace._fill_from_detector(module, Query(query_id="q3", text="a chair", kind="item"))
    # ...and asking for FEWER than the recording is configured for changes nothing: that
    # is the capped count this exists to prevent, arriving from the other direction.
    Hyperspace._fill_from_detector(
        module, Query(query_id="q4", text="a chair", kind="item"), None, 24
    )

    assert spent == [30, 60, 30, 30], "raise only, and for one question"
    assert live.config.detect.max_episodes == 30
