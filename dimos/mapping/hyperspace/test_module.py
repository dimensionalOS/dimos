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
