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

import sqlite3
import tempfile
from types import SimpleNamespace
from typing import TYPE_CHECKING

import numpy as np
import pytest
import sqlite_vec
import torch

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.teleop.memory_world.recording import embedding_stream_name
from dimos.teleop.memory_world.test_recording import seed_embedding_stream
from dimos.teleop.memory_world.tf_tree import pose_matrix
from dimos.teleop.memory_world.visual_search import (
    POSE_FRAME_TAG,
    PatchGrid,
    PatchHit,
    Place,
    VisualMemoryIndex,
    align_patch_tokens,
    body_style_quaternion,
    cluster_hits,
    cluster_places,
    hot_patches,
    index_stream_name_of,
    model_slug,
    patch_world_position,
    score_frames,
    search_phrase,
)

if TYPE_CHECKING:
    from collections.abc import Iterator


def place(x: float, y: float, similarity: float, source_id: int = 0) -> Place:
    return Place(position=(x, y, 0.0), similarity=similarity, source_id=source_id, ts=0.0)


def test_nearby_hits_collapse_to_the_strongest_one() -> None:
    """A robot staring at one object makes many frames; that is still one place."""
    places = cluster_places(
        [place(0.0, 0.0, 0.30), place(0.5, 0.0, 0.42), place(1.0, 0.4, 0.35)],
        radius=2.0,
        max_places=6,
    )
    assert [p.similarity for p in places] == [0.42]


def test_distant_hits_are_separate_places() -> None:
    places = cluster_places(
        [place(0.0, 0.0, 0.30), place(10.0, 0.0, 0.42), place(0.0, 20.0, 0.10)],
        radius=2.0,
        max_places=6,
    )
    assert [p.position for p in places] == [(10.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 20.0, 0.0)]


def test_results_are_ordered_by_similarity_regardless_of_input_order() -> None:
    places = cluster_places(
        [place(0.0, 0.0, 0.1), place(10.0, 0.0, 0.9), place(20.0, 0.0, 0.5)],
        radius=2.0,
        max_places=6,
    )
    assert [p.similarity for p in places] == [0.9, 0.5, 0.1]


def test_max_places_caps_the_answer() -> None:
    candidates = [place(float(i) * 10.0, 0.0, 1.0 - i * 0.01) for i in range(20)]
    places = cluster_places(candidates, radius=2.0, max_places=3)
    assert len(places) == 3
    assert [p.similarity for p in places] == pytest.approx([1.0, 0.99, 0.98])


def test_radius_boundary_is_inclusive_of_exactly_radius_apart() -> None:
    """Two hits exactly `radius` apart are distinct, so the cap is a true minimum spacing."""
    places = cluster_places([place(0.0, 0.0, 0.9), place(2.0, 0.0, 0.8)], radius=2.0, max_places=6)
    assert len(places) == 2


def test_clustering_uses_all_three_axes() -> None:
    """Two floors of a stairwell are different places even at the same x/y."""
    stacked = [
        Place(position=(0.0, 0.0, 0.0), similarity=0.9, source_id=1, ts=0.0),
        Place(position=(0.0, 0.0, 5.0), similarity=0.8, source_id=2, ts=0.0),
    ]
    assert len(cluster_places(stacked, radius=2.0, max_places=6)) == 2


def test_no_candidates_yields_no_places() -> None:
    assert cluster_places([], radius=2.0, max_places=6) == []


@pytest.mark.parametrize(("radius", "max_places"), [(0.0, 6), (-1.0, 6), (2.0, 0), (2.0, -1)])
def test_invalid_parameters_are_rejected(radius: float, max_places: int) -> None:
    with pytest.raises(ValueError):
        cluster_places([place(0.0, 0.0, 0.5)], radius=radius, max_places=max_places)


@pytest.mark.parametrize(
    ("spoken", "expected"),
    [
        ("Where did I see a traffic cone?", "a traffic cone"),
        ("  where is   the whiteboard ", "the whiteboard"),
        ("a chair", "a chair"),
        ("Whereabouts", "Whereabouts"),  # a prefix must be a whole word
        ("", ""),
    ],
)
def test_spoken_questions_reduce_to_the_thing_asked_about(spoken: str, expected: str) -> None:
    assert search_phrase(spoken) == expected


def test_a_long_transcript_is_capped_at_the_result_field_length() -> None:
    assert len(search_phrase("word " * 200)) == 400  # MemoryQueryResult.query_text's max


# ---- scoring ----------------------------------------------------------------


def unit(*values: float) -> torch.Tensor:
    vector = torch.tensor(values, dtype=torch.float32)
    return vector / vector.norm()


def test_one_hot_patch_beats_a_frame_of_lukewarm_patches() -> None:
    """The whole point of per-patch scoring: a small object in one patch must win."""
    query = unit(1.0, 0.0)
    cone_frame = torch.stack([unit(1.0, 0.0), unit(0.0, 1.0), unit(0.0, 1.0), unit(0.0, 1.0)])
    lukewarm = torch.stack([unit(1.0, 1.0)] * 4)
    scores, best = score_frames(torch.stack([lukewarm, cone_frame]), query, torch.zeros(0, 2))
    assert scores[1] > scores[0]
    assert int(best[1]) == 0


def test_background_contrast_removes_what_every_frame_shares() -> None:
    query = unit(1.0, 0.0)
    background = unit(1.0, 0.0).unsqueeze(0)  # the query IS the background
    frame = torch.stack([unit(1.0, 0.0), unit(0.0, 1.0)])
    scores, _ = score_frames(frame.unsqueeze(0), query, background)
    assert float(scores[0]) == pytest.approx(0.0)


# ---- the index --------------------------------------------------------------


@pytest.fixture
def sqlite_store() -> Iterator[SqliteStore]:
    """A throwaway store; skipped only where sqlite-vec really cannot load."""
    probe = sqlite3.connect(":memory:")
    try:
        probe.enable_load_extension(True)
        sqlite_vec.load(probe)
    except (AttributeError, sqlite3.OperationalError) as error:
        pytest.skip(f"sqlite-vec extension not loadable here: {error}")
    finally:
        probe.close()
    with tempfile.NamedTemporaryFile(suffix=".db") as f:
        store = SqliteStore(path=f.name)
        with store:
            yield store


def _seed_index(
    store: SqliteStore,
    model_name: str,
    patches: np.ndarray | None = None,
    ts: float = 1.0,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    source_id: int = 7,
) -> None:
    grid = patches if patches is not None else np.zeros((4, 2), dtype=np.float16)
    store.stream(index_stream_name_of(model_name, "color_image"), PatchGrid).append(
        PatchGrid(source_id=source_id, rows=2, cols=2, patches=grid),
        ts=ts,
        pose=PoseStamped(position=Vector3(*position)),
        tags={"model": model_name, "pose_frame": POSE_FRAME_TAG},
    )


GIANT = "google/siglip2-giant-opt-patch16-384"


def test_each_model_gets_its_own_stream() -> None:
    """Two models' vectors are not comparable, and two cameras' frames are not the
    same evidence, so neither pair may share a stream; siglipify's stays apart too."""
    assert index_stream_name_of(GIANT, "image") == "image_index_siglip2_giant_opt_p16_384"
    assert index_stream_name_of(GIANT, "color_image") != embedding_stream_name("color_image", GIANT)
    assert index_stream_name_of(
        "google/siglip2-so400m-patch16-384", "image"
    ) != index_stream_name_of(GIANT, "image")
    # ...and two cameras' frames are different evidence.
    assert index_stream_name_of(GIANT, "left_image") != index_stream_name_of(GIANT, "right_image")


def test_index_built_by_another_model_is_refused(sqlite_store: SqliteStore) -> None:
    """Naming per model keeps them apart; an explicit override can still collide."""
    other = "google/siglip2-so400m-patch16-384"
    _seed_index(sqlite_store, other)
    with pytest.raises(ValueError, match="so400m"):
        _ = VisualMemoryIndex(
            sqlite_store,
            pose_of=lambda obs: None,
            index_stream_name=index_stream_name_of(other, "color_image"),
            model_name=GIANT,
        ).index_stream


def test_index_built_by_the_same_model_opens(sqlite_store: SqliteStore) -> None:
    _seed_index(sqlite_store, GIANT)
    assert VisualMemoryIndex(sqlite_store, pose_of=lambda obs: None, model_name=GIANT).count() == 1


def test_search_returns_the_frame_and_patch_that_matched(sqlite_store: SqliteStore) -> None:
    """No model needed: the index is seeded with 2-d unit vectors and the query is one too."""
    hot = np.array([[0, 1], [0, 1], [1, 0], [0, 1]], dtype=np.float16)  # patch 2 = row 1, col 0
    cold = np.array([[0, 1]] * 4, dtype=np.float16)
    _seed_index(sqlite_store, GIANT, patches=cold, ts=1.0, position=(0.0, 0.0, 0.0), source_id=1)
    _seed_index(sqlite_store, GIANT, patches=hot, ts=2.0, position=(5.0, 0.0, 0.0), source_id=2)
    index = VisualMemoryIndex(sqlite_store, pose_of=lambda obs: None, model_name=GIANT)
    index._embed = lambda text: unit(1.0, 0.0)  # type: ignore[method-assign]
    index._background = torch.zeros(0, 2)

    places = index.search("a cone", k=5)

    assert [place.source_id for place in places] == [2, 1]
    assert places[0].position == (5.0, 0.0, 0.0)
    assert places[0].similarity == pytest.approx(1.0)
    assert places[0].image_uv == (0.25, 0.75)


def test_index_built_with_body_poses_is_refused(sqlite_store: SqliteStore) -> None:
    sqlite_store.stream(index_stream_name_of(GIANT, "color_image"), PatchGrid).append(
        PatchGrid(source_id=7, rows=2, cols=2, patches=np.zeros((4, 2), dtype=np.float16)),
        ts=1.0,
        pose=PoseStamped(position=Vector3(0.0, 0.0, 0.0)),
        tags={"model": GIANT, "pose_frame": "body"},
    )
    with pytest.raises(ValueError, match="rebuild"):
        _ = VisualMemoryIndex(sqlite_store, pose_of=lambda obs: None, model_name=GIANT).index_stream


def test_index_with_a_frame_on_one_side_only_is_accepted(sqlite_store: SqliteStore) -> None:
    """An untagged (older) index fits any frame; a tagged one fits a caller with no preference."""
    _seed_index(sqlite_store, GIANT)  # no world_frame tag
    assert (
        VisualMemoryIndex(
            sqlite_store, pose_of=lambda obs: None, model_name=GIANT, world_frame="map"
        ).count()
        == 1
    )
    tagged = index_stream_name_of(GIANT, "color_image") + "_tagged"
    sqlite_store.stream(tagged, PatchGrid).append(
        PatchGrid(source_id=7, rows=2, cols=2, patches=np.zeros((4, 2), dtype=np.float16)),
        ts=1.0,
        pose=PoseStamped(position=Vector3(0.0, 0.0, 0.0)),
        tags={"model": GIANT, "world_frame": "odom", "pose_frame": POSE_FRAME_TAG},
    )
    assert (
        VisualMemoryIndex(
            sqlite_store, pose_of=lambda obs: None, index_stream_name=tagged, model_name=GIANT
        ).count()
        == 1
    )


def test_index_built_in_another_world_frame_is_refused(sqlite_store: SqliteStore) -> None:
    sqlite_store.stream(index_stream_name_of(GIANT, "color_image"), PatchGrid).append(
        PatchGrid(source_id=7, rows=2, cols=2, patches=np.zeros((4, 2), dtype=np.float16)),
        ts=1.0,
        pose=PoseStamped(position=Vector3(0.0, 0.0, 0.0)),
        tags={"model": GIANT, "world_frame": "odom", "pose_frame": POSE_FRAME_TAG},
    )
    with pytest.raises(ValueError, match="odom"):
        _ = VisualMemoryIndex(
            sqlite_store, pose_of=lambda obs: None, model_name=GIANT, world_frame="map"
        ).index_stream


def test_index_of_another_camera_is_refused(sqlite_store: SqliteStore) -> None:
    sqlite_store.stream(index_stream_name_of(GIANT, "color_image"), PatchGrid).append(
        PatchGrid(source_id=7, rows=2, cols=2, patches=np.zeros((4, 2), dtype=np.float16)),
        ts=1.0,
        pose=PoseStamped(position=Vector3(0.0, 0.0, 0.0)),
        tags={"model": GIANT, "image_stream": "left_image", "pose_frame": POSE_FRAME_TAG},
    )
    with pytest.raises(ValueError, match="left_image"):
        _ = VisualMemoryIndex(sqlite_store, pose_of=lambda obs: None, model_name=GIANT).index_stream


def test_frames_the_tf_tree_cannot_place_are_skipped(sqlite_store: SqliteStore) -> None:
    images = sqlite_store.stream("realsense_color_image", int)
    images.append(1, ts=10.0, pose=None)
    images.append(2, ts=20.0, pose=None)
    placed = {10.0: pose_matrix((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))}
    index = VisualMemoryIndex(
        sqlite_store,
        pose_of=lambda obs: placed.get(float(obs.ts)),
        image_stream_name="realsense_color_image",
    )

    posed = [(obs.data, tuple(matrix[:3, 3])) for obs, matrix in index._posed_frames()]

    assert posed == [(1, (1.0, 0.0, 0.0))]


# ---- vectors siglipify already wrote into the recording ---------------------


def _fake_head_model(dims: int) -> SimpleNamespace:
    """A stand-in vision head: swaps the first two coordinates, so its use is visible."""
    head = torch.nn.Linear(dims, dims, bias=False)
    with torch.no_grad():
        head.weight.copy_(torch.eye(dims)[[1, 0, *range(2, dims)]])
    return SimpleNamespace(_model=SimpleNamespace(vision_model=SimpleNamespace(head=head)))


def test_align_patch_tokens_runs_the_head_per_token_and_normalises() -> None:
    tokens = torch.tensor([[[3.0, 0.0], [0.0, 4.0]]])  # one frame, two patches
    aligned = align_patch_tokens(_fake_head_model(2), tokens)  # type: ignore[arg-type]
    assert aligned.shape == (1, 2, 2)
    torch.testing.assert_close(aligned[0], torch.tensor([[0.0, 1.0], [1.0, 0.0]]))


def _seed_precomputed(
    sqlite_store: SqliteStore,
    rows: list[tuple[float, int | None, np.ndarray]],
    model_name: str = GIANT,
    text_aligned: bool | None = True,
) -> None:
    images = sqlite_store.stream("color_image", int)
    for source_id, ts in ((1, 1.0), (2, 2.0), (3, 3.0)):
        images.append(source_id, ts=ts, pose=None)
    seed_embedding_stream(
        sqlite_store.config.path,
        f"color_image_{model_slug(model_name)}",
        model_name,
        rows,
        text_aligned=text_aligned,
    )


def _placed(obs: object) -> np.ndarray | None:
    positions = {1.0: (0.0, 0.0, 0.0), 2.0: (5.0, 0.0, 0.0)}  # frame 3 has no pose
    position = positions.get(float(obs.ts))  # type: ignore[attr-defined]
    return None if position is None else pose_matrix(position, (0.0, 0.0, 0.0, 1.0))


def test_precomputed_vectors_are_the_index_and_nothing_is_built(sqlite_store: SqliteStore) -> None:
    """Pooled rows are already text-aligned; frames are matched by id or, failing
    that, by stamp, and a frame the tf tree cannot place is dropped."""
    _seed_precomputed(
        sqlite_store,
        [
            (1.0, 1, np.array([[0.0, 2.0]], np.float32)),  # by id; not unit length on purpose
            (2.0, None, np.array([[1.0, 0.0]], np.float32)),  # by stamp
            (3.0, 3, np.array([[1.0, 0.0]], np.float32)),  # unplaceable
        ],
    )
    index = VisualMemoryIndex(sqlite_store, pose_of=_placed, model_name=GIANT)
    assert index.precomputed_stream_name == "color_image_siglip2_giant_opt_p16_384"
    assert index.count() == 3
    assert index.build() == 0
    index._embed = lambda text: unit(1.0, 0.0)  # type: ignore[method-assign]
    index._background = torch.zeros(0, 2)

    places = index.search("a cone", k=5)

    assert [(p.source_id, p.position) for p in places] == [
        (2, (5.0, 0.0, 0.0)),
        (1, (0.0, 0.0, 0.0)),
    ]
    assert places[0].similarity == pytest.approx(1.0)
    assert places[1].similarity == pytest.approx(0.0, abs=1e-3)


def test_precomputed_patch_grids_are_searched_patch_by_patch(sqlite_store: SqliteStore) -> None:
    grid = np.array([[0.0, 1.0], [0.0, 1.0], [1.0, 0.0], [0.0, 1.0]], np.float32)  # patch 2 differs
    _seed_precomputed(sqlite_store, [(1.0, 1, grid)])
    index = VisualMemoryIndex(sqlite_store, pose_of=_placed, model_name=GIANT)
    index._embed = lambda text: unit(1.0, 0.0)  # type: ignore[method-assign]
    index._background = torch.zeros(0, 2)

    (place,) = index.search("a cone", k=5)

    assert place.image_uv == (0.25, 0.75)
    assert place.similarity == pytest.approx(1.0)


def test_precomputed_raw_tower_tokens_are_refused(sqlite_store: SqliteStore) -> None:
    """A stream siglipify wrote before it applied the head is not text-searchable."""
    _seed_precomputed(sqlite_store, [(1.0, 1, np.ones((4, 2), np.float32))], text_aligned=None)
    index = VisualMemoryIndex(sqlite_store, pose_of=_placed, model_name=GIANT)
    assert index.count() == 1
    with pytest.raises(ValueError, match="raw"):
        index.load()


def test_precomputed_vectors_from_another_model_are_refused(sqlite_store: SqliteStore) -> None:
    other = "google/siglip2-so400m-patch16-384"
    _seed_precomputed(sqlite_store, [(1.0, 1, np.ones((1, 2), np.float32))], model_name=other)
    index = VisualMemoryIndex(
        sqlite_store, pose_of=_placed, model_name=GIANT, image_stream_name="color_image"
    )
    assert index.precomputed_stream_name is None  # named for the other model
    index = VisualMemoryIndex(sqlite_store, pose_of=_placed, model_name=other)
    assert index.count() == 1


# ---- putting the answer on the object ---------------------------------------


def test_patch_centre_back_projects_through_the_median_depth() -> None:
    depth = np.zeros((100, 200), dtype=np.uint16)
    depth[40:60, 90:110] = 2000  # a 2 m surface under the patch, holes elsewhere
    depth[50, 100] = 0  # one hole inside the window is ignored by the median
    intrinsics = (100.0, 100.0, 100.0, 50.0)  # fx, fy, cx, cy
    camera_to_world = np.eye(4)
    camera_to_world[:3, 3] = (10.0, 0.0, 0.0)

    position = patch_world_position((0.5, 0.5), depth, intrinsics, camera_to_world, window_px=16)

    assert position == pytest.approx((10.0, 0.0, 2.0))


def test_patch_over_a_depth_hole_has_no_position() -> None:
    depth = np.zeros((100, 200), dtype=np.uint16)
    assert patch_world_position((0.5, 0.5), depth, (1.0, 1.0, 0.0, 0.0), np.eye(4)) is None


def test_hot_patches_keep_the_object_blob_not_only_the_stray_peak() -> None:
    similarity = torch.full((4,), 0.02)
    similarity[0] = 0.164  # a stray picture frame
    similarity[2] = 0.158  # the cone
    similarity[3] = 0.135  # more cone
    kept = hot_patches(similarity, rows=2, cols=2, floor=0.10, ratio=0.75)
    assert [round(score, 3) for _, score in kept] == [0.164, 0.158, 0.135]
    assert kept[1][0] == (0.25, 0.75)  # index 2 = row 1, col 0


def test_hot_patches_respect_the_absolute_floor() -> None:
    assert hot_patches(torch.tensor([0.05, 0.04]), rows=1, cols=2) == []


def hit(
    x: float, y: float, similarity: float, camera: tuple[float, float, float], frame: int
) -> PatchHit:
    return PatchHit(
        position=(x, y, 0.0),
        similarity=similarity,
        source_id=frame,
        ts=float(frame),
        camera_position=camera,
        camera_orientation=(0.0, 0.0, 0.7071, 0.7071),
    )


def test_an_object_seen_from_several_directions_outranks_a_single_view_stray() -> None:
    cone = [
        hit(5.0, 5.0, 0.15, camera=(0.0, 5.0, 0.0), frame=1),  # seen from the west
        hit(5.1, 5.0, 0.14, camera=(5.0, 0.0, 0.0), frame=2),  # and from the south
        hit(5.0, 5.1, 0.13, camera=(10.0, 5.0, 0.0), frame=3),  # and from the east
    ]
    stray = [hit(-5.0, -5.0, 0.17, camera=(0.0, 0.0, 0.0), frame=4)]
    places = cluster_hits(cone + stray, radius=0.75, max_places=6)
    assert [place.views for place in places] == [3, 1]
    assert places[0].position == pytest.approx((5.033, 5.033, 0.0), abs=0.01)
    assert places[0].similarity == 0.15
    # The best hit's camera pose rides along, so its frame can be shown where it was taken.
    assert places[0].camera_position == (0.0, 5.0, 0.0)
    assert places[0].orientation == (0.0, 0.0, 0.7071, 0.7071)
    assert places[0].source_id == 1


def test_consecutive_frames_from_one_spot_count_as_one_view() -> None:
    hits = [hit(5.0, 5.0, 0.15, camera=(0.0, 5.0, 0.0), frame=f) for f in range(5)]
    assert cluster_hits(hits, radius=0.75, max_places=6)[0].views == 1


def test_body_style_quaternion_puts_x_on_the_optical_axis() -> None:
    # An optical frame looking along world -z (straight down) with image-up = world +x.
    optical = np.eye(4)
    optical[:3, 0] = (0.0, -1.0, 0.0)  # x right
    optical[:3, 1] = (-1.0, 0.0, 0.0)  # y down  -> up is +x
    optical[:3, 2] = (0.0, 0.0, -1.0)  # z forward
    frame = pose_matrix((0, 0, 0), body_style_quaternion(optical))
    assert frame[:3, 0] == pytest.approx((0.0, 0.0, -1.0), abs=1e-6)
    assert frame[:3, 2] == pytest.approx((1.0, 0.0, 0.0), abs=1e-6)
