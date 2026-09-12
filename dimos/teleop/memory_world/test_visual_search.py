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
    sensor_intrinsics,
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
        ("Whereabouts", "Whereabouts"),  # a prefix must be a whole word...
        # ...and "where" is not the only prefix. This case is "find", the shortest one and
        # the likeliest to swallow a real word: with the trailing space dropped from the
        # match, "findings" came back as "ngs". The Whereabouts case above cannot see that,
        # because it only exercises the "where" family.
        ("findings", "findings"),
        ("Find the traffic cone", "the traffic cone"),
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


def test_an_untagged_index_is_refused_and_a_caller_naming_no_frame_takes_anything(
    sqlite_store: SqliteStore,
) -> None:
    """An index with no world_frame was written before the tag, which means old, not
    universal: nothing says its poses are in the frame this caller wants. A caller that
    names no frame has made no claim to contradict, so it takes what it finds."""
    _seed_index(sqlite_store, GIANT)  # no world_frame tag
    with pytest.raises(ValueError, match="rebuild it"):
        _ = VisualMemoryIndex(
            sqlite_store, pose_of=lambda obs: None, model_name=GIANT, world_frame="map"
        ).count()
    assert VisualMemoryIndex(sqlite_store, pose_of=lambda obs: None, model_name=GIANT).count() == 1
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
    """A stream siglipify wrote before it applied the head is not text-searchable.

    It used to be ADOPTED and counted, and only refused later by `load()`. That is what
    told the viewer search was ready while every query failed -- and, on a recording that
    also holds a freshly built index, it let 1108 unusable vectors outrank 5538 good ones,
    because adoption asked how many rows there were and not whether they could be used.
    Refused at adoption now, so `count()` reports the built index instead of these.
    """
    _seed_precomputed(sqlite_store, [(1.0, 1, np.ones((4, 2), np.float32))], text_aligned=None)
    index = VisualMemoryIndex(sqlite_store, pose_of=_placed, model_name=GIANT)

    assert index.precomputed_stream_name is None, "adopted vectors text cannot score"
    assert index.count() == 0, "counted them as an index the viewer could search"

    # ...and pointed straight at it, the loader still says why rather than scoring noise.
    with pytest.raises(ValueError, match="raw"):
        index._load_precomputed(embedding_stream_name("color_image", GIANT))


def test_an_embeddings_stream_under_another_prefix_is_adopted_when_it_is_the_only_one(
    sqlite_store: SqliteStore,
) -> None:
    """siglipify names its stream after the images it was POINTED at, not this rig's name.

    sf_office1_2 holds 1108 vectors in `image_siglip2_giant_opt_p16_384` while its images
    are `realsense_color_image`. The strict name found nothing, so the viewer offered to
    spend minutes building an index that was already sitting in the file.
    """
    rows = [(1.0, 1, np.zeros((4, 2), np.float16)), (2.0, 2, np.zeros((4, 2), np.float16))]
    seed_embedding_stream(sqlite_store.config.path, f"image_{model_slug(GIANT)}", GIANT, rows)

    index = VisualMemoryIndex(
        store=sqlite_store, image_stream_name="realsense_color_image", pose_of=lambda *a: None
    )

    assert index.precomputed_stream_name == f"image_{model_slug(GIANT)}"
    assert index.count() == 2


def test_two_embeddings_streams_for_one_model_are_not_guessed_between(
    sqlite_store: SqliteStore,
) -> None:
    """The strict rule exists because two cameras' frames are not the same evidence.

    With one candidate there is nothing to confuse it with; with two there is, and
    picking either would attach one camera's vectors to the other camera's poses.
    """
    rows = [(1.0, 1, np.zeros((4, 2), np.float16))]
    for camera in ("left_image", "right_image"):
        seed_embedding_stream(
            sqlite_store.config.path, f"{camera}_{model_slug(GIANT)}", GIANT, rows
        )

    index = VisualMemoryIndex(
        store=sqlite_store, image_stream_name="realsense_color_image", pose_of=lambda *a: None
    )

    assert index.precomputed_stream_name is None, "guessed which camera the vectors were for"


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
    """The threshold is the HIGHER of the floor and the ratio of the frame's best.

    One patch below sits between the two -- above the 0.10 floor, below 0.75 x 0.164 --
    and it is the whole point of the fixture. Without it the floor and the ratio select
    the same patches, so `max(floor, best * ratio)` could be written `min(...)`, dropping
    the ratio entirely, and this test still passed.
    """
    similarity = torch.full((6,), 0.02)
    similarity[0] = 0.164  # a stray picture frame
    similarity[1] = 0.110  # over the floor, under the ratio: kept only by a MIN
    similarity[2] = 0.158  # the cone
    similarity[3] = 0.135  # more cone
    kept = hot_patches(similarity, rows=2, cols=3, floor=0.10, ratio=0.75)
    assert [round(score, 3) for _, score in kept] == [0.164, 0.158, 0.135]
    assert kept[1][0] == pytest.approx((2.5 / 3, 0.25))  # index 2 = row 0, col 2


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


def test_a_colour_patch_is_sampled_at_the_depth_camera_s_own_pixel() -> None:
    """The patch coordinates are the colour camera's; the depth image is another camera.

    On the demo rig the two differ by about 1% in fx and several pixels in principal
    point, which sampling at the same normalised position turns into centimetres of
    error at a few metres. Real grocery_stitch.db numbers are used here.
    """
    from dimos.teleop.memory_world.visual_search import patch_world_position

    colour = (644.07, 643.06, 642.15, 363.14)  # camera_info
    depth = (651.56, 651.56, 647.83, 356.01)  # depth_camera_info
    # A wall two metres away, filling the frame, so any pixel reads 2 m.
    depth_mm = np.full((720, 1280), 2000, dtype=np.uint16)
    uv = (0.75, 0.5)

    corrected = patch_world_position(
        uv, depth_mm, depth, np.eye(4), color_intrinsics=colour, color_size=(1280, 720)
    )
    naive = patch_world_position(uv, depth_mm, depth, np.eye(4))
    assert corrected is not None and naive is not None

    # The truth is the colour camera's own ray at two metres.
    truth_x = (uv[0] * 1280 - colour[2]) / colour[0] * 2.0
    assert abs(corrected[0] - truth_x) < 0.005  # within the pixel rounding
    assert abs(naive[0] - truth_x) > 0.025  # what it used to be: 2.9 cm at two metres

    # A patch whose ray leaves the depth camera's view has no depth to read.
    assert (
        patch_world_position(
            (1.02, 0.5), depth_mm, depth, np.eye(4), color_intrinsics=colour, color_size=(1280, 720)
        )
        is None
    )


def test_the_colour_uv_is_scaled_by_the_colour_raster_not_the_depth_one() -> None:
    """The two cameras need not publish the same size, and then the sizes are the bug.

    The demo rig publishes 1280x720 for both, which is why this costs nothing there and
    why a test using one size for both cannot see it. A bag with unaligned depth --
    1280x720 colour, 848x480 depth -- puts the sampled pixel 200-odd columns away.
    """
    from dimos.teleop.memory_world.visual_search import patch_world_position

    colour = (644.07, 643.06, 642.15, 363.14)  # 1280x720
    depth = (424.30, 424.30, 424.90, 237.30)  # 848x480, a different raster
    depth_mm = np.full((480, 848), 2000, dtype=np.uint16)  # a wall two metres off
    uv = (0.75, 0.5)

    got = patch_world_position(
        uv, depth_mm, depth, np.eye(4), color_intrinsics=colour, color_size=(1280, 720)
    )
    assert got is not None
    truth_x = (uv[0] * 1280 - colour[2]) / colour[0] * 2.0
    assert abs(got[0] - truth_x) < 0.01, f"{got[0]} is not the colour ray {truth_x}"

    # Scaling that uv by the DEPTH width instead lands near the optical axis -- a metre
    # of error, not a centimetre -- which is what this test exists to catch.
    assert abs(truth_x) > 0.9

    # And half a calibration is refused rather than silently sampling the wrong pixel.
    with pytest.raises(ValueError):
        patch_world_position(uv, depth_mm, depth, np.eye(4), color_intrinsics=colour)


def test_the_siglip_fallback_answers_end_to_end() -> None:
    """Run the fallback answer, rather than checking a list of names I thought of.

    It moved out of module.py, and a moved method takes its names with it. Nothing else in
    the suite calls it -- it is the path for a recording with no Hyperspace keyframes -- so
    a missing import surfaces in front of whoever runs the demo on an unindexed recording
    and nowhere before that; ruff does not see it, because the names were declared under
    TYPE_CHECKING. My first attempt at this listed four names and missed `time`, which only
    a SUCCESSFUL answer reaches.
    """
    import threading
    from types import SimpleNamespace

    from dimos.teleop.memory_world.visual_answers import VisualAnswers
    from dimos.teleop.memory_world.visual_search import Place

    place = Place(position=(1.0, 2.0, 3.0), similarity=0.42, source_id=3, ts=1.0)
    published = {}

    class Module(VisualAnswers):
        def __init__(self) -> None:
            self._store_lock = threading.RLock()
            # The real host holds this and the answer path publishes `_last_answer` under
            # it, so a stub without one is a stub that has drifted from the mixin's host.
            self._clients_lock = threading.RLock()
            self._last_answer = (None, None)
            self.config = SimpleNamespace(
                store_path="/nowhere/walk.db",
                search_top_k=5,
                place_radius_m=1.0,
                max_places=3,
                object_radius_m=0.25,
            )

        def _ensure_visual_index(self):  # type: ignore[no-untyped-def]
            return SimpleNamespace(count=lambda: 7, search=lambda *a, **k: [])

        def _locate_objects(self, phrase):  # type: ignore[no-untyped-def]
            return [place]

        def _markers_near(self, positions):  # type: ignore[no-untyped-def]
            return [11]

        def _add_route_to_result(self, result) -> None:  # type: ignore[no-untyped-def]
            published["routed"] = True

        def _publish_query_result(self, result) -> str:  # type: ignore[no-untyped-def]
            published["result"] = result
            return "qid"

        def _publish_query_images(self, query_id, phrase, places) -> None:  # type: ignore[no-untyped-def]
            published["images"] = (query_id, phrase, places)

    outcome = Module()._find_with_siglip("a basket", 0.0)

    assert outcome.success, outcome.message
    assert outcome.metadata["query_id"] == "qid"
    assert outcome.metadata["places"][0]["position"] == (1.0, 2.0, 3.0)
    assert outcome.duration_ms > 0  # time.monotonic(), which the name list did not cover
    assert published["result"].engine == "siglip"
    assert published["result"].observation_ids == [11]


def test_one_frame_straddling_a_bearing_boundary_is_still_one_view() -> None:
    """A blob is not two directions because it crosses a 45 degree line.

    The bearing was measured to each hit's OWN world point, so one camera, in one
    frame, looking at one object that happens to sit on a bin boundary reported two
    viewing directions -- and `views` is the primary rank key and the "N views" the
    user reads. A place genuinely seen from two sides then lost to a single stray.
    """
    # One camera at the origin, one frame, a blob either side of the 45 degree line.
    straddle = [
        hit(5.00, 5.05, 0.30, camera=(0.0, 0.0, 0.0), frame=1),  # bearing 45.3 deg
        hit(5.05, 5.00, 0.29, camera=(0.0, 0.0, 0.0), frame=1),  # bearing 44.7 deg
    ]
    assert cluster_hits(straddle, radius=0.75, max_places=6)[0].views == 1

    # And the ranking it was corrupting: two real directions must outrank the blob,
    # even though the blob's similarity is higher.
    two_sides = [
        hit(-5.0, 0.0, 0.20, camera=(-10.0, 0.0, 0.0), frame=2),  # from the west
        hit(-5.0, 0.1, 0.19, camera=(-5.0, -10.0, 0.0), frame=3),  # and from the south
    ]
    places = cluster_hits(straddle + two_sides, radius=0.75, max_places=6)
    assert [place.views for place in places] == [2, 1]
    assert places[0].similarity == 0.20


def test_the_answer_sentence_names_the_place_it_flies_to() -> None:
    """ "best match 0.150" beside a marker reading "0.170" is a contradiction.

    `cluster_hits` ranks by VIEWING DIRECTIONS first, so its `places[0]` -- the place the
    camera flies to and the sentence describes -- is the most-seen one, not the
    highest-scoring one. The sentence called it "best match" anyway, while a higher score
    was drawn in the same world at the same moment with its own number on it. This is the
    fixture two tests above: a cone at 0.15 seen three ways, a stray at 0.17 seen once.
    """
    from dimos.teleop.memory_world.visual_answers import _best_phrase

    cone = [
        hit(5.0, 5.0, 0.15, camera=(0.0, 5.0, 0.0), frame=1),
        hit(5.1, 5.0, 0.14, camera=(5.0, 0.0, 0.0), frame=2),
        hit(5.0, 5.1, 0.13, camera=(10.0, 5.0, 0.0), frame=3),
    ]
    stray = [hit(-5.0, -5.0, 0.17, camera=(0.0, 0.0, 0.0), frame=4)]
    # cluster_hits, not cluster_places: the depth path is the one that ranks by views.
    places = cluster_hits(cone + stray, radius=0.75, max_places=6)

    # The place the answer flies to is NOT the highest-scoring one ...
    assert places[0].similarity < max(place.similarity for place in places)
    # ... so the sentence must not call its number the best match. It names the two
    # numbers the marker beside it shows.
    sentence = _best_phrase(places[0], located=True)
    assert "best match" not in sentence
    assert f"{places[0].similarity:+.3f}" in sentence
    assert f"{places[0].views} views" in sentence

    # On the branch that really does rank by similarity, "best match" is true and stays.
    ranked_by_score = _best_phrase(places[0], located=False)
    assert ranked_by_score == f"best match {places[0].similarity:+.3f}"


def test_an_unmeasured_view_count_does_not_reach_the_metadata_either() -> None:
    """Dropping it from the label and leaving it in the payload is the same defect.

    `views` is measured only on the depth path. On the other one it is the dataclass
    default, and an agent reading the skill result cannot tell a measured 1 from an
    unmeasured one. The marker label stopped printing it; the metadata kept sending it,
    which is a fix reaching one of two call sites -- the shape this loop has produced
    five times.
    """
    from dimos.teleop.memory_world.visual_answers import _place_metadata

    # A Place as the no-depth branch produces it: VisualMemoryIndex.search() never
    # passes views=, so it carries the dataclass default.
    place = Place(position=(1.0, 1.0, 0.0), similarity=0.3, source_id=1, ts=1.0)
    assert place.views == 1  # the default, not a measurement

    assert "views" not in _place_metadata(place, located=False)
    assert _place_metadata(place, located=True)["views"] == place.views
    # The numbers that ARE measured on both branches stay on both.
    for located in (True, False):
        assert _place_metadata(place, located=located)["similarity"] == place.similarity


def _siglip_module(places_from_depth: list, places_from_search: list):
    """The fallback answer path with its two producers stubbed, nothing else changed."""
    import threading
    from types import SimpleNamespace

    from dimos.teleop.memory_world.visual_answers import VisualAnswers

    published: dict = {}

    class Module(VisualAnswers):
        def __init__(self) -> None:
            self._store_lock = threading.RLock()
            # The real host holds this and the answer path publishes `_last_answer` under
            # it, so a stub without one is a stub that has drifted from the mixin's host.
            self._clients_lock = threading.RLock()
            self._last_answer = (None, None)
            self.config = SimpleNamespace(
                store_path="/nowhere/walk.db",
                search_top_k=5,
                place_radius_m=1.0,
                max_places=3,
                object_radius_m=0.25,
            )

        def _ensure_visual_index(self):  # type: ignore[no-untyped-def]
            return SimpleNamespace(count=lambda: 7, search=lambda *a, **k: places_from_search)

        def _locate_objects(self, phrase):  # type: ignore[no-untyped-def]
            return places_from_depth

        def _markers_near(self, positions):  # type: ignore[no-untyped-def]
            return [11]

        def _add_route_to_result(self, result) -> None:  # type: ignore[no-untyped-def]
            pass

        def _publish_query_result(self, result) -> str:  # type: ignore[no-untyped-def]
            published["result"] = result
            return "qid"

        def _publish_query_images(self, query_id, phrase, places) -> None:  # type: ignore[no-untyped-def]
            pass

    return Module(), published


def test_the_real_answer_path_says_which_producer_ranked_it() -> None:
    """Through `_find_with_siglip`, not through the helper it calls.

    The first version of this test called `_best_phrase` directly, so it could not see
    whether production passes the right `located`. Flipping that argument in the source left
    it green -- a test reaching around the code instead of through it, for the fifth time in
    this loop, in the test written to guard against exactly that.
    """
    from dimos.teleop.memory_world.visual_search import Place

    seen_three_ways = Place(position=(1.0, 2.0, 3.0), similarity=0.15, source_id=3, ts=1.0, views=3)
    module, published = _siglip_module([seen_three_ways], [])
    outcome = module._find_with_siglip("a basket", 0.0)
    assert outcome.success, outcome.message
    # Depth path: ranked by viewpoints, so the sentence names both numbers.
    assert "best 0.150 from 3 views" in published["result"].answer.replace("+", "")
    assert "best match" not in published["result"].answer
    assert outcome.metadata["places"][0]["views"] == 3

    # No depth: `cluster_places` really does rank by similarity, and `views` was never
    # measured, so "best match" is true and the count must not be reported.
    found_by_score = Place(position=(9.0, 9.0, 0.0), similarity=0.30, source_id=4, ts=2.0)
    module, published = _siglip_module([], [found_by_score])
    outcome = module._find_with_siglip("a basket", 0.0)
    assert outcome.success, outcome.message
    assert "best match" in published["result"].answer
    assert "views" not in published["result"].answer
    assert "views" not in outcome.metadata["places"][0]


def test_an_empty_siglipify_stream_is_not_adopted_as_the_index(sqlite_store: SqliteStore) -> None:
    """The name is not the vectors, and an empty one is a trap with no way out.

    `detect_streams` has skipped empty streams since round 33, with a comment naming the
    incident: a killed ingest leaves the NAME behind and it outranks a real stream. This
    sibling adopted on the name alone, and an empty adoption is worse than a wrong one --
    `count()` reads 0 so `build()` says "nothing to build" and never builds on any run,
    while `_index_status` reports the vectors PRESENT, which hides the viewer's "Add
    embeddings" button. Nothing inside the product can clear it.

    The module makes that artifact itself: `stop()` terminates the embed job, so a
    `memworld --stop` during an "Add embeddings" run kills siglipify mid-write.
    """
    # The stream exists, with the exact name adoption looks for, and holds nothing --
    # which is what a killed siglipify leaves behind.
    images = sqlite_store.stream("color_image", int)
    for source_id, ts in ((1, 1.0), (2, 2.0)):
        images.append(source_id, ts=ts, pose=None)
    empty = f"color_image_{model_slug(GIANT)}"
    sqlite_store.stream(empty, int)  # created, never appended to
    assert empty in sqlite_store.list_streams()
    assert not any(True for _ in sqlite_store.streams[empty])

    index = VisualMemoryIndex(
        sqlite_store, image_stream_name="color_image", model_name=GIANT, pose_of=_placed
    )
    assert index.precomputed_stream_name is None, "an empty stream is not an index"
    # And with it refused, the ordinary path is available again rather than wedged.
    assert index.count() == 0

    # A registry entry whose TABLE the killed writer never created is not an index
    # either, and must not RAISE: on a SqliteStore the count is a raw SELECT, so it
    # throws "no such table" where the old name lookup simply returned. Turning a
    # dormant trap into a live crash is how the first version of this fix went wrong.
    name = f"color_image_{model_slug(GIANT)}"

    class WillNotCount:
        def __getitem__(self, _name: str) -> object:
            raise sqlite3.OperationalError(f"no such table: {name}")

    unreadable = VisualMemoryIndex(
        SimpleNamespace(list_streams=lambda: [name], streams=WillNotCount()),
        image_stream_name="color_image",
        model_name=GIANT,
        pose_of=_placed,
    )
    assert unreadable.precomputed_stream_name is None, "an unreadable stream is not an index"


def test_intrinsics_are_scaled_to_the_raster_they_are_indexed_against() -> None:
    """A camera_info is a calibration, not a promise about the image beside it.

    With no depth camera_info the recording falls back to the COLOUR one, and the patch
    is then sampled at a depth pixel -- correctly -- and lifted with colour numbers. A
    centre patch in an 848x480 depth image is column 424, and back-projecting it through
    a 1280x720 calibration's cx=640 puts it 43 cm off-axis at 2 m. The sampling end was
    right; the lift was wrong, which is why fixing the sample alone did not fix it.
    """
    depth_mm = np.full((480, 848), 2.0, dtype=np.float32)  # 32FC1 depth is metres
    colour_calibration = (900.0, 900.0, 640.0, 360.0)  # solved on 1280x720

    off = patch_world_position((0.5, 0.5), depth_mm, colour_calibration, np.eye(4), window_px=8)
    assert off is not None
    scaled = patch_world_position(
        (0.5, 0.5),
        depth_mm,
        colour_calibration,
        np.eye(4),
        window_px=8,
        intrinsics_size=(1280, 720),
    )
    assert scaled is not None
    # Dead centre of an aligned depth image is dead ahead, whatever the raster.
    assert abs(scaled[0]) < 0.01 and abs(scaled[1]) < 0.01, scaled
    assert scaled[2] == pytest.approx(2.0)
    # Off-axis, where fx matters and not just cx. Every assertion above is at the dead
    # centre, and there u - cx is zero whatever fx is -- so scaling the principal point
    # while leaving the focal length alone passed all of them, and the whole suite, while
    # putting this patch 38 cm short at 2 m. That is the same order as the 43 cm error
    # this scaling exists to remove, on the same query.
    edge = patch_world_position(
        (0.9, 0.5),
        depth_mm,
        colour_calibration,
        np.eye(4),
        window_px=8,
        intrinsics_size=(1280, 720),
    )
    assert edge is not None
    assert edge[0] == pytest.approx((0.9 * 1280 - 640) * 2.0 / 900.0, abs=0.01), edge
    # And the SAME probe on the other axis, because one off-axis point only pins the axis
    # it is off on: with this one horizontal, `fy * sy -> fy` still passed both intrinsics
    # tests. Fixing x and leaving y is the same mistake one axis over.
    down = patch_world_position(
        (0.5, 0.9),
        depth_mm,
        colour_calibration,
        np.eye(4),
        window_px=8,
        intrinsics_size=(1280, 720),
    )
    assert down is not None
    assert down[1] == pytest.approx((0.9 * 720 - 360) * 2.0 / 900.0, abs=0.01), down
    # And without the scaling it is not: this is the error the scaling removes.
    assert abs(off[0]) > 0.4, off

    # A calibration that already matches the raster must be left exactly alone.
    same = patch_world_position(
        (0.5, 0.5),
        depth_mm,
        (900.0, 900.0, 424.0, 240.0),
        np.eye(4),
        window_px=8,
        intrinsics_size=(848, 480),
    )
    assert same is not None and abs(same[0]) < 0.01 and abs(same[1]) < 0.01

    # A raster whose ASPECT differs from the calibration's, which is what actually tells
    # the two scale factors apart. Everything above runs 848x480 against 1280x720, where
    # sx = 0.6625 and sy = 0.6667 -- 0.6% apart, or about 9 mm at 2 m, well inside the
    # 1 cm tolerances. So scaling y by sx, the exact mistake the paragraph above says it
    # exists to catch, passed all of it. 4:3 against 16:9 puts sx at 0.5 and sy at 0.667.
    four_by_three = np.full((480, 640), 2.0, dtype=np.float32)
    across = patch_world_position(
        (0.9, 0.5),
        four_by_three,
        colour_calibration,
        np.eye(4),
        window_px=8,
        intrinsics_size=(1280, 720),
    )
    assert across is not None
    # fx scales by sx alone: fx = 450, cx = 320.
    assert across[0] == pytest.approx((0.9 * 640 - 320) * 2.0 / 450.0, abs=0.01), across
    downward = patch_world_position(
        (0.5, 0.9),
        four_by_three,
        colour_calibration,
        np.eye(4),
        window_px=8,
        intrinsics_size=(1280, 720),
    )
    assert downward is not None
    # ...and fy by sy alone: fy = 600, cy = 240.
    assert downward[1] == pytest.approx((0.9 * 480 - 240) * 2.0 / 600.0, abs=0.01), downward


def test_a_crop_is_not_a_resize_and_only_a_resize_scales_the_focal_length() -> None:
    """roi and binning are stated in the message; inferring them from sizes is a guess.

    A top-left 640x480 crop of a 1280x960 calibration publishes a smaller image with the
    SAME fx -- only the principal point moves. Reading that as a resize halved fx and cx
    together and moved the sampled pixel from (480, 240) to (240, 120), which on a scene
    with the object at 2 m against a 5 m background returned the background: (-1.0, -1.5,
    5.0) where (-0.4, -0.6, 2.0) was right.
    """
    full = SimpleNamespace(
        K=[900.0, 0.0, 640.0, 0.0, 900.0, 480.0, 0.0, 0.0, 1.0],
        width=1280,
        height=960,
        binning_x=0,
        binning_y=0,
        roi_x_offset=0,
        roi_y_offset=0,
        roi_width=0,
        roi_height=0,
    )

    # No roi, no binning: the numbers and the raster are the calibration's own.
    assert sensor_intrinsics(full) == ((900.0, 900.0, 640.0, 480.0), (1280, 960))

    # A top-left 640x480 crop: fx is untouched, the principal point moves with the window.
    crop = SimpleNamespace(**{**vars(full), "roi_width": 640, "roi_height": 480})
    assert sensor_intrinsics(crop) == ((900.0, 900.0, 640.0, 480.0), (640, 480))

    # An offset crop moves it by the offset, and still does not touch fx.
    offset = SimpleNamespace(**{**vars(crop), "roi_x_offset": 100, "roi_y_offset": 50})
    assert sensor_intrinsics(offset) == ((900.0, 900.0, 540.0, 430.0), (640, 480))

    # 2x binning halves everything, including fx -- binning really is a resize.
    binned = SimpleNamespace(**{**vars(full), "binning_x": 2, "binning_y": 2})
    assert sensor_intrinsics(binned) == ((450.0, 450.0, 320.0, 240.0), (640, 480))

    # And the end-to-end consequence: the crop's own pixel lifts to the crop's own ray.
    depth_mm = np.full((480, 640), 5.0, dtype=np.float32)  # 5 m background
    depth_mm[235:245, 475:485] = 2.0  # the object, at the crop's (480, 240)
    intrinsics, size = sensor_intrinsics(crop)
    where = patch_world_position(
        (480 / 640, 240 / 480),
        depth_mm,
        intrinsics,
        np.eye(4),
        window_px=8,
        intrinsics_size=size,
    )
    assert where is not None
    assert where[2] == pytest.approx(2.0), "read the background instead of the object"
    # The crop's own pinhole: (u - cx) * z / fx with the UNSCALED fx, which is the whole
    # point -- halving fx with the raster would have put it at half this offset.
    assert where[0] == pytest.approx((480 - 640) * 2.0 / 900.0), where
    assert where[1] == pytest.approx((240 - 480) * 2.0 / 900.0), where


def test_an_embedding_answer_carries_the_places_the_viewer_renders() -> None:
    """The client builds its results bar, place stepping and Navigate from `clusters`.

    An answer that carries only `points` leaves all three inert: the bar reads "0 places",
    `results.go(0)` returns false because `this.clusters.length` is 0, and
    `results.navigate()` returns null before it ever reaches the route. Measured live
    against a real recording -- ask succeeded, six places found, six evidence photos hung,
    4308 voxels lit, and Navigate did nothing at all.

    So the embedding answer publishes the same summary shape Hyperspace does, and the
    viewer needs no branch. The keys are `HeatmapCluster.summary()`'s.
    """
    from dimos.teleop.memory_world.hyperspace_search import Cluster

    wanted = set(
        Cluster(
            index=0,
            centre=(0.0, 0.0, 0.0),
            radius=1.0,
            score=0.5,
            peak=0.5,
            n_voxels=1,
            views=1,
            evidence=[],
        ).summary()
    )
    published = {
        "index",
        "centre",
        "radius",
        "score",
        "peak",
        "n_voxels",
        "n_views",
        "n_evidence",
    }

    assert published == wanted, (
        "the embedding answer's cluster summary has drifted from the one the viewer reads"
    )
