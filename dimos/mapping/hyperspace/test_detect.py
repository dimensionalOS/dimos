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

"""The frames-first path: episodes, the 2D box plus depth becoming a 3D box, and what
happens when the detector refuses.

The geometry tests use numbers small enough to check by hand -- a 64x48 camera with
fx = fy = 48 and its centre at (32, 24), and a square of depth planted at a known
distance -- so a wrong answer says which step is wrong rather than only that one is.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.mapping.hyperspace import patches as hs
from dimos.mapping.hyperspace.detect import (
    Box3D,
    DetectConfig,
    Detection,
    Owlv2Boxes,
    RecordingFrames,
    box_from_points,
    detect_episode,
    find,
    merge_duplicates,
    object_points,
    place_of,
    spread_by_place,
)
from dimos.mapping.hyperspace.frames import Episode, Frame, Hit, episodes, ranked_episodes
from dimos.mapping.hyperspace.ingest import filled_stream_for
from dimos.memory.codecs.lcm import LcmCodec
from dimos.memory.codecs.lz4 import Lz4Codec
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

WIDTH, HEIGHT = 64, 48
FOCAL, CX, CY = 48.0, 32.0, 24.0
CAMERA = "camera_optical"
WORLD = "odom"

# The planted square, in pixels, and how far away it is.
SQUARE = (28, 36, 20, 28)  # left, right, top, bottom
SQUARE_DEPTH = 2.0
BACKGROUND_DEPTH = 8.0

# Eight columns 1/48 of a radian apart, at two metres: the outermost centres sit
# +-3.5/48 * 2 m from the axis, and the 2nd/98th percentiles of eight repeated values
# land on those same outermost ones.
HALF_EXTENT = 3.5 / FOCAL * SQUARE_DEPTH


def intrinsics() -> hs.Intrinsics:
    return hs.Intrinsics(width=WIDTH, height=HEIGHT, fx=FOCAL, fy=FOCAL, cx=CX, cy=CY)


def planted_depth(right: int = SQUARE[1]) -> np.ndarray:
    """Background everywhere, with a square of near depth in the middle."""
    depth = np.full((HEIGHT, WIDTH), BACKGROUND_DEPTH, dtype=np.float32)
    depth[SQUARE[2] : SQUARE[3], SQUARE[0] : right] = SQUARE_DEPTH
    return depth


def frame_at(ts: float, score: float = 1.0, member: str = "stub") -> Frame:
    hit = Hit(
        member=member,
        frame=CAMERA,
        ts=ts,
        cell=0,
        grid=(2, 2),
        ray=(0.0, 0.0),
        depth=SQUARE_DEPTH,
        score=score,
    )
    return Frame(frame=CAMERA, ts=ts, hits=[hit])


# --- episodes -----------------------------------------------------------------------


def test_episodes_split_on_a_gap_in_time() -> None:
    frames = [frame_at(ts) for ts in (0.0, 0.25, 0.5, 5.0, 5.25)]
    found = episodes(frames, gap_s=1.0)
    assert [len(episode.frames) for episode in found] == [3, 2]
    assert found[0].span == pytest.approx(0.5)
    assert found[1].start == pytest.approx(5.0)


def test_a_wider_gap_joins_what_a_narrow_one_split() -> None:
    frames = [frame_at(ts) for ts in (0.0, 0.25, 5.0)]
    assert len(episodes(frames, gap_s=1.0)) == 2
    assert len(episodes(frames, gap_s=10.0)) == 1


def test_the_peak_frame_is_the_one_with_the_most_match_in_it() -> None:
    frames = [frame_at(0.0, score=0.1), frame_at(0.25, score=0.9), frame_at(0.5, score=0.4)]
    episode = Episode(frames=frames)
    assert episode.peak.ts == pytest.approx(0.25)
    assert [frame.ts for frame in episode.by_weight()] == [0.25, 0.5, 0.0]


def test_ranked_episodes_drop_strays_and_rank_by_score() -> None:
    frames = [
        frame_at(0.0, score=0.2),
        frame_at(0.25, score=0.2),
        frame_at(9.0, score=5.0),  # one lone frame, however hot
        frame_at(20.0, score=0.9),
        frame_at(20.25, score=0.9),
    ]
    found = ranked_episodes(frames, gap_s=1.0, min_frames=2)
    assert [episode.start for episode in found] == [20.0, 0.0]
    assert all(len(episode.frames) >= 2 for episode in found)


# --- a 2D box and a depth image becoming a 3D box -----------------------------------


def test_object_points_measure_the_planted_square() -> None:
    found = object_points((28.0, 20.0, 36.0, 28.0), (WIDTH, HEIGHT), planted_depth(), intrinsics())
    assert found, found.why
    points, median = found.points, found.median
    assert median == pytest.approx(SQUARE_DEPTH)
    assert len(points) == 8 * 8
    assert points[:, 2].min() == pytest.approx(SQUARE_DEPTH)
    assert points[:, 0].min() == pytest.approx(-HALF_EXTENT)
    assert points[:, 0].max() == pytest.approx(HALF_EXTENT)


def test_the_depth_band_drops_the_background_showing_through_the_box() -> None:
    """A box that is two thirds object and one third the aisle behind it."""
    depth = planted_depth()
    box = (28.0, 20.0, 40.0, 28.0)  # four columns wider than the square
    found = object_points(box, (WIDTH, HEIGHT), depth, intrinsics())
    assert found, found.why
    points, median = found.points, found.median
    assert median == pytest.approx(SQUARE_DEPTH)
    # Only the square's own 64 pixels survive; the 32 background ones are 6 m away.
    assert len(points) == 8 * 8
    assert points[:, 2].max() == pytest.approx(SQUARE_DEPTH)


def test_no_usable_depth_inside_the_box_is_reported_rather_than_guessed() -> None:
    empty = np.zeros((HEIGHT, WIDTH), dtype=np.float32)
    found = object_points((28.0, 20.0, 36.0, 28.0), (WIDTH, HEIGHT), empty, intrinsics())
    assert not found
    assert found.why == "the depth image has no reading at all inside the box"


def test_depth_past_the_cut_off_is_not_reported_as_no_depth() -> None:
    """The distinction Jeff asked for: a working sensor, refused, says so.

    Everything in the box reads 40 m. That is not a hole -- depth2depth has nothing to
    fix -- it is the far cut-off doing its job, and the two used to arrive under the
    same sentence, which is how "no usable depth" came to look like a depth bug.
    """
    far = np.full((HEIGHT, WIDTH), 40.0, dtype=np.float32)
    found = object_points((28.0, 20.0, 36.0, 28.0), (WIDTH, HEIGHT), far, intrinsics())
    assert not found
    assert (
        found.why == "every depth reading inside the box is past the 10 m cut-off (nearest 40.0 m)"
    )


def test_too_few_readings_says_how_many_there_were() -> None:
    depth = np.zeros((HEIGHT, WIDTH), dtype=np.float32)
    depth[20:28, 28:31] = SQUARE_DEPTH  # 24 pixels of object, spread over three columns
    found = object_points(
        (28.0, 20.0, 36.0, 28.0), (WIDTH, HEIGHT), depth, intrinsics(), min_pixels=30
    )
    assert not found
    assert found.why == "only 24 depth reading(s) inside the box within 10 m, 30 needed"


def test_box_from_points_under_the_identity_pose() -> None:
    found = object_points((28.0, 20.0, 36.0, 28.0), (WIDTH, HEIGHT), planted_depth(), intrinsics())
    assert found, found.why
    points, median = found.points, found.median
    box = box_from_points(points, np.eye(4), WORLD, median)
    assert box.frame == WORLD
    assert box.centre == pytest.approx((0.0, 0.0, SQUARE_DEPTH), abs=1e-6)
    assert box.extent == pytest.approx((2 * HALF_EXTENT, 2 * HALF_EXTENT, 0.0), abs=1e-6)
    assert box.pixels == 64
    assert box.depth_m == pytest.approx(SQUARE_DEPTH)


def test_box_from_points_carries_the_camera_pose() -> None:
    """Camera at (10, -4, 1) yawed 90 degrees: optical +z points along world +y.

    Optical axes are x right, y down, z forward. A yaw of 90 degrees about world z
    sends camera x -> world -x... so the columns below spell out where each optical
    axis goes, and the object two metres ahead lands two metres along world +y.
    """
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack(
        [
            [-1.0, 0.0, 0.0],  # optical x (right) -> world -x
            [0.0, 0.0, -1.0],  # optical y (down)  -> world -z
            [0.0, 1.0, 0.0],  # optical z (ahead) -> world +y
        ]
    )
    pose[:3, 3] = [10.0, -4.0, 1.0]
    found = object_points((28.0, 20.0, 36.0, 28.0), (WIDTH, HEIGHT), planted_depth(), intrinsics())
    assert found, found.why
    points, median = found.points, found.median
    box = box_from_points(points, pose, WORLD, median)
    assert box.centre == pytest.approx((10.0, -4.0 + SQUARE_DEPTH, 1.0), abs=1e-6)
    # The square's width is now along world x and its height along world z.
    assert box.extent == pytest.approx((2 * HALF_EXTENT, 0.0, 2 * HALF_EXTENT), abs=1e-6)


def test_a_box_on_a_different_grid_than_the_intrinsics_still_lands() -> None:
    """Depth at half the colour camera's resolution: the box must land in the same place.

    The square is then 4x4 pixels instead of 8x8, so its extent is measured between the
    centres of the outermost of four columns rather than of eight: +-1.5 px at a
    half-grid focal length of 24, which is +-0.125 m at two metres.
    """
    half = planted_depth()[::2, ::2]
    found = object_points(
        (28.0, 20.0, 36.0, 28.0), (WIDTH, HEIGHT), half, intrinsics(), min_pixels=8
    )
    assert found, found.why
    points, median = found.points, found.median
    assert len(points) == 4 * 4
    box = box_from_points(points, np.eye(4), WORLD, median)
    assert box.centre == pytest.approx((0.0, 0.0, SQUARE_DEPTH), abs=1e-6)
    assert box.extent[0] == pytest.approx(2 * (1.5 / (FOCAL / 2)) * SQUARE_DEPTH, abs=1e-6)


# --- the whole episode, against a real store ----------------------------------------


class StubBoxes:
    """A detector that answers with a fixed box, or refuses.

    Counts images shown and forward passes separately: batching is exactly the
    difference between the two, so a test can hold one fixed and assert on the other.
    """

    def __init__(self, box: tuple[float, float, float, float] | None, score: float = 0.5) -> None:
        self.box = box
        self.score = score
        self.calls = 0
        self.passes = 0

    def answer(self) -> tuple[tuple[float, float, float, float], float] | None:
        return None if self.box is None else (self.box, self.score)

    def all_many(
        self, images: Sequence[Image], text: str
    ) -> list[list[tuple[tuple[float, float, float, float], float]]]:
        del text
        self.passes += 1
        answers = []
        for _ in images:
            self.calls += 1
            found = self.answer()
            answers.append([] if found is None else [found])
        return answers

    def best_many(
        self, images: Sequence[Image], text: str
    ) -> list[tuple[tuple[float, float, float, float], float] | None]:
        return [found[0] if found else None for found in self.all_many(images, text)]

    def best(
        self, image: Image, text: str
    ) -> tuple[tuple[float, float, float, float], float] | None:
        return self.best_many([image], text)[0]


class RefusesThenAnswers(StubBoxes):
    """Refuses the first frame it is shown and answers the second."""

    def answer(self) -> tuple[tuple[float, float, float, float], float] | None:
        if self.calls == 1:
            return None
        return (self.box, self.score)  # type: ignore[return-value]


def camera_info(frame_id: str = CAMERA) -> CameraInfo:
    info = CameraInfo(width=WIDTH, height=HEIGHT, distortion_model="plumb_bob", frame_id=frame_id)
    info.set_K_matrix(np.array([[FOCAL, 0.0, CX], [0.0, FOCAL, CY], [0.0, 0.0, 1.0]]))
    return info


@pytest.fixture
def recording(tmp_path: Path) -> SqliteStore:
    """A minimal recording: four colour frames, their depth, intrinsics and one tf.

    The frames come in two pairs a long way apart in time, so the same fixture serves a
    single-episode test and one that needs two episodes.
    """
    store = SqliteStore(path=str(tmp_path / "recording.db"))
    store.start()
    store.stream("camera_info", CameraInfo).append(camera_info(), ts=10.0)
    store.stream("depth_camera_info", CameraInfo).append(camera_info(), ts=10.0)
    colors = store.stream("color_image", Image)
    # A real recording stores depth lz4+lcm; the default Image codec is JPEG, which
    # would quietly hand back an 8-bit RGB picture of the depth instead of metres.
    depths = store.stream("depth_image", Image, codec=Lz4Codec(LcmCodec(Image)))
    for ts in (10.0, 10.25, 30.0, 30.25):
        colors.append(
            Image.from_numpy(
                np.full((HEIGHT, WIDTH, 3), 128, dtype=np.uint8), frame_id=CAMERA, ts=ts
            ),
            ts=ts,
        )
        depths.append(
            Image.from_numpy(
                (planted_depth() * 1000).astype(np.uint16),
                format=ImageFormat.DEPTH16,
                frame_id=CAMERA,
                ts=ts,
            ),
            ts=ts,
        )
    store.stream("tf", TFMessage).append(
        TFMessage(
            Transform(
                translation=Vector3(1.0, 2.0, 3.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id=WORLD,
                child_frame_id=CAMERA,
                ts=10.0,
            )
        ),
        ts=10.0,
    )
    yield store
    store.stop()


def test_detect_episode_places_the_box_in_the_world(recording: SqliteStore) -> None:
    episode = Episode(frames=[frame_at(10.0, 0.9), frame_at(10.25, 0.4)])
    boxes = StubBoxes((28.0, 20.0, 36.0, 28.0))
    config = DetectConfig(world_frame=WORLD)
    found = detect_episode(
        episode, "a square", RecordingFrames(recording, config=config), boxes, rank=1, config=config
    )
    assert found.found
    assert boxes.calls == 1, "the peak frame answered, so nothing else should be tried"
    assert found.ts == pytest.approx(10.0)
    assert found.score == pytest.approx(0.5)
    assert found.box3d is not None
    # The camera sits at (1, 2, 3) with no rotation, so the square is two metres along
    # its optical z, which is world z.
    assert found.box3d.centre == pytest.approx((1.0, 2.0, 3.0 + SQUARE_DEPTH), abs=1e-6)
    assert found.box3d.frame == WORLD
    assert found.box3d.depth_m == pytest.approx(SQUARE_DEPTH)


def test_an_episode_the_detector_refuses_is_reported_not_dropped(recording: SqliteStore) -> None:
    episode = Episode(frames=[frame_at(10.0, 0.9), frame_at(10.25, 0.4)])
    boxes = StubBoxes(None)
    config = DetectConfig(world_frame=WORLD, attempts=2)
    found = detect_episode(
        episode, "a square", RecordingFrames(recording, config=config), boxes, rank=3, config=config
    )
    assert isinstance(found, Detection)
    assert not found.found
    assert found.box2d is None and found.box3d is None
    assert found.rank == 3 and found.episode_frames == 2
    assert boxes.calls == 2, "both frames of the episode should have been tried"
    assert "refused" in found.note


def test_the_second_frame_is_tried_when_the_peak_frame_fails(recording: SqliteStore) -> None:
    episode = Episode(frames=[frame_at(10.0, 0.9), frame_at(10.25, 0.4)])
    boxes = RefusesThenAnswers((28.0, 20.0, 36.0, 28.0))
    config = DetectConfig(world_frame=WORLD, attempts=3)
    found = detect_episode(
        episode, "a square", RecordingFrames(recording, config=config), boxes, rank=1, config=config
    )
    assert found.found
    assert found.attempts == 2
    assert found.ts == pytest.approx(10.25), "the answer came from the second-best frame"


def test_a_detection_without_depth_still_reports_the_2d_box(recording: SqliteStore) -> None:
    """Depth that is all holes: the image answer survives, the world answer says why."""
    blank = recording.stream("depth_image", Image, codec=Lz4Codec(LcmCodec(Image)))
    blank.append(
        Image.from_numpy(
            np.zeros((HEIGHT, WIDTH), dtype=np.uint16),
            format=ImageFormat.DEPTH16,
            frame_id=CAMERA,
            ts=50.0,
        ),
        ts=50.0,
    )
    recording.stream("color_image", Image).append(
        Image.from_numpy(
            np.full((HEIGHT, WIDTH, 3), 128, dtype=np.uint8), frame_id=CAMERA, ts=50.0
        ),
        ts=50.0,
    )
    episode = Episode(frames=[frame_at(50.0, 0.9)])
    config = DetectConfig(world_frame=WORLD)
    found = detect_episode(
        episode,
        "a square",
        RecordingFrames(recording, config=config),
        StubBoxes((28.0, 20.0, 36.0, 28.0)),
        rank=1,
        config=config,
    )
    assert found.found and found.box2d is not None
    assert found.box3d is None
    assert found.note == "the depth image has no reading at all inside the box"


def test_one_model_falls_back_to_episodes(recording: SqliteStore, monkeypatch) -> None:
    """Agreement needs somebody to agree with; one model alone keeps the old path."""
    frames = [frame_at(ts, 0.9) for ts in (10.0, 10.25)]
    monkeypatch.setattr("dimos.mapping.hyperspace.frames.hot_frames", lambda *a, **k: frames)
    boxes = StubBoxes((28.0, 20.0, 36.0, 28.0))
    config = DetectConfig(world_frame=WORLD)
    assert config.agreement, "on by default, and this test is that it still yields"
    answers = list(
        find(
            recording,
            recording,
            "a square",
            config=config,
            frames=RecordingFrames(recording, config=config),
            boxes=boxes,
        )
    )
    assert answers, "one model is not a reason to answer nothing"


def test_a_batch_takes_every_episode_through_one_forward_pass(
    recording: SqliteStore, monkeypatch
) -> None:
    """When a batch is worth having, the round of episodes goes through it together."""
    frames = [frame_at(ts, 0.9) for ts in (10.0, 10.25)] + [
        frame_at(ts, 0.4) for ts in (30.0, 30.25)
    ]
    monkeypatch.setattr("dimos.mapping.hyperspace.frames.hot_frames", lambda *a, **k: frames)
    boxes = StubBoxes((28.0, 20.0, 36.0, 28.0))
    config = DetectConfig(world_frame=WORLD, batch=8)
    answers = list(
        find(
            recording,
            recording,
            "a square",
            config=config,
            frames=RecordingFrames(recording, config=config),
            boxes=boxes,
        )
    )
    assert [answer.rank for answer in answers] == [1, 2]
    assert boxes.calls == 2, "one image per episode"
    assert boxes.passes == 1, "and both of them in the same forward pass"


def test_without_a_batch_an_answer_arrives_before_the_next_is_started(
    recording: SqliteStore, monkeypatch
) -> None:
    """The default path hands back each answer as it settles.

    Waiting for the whole round would put the first answer after twelve detector calls
    instead of one, and at a batch of one the wait buys nothing.
    """
    frames = [frame_at(ts, 0.9) for ts in (10.0, 10.25)] + [
        frame_at(ts, 0.4) for ts in (30.0, 30.25)
    ]
    monkeypatch.setattr("dimos.mapping.hyperspace.frames.hot_frames", lambda *a, **k: frames)
    boxes = StubBoxes((28.0, 20.0, 36.0, 28.0))
    config = DetectConfig(world_frame=WORLD)
    assert config.batch == 1, "the default this test is about"
    answers = find(
        recording,
        recording,
        "a square",
        config=config,
        frames=RecordingFrames(recording, config=config),
        boxes=boxes,
    )
    first = next(answers)
    assert first.rank == 1
    assert boxes.calls == 1, "the second episode has not been shown to the detector yet"
    rest = list(answers)
    assert [answer.rank for answer in rest] == [2]
    assert boxes.calls == 2


def test_an_episode_that_answered_is_not_shown_a_second_frame(
    recording: SqliteStore, monkeypatch
) -> None:
    """Batching must not cost extra work: a settled episode sits out later rounds."""
    frames = [frame_at(ts, 0.9) for ts in (10.0, 10.25, 10.5)]
    monkeypatch.setattr("dimos.mapping.hyperspace.frames.hot_frames", lambda *a, **k: frames)
    boxes = StubBoxes((28.0, 20.0, 36.0, 28.0))
    config = DetectConfig(world_frame=WORLD, attempts=3)
    answers = list(
        find(
            recording,
            recording,
            "a square",
            config=config,
            frames=RecordingFrames(recording, config=config),
            boxes=boxes,
        )
    )
    assert len(answers) == 1 and answers[0].box3d is not None
    assert boxes.calls == 1, "it answered on the first frame; the other two are never shown"
    assert boxes.passes == 1


class CountingDetector:
    """Stands in for core's OWLv2, recording the size of every forward pass."""

    def __init__(self) -> None:
        self.passes: list[int] = []

    def query_detections_batch(self, images, queries, threshold):  # type: ignore[no-untyped-def]
        del queries, threshold
        self.passes.append(len(images))
        return [SimpleNamespace(detections=[]) for _ in images]


def test_a_round_is_split_at_the_configured_batch_size() -> None:
    """The cap bounds how much of the GPU one round can ask for.

    Ten frames at a cap of four is three passes of 4, 4 and 2 -- not ten passes, and
    not one pass of ten.
    """
    boxes = Owlv2Boxes(DetectConfig(batch=4))
    detector = CountingDetector()
    boxes._detector = detector
    answers = boxes.best_many([object()] * 10, "a square")  # type: ignore[list-item]
    assert detector.passes == [4, 4, 2]
    assert answers == [None] * 10, "every image was answered, in order"


def test_one_image_is_still_one_pass() -> None:
    """`best` is the single-image case of the same path, not a second code path."""
    boxes = Owlv2Boxes(DetectConfig(batch=8))
    detector = CountingDetector()
    boxes._detector = detector
    assert boxes.best(object(), "a square") is None  # type: ignore[arg-type]
    assert detector.passes == [1]


class PlacesAt:
    """`RecordingFrames`' pose lookup, with the answer chosen by the test.

    `spread_by_place` asks nothing else of a recording, so standing in for that one
    method keeps the ordering under test instead of the fixture's transforms.
    """

    def __init__(self, by_ts: dict[float, tuple[float, float, float] | None]) -> None:
        self.by_ts = by_ts

    def pose(self, camera_frame: str, ts: float, world_frame: str):  # type: ignore[no-untyped-def]
        del camera_frame, world_frame
        where = self.by_ts.get(round(ts, 3))
        if where is None:
            return None
        pose = np.eye(4)
        pose[:3, 3] = where
        return pose


def two_looks_each_at_two_places() -> tuple[list[Episode], PlacesAt]:
    """Four episodes: near, near again, far, far again -- strongest first."""
    made = [
        Episode(frames=[frame_at(10.0, 0.9)]),
        Episode(frames=[frame_at(10.5, 0.8)]),
        Episode(frames=[frame_at(30.0, 0.4)]),
        Episode(frames=[frame_at(30.5, 0.3)]),
    ]
    return made, PlacesAt({10.0: (0, 0, 0), 10.5: (0.1, 0, 0), 30.0: (9, 0, 0), 30.5: (9.1, 0, 0)})


def test_one_look_at_each_place_before_a_second_look_at_any() -> None:
    """Strongest-first would detect both looks at the near thing before seeing the far.

    That is how a real second cone went unanswered while the first collected seven
    boxes: the budget was spent on the loudest place rather than on distinct ones.
    """
    made, frames = two_looks_each_at_two_places()
    config = DetectConfig(world_frame=WORLD, place_radius_m=0.75)
    ordered = spread_by_place(made, frames, config=config)  # type: ignore[arg-type]
    assert [episode.peak.ts for episode in ordered] == [10.0, 30.0, 10.5, 30.5]


def test_without_spreading_the_loudest_place_takes_the_whole_budget() -> None:
    """The behaviour being replaced, stated so the fix cannot be mistaken for a no-op."""
    made, _ = two_looks_each_at_two_places()
    assert [episode.peak.ts for episode in made] == [10.0, 10.5, 30.0, 30.5]
    assert [episode.peak.ts for episode in made][:2] == [10.0, 10.5], "both at one place"


def test_an_unplaceable_episode_keeps_its_turn() -> None:
    """Not knowing where an episode is says nothing about whether it is worth detecting."""
    made = [Episode(frames=[frame_at(10.0, 0.9)]), Episode(frames=[frame_at(30.0, 0.8)])]
    frames = PlacesAt({10.0: (0, 0, 0), 30.0: None})
    ordered = spread_by_place(made, frames, config=DetectConfig(world_frame=WORLD))  # type: ignore[arg-type]
    assert len(ordered) == 2, "nothing was dropped for being unplaceable"
    assert place_of(ordered[1], frames, WORLD) is None  # type: ignore[arg-type]


def test_spreading_never_changes_which_episodes_exist() -> None:
    """It is an ordering. Every episode in, every episode out, exactly once."""
    made, frames = two_looks_each_at_two_places()
    ordered = spread_by_place(made, frames, config=DetectConfig(world_frame=WORLD))  # type: ignore[arg-type]
    assert sorted(id(episode) for episode in ordered) == sorted(id(episode) for episode in made)


def test_geometry_matches_a_hand_computation() -> None:
    """The whole chain, written out longhand, against the code.

    A pixel at column *c* of a camera with focal length f and centre cx sees a ray of
    slope ``(c + 0.5 - cx) / f``; at distance d that is ``slope * d`` metres off the
    axis. Nothing else is involved.
    """
    left, right, top, bottom = SQUARE
    xs, ys = [], []
    for col in range(left, right):
        for row in range(top, bottom):
            xs.append((col + 0.5 - CX) / FOCAL * SQUARE_DEPTH)
            ys.append((row + 0.5 - CY) / FOCAL * SQUARE_DEPTH)
    by_hand_x = np.percentile(xs, 98) - np.percentile(xs, 2)
    by_hand_y = np.percentile(ys, 98) - np.percentile(ys, 2)

    found = object_points((28.0, 20.0, 36.0, 28.0), (WIDTH, HEIGHT), planted_depth(), intrinsics())
    assert found, found.why
    box = box_from_points(found.points, np.eye(4), WORLD, found.median)
    assert box.extent[0] == pytest.approx(by_hand_x)
    assert box.extent[1] == pytest.approx(by_hand_y)
    assert math.isclose(box.extent[0], 2 * HALF_EXTENT, rel_tol=1e-9)


def test_an_unplaceable_frame_falls_through_to_the_next_one(recording: SqliteStore) -> None:
    """Detected but with no depth behind the box: try the next look at the same thing.

    Stereo gives nothing back off glass, a dark shelf or a shiny floor, and that is a
    property of one frame, not of the object. The next-best frame of the episode is
    another chance at the same thing, so an empty depth image must not end the episode.
    """
    colors = recording.stream("color_image", Image)
    depths = recording.stream("depth_image", Image, codec=Lz4Codec(LcmCodec(Image)))
    for ts, depth in ((60.0, np.zeros((HEIGHT, WIDTH), np.uint16)), (60.25, None)):
        colors.append(
            Image.from_numpy(
                np.full((HEIGHT, WIDTH, 3), 128, dtype=np.uint8), frame_id=CAMERA, ts=ts
            ),
            ts=ts,
        )
        pixels = (planted_depth() * 1000).astype(np.uint16) if depth is None else depth
        depths.append(
            Image.from_numpy(pixels, format=ImageFormat.DEPTH16, frame_id=CAMERA, ts=ts), ts=ts
        )

    episode = Episode(frames=[frame_at(60.0, 0.9), frame_at(60.25, 0.4)])
    boxes = StubBoxes((28.0, 20.0, 36.0, 28.0))
    config = DetectConfig(world_frame=WORLD, attempts=3)
    found = detect_episode(
        episode, "a square", RecordingFrames(recording, config=config), boxes, rank=1, config=config
    )
    assert found.found
    assert found.box3d is not None, "the second frame's depth should have placed it"
    assert found.ts == pytest.approx(60.25)
    assert boxes.calls == 2


def test_a_detection_that_can_never_be_placed_is_still_returned(recording: SqliteStore) -> None:
    """Every frame of the episode is unplaceable: report the 2D answer and say why."""
    colors = recording.stream("color_image", Image)
    depths = recording.stream("depth_image", Image, codec=Lz4Codec(LcmCodec(Image)))
    for ts in (70.0, 70.25):
        colors.append(
            Image.from_numpy(
                np.full((HEIGHT, WIDTH, 3), 128, dtype=np.uint8), frame_id=CAMERA, ts=ts
            ),
            ts=ts,
        )
        depths.append(
            Image.from_numpy(
                np.zeros((HEIGHT, WIDTH), np.uint16),
                format=ImageFormat.DEPTH16,
                frame_id=CAMERA,
                ts=ts,
            ),
            ts=ts,
        )
    episode = Episode(frames=[frame_at(70.0, 0.9), frame_at(70.25, 0.4)])
    config = DetectConfig(world_frame=WORLD, attempts=3)
    found = detect_episode(
        episode,
        "a square",
        RecordingFrames(recording, config=config),
        StubBoxes((28.0, 20.0, 36.0, 28.0)),
        rank=1,
        config=config,
    )
    assert found.found and found.box3d is None
    assert found.ts == pytest.approx(70.0), "the first, strongest attempt is the one kept"
    assert found.note == "the depth image has no reading at all inside the box"


def placed(rank: int, score: float, centre: tuple[float, float, float]) -> Detection:
    detection = Detection(
        query="a square",
        rank=rank,
        ts=float(rank),
        camera_frame=CAMERA,
        episode_frames=3,
        episode_span=1.0,
        episode_score=score,
        models=["stub"],
        attempts=1,
        score=score,
    )
    detection.box2d = (0.0, 0.0, 1.0, 1.0)
    detection.box3d = Box3D(
        frame=WORLD, centre=centre, extent=(0.2, 0.2, 0.2), pixels=50, depth_m=1.0
    )
    return detection


class FindsTwoThings(StubBoxes):
    """A photograph with two of the thing in it, which is what OWLv2 really returns."""

    def all_many(self, images, text):  # type: ignore[no-untyped-def]
        del text
        self.passes += 1
        answers = []
        for _ in images:
            self.calls += 1
            answers.append([((28.0, 20.0, 36.0, 28.0), 0.8), ((4.0, 4.0, 12.0, 12.0), 0.6)])
        return answers


def test_two_things_in_one_photograph_are_two_answers(recording: SqliteStore, monkeypatch) -> None:
    """Keeping only the strongest box lost every second instance sharing a view."""
    frames = [frame_at(ts, 0.9) for ts in (10.0, 10.25)]
    monkeypatch.setattr("dimos.mapping.hyperspace.frames.hot_frames", lambda *a, **k: frames)
    boxes = FindsTwoThings((28.0, 20.0, 36.0, 28.0))
    config = DetectConfig(world_frame=WORLD)
    answers = list(
        find(
            recording,
            recording,
            "a square",
            config=config,
            frames=RecordingFrames(recording, config=config),
            boxes=boxes,
        )
    )
    assert len(answers) == 2, "one episode, two things seen in it"
    assert [answer.score for answer in answers] == [0.8, 0.6], "strongest is the episode's"
    assert len({answer.rank for answer in answers}) == 2, "and the second gets its own rank"
    assert all(answer.box3d is not None for answer in answers)


def test_a_second_look_sharpens_the_place_instead_of_adding_a_box() -> None:
    """Two looks at one shelf are one place whose box is the average of the looks."""
    near = placed(1, 0.5, (1.0, 0.0, 0.0))
    again = placed(2, 0.5, (1.2, 0.0, 0.0))
    assert near.box3d is not None and again.box3d is not None
    near.box3d = replace(near.box3d, extent=(0.4, 0.4, 0.4))
    again.box3d = replace(again.box3d, extent=(0.6, 0.6, 0.6))
    assert merge_duplicates([near, again], merge_m=0.75) == 1
    assert near.place_id == again.place_id == 1
    assert again.duplicate_of == 1, "the second look points at the first"
    assert near.refined is not None and again.refined is not None
    assert near.refined.centre == pytest.approx((1.0, 0.0, 0.0)), "the first look is itself"
    assert again.refined.centre == pytest.approx((1.1, 0.0, 0.0)), "then the average of both"
    assert again.refined.extent == pytest.approx((0.5, 0.5, 0.5))
    assert again.box3d is not None and again.box3d.centre == pytest.approx((1.2, 0.0, 0.0)), (
        "the look keeps its own box; refining does not rewrite the evidence"
    )


def test_a_confident_look_pulls_the_place_further_than_a_doubtful_one() -> None:
    """Averaging by score, so a 0.9 answer is not dragged about by a 0.2 one."""
    sure = placed(1, 0.9, (1.0, 0.0, 0.0))
    unsure = placed(2, 0.1, (1.5, 0.0, 0.0))
    merge_duplicates([sure, unsure], merge_m=0.75)
    assert unsure.refined is not None
    assert unsure.refined.centre[0] == pytest.approx(1.05), "a tenth of the way, not halfway"


def test_a_separate_place_gets_its_own_id() -> None:
    """The id is what tells a caller 'new thing' from 'better look at the same thing'."""
    here = placed(1, 0.5, (0.0, 0.0, 0.0))
    far = placed(2, 0.5, (5.0, 0.0, 0.0))
    assert merge_duplicates([here, far], merge_m=0.75) == 2
    assert here.place_id == 1 and far.place_id == 2
    assert far.duplicate_of is None


def test_several_looks_at_one_shelf_become_one_place() -> None:
    """The real "cheese" answers on grocery.db: five episodes, three places.

    Three of them are the same metre of the cheese fridge and collapse onto the
    strongest. The fourth is 1.2 m along the same aisle and stays its own answer --
    grouping is around a representative, not single-link, so a chain of near-neighbours
    cannot swallow the length of a shelf.
    """
    found = [
        placed(1, 0.34, (8.3, 20.8, -2.1)),
        placed(2, 0.43, (8.2, 20.9, -2.2)),
        placed(3, 0.47, (8.2, 20.9, -2.2)),
        placed(4, 0.40, (7.5, 19.9, -2.1)),
        placed(5, 0.48, (-1.3, 24.9, -1.3)),
    ]
    assert merge_duplicates(found, merge_m=0.75) == 3
    # The place belongs to the look that found it, and later looks point back at that
    # one -- answers are folded in the order they arrived, so that a caller replaying a
    # query sees the same thing a caller watching it saw.
    assert found[0].duplicate_of is None
    assert [d.duplicate_of for d in found] == [None, 1, 1, None, None]
    assert [d.place_id for d in found] == [1, 1, 1, 2, 3]
    # Widen it and the fourth joins them.
    assert merge_duplicates(found, merge_m=1.5) == 2


def test_merging_keeps_places_further_apart_than_the_radius() -> None:
    found = [placed(1, 0.5, (0.0, 0.0, 0.0)), placed(2, 0.4, (0.0, 0.8, 0.0))]
    assert merge_duplicates(found, merge_m=0.75) == 2
    assert merge_duplicates(found, merge_m=1.0) == 1


def test_an_unplaced_detection_is_never_called_a_duplicate() -> None:
    """Without a 3D box there is no "same place" to test, so it stays on its own."""
    flat = Detection(
        query="a square",
        rank=2,
        ts=2.0,
        camera_frame=CAMERA,
        episode_frames=3,
        episode_span=1.0,
        episode_score=0.1,
        models=["stub"],
        attempts=1,
    )
    found = [placed(1, 0.5, (0.0, 0.0, 0.0)), flat]
    assert merge_duplicates(found) == 1
    assert flat.duplicate_of is None


def test_live_the_detector_is_shown_the_frames_the_ingest_kept(tmp_path: Path) -> None:
    """A live run has no colour stream, and the detector still has to see a picture.

    The camera's thirty frames a second exist on the wire and nowhere else, so the only
    pictures a robot can be shown later are the ones an embedding frame was made from.
    This store has no `color_image` at all -- which is exactly the live shape -- and the
    box still has to be placed.
    """
    from dimos.mapping.hyperspace.ingest import filled_stream_for, frame_stream_for

    store = SqliteStore(path=str(tmp_path / "live.db"))
    store.start()
    store.stream("camera_info", CameraInfo).append(camera_info(), ts=10.0)
    kept = store.stream(frame_stream_for(""), Image)
    filled = store.stream(filled_stream_for(""), dict)
    for ts in (10.0, 10.25):
        kept.append(
            Image.from_numpy(
                np.full((HEIGHT, WIDTH, 3), 128, dtype=np.uint8), frame_id=CAMERA, ts=ts
            ),
            ts=ts,
            tags={"camera_frame": CAMERA},
        )
        filled.append(
            {
                "camera_frame": CAMERA,
                "ts": ts,
                "depth_mm": (planted_depth() * 1000).astype(np.uint16),
            },
            ts=ts,
            tags={"camera_frame": CAMERA},
        )
    store.stream("tf", TFMessage).append(
        TFMessage(
            Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id=WORLD,
                child_frame_id=CAMERA,
                ts=10.0,
            )
        ),
        ts=10.0,
    )

    frames = RecordingFrames(store, config=DetectConfig(world_frame=WORLD))
    assert "color_image" not in store.list_streams(), "the live shape: no colour stream"
    assert frames.color(10.0) is not None, "the kept frame is the picture"
    assert frames.depth(CAMERA, 10.0) is not None

    boxes = StubBoxes((SQUARE[0], SQUARE[2], SQUARE[1], SQUARE[3]), score=0.9)
    episode = Episode(
        frames=[
            Frame(
                frame=CAMERA,
                ts=ts,
                hits=[
                    Hit(
                        member="stub",
                        frame=CAMERA,
                        ts=ts,
                        cell=0,
                        grid=(8, 8),
                        ray=(0.0, 0.0),
                        depth=2.0,
                        score=1.0,
                    )
                ],
            )
            for ts in (10.0, 10.25)
        ]
    )
    answer = detect_episode(
        episode,
        "a square",
        frames,
        boxes,
        rank=1,
        config=DetectConfig(world_frame=WORLD),
        keep_image=True,
    )
    assert answer.found, answer.note
    assert answer.box3d is not None, "placed off the kept frame and the filled depth"
    assert answer.image is not None, "and it carries the picture it was placed from"
    store.stop()


def test_a_recording_that_names_its_streams_differently_can_still_place(tmp_path: Path) -> None:
    """Intrinsics are read in the constructor, so a stream name set afterwards is late.

    bike.db calls its camera info `realsense_camera_info`, not `camera_info`. Told the
    wrong name, `RecordingFrames` finds the images and no intrinsics, and every answer
    dies as "no camera_info for <frame>" -- a warning per episode, no exception, and a
    query that quietly returns nothing on a recording full of the thing asked for.
    """
    store = SqliteStore(path=str(tmp_path / "oddly_named.db"))
    store.start()
    store.stream("realsense_camera_info", CameraInfo).append(camera_info(), ts=10.0)
    colors = store.stream("realsense_color_image_compressed", Image)
    depths = store.stream("realsense_depth_image", Image, codec=Lz4Codec(LcmCodec(Image)))
    for ts in (10.0, 10.25):
        colors.append(
            Image.from_numpy(
                np.full((HEIGHT, WIDTH, 3), 128, dtype=np.uint8), frame_id=CAMERA, ts=ts
            ),
            ts=ts,
        )
        depths.append(
            Image.from_numpy(
                (planted_depth() * 1000).astype(np.uint16),
                format=ImageFormat.DEPTH16,
                frame_id=CAMERA,
                ts=ts,
            ),
            ts=ts,
        )

    config = DetectConfig(world_frame=WORLD)
    guessed = RecordingFrames(store, config=config)
    assert not guessed.intrinsics, "the defaults do not match this recording"

    told = RecordingFrames(
        store,
        color_stream="realsense_color_image_compressed",
        depth_stream="realsense_depth_image",
        color_info_stream="realsense_camera_info",
        depth_info_stream="realsense_camera_info",
        config=config,
    )
    assert CAMERA in told.intrinsics, "told the right names, it can place a box"
    assert told.color(10.0) is not None
    assert told.depth(CAMERA, 10.0) is not None
    store.stop()


def test_the_ingests_own_camera_record_is_enough_to_place_a_box(tmp_path: Path) -> None:
    """Live there is no camera_info stream at all -- the ingest's record is the only one.

    The intrinsics arrive on a port and live in the ingest's memory; if it does not write
    them down, a query against its store can find the pictures and the depth and still
    not turn a 2D box into a place in the world.
    """
    from dimos.mapping.hyperspace.ingest import info_stream_for

    store = SqliteStore(path=str(tmp_path / "live_only.db"))
    store.start()
    store.stream(info_stream_for(""), CameraInfo).append(camera_info(), ts=10.0)
    frames = RecordingFrames(store, config=DetectConfig(world_frame=WORLD))
    assert CAMERA in frames.intrinsics, "the ingest's own record is read"
    store.stop()


# --- depth2depth where the recording never had it run -------------------------------


class StubFusion:
    """Stands in for the depth model: reports every hole filled at a fixed depth."""

    def __init__(self, **named: object) -> None:
        self.named = named
        self.started = 0
        self.calls: list[tuple[int, int]] = []

    def start(self) -> None:
        self.started += 1

    def fuse(self, rgb: np.ndarray, raw: np.ndarray) -> SimpleNamespace:
        self.calls.append(rgb.shape[:2])
        filled = np.array(raw, dtype=np.float32)
        filled[filled == 0.0] = 4.0
        return SimpleNamespace(fused=filled)


@pytest.fixture
def stub_fusion(monkeypatch) -> StubFusion:
    held: list[StubFusion] = []

    def build(**named: object) -> StubFusion:
        held.append(StubFusion(**named))
        return held[-1]

    monkeypatch.setattr("dimos.perception.depth2depth.fusion.Depth2Depth", build)
    return held  # type: ignore[return-value]


def test_a_recording_with_no_filled_stream_fills_its_depth_as_it_places(
    recording: SqliteStore, stub_fusion
) -> None:
    """Jeff, 2026-09-14: "we should be able to run depth2depth live".

    Before this the placer only ever READ filled depth, so a recording nobody had run
    `fill_depth` over was placed off stereo's holes and said nothing about it.
    """
    holed = planted_depth()
    holed[:4, :] = 0.0  # four rows the stereo gave up on
    recording.streams["depth_image"].append(
        Image.from_numpy(
            (holed * 1000).astype(np.uint16), format=ImageFormat.DEPTH16, frame_id=CAMERA, ts=50.0
        ),
        ts=50.0,
    )
    recording.streams["color_image"].append(
        Image.from_numpy(
            np.full((HEIGHT, WIDTH, 3), 128, dtype=np.uint8), frame_id=CAMERA, ts=50.0
        ),
        ts=50.0,
    )
    config = DetectConfig(world_frame=WORLD, depth2depth="auto")
    frames = RecordingFrames(recording, config=config)
    depth = frames.depth(CAMERA, 50.0)
    assert depth is not None
    assert stub_fusion[0].started == 1
    # Everything the stereo did not see reads 4 m now, and what it saw is untouched.
    assert float(depth[0, 0]) == pytest.approx(4.0)
    assert float(depth[24, 32]) == pytest.approx(SQUARE_DEPTH)


def test_the_model_is_paid_once_a_photograph_however_many_boxes_land_on_it(
    recording: SqliteStore, stub_fusion
) -> None:
    config = DetectConfig(world_frame=WORLD, depth2depth="auto")
    frames = RecordingFrames(recording, config=config)
    for _ in range(4):
        frames.depth(CAMERA, 10.0)
    frames.depth(CAMERA, 30.0)
    assert len(stub_fusion[0].calls) == 2, "one call per distinct frame, not per placement"


def test_auto_does_not_pay_for_a_model_on_a_recording_already_filled(
    recording: SqliteStore, stub_fusion
) -> None:
    """`fill_depth` and this do the same arithmetic; doing it twice buys nothing."""
    recording.stream(filled_stream_for(""), dict).append(
        {
            "camera_frame": CAMERA,
            "ts": 10.0,
            "depth_mm": np.full((HEIGHT, WIDTH), 7000, dtype=np.uint16),
        },
        ts=10.0,
    )
    config = DetectConfig(world_frame=WORLD, depth2depth="auto")
    depth = RecordingFrames(recording, config=config).depth(CAMERA, 10.0)
    assert depth is not None and float(depth[0, 0]) == pytest.approx(7.0)
    assert not stub_fusion, "the model must not even be built"


def test_a_model_that_will_not_load_places_off_raw_depth_rather_than_failing(
    recording: SqliteStore, monkeypatch
) -> None:
    """Worse depth beats no answer, and it must not be tried again on the next frame."""
    tries = []

    def explode(**named: object) -> object:
        tries.append(named)
        raise RuntimeError("no such checkpoint")

    monkeypatch.setattr("dimos.perception.depth2depth.fusion.Depth2Depth", explode)
    config = DetectConfig(world_frame=WORLD, depth2depth="auto")
    frames = RecordingFrames(recording, config=config)
    depth = frames.depth(CAMERA, 10.0)
    assert depth is not None and float(depth[24, 32]) == pytest.approx(SQUARE_DEPTH)
    frames.depth(CAMERA, 30.0)
    assert len(tries) == 1, "one failure is enough; the knob turns itself off"


def test_far_depth_survives_the_read_so_the_cut_off_can_be_reported(
    recording: SqliteStore,
) -> None:
    """The 10 m cut used to happen at read time, which erased the evidence for it."""
    store = recording
    depths = store.streams["depth_image"]
    far = np.full((HEIGHT, WIDTH), 40000, dtype=np.uint16)  # 40 m, well past the cut
    depths.append(
        Image.from_numpy(far, format=ImageFormat.DEPTH16, frame_id=CAMERA, ts=50.0), ts=50.0
    )
    config = DetectConfig(world_frame=WORLD)
    depth = RecordingFrames(store, config=config).depth(CAMERA, 50.0)
    assert depth is not None
    assert float(depth[0, 0]) == pytest.approx(40.0), "not zeroed on the way out"
    found = object_points((28.0, 20.0, 36.0, 28.0), (WIDTH, HEIGHT), depth, intrinsics())
    assert not found and "past the 10 m cut-off" in found.why


def test_a_detector_precision_is_named_or_refused() -> None:
    """A typo'd dtype must say so, not quietly run float32.

    The whole point of the knob is a measured 2x on CUDA; a silent fallback looks exactly
    like a speedup that did not arrive, which is the most expensive kind of bug to chase.
    """
    import pytest
    import torch

    from dimos.mapping.hyperspace.detect import DetectConfig, _torch_dtype

    assert DetectConfig.dtype == "auto", "the device decides; '' still forces float32"
    assert _torch_dtype("fp16") is torch.float16
    assert _torch_dtype("bf16") is torch.bfloat16
    with pytest.raises(ValueError, match="unknown detector dtype"):
        _torch_dtype("float8")


def test_the_fast_image_path_is_cuda_only_and_never_metal() -> None:
    """The fast image path is not bit-identical, and on MPS it does not exist at all.

    `Owlv2ImageProcessor` resizes with a gaussian pre-filter and an order-1 zoom; the torch
    version uses an antialiased bilinear. Same intent, different kernel, so the pixels
    differ a little and the scores move with them -- which is why it took a whole run's
    answers to turn on, and only where it was measured.

    MPS cannot run those steps at all -- no `aten::_upsample_bilinear2d_aa` there -- but
    that is a question of WHERE they run, not whether. MEASURED on an M-series Mac, one
    848x480 frame: the processor's preprocessing 155 ms against 4.0 ms for the same four
    steps in torch on the CPU, beside a 141 ms forward pass. So the win was never the GPU,
    and a machine that cannot use the GPU for it still gets 39x by preparing on the cpu.
    """
    from dimos.mapping.hyperspace.detect import (
        DetectConfig,
        gpu_preprocess_for,
        preprocess_device_for,
    )
    from dimos.perception.detection.detectors.owlv2 import Owlv2Config

    assert DetectConfig.gpu_preprocess == "auto"
    # CUDA only, and for a RECALL reason rather than a speed one: on Metal the fast path
    # is twice as quick and cost four of bike's eleven traffic lights. See the docstring.
    assert gpu_preprocess_for("auto", "cuda") is True
    assert gpu_preprocess_for("auto", "mps") is False
    assert gpu_preprocess_for("auto", "cpu") is False
    assert gpu_preprocess_for("on", "mps") is True, "an explicit ask is still obeyed"
    assert gpu_preprocess_for("off", "cuda") is False

    # Metal has no antialiased resize, so it prepares on the cpu and moves the result.
    assert preprocess_device_for("mps") == "cpu"
    assert preprocess_device_for("cuda") == "", "cuda does it in place"
    assert preprocess_device_for("cpu") == ""

    # Owlv2Config is a pydantic model, so the default lives in the field, not on the class.
    assert Owlv2Config().gpu_preprocess is False, (
        "every other caller of the shared detector still opts in deliberately"
    )


def test_half_precision_is_taken_only_where_it_was_measured_to_pay() -> None:
    """fp16 halves the forward pass on an RTX 5070 and is a wash on Metal.

    Measured on the Mac over sf_office's four queries: detect 10.30 / 8.34 / 6.69 / 1.88 s
    at fp16 against 10.46 / 8.36 / 6.21 / 1.86 at float32, same places both ways. So Metal
    carries the score risk of three fewer mantissa bits for nothing, and "auto" declines.
    """
    from dimos.mapping.hyperspace.detect import detector_dtype_for

    assert detector_dtype_for("auto", "cuda") == "fp16"
    assert detector_dtype_for("auto", "mps") == ""
    assert detector_dtype_for("auto", "cpu") == ""
    assert detector_dtype_for("", "cuda") == "", "'' still means float32 outright"
    assert detector_dtype_for("bf16", "mps") == "bf16"


def test_the_first_answer_is_timed_from_the_question(recording: SqliteStore, monkeypatch) -> None:
    """`first_result` is the wait a person could time with a stopwatch.

    It used to start at the beginning of detection, which left out the patch search and
    the episode grouping in front of it. On bike.db that reported 0.4 s for a wait that
    was really 7 -- and a number that flattering sends the next afternoon of tuning at
    the wrong half of the query. So it has to be at least as large as the search it
    waited for.
    """
    import time as clock

    def slow_search(*args, **kwargs):
        clock.sleep(0.15)
        return [frame_at(10.0, 0.9), frame_at(10.25, 0.9)]

    monkeypatch.setattr("dimos.mapping.hyperspace.frames.hot_frames", slow_search)
    config = DetectConfig(world_frame=WORLD)
    timings: dict[str, float] = {}
    answers = list(
        find(
            recording,
            recording,
            "a square",
            config=config,
            frames=RecordingFrames(recording, config=config),
            boxes=StubBoxes((28.0, 20.0, 36.0, 28.0)),
            timings=timings,
        )
    )
    assert answers, "the stub answers, so there is a first result to time"
    assert timings["search"] >= 0.15, "the stubbed search really did take that long"
    assert timings["first_result"] >= timings["search"], (
        f"first_result {timings['first_result']:.3f}s is less than the "
        f"{timings['search']:.3f}s search in front of it, so it is not timed from the question"
    )
    assert timings["first_result"] == answers[0].arrived, (
        "the reported wait and the answer's own arrival have to be the same number"
    )


def test_the_background_contrast_can_be_turned_off() -> None:
    """The floor/wall/ceiling subtraction is a setting, not a law.

    It is on by default and should stay that way -- CLIP scores nearly everything
    somewhat highly, so without it a wall answers most questions moderately well and the
    frame ranking stops discriminating. But it is a correction with a guess inside it,
    and there are two times to turn it off: to check whether it is earning its place on
    a given recording, and when the thing being asked for IS a wall or a floor, where
    the contrast subtracts the target.

    With no background rows the score has to fall back to the plain similarity.
    """
    import numpy as np

    from dimos.mapping.hyperspace.detect import DetectConfig
    from dimos.mapping.hyperspace.resident import ResidentPatches

    assert DetectConfig.contrast is True, "on unless asked otherwise"

    vectors = np.array([[1.0, 0.0], [0.0, 1.0], [0.6, 0.8]], dtype=np.float32)
    held = ResidentPatches(
        tag="t",
        stream="s",
        vectors=vectors,
        last_id=3,
        frame_of=np.zeros(3, np.int32),
        ts=np.zeros(3, np.float64),
        cell=np.arange(3, dtype=np.int32),
        grid=np.ones((3, 2), np.int16),
        ray=np.zeros((3, 2), np.float32),
        depth=np.ones(3, np.float32),
        camera_frames=["cam"],
    )
    query = np.array([1.0, 0.0], dtype=np.float32)
    background = np.array([[0.0, 1.0]], dtype=np.float32)

    plain = held.scores(query, np.empty((0, 2), np.float32))
    contrasted = held.scores(query, background)

    assert np.allclose(plain, [1.0, 0.0, 0.6]), "no background rows means the raw similarity"
    assert np.allclose(contrasted, [1.0, -1.0, -0.2]), "the background is subtracted off"


def test_narrowing_a_search_returns_row_numbers_into_the_whole_index() -> None:
    """`rows=` scores a subset, and the indices handed back have to be absolute.

    This is the bookkeeping that puts a patch on the wrong frame if it is wrong: the
    caller looks `held.frame_of[index]` up directly, so an index that was relative to
    the subset would silently name a different photograph. The subset here is chosen so
    that the relative and absolute answers are different numbers -- a test where they
    coincide would pass either way.
    """
    import numpy as np

    from dimos.mapping.hyperspace.resident import ResidentPatches

    vectors = np.array(
        [[1.0, 0.0], [0.0, 1.0], [0.9, 0.1], [0.0, 1.0], [0.8, 0.2]], dtype=np.float32
    )
    held = ResidentPatches(
        tag="t",
        stream="s",
        vectors=vectors,
        last_id=5,
        frame_of=np.zeros(5, np.int32),
        ts=np.arange(5, dtype=np.float64),
        cell=np.arange(5, dtype=np.int32),
        grid=np.ones((5, 2), np.int16),
        ray=np.zeros((5, 2), np.float32),
        depth=np.ones(5, np.float32),
        camera_frames=["cam"],
    )
    query = np.array([1.0, 0.0], dtype=np.float32)
    nothing = np.empty((0, 2), np.float32)

    everywhere, _ = held.hot(query, nothing, threshold=0.5)
    assert sorted(everywhere.tolist()) == [0, 2, 4]

    # Rows 2 and 4 only. As a subset they are positions 0 and 1, so an off-by-subset
    # bug would answer [0, 1] -- which is also a valid-looking pair of row numbers.
    narrowed, scored = held.hot(query, nothing, threshold=0.5, rows=np.array([2, 4], np.intp))
    assert sorted(narrowed.tolist()) == [2, 4], "row numbers are into the index, not the subset"
    assert np.allclose(sorted(scored.tolist()), [0.8, 0.9])


def test_ranking_with_one_model_only_looks_where_that_model_looked() -> None:
    """`_rows_on` picks the rows of the frames named, and nothing else.

    The whole saving is that the other members never read the vectors of a frame the
    ranking member did not like, so what this pins is which ROWS come back -- matched on
    (camera frame, ts) together, because two cameras can share a timestamp and a match
    on the stamp alone would drag in the other camera's patches.
    """
    import numpy as np

    from dimos.mapping.hyperspace.frames import _rows_on
    from dimos.mapping.hyperspace.resident import ResidentPatches

    held = ResidentPatches(
        tag="t",
        stream="s",
        vectors=np.zeros((6, 2), np.float32),
        last_id=6,
        # Two cameras, and both of them saw something at t=1.0.
        frame_of=np.array([0, 0, 1, 1, 0, 1], np.int32),
        ts=np.array([1.0, 2.0, 1.0, 3.0, 3.0, 2.0], np.float64),
        cell=np.arange(6, dtype=np.int32),
        grid=np.ones((6, 2), np.int16),
        ray=np.zeros((6, 2), np.float32),
        depth=np.ones(6, np.float32),
        camera_frames=["left", "right"],
    )

    assert _rows_on(held, {("left", 1.0)}).tolist() == [0], "not the right camera's t=1.0"
    assert _rows_on(held, {("left", 1.0), ("right", 2.0)}).tolist() == [0, 5]
    assert _rows_on(held, {("left", 9.0)}).tolist() == [], "a frame it never saw"
    assert _rows_on(held, {("nope", 1.0)}).tolist() == [], "a camera it does not have"


def test_narrowed_rows_become_contiguous_spans_not_a_gather() -> None:
    """The saving is only real if the scorer SLICES, so the runs have to be right.

    MEASURED, same query and index on two machines: fancy-indexing the wanted rows made
    CudaLaptop's search go 4.11 s -> 17.20 s, four times slower than reading the whole
    19.2 GB, while the Mac went 0.81 s -> 0.83 s and hid the problem completely. A gather
    reads one row at a time wherever they are; a slice streams. An off-by-one here is the
    difference between scoring the right patches and scoring their neighbours, so the
    boundaries are pinned rather than the count.
    """
    import numpy as np

    from dimos.mapping.hyperspace.resident import _spans

    assert _spans(None, 7) == [(0, 7)], "no narrowing means one span over everything"
    assert _spans(np.array([], np.intp), 7) == []
    assert _spans(np.array([3], np.intp), 7) == [(3, 4)], "half-open, so one row is (n, n+1)"
    assert _spans(np.array([0, 1, 2, 3], np.intp), 7) == [(0, 4)], "one run, not four"
    assert _spans(np.array([0, 1, 4, 5, 6], np.intp), 7) == [(0, 2), (4, 7)]
    assert _spans(np.array([1, 3, 5], np.intp), 7) == [(1, 2), (3, 4), (5, 6)], (
        "alternating rows are the worst case and still have to be exact"
    )


def test_a_narrowed_score_equals_the_same_rows_scored_whole() -> None:
    """Slicing is an optimisation, so it has to answer what the slow way answers.

    The rows chosen are deliberately in two runs with a gap, because a single run would
    pass under an implementation that ignored `rows` altogether.
    """
    import numpy as np

    from dimos.mapping.hyperspace.resident import ResidentPatches

    rng = np.random.default_rng(7)
    vectors = rng.standard_normal((9, 4)).astype(np.float32)
    held = ResidentPatches(
        tag="t",
        stream="s",
        vectors=vectors,
        last_id=9,
        frame_of=np.zeros(9, np.int32),
        ts=np.arange(9, dtype=np.float64),
        cell=np.arange(9, dtype=np.int32),
        grid=np.ones((9, 2), np.int16),
        ray=np.zeros((9, 2), np.float32),
        depth=np.ones(9, np.float32),
        camera_frames=["cam"],
    )
    query = rng.standard_normal(4).astype(np.float32)
    background = rng.standard_normal((2, 4)).astype(np.float32)
    wanted = np.array([1, 2, 3, 6, 7], np.intp)

    whole = held.scores(query, background)
    narrowed = held.scores(query, background, wanted)

    assert narrowed.shape == (len(wanted),), "one score per row asked for"
    assert np.allclose(narrowed, whole[wanted]), "narrowing must not change the arithmetic"


def test_the_pass_count_is_per_episode_not_per_answer() -> None:
    """Two boxes in one photograph are two answers and ONE episode's cost.

    `_Try.finish()` returns the episode's own detection plus a `beside` copy for every
    extra box the detector saw in the same frame, and those copies carry the SAME
    `attempts`. Summing over the yielded answers would multiply an episode's cost by how
    many things happened to be in shot -- so the count is taken over `tries`, and this
    pins that it is.

    The number exists because "more work" and "dearer work" are different diagnoses and
    nothing outside could tell them apart: three queries on grocery ranged 138-274 s
    while the pass count was ASSUMED constant, and the assumption was an upper bound
    (`candidates` is capped at `min(attempts, len(episode.frames))`) treated as a count.
    """
    from dataclasses import replace

    from dimos.mapping.hyperspace.detect import Detection, _Try

    one = _Try(
        detection=Detection(
            query="a cone",
            rank=1,
            ts=1.0,
            camera_frame="cam",
            episode_frames=4,
            episode_span=0.5,
            episode_score=1.0,
            models=["m"],
            attempts=3,
        ),
        candidates=[],
    )
    one.answer = replace(one.detection)
    one.beside = [replace(one.detection), replace(one.detection)]

    assert len(one.finish()) == 3, "one episode, three answers"
    assert sum(answer.attempts for answer in one.finish()) == 9, (
        "this is the wrong sum, and it is the one a caller would reach for"
    )
    assert one.detection.attempts == 3, "the episode cost three passes, not nine"


def test_the_cheap_member_ranks_by_default_and_can_be_turned_off() -> None:
    """Ranking with one member and confirming with the rest is the shipped behaviour.

    It was off while it had only three queries of one recording behind it. Eleven queries
    across all three now say it is faster on every one of them and finds four more places
    than it loses -- the numbers are written out on the field itself. "" is still the way
    back to searching every member over everything, which is what the measurements are
    compared against.
    """
    from dimos.mapping.hyperspace.detect import DetectConfig
    from dimos.mapping.hyperspace.module import HyperspaceConfig

    assert DetectConfig.rank_with == "auto"
    assert HyperspaceConfig.model_fields["rank_with"].default == "auto", (
        "the live module and the offline CLI must not disagree about this again"
    )
    assert DetectConfig.rank_frames == 400, "the cut is what makes ranking worth anything"
