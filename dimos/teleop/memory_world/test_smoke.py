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

"""Smoke tests for the memory world.

Deliberately small. This is a visual demo, and the viewer is checked by looking at it --
what is here is the arithmetic underneath that a person cannot see going wrong: the route
planner, the tf interpolation, the replay's scan accumulation, the shapes that go on the
wire, and the answer path end to end. A green run here means the pieces still fit
together; it does not mean the demo looks right.
"""

from __future__ import annotations

import itertools
import json
import math
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.teleop.memory_world.answers import WorldAnswers
from dimos.teleop.memory_world.query import ClusterSummary, MemoryQueryResult
from dimos.teleop.memory_world.route import LETHAL, RoutePlanner
from dimos.teleop.memory_world.tf_tree import TfTree, pose_matrix, quaternion_from_matrix
from dimos.teleop.memory_world.visual_answers import VisualAnswers
from dimos.teleop.memory_world.visual_search import (
    Place,
    VisualMemoryIndex,
    cluster_places,
    search_phrase,
)
from dimos.teleop.memory_world.world_cache import WorldCache

VOXEL = 0.1
BODY_Z = 0.4  # the robot's base height above the floor at z = 0


# ---- the module itself ------------------------------------------------------------


def test_the_module_composes_and_serves_its_routes(memory_world) -> None:  # type: ignore[no-untyped-def]
    """It is five mixins over one config; a name two of them disagree about breaks at import.

    Cheap, and it is the test that would have caught every rename made while the
    hyperspace half was being cut out of this package.
    """
    app = SimpleNamespace(
        routes=[],
        exception_handler=lambda *a, **k: (lambda fn: fn),
        get=lambda path, **k: (lambda fn: app.routes.append(("GET", path)) or fn),
        post=lambda path, **k: (lambda fn: app.routes.append(("POST", path)) or fn),
    )
    memory_world._setup_answer_routes(app)

    assert ("POST", "/memory_world/ask") in app.routes
    assert ("POST", "/memory_world/navigate") in app.routes
    assert ("GET", "/memory_world/frames") in app.routes
    assert ("GET", "/memory_world/orbit") in app.routes
    assert ("GET", "/memory_world/answer") in app.routes
    # The engine is not configurable and the module says so to anything that asks.
    assert memory_world.find_in_memory("").success is False


# ---- the route planner -------------------------------------------------------------


def _path(*waypoints: tuple[float, float]) -> np.ndarray:
    """The robot's base along straight legs between waypoints, a point every 5 cm."""
    points = []
    for (x0, y0), (x1, y1) in itertools.pairwise(waypoints):
        n = max(int(math.dist((x0, y0), (x1, y1)) / 0.05), 2)
        for t in np.linspace(0, 1, n):
            points.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, BODY_Z))
    return np.asarray(points)


def _floor(x0: float, x1: float, y0: float, y1: float) -> np.ndarray:
    xs = np.arange(x0, x1, VOXEL)
    ys = np.arange(y0, y1, VOXEL)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    return np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], axis=1)


def _wall(x0: float, x1: float, y0: float, y1: float, height: float = 1.5) -> np.ndarray:
    xs = np.arange(x0, x1, VOXEL)
    ys = np.arange(y0, y1, VOXEL)
    zs = np.arange(VOXEL, height, VOXEL)
    gx, gy, gz = np.meshgrid(xs, ys, zs, indexing="ij")
    return np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)


# A 10 x 6 m floor with a wall across x = 5 that has a 1.2 m gap at y in [4, 5.2], and a
# drive that goes through the gap and back down the far side.
ROOM = np.concatenate([_floor(0, 10, 0, 6), _wall(4.9, 5.1, 0, 4.0), _wall(4.9, 5.1, 5.2, 6.0)])
DRIVE = _path((1, 1), (1, 4.6), (9, 4.6), (9, 1))


def test_the_route_goes_through_the_doorway_and_never_through_the_wall() -> None:
    """The one thing Navigate must not do is draw a line through a wall.

    The green tube is presented as what the navigation stack would drive, so a route that
    crosses the wall is not a cosmetic defect: it is the demo asserting something false
    about the map it is standing in.
    """
    planner = RoutePlanner.from_voxels(ROOM, DRIVE, voxel_size=VOXEL)

    assert planner.costs[planner.cell_of((5.0, 3.5))] == LETHAL, "the wall is not free space"
    assert 0 <= planner.costs[planner.cell_of((5.0, 4.6))] < LETHAL, "the doorway is passable"

    route = planner.plan((1.5, 1.5), (8.5, 1.5))
    assert route is not None, "no route between two points the robot drove between"
    points = np.asarray([(x, y) for x, y, *_ in route.points])
    # Every leg that crosses x = 5 has to do it in the doorway.
    for (x0, y0), (x1, y1) in itertools.pairwise(points):
        if (x0 - 5.0) * (x1 - 5.0) < 0:
            span = (x1 - x0) or 1e-9
            y_at_wall = y0 + (y1 - y0) * (5.0 - x0) / span
            assert 3.9 < y_at_wall < 5.3, f"the route crosses the wall at y={y_at_wall:.2f}"
    assert route.length_m > 7.0, f"a route around the wall cannot be {route.length_m:.1f} m"


def test_a_goal_nothing_connects_to_is_refused_rather_than_faked() -> None:
    """Outside the corridor the map is unknown, and unknown is not free."""
    planner = RoutePlanner.from_voxels(ROOM, DRIVE, voxel_size=VOXEL)
    assert planner.plan((1.5, 1.5), (200.0, 200.0)) is None


# ---- tf ---------------------------------------------------------------------------


def test_a_pose_between_two_tf_samples_is_interpolated_not_snapped() -> None:
    """Every photograph is hung where tf says its camera was, at the frame's own stamp.

    Nearest-sample instead of interpolation puts the evidence a whole tf period away from
    where it was taken, which at 10 Hz and walking pace is tens of centimetres.
    """
    tree = TfTree()
    tree.add("odom", "base_link", 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    tree.add("odom", "base_link", 1.0, (2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    half = tree.lookup("odom", "base_link", 0.5)

    assert half is not None, "tf could not place a frame between two samples it holds"
    assert half[0][0] == pytest.approx(1.0, abs=1e-6), f"x was {half[0][0]}, not the midpoint"


def test_a_frame_tf_never_saw_has_no_pose_rather_than_the_origin() -> None:
    """The origin is a place in the world; "I do not know" is not."""
    tree = TfTree()
    tree.add("odom", "base_link", 0.0, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    assert tree.lookup("odom", "camera_link", 0.0) is None


def test_a_pose_survives_the_matrix_round_trip() -> None:
    """Poses go to the viewer as a quaternion and come back as a matrix all through here."""
    quaternion = (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))  # 45 degrees about z
    matrix = pose_matrix((1.0, 2.0, 3.0), quaternion)
    back = quaternion_from_matrix(matrix)
    assert np.allclose(matrix[:3, 3], (1.0, 2.0, 3.0))
    # A quaternion and its negation are the same rotation, so compare up to sign.
    assert np.allclose(back, quaternion, atol=1e-6) or np.allclose(
        np.negative(back), quaternion, atol=1e-6
    ), f"{back} is not {quaternion}"


# ---- the replay -------------------------------------------------------------------


def test_the_map_stream_s_own_messages_are_the_timeline_when_there_are_several() -> None:
    """A mapper that publishes `global_map` as it grows has already recorded the timeline.

    This is the whole reason the ray-traced replay builder could be deleted: roscon's
    recording carries 119 of those messages, 812 KB to 50.9 MB, and scrubbing it is a
    read. The two halves worth asserting are the DECISION (several messages, so this
    recording is scrubbed through them; one message, so it is not) and the SELECTION --
    seeking to a stamp must hand back the map as it was then, not the finished one, which
    is a failure that looks like a working scrub over a map that never changes.
    """
    from dimos.teleop.memory_world.world_cache import WorldCache

    class Stream:
        def __init__(self, clouds: dict[float, np.ndarray]) -> None:
            self.clouds = clouds

        def order_by(self, field: str, desc: bool = False) -> Stream:
            return self

        def before(self, t: float) -> Stream:
            return Stream({ts: xyz for ts, xyz in self.clouds.items() if ts < t})

        def __iter__(self):  # type: ignore[no-untyped-def]
            for ts, xyz in sorted(self.clouds.items()):
                yield self._obs(ts, xyz)

        def last(self):  # type: ignore[no-untyped-def]
            ts = max(self.clouds)
            return self._obs(ts, self.clouds[ts])

        @staticmethod
        def _obs(ts: float, xyz: np.ndarray):  # type: ignore[no-untyped-def]
            return SimpleNamespace(
                ts=ts, data=SimpleNamespace(points_f32=lambda: xyz, frame_id="odom")
            )

    def reading(**streams: Stream) -> WorldCache:
        world = WorldCache()
        world.config = SimpleNamespace(global_map_stream_name="global_map", world_frame="odom")
        world._ensure_store = lambda: SimpleNamespace(
            list_streams=lambda: list(streams), streams=streams
        )
        return world

    growing = Stream(
        {
            10.0: np.zeros((1, 3), dtype=np.float32),
            20.0: np.zeros((2, 3), dtype=np.float32),
            30.0: np.zeros((3, 3), dtype=np.float32),
        }
    )
    world = reading(global_map=growing)
    assert world._map_timeline() == ("global_map", [10.0, 20.0, 30.0])
    # The stamp asked for is one the stream wrote, and `before` is exclusive: off by one
    # message here would scrub a map that is always one step stale.
    assert len(world._named_map_cloud("global_map", at=20.0)) == 2
    assert len(world._named_map_cloud("global_map", at=29.9)) == 2
    assert len(world._named_map_cloud("global_map")) == 3, "no stamp means the finished map"

    one = reading(global_map=Stream({10.0: np.zeros((1, 3), dtype=np.float32)}))
    assert one._map_timeline() is None, "one message is a map, not a timeline"


def test_a_tf_gap_holds_the_last_position_instead_of_dropping_to_the_origin() -> None:
    """The same array is the orbit path AND the free corridor a route is planned over.

    A run of origins in it is a straight line through unmapped space that the planner
    cannot tell from somewhere the robot actually drove.
    """
    from dimos.teleop.memory_world.replay import frame_positions

    known = {1.0: (5.0, 0.0, 0.0), 3.0: (7.0, 0.0, 0.0)}
    positions = frame_positions(
        [0.0, 1.0, 2.0, 3.0],
        lambda ts: None if ts not in known else pose_matrix(known[ts], (0.0, 0.0, 0.0, 1.0)),
    )

    assert positions[0] == [5.0, 0.0, 0.0], "the gap before the first fix became the origin"
    assert positions[2] == [5.0, 0.0, 0.0], "the gap did not hold the last known position"
    assert positions[3] == [7.0, 0.0, 0.0]


# ---- the wire ---------------------------------------------------------------------


def test_the_engine_that_answered_is_named_on_the_wire() -> None:
    """Which engine answered is a fact the client needs, not decoration.

    This branch has two, and they do not mean the same thing: a siglip place is where a
    thing was SEEN FROM, a hyperspace place is the thing's own measured position. A
    viewer that described both the same way would be wrong about one of them, so the
    name rides on every answer and the default stays siglip for a recording with no
    Hyperspace connected. Anything else is still refused -- the point of the literal is
    that a third engine cannot arrive unnamed.
    """
    assert MemoryQueryResult(answer="found it").engine == "siglip"
    assert MemoryQueryResult(answer="found it", engine="hyperspace").engine == "hyperspace"
    with pytest.raises(ValueError):
        MemoryQueryResult(answer="found it", engine="clip")  # type: ignore[arg-type]


def test_a_score_that_cannot_be_written_as_json_is_refused_at_the_model() -> None:
    """`json.dumps` writes `NaN`, which `JSON.parse` throws on.

    The viewer's `decodeText` then returns null and `main.js` drops the WHOLE
    `query_result`: nothing on screen, and a success reported by the server.
    """
    for bad in (float("nan"), float("inf")):
        with pytest.raises(ValueError):
            ClusterSummary(index=0, centre=(0.0, 0.0, 0.0), radius=1.0, score=bad, peak=0.5)


def test_a_spoken_question_is_reduced_to_the_thing_asked_about() -> None:
    assert search_phrase("where did I see a fire extinguisher?") == "a fire extinguisher"
    assert search_phrase("   ") == ""


# ---- places -----------------------------------------------------------------------


def _place(x: float, similarity: float, ts: float = 0.0) -> Place:
    return Place(position=(x, 0.0, 0.0), similarity=similarity, source_id=0, ts=ts)


def test_near_identical_frames_of_one_thing_become_one_place() -> None:
    """The robot lingers, so one object produces dozens of near-identical frames.

    Clustering keeps the best-scoring frame per location and drops the rest, which is
    what turns a ranked list of frames into the handful of places a person asked for.
    """
    one_spot = [_place(0.0, 0.30), _place(0.4, 0.55), _place(-0.3, 0.21)]
    somewhere_else = [_place(20.0, 0.40)]

    places = cluster_places(one_spot + somewhere_else, radius=2.0, max_places=5)

    assert len(places) == 2, "the frames of one object were not collapsed"
    assert places[0].similarity == pytest.approx(0.55), "the best frame of the place was lost"
    assert places[0].position[0] == pytest.approx(0.4)


def test_a_place_is_ranked_by_similarity_alone() -> None:
    """There is no view count to rank by any more: one vector per image cannot say how
    many distinct directions saw a thing, so the score is the whole order."""
    places = cluster_places([_place(0.0, 0.10), _place(9.0, 0.80)], radius=1.0, max_places=5)

    assert [round(p.similarity, 2) for p in places] == [0.80, 0.10]


# ---- building the index -----------------------------------------------------------


@pytest.mark.parametrize(
    ("present", "via_agent", "hyperspace_live", "ask"),
    [
        (False, False, False, False),
        (False, True, False, False),
        (False, True, True, True),
        (True, False, False, True),
    ],
)
def test_the_ask_box_opens_for_hyperspace_without_a_siglip_index(
    present, via_agent, hyperspace_live, ask
) -> None:  # type: ignore[no-untyped-def]
    """The viewer gates the ask box and the mic on the status payload's `ask`.

    It used to gate on `present` -- the siglip index holding vectors -- so the hyperspace
    blueprint, which answers through the agent and Hyperspace and has no siglip index,
    sat on "Search not ready" while every question sent by curl was answered and drawn.
    Seen live on grocery.db 2026-09-20. An agent without Hyperspace still needs the index:
    its only lookup is `find_in_memory`.
    """
    from dimos.teleop.memory_world.visual_answers import VisualAnswers

    class Host(VisualAnswers):
        def __init__(self) -> None:
            self.config = SimpleNamespace(ask_via_agent=via_agent)
            self._store_lock = threading.Lock()
            self._index_progress = "no embeddings"
            self._embed_job = SimpleNamespace(status=lambda: {"embedding": "idle"})
            if hyperspace_live:
                self._hyperspace_live = True

        def _ensure_visual_index(self):  # type: ignore[no-untyped-def]
            return SimpleNamespace(
                precomputed_stream_name="s" if present else None, count=lambda: 0
            )

    status = Host()._index_status()
    assert status["ask"] is ask
    assert status["present"] is present
    assert status["embedding"] == "idle"
    if ask and not present:
        assert status["index"].startswith("not needed")
    else:
        assert status["index"] == "no embeddings"


@pytest.mark.parametrize(
    ("recorded", "expect_siglipify"),
    [("Image", True), ("CompressedImage", False), (None, False)],
)
def test_an_mcap_goes_to_the_indexer_that_can_read_it(recorded, expect_siglipify) -> None:  # type: ignore[no-untyped-def]
    """ "Add embeddings" is the viewer's only offer to make a recording searchable.

    siglipify reads the file itself and knows only plain `Image` streams. The choice used
    to be "not Image -> our own indexer", and `recorded_payload` answers **None for every
    mcap**, because an mcap cannot say. So `grocery.mcap` -- the recording DEMO.md tells
    you to run -- sent its `CompressedImage` colour to siglipify, which failed with "no
    image stream 'realsense_color_image_compressed' in the recording; it has
    ['/realsense/depth_image']": the colour reported missing when it is right there, and
    no way left to search the demo recording. Measured live on the demo before this test
    existed. Only a POSITIVE "Image" may take the siglipify path.
    """
    from dimos.teleop.memory_world import visual_answers as mod
    from dimos.teleop.memory_world.visual_answers import VisualAnswers

    started: dict[str, object] = {}

    class Host(VisualAnswers):
        def __init__(self) -> None:
            self.config = SimpleNamespace(
                store_path="/nowhere/grocery.mcap",
                image_stream_name="colour",
                siglip_model_name="m",
                image_index_stride=3,
                tf_stream_name="tf",
                world_frame="odom",
                camera_optical_frame="",
                image_index_stream_name="",
                tf_tolerance_s=0.2,
                siglipify_flake="flake",
            )
            self._embed_job = SimpleNamespace(
                start=lambda command, config, adopt: started.update(command=command, config=config)
                or True
            )

        def _ensure_store(self):  # type: ignore[no-untyped-def]
            return None

    original = mod.recorded_payload
    mod.recorded_payload = lambda *a, **k: recorded  # type: ignore[assignment]
    try:
        assert Host()._start_embedding() is True
    finally:
        mod.recorded_payload = original  # type: ignore[assignment]

    # siglipify is the one that takes a config alongside its command; ours takes None.
    went_to_siglipify = started["config"] is not None
    assert went_to_siglipify is expect_siglipify, (
        f"payload {recorded!r} went to {'siglipify' if went_to_siglipify else 'our indexer'}"
    )


def test_the_planner_walks_the_raw_map_even_when_the_smoothed_one_is_on_screen() -> None:
    """A closing is the right thing to look at and the wrong thing to plan over.

    Filling a gap narrower than the structuring element seals the space between a floor
    and the shelf above it, and standable surface is what the MLS planner reads out of
    that space -- so the map that looks more solid offers FEWER places to stand. Measured
    on grocery.db over the same 25 early poses: the raw map has 308,080 surface cells and
    routes to a basket in 42.93 m, the closed one has 271,037 and routes nowhere.
    """
    drawn = np.ones((4, 3), dtype=np.float32)
    raw = np.zeros((9, 3), dtype=np.float32)
    world = WorldCache()
    world.config = SimpleNamespace(map_z_min=None, map_z_max=None)
    world._named_map_cloud = lambda name: raw if name == "global_map" else None

    assert len(world._planning_map(drawn, "global_map_smoothed")) == 9

    # A raw source is already the map to plan over, and nothing else is fetched.
    assert len(world._planning_map(drawn, "global_map")) == 4
    # So is the drawn cloud when the raw sibling is not in the recording at all.
    assert len(world._planning_map(drawn, "voxel_keyframe_smoothed")) == 4
    # And when the cloud came from the replay or an accumulation, not a stream.
    assert len(world._planning_map(drawn, None)) == 4


def test_a_viewer_opening_an_empty_world_is_not_shown_the_last_visitor_s_answer() -> None:
    """The answer on screen is replayed to every new websocket and was never cleared.

    So it outlived its viewers: a curl, an agent turn or a closed tab left markers and
    evidence photographs on the next person's opening screen, answering a question they
    had not asked. The join is what has to distinguish the two cases, because a headset
    picked up beside a laptop is a second viewer of a LIVE answer and must still get it.
    """
    world = WorldAnswers()
    world._world_clients = set()
    world._active_query_result = {"query_id": "abc"}
    world._active_query_images = [({"index": 0}, b"jpeg")]
    world._last_answer = ("places", "abc")

    world._join_world_clients("the first viewer")
    assert world._active_query_result is None
    assert world._active_query_images == []
    assert world._last_answer == (None, None)
    assert world._world_clients == {"the first viewer"}

    # ...and the live case the clearing must not break.
    live = {"query_id": "xyz"}
    world._active_query_result = live
    world._last_answer = ("places", "xyz")
    world._join_world_clients("a headset joining the same demo")
    assert world._active_query_result is live
    assert world._last_answer == ("places", "xyz")
    assert len(world._world_clients) == 2


def test_an_index_the_recording_already_holds_is_used_not_rebuilt() -> None:
    """`build_image_index_on_start` off with vectors present means USE them.

    Topping up on every start is minutes to hours of cpu embedding under the store lock,
    and the viewer says "Search not ready" for all of it over an index that could already
    have answered. An EMPTY index still refuses, because there is nothing to answer with.
    """
    built = []

    class Index:
        precomputed_stream_name = None
        model = SimpleNamespace(embed_text=lambda _text: None)

        def __init__(self, rows: int) -> None:
            self.rows = rows

        def count(self) -> int:
            return self.rows

        def build(self, stride: int = 1) -> int:
            built.append(stride)
            return 0

        def load(self) -> None:
            pass

    def run(rows: int, on_start: bool) -> str:
        host = SimpleNamespace(
            config=SimpleNamespace(
                image_stream_name="color_image",
                build_image_index_on_start=on_start,
                image_index_stride=3,
            ),
            _store_lock=threading.RLock(),
            _index_lock=threading.RLock(),
            _index_progress="not started",
            _ensure_store=lambda: SimpleNamespace(list_streams=lambda: ["color_image"]),
            _ensure_visual_index=lambda: Index(rows),
            whisper=None,
        )
        VisualAnswers._build_visual_index(host)
        return host._index_progress

    assert run(4826, on_start=False) == "ready (4826 frames)"
    assert built == [], "an index that is already there was rebuilt"

    assert run(4826, on_start=True) == "ready (4826 frames)"
    assert built == [3], "an explicit build did not run"

    assert "no embeddings" in run(0, on_start=False)


def test_which_frames_are_indexed_does_not_move_when_tf_places_fewer_of_them() -> None:
    """The stride counts frames of the image stream, never of the posed survivors.

    Striding the survivors makes every pick a function of how many frames tf happened to
    place, so a run that places one fewer re-phases all the rest and the index already in
    the recording matches almost nothing the next build wants -- which is ~2/3 of the
    recording re-embedded on cpu, holding the store lock, under a viewer that says only
    "Search not ready". Measured on grocery.db: 4,826 rows stored and 6,431 wanted by the
    very next build of the same file.
    """
    frames = [SimpleNamespace(id=i, ts=float(i)) for i in range(30)]
    index = VisualMemoryIndex.__new__(VisualMemoryIndex)
    index.image_stream_name = "color_image"
    index.store = SimpleNamespace(
        streams={"color_image": SimpleNamespace(order_by=lambda _field: iter(frames))}
    )

    def posed_ids(unplaceable: set[int]) -> list[int]:
        index.pose_of = lambda obs: None if obs.id in unplaceable else np.eye(4)
        return [obs.id for obs, _ in index._posed_frames(stride=3)]

    everything = posed_ids(set())
    assert everything == list(range(0, 30, 3))

    # tf loses three of them. The rest must keep the numbers they already had -- the
    # failure this guards is not "fewer frames" but "different frames".
    fewer = posed_ids({0, 9, 21})
    assert fewer == [3, 6, 12, 15, 18, 24, 27]
    assert set(fewer) < set(everything)


def test_the_smoothed_global_map_wins_but_an_empty_one_falls_back_to_the_raw() -> None:
    """`global_map_smooth` writes `<name>_smoothed`, and that closing is the map to draw.

    The fallback is the half worth testing: preferring the smoothed stream on its NAME
    alone would take the whole world down whenever one was declared and never written,
    with a perfectly good raw map sitting beside it in the same recording.
    """

    def store_of(**clouds: np.ndarray) -> SimpleNamespace:
        streams = {}
        for name, points in clouds.items():
            # `order_by("ts")` first, because the reader takes the last message IN TIME:
            # a map stream with several messages is the map as it grew.
            stream = SimpleNamespace(
                last=lambda points=points: SimpleNamespace(
                    ts=0.0,
                    data=SimpleNamespace(points_f32=lambda: points, frame_id="odom"),
                )
            )
            stream.order_by = lambda field, stream=stream: stream
            streams[name] = stream
        return SimpleNamespace(list_streams=lambda: list(streams), streams=streams)

    def reading(store: SimpleNamespace) -> WorldCache:
        world = WorldCache()
        world.config = SimpleNamespace(
            global_map_stream_name="global_map_smoothed", world_frame="odom"
        )
        world._ensure_store = lambda: store
        return world

    raw = np.zeros((1, 3), dtype=np.float32)
    smoothed = np.ones((2, 3), dtype=np.float32)
    both = store_of(global_map=raw, global_map_smoothed=smoothed)

    name, xyz = reading(both)._global_map_cloud()
    assert (name, len(xyz)) == ("global_map_smoothed", 2)

    # The flag names the map and nothing overrules it, or there is no way back to the raw
    # one on a recording that has been smoothed.
    asked_for_raw = reading(both)
    asked_for_raw.config.global_map_stream_name = "global_map"
    assert asked_for_raw._global_map_cloud()[0] == "global_map"

    nothing_written = store_of(global_map=raw, global_map_smoothed=np.empty((0, 3), np.float32))
    name, xyz = reading(nothing_written)._global_map_cloud()
    assert (name, len(xyz)) == ("global_map", 1)

    only_raw = reading(store_of(global_map=raw))
    assert only_raw._global_map_cloud()[0] == "global_map"

    assert reading(store_of(other_map=raw))._global_map_cloud() is None


def test_adopting_embeddings_puts_the_planner_s_map_back() -> None:
    """A reopen clears `_map_xyz`, and `_map_xyz` is the map Navigate walks over.

    On an mcap the finished index is picked up by reopening the recording, which drops
    every world cache with the store that built them. Nothing rebuilt them until the next
    viewer connected -- so on the demo's own path (add embeddings, ask, press Navigate)
    the answer came back and Navigate answered 503 "the map is still building" for ever,
    over a map drawn on screen in front of you. Measured live on grocery.mcap.
    """
    from dimos.teleop.memory_world.visual_answers import VisualAnswers

    done: list[str] = []

    class Host(VisualAnswers):
        def __init__(self) -> None:
            self.config = SimpleNamespace(store_path="/nowhere/grocery.mcap")

        def _reopen_recording(self) -> None:
            done.append("reopen")

        def _build_visual_index(self) -> None:
            done.append("index")

        def _ensure_world_cache(self):  # type: ignore[no-untyped-def]
            done.append("world_cache")

    Host()._adopt_embeddings()

    assert "world_cache" in done, "the reopen left the planner with no map"
    assert done.index("reopen") < done.index("world_cache"), "rebuilt before it was cleared"


# ---- the answer, end to end -------------------------------------------------------


def test_a_question_becomes_places_with_photographs() -> None:
    """Drive the whole answer path over a stubbed index: question in, places and pictures out.

    The stubs are the model and the store, which is everything slow; the path between
    them -- scoring, clustering, the result the viewer renders, the evidence headers it
    hangs -- is the real code. This is the one test that fails if a rename in
    `visual_answers` or `query` breaks the demo's only feature.
    """
    from dimos.teleop.memory_world.visual_answers import VisualAnswers

    found = [
        Place(position=(1.0, 2.0, 0.0), similarity=0.42, source_id=3, ts=101.0),
        Place(position=(8.0, 2.0, 0.0), similarity=0.21, source_id=9, ts=102.0),
    ]
    asked: dict[str, object] = {}
    published: dict[str, object] = {}

    class Host(VisualAnswers):
        def __init__(self) -> None:
            self._store_lock = threading.RLock()
            self._clients_lock = threading.RLock()
            self._last_answer = (None, None)
            self._active_query_images: list[tuple[dict, bytes]] = []
            self.config = SimpleNamespace(
                store_path="/nowhere/walk.db",
                search_top_k=5,
                place_radius_m=1.0,
                max_places=3,
                min_similarity=0.05,
                world_frame="odom",
                image_stream_name="color_image",
                query_image_max_size=240,
                thumbnail_jpeg_quality=60,
                query_image_distance_m=1.5,
            )

        def _ensure_visual_index(self):  # type: ignore[no-untyped-def]
            def search(text, k=200, window=None):  # type: ignore[no-untyped-def]
                asked["window"] = window
                return found

            return SimpleNamespace(
                count=lambda: 7,
                time_span=lambda: (100.0, 200.0),
                search=search,
            )

        def _markers_near(self, positions):  # type: ignore[no-untyped-def]
            return [11]

        def _add_route_to_result(self, result) -> None:  # type: ignore[no-untyped-def]
            published["routed"] = True

        def _query_is_current(self, query_id):  # type: ignore[no-untyped-def]
            return True

        def _publish_query_result(self, result) -> str:  # type: ignore[no-untyped-def]
            published["result"] = result
            return "qid"

        def _camera_hfov(self) -> float:
            return 70.0

        def _broadcast(self, message) -> None:  # type: ignore[no-untyped-def]
            published.setdefault("broadcasts", []).append(message)  # type: ignore[union-attr]

        def _ensure_store(self):  # type: ignore[no-untyped-def]
            frame = SimpleNamespace(data=np.zeros((4, 6, 3), dtype=np.uint8), ts=1.0)
            stream = SimpleNamespace(at=lambda ts, tolerance: [frame])
            return SimpleNamespace(streams={"color_image": stream})

        @staticmethod
        def _encode_jpeg(img, max_size, quality) -> bytes:  # type: ignore[no-untyped-def]
            return b"jpeg"

    host = Host()
    outcome = host._find_with_siglip("a shelf of bottles", 0.0)

    assert outcome.success, outcome.message
    assert outcome.metadata["engine"] == "siglip"
    assert len(outcome.metadata["places"]) == 2
    assert outcome.duration_ms > 0
    assert asked["window"] is None, "a question about the whole recording narrowed it"
    # Where in the recording each place sits, which is what answers "the FIRST one":
    # the list is ranked by score, so without this there is no time order to read.
    assert outcome.metadata["places"][0]["seconds_into_recording"] == pytest.approx(1.0)

    result = published["result"]
    assert result.engine == "siglip"
    assert len(result.clusters) == 2, "the viewer builds its results bar from these"
    assert result.observation_ids == [11], "no marker was lit, so no photograph is shown"
    assert result.focus_point == (1.0, 2.0, 0.0), "the answer flies to the wrong place"

    # One evidence photograph per place, each carrying where its camera stood, which
    # pixel matched, and the point that pixel produced -- the viewer draws all three.
    images = host._active_query_images
    assert len(images) == 2, f"{len(images)} evidence headers for 2 places"
    for index, (header, jpeg) in enumerate(images):
        assert jpeg == b"jpeg"
        assert header["cluster"] == index, "an image not in a place is refused by /navigate"
        assert header["query_id"] == "qid"
        assert len(header["position"]) == 3
        # No `uv`: one vector per image scores the whole picture, so there is no in-frame
        # hotspot, and the viewer only rings one when the server sends both keys.
        assert "uv" not in header, "an in-frame hotspot was published for a whole-image match"
        assert header["point"] == [float(v) for v in found[index].position]


def test_prebuild_refuses_a_recording_it_cannot_build_instead_of_leaving_half_of_it(
    memory_world,  # type: ignore[no-untyped-def]
) -> None:
    # The empty store has no camera frames, so there is nothing to embed; prebuild must
    # say so and exit non-zero rather than leave a half-prepared recording behind looking
    # ready. (It used to fail one step earlier, on the replay the timeline needed built;
    # nothing builds a replay any more, so the index is the whole of prebuild.)
    from dimos.teleop.memory_world import prebuild

    with pytest.raises(SystemExit) as refused:
        prebuild.main([memory_world.config.store_path])
    assert "prebuild failed" in str(refused.value)
    assert "visual index" in str(refused.value)


# ---- the chat panel ---------------------------------------------------------------


def test_a_question_is_not_mistaken_for_a_program_and_a_program_is_found() -> None:
    """Which tool calls get summarised rather than dumped.

    The failure that matters is the false positive: `{"query": "a fire extinguisher"}` read
    as code would replace the one readable row in the transcript with a model's guess about
    a phrase. So a python token alone is not enough and a field NAME alone is not either.
    """
    from dimos.teleop.memory_world.chat import python_in_args

    assert python_in_args('{"query":"a fire extinguisher"}') is None
    assert python_in_args('{"text":"where is the entrance, and how tall is it"}') is None
    assert python_in_args("not json at all") is None
    assert python_in_args('{"n":3}') is None

    program = "import numpy as np\nxyz = np.asarray(cloud)\nreturn xyz.mean(axis=0)"
    found = python_in_args(json.dumps({"code": program}))
    assert found == ("code", program)
    # ...and under a name nobody would guess, because it is multi-line python.
    found = python_in_args(json.dumps({"payload": program}))
    assert found == ("payload", program)


def test_a_summary_that_cannot_be_had_leaves_the_program_showing() -> None:
    """The summariser is a nicety over a subprocess, and must never break a transcript."""
    from dimos.teleop.memory_world import chat

    was = chat.SUMMARY_COMMAND
    try:
        chat.SUMMARY_COMMAND = ("definitely-not-a-command-on-this-machine",)
        assert chat.summarise_python("import numpy as np\nprint(np.pi)") is None
    finally:
        chat.SUMMARY_COMMAND = was


# ---- analyze_memory: the agent's own program ---------------------------------------


def test_an_analysis_runs_in_a_child_and_its_result_is_published(memory_world) -> None:
    """The whole path: source in, child process, validated result, drawn answer.

    Runs a REAL subprocess against a real (empty) store rather than a stub, because the
    parts most likely to break are the ones a stub cannot have -- the bootstrap importing
    `open_recording`, `stepwise` being loadable by path, and the sentinel surviving the
    round trip through stdio.
    """
    published: list[MemoryQueryResult] = []
    memory_world._publish_query_result = lambda result: (  # type: ignore[method-assign]
        published.append(result) or "query-1"
    )

    outcome = memory_world.analyze_memory(
        "names = store.list_streams()\n"
        "result = {'answer': f'{len(names)} streams', 'focus_point': [1.0, 2.0, 3.0]}\n",
        timeout=60,
    )

    assert outcome.success, outcome.message
    assert outcome.message == "0 streams"
    assert published and published[0].focus_point == (1.0, 2.0, 3.0)


def test_an_analysis_that_assigns_nothing_fails_with_what_the_child_said(memory_world) -> None:
    """A program that forgets `result` must come back as a readable failure.

    The agent's next move is to fix its program, and it can only do that from the text in
    the failure -- so the child's own message has to reach it rather than "no result".
    """
    outcome = memory_world.analyze_memory("total = 1 + 1\n", timeout=60)

    assert not outcome.success
    assert outcome.error_code == "EXECUTION_FAILED"
    assert "dictionary" in outcome.message


def test_an_analysis_that_never_stops_is_killed_rather_than_waited_for(memory_world) -> None:
    """The timeout is the whole reason this runs in a child. A loop must not take the demo."""
    outcome = memory_world.analyze_memory("while True:\n    pass\n", timeout=1.0)

    assert not outcome.success
    assert outcome.error_code == "EXECUTION_TIMEOUT"


def test_every_step_of_a_program_is_reported_as_it_runs(memory_world) -> None:
    """The steps are what the panel shows while an analysis is running.

    Asserted as a SEQUENCE of (index, status), not as a count: a relay that reported every
    step twice, or reported them all at the end, would still pass a count.
    """
    seen: list[tuple[int, str]] = []
    memory_world._on_analysis_step = lambda step: seen.append((step["index"], step["status"]))  # type: ignore[method-assign]
    memory_world._publish_query_result = lambda result: "query-1"  # type: ignore[method-assign]

    outcome = memory_world.analyze_memory(
        "a = 1\nfor _ in range(2):\n    a += 1\nresult = {'answer': str(a)}\n",
        timeout=60,
    )

    assert outcome.success, outcome.message
    assert outcome.message == "3"
    assert seen == [(0, "start"), (0, "done"), (1, "start"), (1, "done"), (2, "start"), (2, "done")]


def test_a_result_the_viewer_could_not_draw_is_refused_with_one_line_per_complaint() -> None:
    """Pydantic says the same thing once per offending item; the agent needs it once."""
    from pydantic import ValidationError

    from dimos.teleop.memory_world.query import validation_summary

    try:
        MemoryQueryResult(
            answer="here",
            points=[{"position": [0.0, 0.0]}, {"position": [1.0, 1.0]}],  # type: ignore[list-item]
        )
    except ValidationError as exc:
        summary = validation_summary(exc)
    else:
        raise AssertionError("a two-coordinate point should not validate")

    assert summary.count(";") == 0  # one complaint, not one per point
    assert "(2 of them)" in summary


def _hyperspace_host(published: dict):  # type: ignore[no-untyped-def]
    """The smallest thing `_draw_hyperspace_answer` will run against."""
    from dimos.teleop.memory_world.hyperspace_answers import HyperspaceAnswers

    class Host(HyperspaceAnswers):
        config = SimpleNamespace(place_radius_m=2.5, world_frame="odom")

        def __init__(self) -> None:
            self._clients_lock = threading.Lock()
            self._last_answer = (None, None)

        def _publish_query_result(self, result) -> str:  # type: ignore[no-untyped-def]
            published["result"] = result
            return "qid"

        def _markers_near(self, positions):  # type: ignore[no-untyped-def]
            return []

        def _query_is_current(self, query_id) -> bool:  # type: ignore[no-untyped-def]
            return True

    return Host()


def test_a_measured_object_is_drawn_as_a_box_and_an_unmeasured_place_is_not() -> None:
    """The detector's own size reaches the viewer instead of dying as a radius.

    Hyperspace backprojects the 2-D box through depth, so `extent` is the thing's
    measured size -- and the answer used to collapse it to `max(extent)/2` and send a
    sphere. A heatmap place measured nothing (zero extent) and must still get NO box:
    a box invented from the configured radius is indistinguishable, on screen, from
    one something actually measured.
    """
    from dimos.mapping.hyperspace.msgs import FoundObject, FoundObjects

    published: dict = {}
    _hyperspace_host(published)._draw_hyperspace_answer(
        FoundObjects(
            query="a fire extinguisher",
            kind="item",
            frame="odom",
            objects=[
                FoundObject(
                    frame="odom",
                    centre=(1.0, 2.0, 0.5),
                    extent=(0.3, 0.25, 0.73),
                    confidence=0.9,
                    place_id=1,
                ),
                FoundObject(
                    frame="odom",
                    centre=(9.0, 4.0, 0.0),
                    extent=(0.0, 0.0, 0.0),
                    confidence=0.4,
                    place_id=2,
                ),
            ],
        )
    )

    result = published["result"]
    assert len(result.clusters) == 2, "both places are still named"
    assert len(result.boxes) == 1, "the unmeasured place invented a size"
    assert result.boxes[0].centre == (1.0, 2.0, 0.5)
    assert result.boxes[0].extent == (0.3, 0.25, 0.73), "the extent was halved or rounded"
    # The point's radius is still half the longest side, so the voxels under the box
    # light up too; the box is added beside it, not instead of it.
    assert result.points[0].radius == pytest.approx(0.365)
    assert result.points[1].radius is None, "a fallback radius is not a measurement"


def test_a_box_too_big_for_the_viewer_is_clamped_rather_than_losing_the_whole_answer() -> None:
    """One oversized measurement used to raise and take every other place with it."""
    from dimos.mapping.hyperspace.msgs import FoundObject, FoundObjects
    from dimos.teleop.memory_world.query import MAX_HIGHLIGHT_RADIUS_M

    published: dict = {}
    _hyperspace_host(published)._draw_hyperspace_answer(
        FoundObjects(
            query="the wall",
            kind="item",
            frame="odom",
            # Flat on the view axis (every depth reading on one plane) AND longer than
            # the viewer's metre budget: the two ends a measurement can fail at.
            objects=[
                FoundObject(
                    frame="odom",
                    centre=(0.0, 0.0, 0.0),
                    extent=(40.0, 0.0, 2.0),
                    confidence=0.8,
                    place_id=1,
                )
            ],
        )
    )

    box = published["result"].boxes[0]
    assert box.extent[0] == 2 * MAX_HIGHLIGHT_RADIUS_M
    assert box.extent[1] > 0.0, "a zero side is un-drawable and would have raised"
    assert box.extent[2] == 2.0, "a side inside the budget was changed"
    assert published["result"].points[0].radius == MAX_HIGHLIGHT_RADIUS_M
