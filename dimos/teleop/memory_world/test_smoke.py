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


def test_a_world_frame_scan_is_moved_back_to_where_the_rays_started() -> None:
    """The map is ray-traced, so a scan has to be in the SENSOR's frame to be traced.

    Some recordings store the cloud already registered in the world. Taking those points
    as if they were sensor-relative casts every ray from the wrong origin, and the map
    that comes out is carved in the wrong places -- a quiet, total corruption that still
    renders as a plausible building.
    """
    from dimos.teleop.memory_world.replay import sensor_scan

    world_from_sensor = pose_matrix((10.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    in_world = np.array([[11.0, 0.0, 0.0], [12.0, 0.0, 0.0]])

    scan = sensor_scan(in_world, world_from_sensor, in_world=True)

    assert scan.position == (10.0, 0.0, 0.0), "the rays do not start at the sensor"
    assert np.allclose(scan.points, [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]), (
        "a world-frame scan was traced from the origin of the world, not the sensor"
    )
    # ...and a scan already in the sensor frame is left exactly alone.
    same = sensor_scan(in_world, world_from_sensor, in_world=False)
    assert np.allclose(same.points, in_world)


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
        streams = {
            name: SimpleNamespace(
                last=lambda points=points: SimpleNamespace(
                    ts=0.0,
                    data=SimpleNamespace(points_f32=lambda: points, frame_id="odom"),
                )
            )
            for name, points in clouds.items()
        }
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
    # The empty store has no camera frames, which the replay needs; prebuild must say so
    # and exit non-zero rather than move on to the index over a recording with no replay.
    from dimos.teleop.memory_world import prebuild

    with pytest.raises(SystemExit) as refused:
        prebuild.main([memory_world.config.store_path])
    assert "prebuild failed" in str(refused.value)
    assert "image stream" in str(refused.value)
