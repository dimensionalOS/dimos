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

"""The query surface: three shapes, one handle, and answers you can come back for."""

from __future__ import annotations

from dimos.mapping.hyperspace.queries import (
    AREA_PROMPTS,
    Place,
    Query,
    QueryBook,
    near_enough,
    negative_prompts,
)


def a_place(x: float, score: float = 1.0) -> Place:
    return Place(where=(x, 0.0, 0.0), frame="odom", kind="heatmap", score=score)


def test_coming_back_for_more_walks_the_list_instead_of_repeating_it() -> None:
    """Two calls for the rest must not hand the same answer twice.

    The whole point of the handle is that an agent takes the first place, tries it, and
    comes back. If the second call replayed the first answer it would drive to the same
    spot forever.
    """
    book = QueryBook()
    query = book.put(
        Query(query_id="item-1", text="a cone", kind="item", places=[a_place(i) for i in range(5)])
    )
    query.taken = 1

    first_rest = query.places[query.taken :][:2]
    query.taken += len(first_rest)
    second_rest = query.places[query.taken :]
    query.taken += len(second_rest)

    assert [place.where[0] for place in first_rest] == [1.0, 2.0]
    assert [place.where[0] for place in second_rest] == [3.0, 4.0]
    assert query.remaining == 0


def test_a_forgotten_query_says_so_rather_than_looking_empty() -> None:
    """ "There were no more" and "I no longer remember" are different facts.

    The book is bounded, so a long-running robot drops old questions. An agent told
    "no more results" would conclude it had seen everything.
    """
    book = QueryBook(keep=2)
    for number in range(3):
        book.put(Query(query_id=f"item-{number}", text="a cone", kind="item"))

    assert book.get("item-0") is None, "the oldest was dropped"
    assert book.get("item-2") is not None
    assert book.ids() == ["item-1", "item-2"]


def test_ids_do_not_collide_between_kinds_or_calls() -> None:
    book = QueryBook()
    made = {book.next_id("item") for _ in range(3)} | {book.next_id("area") for _ in range(3)}
    assert len(made) == 6, f"ids repeated: {made}"


def test_a_radius_keeps_what_is_near_and_an_unknown_position_keeps_everything() -> None:
    """A radius measured from a position nobody recorded would answer about the wrong
    part of the map, so no origin means no filtering rather than a filter from zero."""
    places = [a_place(0.0), a_place(5.0), a_place(50.0)]

    assert len(near_enough(places, (0.0, 0.0, 0.0), 10.0)) == 2
    assert len(near_enough(places, (0.0, 0.0, 0.0), 0.0)) == 3, "no radius keeps everything"
    assert len(near_enough(places, None, 10.0)) == 3, "no known position cannot filter"


def test_a_heatmap_place_does_not_claim_a_size_it_never_measured() -> None:
    """A box has an extent because a detector drew one. A voxel does not.

    Reporting a size for a heatmap answer would have an agent plan around a number that
    came from nowhere.
    """
    voxel = a_place(1.0)
    assert voxel.extent is None
    assert "extent" not in voxel.as_dict()

    box = Place(where=(1.0, 2.0, 3.0), frame="odom", kind="item", score=0.8, extent=(0.5, 0.5, 1.0))
    assert box.as_dict()["extent"] == [0.5, 0.5, 1.0]


def test_an_area_is_contrasted_against_objects_and_surfaces_both() -> None:
    """This test used to assert the OPPOSITE, on the plausible-sounding grounds that
    subtracting "a wall" would subtract the room an area query asks for. Judging by the
    top six frames could not tell the two apart -- they were right either way -- so the
    claim went untested until the cells were counted against the kitchen's real
    rectangle: object-ness alone gave 22 cells inside it and 268 outside, the surface set
    alone 20 and 40, and both together 20 and 29. Subtracting object-ness alone left
    nothing subtracting walls, and the false positives were walls.

    So an area contrast needs BOTH halves, and a set missing either one is the bug this
    test exists to catch."""
    assert any("object" in prompt or "equipment" in prompt for prompt in AREA_PROMPTS), (
        f"an area contrast needs the object-ness terms that favour a wide shot: {AREA_PROMPTS}"
    )
    for surface in ("floor", "wall", "ceiling"):
        assert any(surface in prompt for prompt in AREA_PROMPTS), (
            f"an area contrast must subtract {surface!r} or walls answer: {AREA_PROMPTS}"
        )


def test_the_query_module_can_configure_everything_its_start_reads() -> None:
    """Every `self.config.X` on the start path has to exist on the config.

    `Hyperspace.start()` crashed on `max_depth_m` -- present on the ingest's config and
    missing from the query module's -- so the detector never loaded and the module was
    unusable, while `map live` and `map find` were fine because they build the query
    object directly and never go through the module. Nothing caught it because nothing
    started the module. A missing field is an AttributeError at start, which on a robot
    is a module that is simply not there.
    """
    from pathlib import Path
    import re

    from dimos.mapping.hyperspace.module import HyperspaceConfig

    source = Path(__file__).with_name("module.py").read_text()
    # The query module's half of the file: from its config to the end.
    start = source.index("class HyperspaceConfig(")
    read = set(re.findall(r"self\.config\.([a-z_0-9]+)", source[start:]))
    known = set(HyperspaceConfig.model_fields)
    missing = sorted(read - known)
    assert not missing, f"Hyperspace reads config fields it does not declare: {missing}"


def test_an_item_place_is_built_from_fields_a_found_object_actually_has() -> None:
    """A Place is assembled by hand from a FoundObject, so a renamed or imagined field
    is an AttributeError at the moment of answering -- after the detector has already
    been paid for. `found.arrived` was invented; the real one is `stamp`."""
    from dimos.mapping.hyperspace.msgs import FoundObject

    fields = set(FoundObject.__dataclass_fields__)
    for name in ("centre", "frame", "confidence", "depth_m", "extent", "views", "stamp"):
        assert name in fields, f"_fill_from_detector reads {name!r} and it is gone"


def test_a_hot_patch_lands_where_its_frame_was_looking() -> None:
    """The heatmap and area answers are placed by hand, so the arithmetic is the risk.

    A patch carries the ray it sat on and how far the depth said that was. The point is
    `(ray.x * d, ray.y * d, d)` in the camera, then through that frame's pose -- the same
    arithmetic `object_points` does for a box. Get the convention wrong and every answer
    is confidently in the wrong place, which is worse than no answer.
    """
    import numpy as np

    # A camera two metres up, looking down the world's +x with z forward in its own frame.
    pose = np.array(
        [
            [0.0, 0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 2.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    # Dead centre of the image, three metres out.
    centre = pose @ np.array([0.0 * 3.0, 0.0 * 3.0, 3.0, 1.0])
    assert np.allclose(centre[:3], [3.0, 0.0, 2.0]), (
        f"straight ahead at 3 m should be 3 m along +x, got {centre[:3]}"
    )

    # Up and to the right in the image: +x right, +y DOWN in an optical frame, so this
    # must come back to the camera's right and ABOVE it in the world.
    off = pose @ np.array([0.5 * 4.0, -0.25 * 4.0, 4.0, 1.0])
    assert np.allclose(off[:3], [4.0, -2.0, 3.0]), f"right and up in the image put it at {off[:3]}"

    size = 0.1
    cell = tuple(int(np.floor(value / size)) for value in centre[:3])
    assert cell == (30, 0, 20)
    back = tuple((index + 0.5) * size for index in cell)
    assert all(abs(a - b) <= size for a, b in zip(back, centre[:3], strict=False)), (
        "a voxel's centre has to be within one voxel of the point that made it"
    )


def test_detect_models_names_member_tags_not_checkpoints() -> None:
    """`detect_models` is a list of MEMBER TAGS, the same thing `--models` takes.

    It used to be handed to `warm()` as if it were checkpoint names, so setting it at all
    sent a tag like "base_patch16_224" to the Hugging Face hub as a repository and the
    module died on a 404 -- while the SAME value was simultaneously used as a tag to
    filter the streams. One value cannot be both.
    """
    from pathlib import Path
    import re

    source = Path(__file__).with_name("module.py").read_text()
    assert "self.live.warm(self.config.detect_models" not in source, (
        "detect_models holds tags; warm() takes checkpoints"
    )
    # What warm is given has to come from spec_of, which is the tag -> checkpoint map.
    warm_call = re.search(
        r"wanted = \[spec_of\(tag\) for tag, _ in self\.live\.members\(\)\]", source
    )
    assert warm_call, "the checkpoints warm() loads should be derived from the tags searched"


def test_a_query_without_a_radius_never_looks_for_the_robot() -> None:
    """Finding the robot is a walk of the transform tree, and on a recording whose
    stamps are all in the past, asking for "now" is a lookup that can take forever. A
    query that never mentioned proximity must not pay for it, or hang on it."""
    from pathlib import Path

    source = Path(__file__).with_name("module.py").read_text()
    assert "self._where_the_robot_is(at_time) if within_m else None" in source, (
        "the robot's pose is only needed when a radius was asked for"
    )


def test_starting_the_module_does_not_build_the_dense_voxel_path() -> None:
    """The three query skills never touch the dense engine, so start() must not build it.

    It used to: every module paid a so400m text tower and its GPU memory at boot, which
    on an 8 GB card beside OWLv2 and the patch index is most of what there is -- and it
    is the slowest thing in the startup for a path most callers never use.
    """
    from pathlib import Path

    source = Path(__file__).with_name("module.py").read_text()
    body = source[source.index("    def start(self) -> None:\n        # What every question") :]
    body = body[: body.index("    @property")]
    assert "PatchEnsemble(" not in body, "start() is building the dense path's text towers"
    assert "HyperspaceQuery(" not in body, "start() is building the dense engine"
    # And a transform arriving must not build it either.
    # BOTH modules define handle_tf; this is about the query module's.
    query_module = source[source.index("class Hyperspace(MemoryModule)") :]
    tf_handler = query_module[query_module.index("async def handle_tf") :]
    tf_handler = tf_handler[: tf_handler.index("async def handle_query")]
    # Code only: the comment there explains why it must not, and says the name to do it.
    code = "\n".join(line for line in tf_handler.splitlines() if not line.strip().startswith("#"))
    assert "self.engine" not in code, (
        "touching self.engine in handle_tf builds the dense path on the first transform"
    )


def test_start_does_not_read_anything_the_lazy_engine_owns() -> None:
    """`start()` logged `self.model.tags` after the model stopped being built there.

    Making something lazy means every reference to it is now a construction or a crash,
    and a leftover log line is the easiest one to miss -- it was the last statement in
    `start()`, so the module came up and then died on its own success message.
    """
    from pathlib import Path

    source = Path(__file__).with_name("module.py").read_text()
    body = source[source.index("    def start(self) -> None:\n        # What every question") :]
    body = body[: body.index("    def _load_the_detector")]
    assert "self.model" not in body, "start() touches the lazily built model"
    assert "self.engine" not in body, "start() touches the lazily built engine"


def test_one_unplaceable_patch_does_not_take_the_whole_query_down() -> None:
    """A patch whose ray or pose carries a NaN has no position, and binning it raises.

    Out of two million patches, one is enough: `int(np.floor(nan))` is a ValueError and
    the query that was otherwise fine returns nothing. It also came back to the caller
    as INVALID_INPUT, telling them to fix a question that was never the problem.
    """
    import numpy as np

    good = np.array([1.0, 2.0, 3.0])
    bad = np.array([float("nan"), 2.0, 3.0])
    assert np.isfinite(good).all()
    assert not np.isfinite(bad).all()

    from pathlib import Path

    source = Path(__file__).with_name("module.py").read_text()
    assert "if not np.isfinite(here[:3]).all():" in source, (
        "every placed point has to be checked before it is binned"
    )
    assert "QUERY_FAILED" in source, (
        "a fault inside the query must not be reported as the caller's bad input"
    )


def test_negative_terms_replace_the_default_rather_than_adding_to_it() -> None:
    """What a caller names is the whole contrast, not an extra on top of ours.

    An agent that says "subtract an office" and gets the office subtracted *plus* the
    three object prompts cannot tell what its own words did, and the one thing worth
    having here is a knob whose effect is legible.
    """
    assert negative_prompts("an office, a hallway", "area") == ("an office", "a hallway")
    assert negative_prompts("an office, a hallway", "item") == ("an office", "a hallway")


def test_nothing_named_keeps_each_kind_default() -> None:
    """Empty is not "no contrast": dropping it was measured at 2 of 6 on "kitchen"."""
    assert negative_prompts("", "area") == AREA_PROMPTS
    assert negative_prompts("   ,  ,", "area") == AREA_PROMPTS
    assert negative_prompts("", "item") is None
    assert negative_prompts("", "heatmap") is None


def test_negative_terms_survive_the_way_an_agent_writes_them() -> None:
    """A language model writes one string with loose spacing, and sometimes a list."""
    assert negative_prompts("a poster ,  a screen", "item") == ("a poster", "a screen")
    assert negative_prompts(["a poster", " a screen "], "item") == ("a poster", "a screen")


def test_every_query_skill_takes_negative_terms() -> None:
    """All three, or an agent has to know which of them listens -- and the rpc too,
    since the skills are meant to be thin wrappers over one implementation."""
    import inspect

    from dimos.mapping.hyperspace.module import Hyperspace

    for name in ("start_item_query", "start_heatmap_query", "start_area_query", "run_query"):
        taken = inspect.signature(getattr(Hyperspace, name)).parameters
        assert "negative_terms" in taken, f"{name} cannot be given negative terms"
        assert taken["negative_terms"].default == "", f"{name} must default to its kind's own"


def test_the_search_is_actually_told_what_to_subtract() -> None:
    """The argument has to reach `hot_frames`; a skill that accepts it and drops it is
    worse than one that never offered. This has happened here once already -- the
    per-call prompts were added to the docstring and never to the call."""
    from pathlib import Path

    source = Path(__file__).with_name("module.py").read_text()
    assert "background_prompts=negatives" in source
    assert "self.live.ask(query.text, background_prompts=negatives)" in source
    detect = Path(__file__).with_name("detect.py").read_text()
    assert "background_prompts=background_prompts," in detect


def test_a_cell_is_scored_by_how_well_it_matches_not_by_how_much_landed_in_it() -> None:
    """Summing every patch that lands in a cell answers with whatever was looked at most.

    MEASURED on "kitchen" over sf_office_drive1, against the kitchen's real rectangle and
    under four different contrasts: the sum put its best answer 7.5-9.7 m outside the
    kitchen every time. A wall band the robot drove past a hundred times collects more
    patches than a kitchen it saw well from ten, and a sum cannot tell those apart.

    This is that case in miniature: a grazed cell with many weak patches against a
    well-matched one with a few strong ones.
    """
    from dimos.mapping.hyperspace.module import Hyperspace, HyperspaceConfig

    grazed, matched = (0, 0, 0), (1, 1, 1)
    placed = [(grazed, 0.02, ("cam", float(index % 20))) for index in range(40)]
    placed += [(matched, 0.09, ("cam", 100.0 + index)) for index in range(6)]
    module = Hyperspace.__new__(Hyperspace)
    module.config = HyperspaceConfig(db_path="unused")

    scored = module._cells_worth_answering(placed, module.config.heat_min_views)
    assert max(scored, key=lambda cell: scored[cell]) == matched, (
        "the well-matched cell has to win; summing gives it to the grazed one"
    )
    assert 0.02 * 40 > 0.09 * 6, (
        "this test is only meaningful while the sum would have picked the other one"
    )


def test_one_stray_patch_cannot_be_a_place() -> None:
    """Scoring by the mean alone answered with a cell holding one patch seen once.

    The view gate is what makes an answer a place rather than a pixel -- and it has to
    give way rather than answer nothing, since a short recording may have no cell seen
    from three viewpoints at all.
    """
    from dimos.mapping.hyperspace.module import Hyperspace, HyperspaceConfig

    stray, real = (0, 0, 0), (1, 1, 1)
    placed = [(stray, 0.5, ("cam", 1.0))]
    placed += [(real, 0.3, ("cam", float(index))) for index in (1.0, 2.0, 3.0)]
    module = Hyperspace.__new__(Hyperspace)
    module.config = HyperspaceConfig(db_path="unused")

    gated = module._cells_worth_answering(placed, module.config.heat_min_views)
    assert stray not in gated and real in gated

    everything = module._cells_worth_answering(placed, 1)
    assert stray in everything, "the gate has to be droppable, or a short recording answers nothing"


def test_the_patch_path_answers_end_to_end_over_a_fake_search(tmp_path) -> None:
    """Run `_fill_from_patches` for real, because its bugs are never in the arithmetic.

    Twice now this path has been broken by a name rather than a number -- `found.arrived`
    on a field that does not exist, and a `seen` dictionary that a refactor left behind --
    and neither showed up in a test that only checked the scoring. This drives the whole
    method with a search of two frames and a pose that is the identity, so anything the
    method reads has to exist.
    """
    import numpy as np

    from dimos.mapping.hyperspace import frames as frames_module
    from dimos.mapping.hyperspace.frames import Frame, Hit
    from dimos.mapping.hyperspace.module import Hyperspace, HyperspaceConfig
    from dimos.mapping.hyperspace.queries import Query

    def a_frame(name: str, ts: float, ray: tuple[float, float], score: float) -> Frame:
        frame = Frame(frame=name, ts=ts)
        frame.hits = [
            Hit(member="m", frame=name, ts=ts, cell=0, grid=(0, 0), ray=ray, depth=2.0, score=score)
        ]
        return frame

    # Three viewpoints of one patch of space, so the view gate has something to pass, and
    # one stray that nothing agrees with.
    made = [
        a_frame("cam", 1.0, (0.0, 0.0), 0.9),
        a_frame("cam", 2.0, (0.001, 0.0), 0.8),
        a_frame("cam", 3.0, (0.0, 0.001), 0.85),
        a_frame("cam", 4.0, (2.0, 2.0), 0.4),
    ]

    class OneFrame:
        def pose(self, frame: str, ts: float, world: str):
            del frame, ts, world
            return np.eye(4)

    class Live:
        towers = None
        held = None
        frames = OneFrame()

        def members(self):
            return [("m", "stream")]

    module = Hyperspace.__new__(Hyperspace)
    # A REAL PATH, because `_fill_from_patches` hands `self.store` to the search and the
    # `store` property then opens whatever `db_path` names. Pointed at a bare "unused" it
    # created a 12 KB sqlite file in the working directory on every run.
    module.config = HyperspaceConfig(db_path=str(tmp_path / "patches.db"))
    module._store = None
    module.live = Live()

    # The port a blueprint would bind. Stubbed rather than guarded in production: a
    # heatmap answer that reaches nobody is the bug this publish exists to fix, so the
    # code must not quietly tolerate a missing port.
    published: list = []

    class Port:
        def publish(self, message: object) -> None:
            published.append(message)

    module.found = Port()

    original = frames_module.hot_frames
    frames_module.hot_frames = lambda *args, **kwargs: made
    try:
        query = Query(query_id="area-1", text="kitchen", kind="area")
        module._fill_from_patches(query, "area")
    finally:
        frames_module.hot_frames = original

    assert query.places, f"the patch path answered nothing: {query.note!r}"
    best = query.places[0]
    assert best.frame == module.config.world_frame
    assert best.views >= 1 and best.score > 0
    assert best.kind == "area"
    assert query.timings.get("search") is not None

    # Published too, and SAYING WHICH KIND. MemWorld subscribed to `found`, got only the
    # detector's answers because this path published nothing, and their heatmap queries
    # answered correctly in words over an empty world -- which reads as the world being
    # broken rather than the wiring. A subscriber also has to be able to tell a scored
    # cell from a detection before it reads `confidence`, which is a different quantity
    # on a different scale for each.
    assert len(published) == 1, "the patch path has to publish what it found"
    answer = published[0]
    assert answer.kind == "area", "a subscriber cannot read `confidence` without this"
    assert answer.query == "kitchen"
    assert len(answer.objects) == len(query.places)
    first = answer.objects[0]
    assert tuple(first.centre) == tuple(best.where)
    assert first.confidence == best.score, "the cell's score, not a detector's"
    assert first.extent == (0.0, 0.0, 0.0), "nothing measured a size"
    assert first.camera_frame == "" and first.box2d == (0.0, 0.0, 0.0, 0.0), (
        "no photograph was chosen and no box was drawn"
    )
    assert first.stamp == best.seen_at, "a real frame stamp, for pulling the evidence"


def test_the_query_module_passes_its_ranking_knobs_to_the_search() -> None:
    """A knob the module declares and never hands on is a knob that does nothing.

    `rank_with` and `rank_frames` live on `DetectConfig`, which the module builds -- so
    declaring them on `HyperspaceConfig` is only half the wiring, and the missing half
    fails silently at the worst possible moment: the setting appears to be accepted and
    the search ignores it. Same family as `detect_models`, which was read as a member tag
    in one place and a Hugging Face checkpoint in another and died on a 404.
    """
    from pathlib import Path

    from dimos.mapping.hyperspace.module import HyperspaceConfig

    for name in ("rank_with", "rank_frames"):
        assert name in HyperspaceConfig.model_fields, f"the module cannot configure {name}"

    source = Path(__file__).with_name("module.py").read_text()
    for name in ("rank_with", "rank_frames"):
        assert f"{name}=self.config.{name}" in source, (
            f"{name} is declared on the config and never reaches DetectConfig"
        )
