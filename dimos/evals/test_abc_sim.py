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

from collections.abc import Mapping, Sequence
import math
from pathlib import Path

import pytest

from dimos.evals.scorers import (
    all_of,
    count_inside,
    crossed_table,
    held_on_tray,
    in_box,
    in_cylinder,
    placed_beside,
    set_up,
    spelled,
    standing,
    tower,
    upright,
)
from dimos.evals.suites import abc_sim, abc_sim_single_arm
from dimos.evals.suites.abc_sim import COUNT_BOX
from dimos.evals.types import Outcome
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

Xyz = tuple[float, float, float]
Pose = Xyz | tuple[Xyz, tuple[float, float, float, float]]  # xyz, or (xyz, quat xyzw)
UP = (0.0, 0.0, 0.0, 1.0)
UPSIDE_DOWN = (1.0, 0.0, 0.0, 0.0)  # 180 degrees about x
ON_EDGE = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))  # 90 degrees about x


def _outcome(
    tmp_path: Path, name: str, frames: Sequence[Mapping[str, Pose]], dt: float = 1.0
) -> Outcome:
    """A recording with one body_poses message per frame of ``{body: pose}``."""
    db = tmp_path / f"{name}.db"
    with SqliteStore(path=str(db)) as store:
        stream = store.stream("body_poses", TFMessage)
        for i, frame in enumerate(frames):
            transforms = []
            for body, pose in frame.items():
                xyz, quat = (pose, UP) if len(pose) == 3 else pose
                transforms.append(
                    Transform(
                        translation=Vector3(*xyz),
                        rotation=Quaternion(*quat),
                        frame_id="world",
                        child_frame_id=body,
                        ts=i * dt,
                    )
                )
            stream.append(TFMessage(*transforms), ts=i * dt)
    # These graders read only the recording.
    return Outcome(trajectory=None, artifacts={"recording": db})  # type: ignore[arg-type]


def test_count_inside_needs_exactly_the_wanted_number(tmp_path: Path) -> None:
    grade = count_inside(("a", "b", "c"), "box", COUNT_BOX, want=2)
    box = (0.70, 0.0, 0.75)
    inside, outside = (0.70, 0.05, 0.80), (0.45, -0.3, 0.776)
    on_lid = (0.70, 0.0, 0.96)  # above the lid underside does not count

    def score(name: str, a: Xyz, b: Xyz, c: Xyz) -> float:
        return grade(_outcome(tmp_path, name, [{"box": box, "a": a, "b": b, "c": c}]))

    assert score("two", inside, inside, outside) == 1.0
    assert score("one", inside, outside, on_lid) == 0.0
    assert score("three", inside, inside, inside) == 0.0


def test_placed_beside_is_directional_and_placed(tmp_path: Path) -> None:
    grade = placed_beside("m", "r", (0.0, 1.0))
    ref, start = (0.44, -0.24, 0.775), (0.64, -0.24, 0.775)

    def score(name: str, end: Xyz) -> float:
        return grade(_outcome(tmp_path, name, [{"m": start, "r": ref}, {"m": end, "r": ref}]))

    assert score("left", (0.46, -0.12, 0.775)) == 1.0
    assert score("right", (0.46, -0.36, 0.775)) == 0.0, "wrong side"
    assert score("held", (0.46, -0.12, 0.90)) == 0.0, "still carried"
    assert score("touching", (0.44, -0.21, 0.775)) == 0.0, "too close"


def test_in_box_follows_the_container_and_keeps_others_out(tmp_path: Path) -> None:
    # The bin lies rotated so its local +y is world up, like ABC's put/sweep bin.
    grade = in_box(("a", "b"), "bin", ((0.0, 0.02, 0.0), (0.08, 0.11, 0.08)), outside=("c",))
    bin_pose = ((0.65, 0.0, 0.83), ON_EDGE)
    inside, outside = (0.66, 0.01, 0.87), (0.40, 0.30, 0.76)

    def score(name: str, a: Xyz, b: Xyz, c: Xyz) -> float:
        return grade(_outcome(tmp_path, name, [{"bin": bin_pose, "a": a, "b": b, "c": c}]))

    assert score("both", inside, inside, outside) == 1.0
    assert score("one", inside, outside, outside) == 0.0
    assert score("extra", inside, inside, inside) == 0.0, "an excluded item went in"
    assert score("above", inside, (0.65, 0.0, 1.0), outside) == 0.0, "above the rim"


def test_in_cylinder_counts_at_least(tmp_path: Path) -> None:
    grade = in_cylinder(("a", "b", "c"), None, (0.63, 0.0), 0.115, (0.85, 1.10), at_least=2)
    hung, on_table, far = (0.70, 0.05, 0.95), (0.70, 0.05, 0.80), (0.63, 0.30, 0.95)

    def score(name: str, *poses: Xyz) -> float:
        return grade(_outcome(tmp_path, name, [dict(zip("abc", poses, strict=True))]))

    assert score("two", hung, hung, far) == 1.0
    assert score("table", hung, on_table, far) == 0.0, "a mug on the table is not hung"


def test_upright_needs_rest_and_orientation(tmp_path: Path) -> None:
    grade = upright(("m",), max_lift_m=0.03)
    start = {"m": ((0.6, 0.0, 0.81), UPSIDE_DOWN)}

    def score(name: str, end: Pose) -> float:
        return grade(_outcome(tmp_path, name, [start, {"m": end}]))

    assert score("flipped", ((0.6, 0.0, 0.81), UP)) == 1.0
    assert score("not_flipped", ((0.6, 0.0, 0.81), UPSIDE_DOWN)) == 0.0
    assert score("held_up", ((0.6, 0.0, 0.95), UP)) == 0.0, "still in the gripper"


def test_standing_plates_and_all_of(tmp_path: Path) -> None:
    rack = ((0.62, -0.24, 0.75), UP)
    in_rack = in_box(("p",), "rack", ((0.0, 0.0, 0.2), (0.22, 0.27, 0.25)))
    grade = all_of(in_rack, standing(("p",)))

    def score(name: str, plate: Pose) -> float:
        return grade(_outcome(tmp_path, name, [{"rack": rack, "p": plate}]))

    assert score("standing", ((0.62, -0.24, 0.86), ON_EDGE)) == 1.0
    assert score("flat", ((0.62, -0.24, 0.80), UP)) == 0.0, "lying flat in the rack"
    assert score("away", ((0.40, 0.20, 0.86), ON_EDGE)) == 0.0, "standing, not in the rack"


def test_spelled_reads_left_to_right_face_up(tmp_path: Path) -> None:
    face_up = (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))  # 90 degrees about y: local x down
    grade = spelled(("C", "A", "T"), rest_z=0.765)

    def score(
        name: str, ys: tuple[float, ...], quat: tuple[float, float, float, float] = face_up
    ) -> float:
        frame: dict[str, Pose] = {
            c: ((0.6, y, 0.765), quat) for c, y in zip("CAT", ys, strict=True)
        }
        return grade(_outcome(tmp_path, name, [frame]))

    assert score("cat", (0.10, 0.06, 0.02)) == 1.0
    assert score("tac", (0.02, 0.06, 0.10)) == 0.0, "reads right to left"
    assert score("spread", (0.10, 0.0, -0.10)) == 0.0, "too far apart"
    assert score("blank", (0.10, 0.06, 0.02), UP) == 0.0, "letters face sideways"


def test_set_up_accepts_any_identical_piece(tmp_path: Path) -> None:
    board = ((0.6, 0.0, 0.75), (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)))  # local y up
    goals = {"pawn": [(0.10, 0.05, 0.0)]}
    grade = set_up("board", goals, {"pawn": ("pawn_1", "pawn_2")})
    on_square, home, aside = (0.70, 0.0, 0.80), (0.65, 0.05, 0.80), (0.50, -0.30, 0.75)

    def score(name: str, p1: Pose, p2: Pose) -> float:
        return grade(_outcome(tmp_path, name, [{"board": board, "pawn_1": p1, "pawn_2": p2}]))

    assert score("first", on_square, aside) == 1.0
    assert score("other", home, on_square) == 1.0, "either pawn fills the square"
    assert score("none", home, aside) == 0.0
    assert score("fallen", (on_square, UPSIDE_DOWN), aside) == 0.0


def test_crossed_table_needs_the_far_side_at_rest(tmp_path: Path) -> None:
    grade = crossed_table(
        "o", start_sign=1.0, table_z=0.75, x_bounds=(0.3, 0.9), y_bounds=(-0.65, 0.65)
    )

    def score(name: str, end: Xyz) -> float:
        return grade(_outcome(tmp_path, name, [{"o": (0.6, 0.25, 0.81)}, {"o": end}]))

    assert score("crossed", (0.6, -0.25, 0.81)) == 1.0
    assert score("middle", (0.6, -0.05, 0.81)) == 0.0
    assert score("held", (0.6, -0.25, 1.0)) == 0.0
    assert score("off_table", (0.6, -0.80, 0.81)) == 0.0


def test_held_on_tray_needs_a_lift_held_to_the_end(tmp_path: Path) -> None:
    grade = held_on_tray("tray", "ball", half_extents_m=(0.126, 0.1215), ball_radius_m=0.022)
    rest = {"tray": (0.6, 0.0, 0.754), "ball": (0.6, 0.0, 0.781)}
    up = {"tray": (0.6, 0.0, 0.85), "ball": (0.62, 0.01, 0.877)}
    dropped = {"tray": (0.6, 0.0, 0.85), "ball": (0.75, 0.0, 0.77)}

    def score(name: str, frames: Sequence[Mapping[str, Pose]]) -> float:
        return grade(_outcome(tmp_path, name, frames, dt=1.0))

    assert score("held", [rest, *[up] * 12]) == 1.0
    assert score("idle", [rest] * 13) == 0.0, "ABC's own rule passes this"
    assert score("short", [rest, *[up] * 5]) == 0.0, "not held for 10 s"
    assert score("dropped", [rest, *[up] * 10, dropped, up]) == 0.0


def test_tower_needs_three_levels(tmp_path: Path) -> None:
    grade = tower(("a", "b", "c", "d"), table_z=0.75)
    spread = {
        "a": (0.5, 0.0, 0.77),
        "b": (0.6, 0.1, 0.77),
        "c": (0.7, 0.0, 0.77),
        "d": (0.6, -0.2, 0.77),
    }
    stacked = {**spread, "b": (0.51, 0.0, 0.81), "c": (0.50, 0.01, 0.85)}
    two = {**spread, "b": (0.51, 0.0, 0.81)}
    assert grade(_outcome(tmp_path, "stacked", [stacked])) == 1.0
    assert grade(_outcome(tmp_path, "two", [two])) == 0.0
    assert grade(_outcome(tmp_path, "spread", [spread])) == 0.0


def test_missing_bodies_score_zero(tmp_path: Path) -> None:
    grade = in_box(("a",), "box", COUNT_BOX)
    assert grade(_outcome(tmp_path, "empty", [{"other": (0.0, 0.0, 0.0)}])) == 0.0


def test_suite_has_every_abc_task_once() -> None:
    names = {case.id.removeprefix("abc_") for case in abc_sim.SUITE}
    counts = {f"count_{w}_into_opaque_box" for w in ("one", "two", "three", "four")}
    spells = {f"spell_{w}" for w in ("cat", "dog", "fish", "bair", "xdof", "abc", "yam", "agi")}
    drawers = {f"put_markers_in_{d}_drawer" for d in ("top", "middle", "bottom")}
    assert names == {
        "throw_plastic_bottles_in_bin",
        "put_plastic_bottles_in_bin",
        "conveyor_pick",
        "count_into_opaque_box",
        *counts,
        "put_relative",
        "grab_specific_object_from_clutter",
        "ball_tray_balancing",
        "inhand_transfer_item_to_other_side",
        "multi_drawer_search",
        *drawers,
        "pouring",
        "hang_mug_on_mug_rack",
        "turn_mug_right_side_up",
        "sweep_away_paper_scraps_from_table",
        "sort_nuts_and_bolts",
        "sort_lego_blocks",
        "load_plates_into_dish_rack",
        "set_up_chess_pieces_on_board",
        "build_wood_block_tower",
        *spells,
    }
    assert len(abc_sim.SUITE) == 33


@pytest.mark.parametrize("case", abc_sim.SUITE, ids=lambda case: case.id)
def test_case_is_tagged_with_its_arms(case) -> None:  # type: ignore[no-untyped-def]
    assert len(case.tags & {"single_arm", "dual_arm"}) == 1
    assert case.environment.config.blueprint[0] == "dual-openyam-abc-sim"


def test_single_arm_suite_builds() -> None:
    ids = {case.id for case in abc_sim_single_arm.SUITE}
    assert ids == {"abc_count_two", "abc_grab_clutter", "abc_put_relative"}
