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

"""All 33 ABC Sim tasks (amazon-far/abc, abc_sim/task_specs.py) with both OpenYAM arms.

    dimos evals run dimos.evals.suites.abc_sim --agent dimos.evals.agents.dimos_agent --tags count

One case per ABC task spec, ``abc_<spec name>``, tagged with its ABC scene (``env_task``)
and with ``dual_arm`` when it needs both arms (they hold, hand over or work both sides of
the table) or ``single_arm`` when one arm can do it. Each runs ``dual-openyam-abc-sim`` in
its scene (``data/abc_<env_task>``, see ``data/abc_yam/SOURCE.md``) and is graded on the
recorded body poses, following ``abc_sim/task_eval``. Where we depart from ABC:

- Scenes use one fixed layout (ABC's template, or ABC's nominal poses where its randomizer
  fills the scene in), not ABC's randomized episodes; prompts name the objects.
- ``conveyor`` (``static_belt``): the belt does not move; ABC drives it from Python.
- ``ball_tray_balancing``: ABC's evaluator passes a robot that never moves, so the grade
  needs the tray lifted with the ball on it for the last 10 s. ABC's perturbation pulses
  are not applied.
- ``put_markers_in_*_drawer`` and ``build_wood_block_tower`` have no ABC evaluator
  (``ours_grader``); their graders are ours.
"""

from __future__ import annotations

from collections.abc import Callable
import json

from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import (
    Box,
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
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.robot.manipulators.dual_openyam.abc_sim import (
    DUAL_OPENYAM_SIM_BASE_XY,
    abc_scene,
)

TABLE_Z = 0.75
GUIDANCE = (
    "You control two YAM arms with parallel grippers, and a fixed overhead RGB-D camera. "
    "Find objects with the localize module, get grasps for an object's point cloud from the grasp "
    "module, and move the arms and grippers through the manipulation module."
)
# The arm bases stand on the table top in some ABC scenes and 1 cm above it in the rest.
_BASE_Z = {
    "count_into_opaque_box": 0.75,
    "put_relative": 0.75,
    "grab_clutter": 0.75,
    "ball_tray_balancing": 0.75,
    "nuts_bolts_sorting": 0.75,
    "lego_blocks_sorting": 0.75,
}


def environment(env_task: str, bodies: tuple[str, ...]) -> MujocoEnvironment:
    base = [*DUAL_OPENYAM_SIM_BASE_XY, _BASE_Z.get(env_task, 0.76)]
    return MujocoEnvironment(
        blueprint=["dual-openyam-abc-sim", "manipulation-localize"],
        scene=abc_scene(f"abc_{env_task}"),
        tracked_bodies=bodies,
        module_env={
            "MANIPULATIONMODULE__MODEL__BASE_POSE": json.dumps(
                {"frame_id": "world", "position": base}
            ),
            # The overhead camera never moves, so one view must confirm an object.
            "MANIPULATIONLOCALIZE__PARAMS__MIN_VIEWS": "1",
        },
    )


def case(
    name: str,
    env_task: str,
    prompt: str,
    bodies: tuple[str, ...],
    grade: Callable[[Outcome], float],
    *,
    arms: str,
    tags: tuple[str, ...] = (),
    timeout_s: float = 1200.0,
) -> EvalCase:
    return EvalCase(
        id=f"abc_{name}",
        inputs=f"{prompt} {GUIDANCE}",
        environment=environment(env_task, bodies),
        grade=grade,
        timeout_s=timeout_s,
        tags=frozenset({"mujoco", "abc", "openyam", "manipulation", env_task, arms, *tags}),
    )


def _names(prefix: str, count: int, suffix: str = "") -> tuple[str, ...]:
    return tuple(f"{prefix}{i}{suffix}" for i in range(1, count + 1))


# Interiors, as (centre, half sizes) in the container's frame, measured from each scene's
# collision geometry or taken from ABC's goal-region sites.
THROW_BIN: Box = ((0.0, 0.0, 0.025), (0.12, 0.12, 0.155))
# The put/sweep bin is rotated so its local +y is world up.
PUT_BIN: Box = ((0.0, 0.02, 0.0), (0.08, 0.11, 0.08))
CONVEYOR_BIN: Box = ((0.0, 0.0, 0.09), (0.125, 0.18, 0.10))
# Inside the lidded box, below the lid (abc_sim/task_eval/count_box.py).
COUNT_BOX: Box = ((0.0, 0.0, 0.104), (0.106, 0.138, 0.088))
CLUTTER_BOX: Box = ((0.0, 0.0, 0.110), (0.060, 0.055, 0.180))
DRAWER_SEARCH_BIN: Box = ((0.0, 0.0, 0.100), (0.056, 0.052, 0.160))
SORTING_BIN: Box = ((0.0, 0.0, 0.048), (0.068, 0.080, 0.042))
LEGO_BIN: Box = ((0.0, 0.0, 0.044), (0.054, 0.050, 0.040))
# Inside a drawer, in the drawer's frame.
DRAWER: Box = ((0.0, -0.035, 0.010), (0.100, 0.095, 0.050))
# ABC's dish-rack footprint, 5 cm margin, -6..45 cm up (abc_sim/task_specs.py).
DISH_RACK: Box = ((-0.0065, 0.002, 0.195), (0.2215, 0.273, 0.255))

COUNT_ITEMS = _names("count_item_", 6)
COUNT_CUBES = COUNT_ITEMS[:4]
BOTTLES = _names("bottle_", 4)
CONVEYOR_OBJECTS = _names("conveyor_object_", 3)
MARKERS = _names("marker_", 5)
BEADS = _names("bead_", 10)
MUGS_3 = _names("mug_", 3)
MUGS_4 = _names("mug_", 4)
TRASH = _names("trash_", 7)
NUTS = _names("nut_", 10)
BOLTS = _names("bolt_", 10)
PLATES = ("plate", "plate_1")
WOOD_BLOCKS = (
    "arch_block_1",
    "arch_block_2",
    "bevel_block_1",
    "bevel_block_2",
    "slant_block_1",
    "slant_block_2",
    "L_block_1",
    "hole_block_1",
    "pCube_1",
)
LEGO = {color: _names(f"{color}_lego_", 3) for color in ("red", "yellow", "blue")}

# The six pieces the scene takes off the board, by kind, and their home squares in the
# board's frame (its local y is the board normal).
CHESS_GOALS = {
    "white_queen": [(0.1794, 0.0469, -0.0278)],
    "white_knight": [(0.1794, 0.0469, -0.1302)],
    "black_bishop": [(-0.1931, 0.0469, -0.0805)],
    "black_pawn": [(-0.1399, 0.0469, 0.1281)],
    "white_pawn": [(0.1261, 0.0469, -0.0278)],
    "black_rook": [(-0.1931, 0.0469, 0.1824)],
}
CHESS_PIECES = {
    "white_queen": ("white_queen_1",),
    "white_knight": ("white_knight_1", "white_knight_2"),
    "black_bishop": ("black_bishop_1", "black_bishop_2"),
    "black_pawn": _names("black_pawn_", 8),
    "white_pawn": _names("white_pawn_", 8),
    "black_rook": ("black_rook_1", "black_rook_2"),
}
CHESS_BODIES = ("chessboard", *(p for pieces in CHESS_PIECES.values() for p in pieces))

COUNT_WORDS = {1: "one", 2: "two", 3: "three", 4: "four"}
SPELL_WORDS = ("cat", "dog", "fish", "bair", "xdof", "abc", "yam", "agi")
LETTER_REST_Z = TABLE_Z + 0.015  # a 3 cm letter cube lying on the table


def _count_case(n: int) -> EvalCase:
    return case(
        f"count_{COUNT_WORDS[n]}_into_opaque_box",
        "count_into_opaque_box",
        f"Put exactly {COUNT_WORDS[n]} object{'s' if n > 1 else ''} into the dark box "
        "through the slot in its lid. Leave every other object outside the box.",
        (*COUNT_ITEMS, "opaque_count_box"),
        count_inside(COUNT_ITEMS, "opaque_count_box", COUNT_BOX, want=n),
        arms="single_arm",
        tags=("count",),
    )


def _drawer_case(drawer: str) -> EvalCase:
    body = f"link_{drawer}_drawer_low"
    return case(
        f"put_markers_in_{drawer}_drawer",
        "drawer",
        f"Put all five markers in the {drawer} drawer of the cabinet. Open the drawer first.",
        (*MARKERS, body),
        in_box(MARKERS, body, DRAWER),
        arms="dual_arm",
        tags=("drawer", "ours_grader"),
    )


def _spell_case(word: str) -> EvalCase:
    letters = tuple(f"block_{c.upper()}" for c in word)
    return case(
        f"spell_{word}",
        "blocks",
        f"Spell '{word}' with the letter cubes: put the cubes {', '.join(word.upper())} in one "
        "row on the table, reading left to right from the robot's side (first letter at "
        "the robot's left), letters facing up, about 2-5 cm apart.",
        letters,
        spelled(letters, rest_z=LETTER_REST_Z),
        arms="dual_arm",
        tags=("spell",),
    )


SUITE: Suite = [
    case(
        "throw_plastic_bottles_in_bin",
        "bottles",
        "Throw all four plastic bottles into the bin in the middle of the table.",
        (*BOTTLES, "bin_container"),
        in_box(BOTTLES, "bin_container", THROW_BIN),
        arms="dual_arm",
        tags=("bottles",),
    ),
    case(
        "put_plastic_bottles_in_bin",
        "put_bottles",
        "Put all four plastic water bottles into the bin in the middle of the table.",
        (*BOTTLES, "bin_container"),
        in_box(BOTTLES, "bin_container", PUT_BIN),
        arms="dual_arm",
        tags=("bottles",),
    ),
    case(
        "conveyor_pick",
        "conveyor_pick",
        "Pick the three objects off the conveyor belt and put them in the bin at the end of "
        "the table.",
        (*CONVEYOR_OBJECTS, "conveyor_bin"),
        in_box(CONVEYOR_OBJECTS, "conveyor_bin", CONVEYOR_BIN),
        arms="single_arm",
        tags=("static_belt",),
    ),
    case(
        "count_into_opaque_box",
        "count_into_opaque_box",
        "Put all the cubes into the dark box through the slot in its lid. Leave the other "
        "objects outside the box.",
        (*COUNT_ITEMS, "opaque_count_box"),
        in_box(COUNT_CUBES, "opaque_count_box", COUNT_BOX, outside=COUNT_ITEMS[4:]),
        arms="single_arm",
        tags=("count",),
    ),
    *(_count_case(n) for n in COUNT_WORDS),
    case(
        "put_relative",
        "put_relative",
        "Put the orange cube to the left of the green cube, from the robot's point of view.",
        ("relative_obj_06", "relative_obj_04"),
        placed_beside("relative_obj_06", "relative_obj_04", (0.0, 1.0)),
        arms="single_arm",
    ),
    case(
        "grab_specific_object_from_clutter",
        "grab_clutter",
        "Pick up the red cube from the clutter and place it in the box.",
        ("target_object", "grab_clutter_target_box"),
        in_box(("target_object",), "grab_clutter_target_box", CLUTTER_BOX),
        arms="single_arm",
    ),
    case(
        "ball_tray_balancing",
        "ball_tray_balancing",
        "Grab the tray by both handles, one arm on each, lift it at least 5 cm and keep the "
        "ball balanced on it. Hold it up, ball on the tray, until the episode ends: the "
        "last 10 seconds are judged.",
        ("ball_tray", "ball_tray_ball"),
        held_on_tray(
            "ball_tray", "ball_tray_ball", half_extents_m=(0.126, 0.1215), ball_radius_m=0.022
        ),
        arms="dual_arm",
        timeout_s=900.0,
    ),
    case(
        "inhand_transfer_item_to_other_side",
        "inhand_transfer",
        "Pick up the can on the left side of the table with the left arm, hand it to the "
        "right arm, and set it down on the right side of the table.",
        ("task_object",),
        crossed_table(
            "task_object",
            start_sign=1.0,
            table_z=TABLE_Z,
            x_bounds=(0.3025, 0.8975),
            y_bounds=(-0.65, 0.65),
        ),
        arms="dual_arm",
    ),
    case(
        "multi_drawer_search",
        "multi_drawer_search",
        "The letter cube A is hidden in one of the cabinet's drawers. Find it and place it in "
        "the bin; leave the other cubes where they are.",
        ("drawer_search_target", "drawer_search_goal_bin"),
        in_box(("drawer_search_target",), "drawer_search_goal_bin", DRAWER_SEARCH_BIN),
        arms="dual_arm",
        tags=("drawer",),
    ),
    *(_drawer_case(d) for d in ("top", "middle", "bottom")),
    case(
        "pouring",
        "pour",
        "Pour the beads from the mug into the cup, keeping the cup upright.",
        (*BEADS, "cup_1"),
        all_of(
            in_cylinder(BEADS, "cup_1", (0.0, 0.0), 0.034, (-0.045, 0.08), at_least=9),
            upright(("cup_1",), min_up=0.5),
        ),
        arms="dual_arm",
    ),
    case(
        "hang_mug_on_mug_rack",
        "mug_tree",
        "Hang all three mugs on the pegs of the mug tree.",
        MUGS_3,
        # Within 11.5 cm of the trunk and off the table (abc_sim/task_eval/mug_tree.py).
        in_cylinder(MUGS_3, None, (0.632, 0.0), 0.115, (0.85, 1.10)),
        arms="dual_arm",
        tags=("mugs",),
    ),
    case(
        "turn_mug_right_side_up",
        "mug_flip",
        "Turn all four mugs right side up and set them back down on the table.",
        MUGS_4,
        upright(MUGS_4, min_up=0.95, max_lift_m=0.03),
        arms="dual_arm",
        tags=("mugs",),
    ),
    case(
        "sweep_away_paper_scraps_from_table",
        "sweep",
        "Sweep the paper scraps off the table into the bin, using the brush and the dustpan.",
        (*TRASH, "bin_container"),
        in_box(TRASH, "bin_container", PUT_BIN),
        arms="dual_arm",
    ),
    case(
        "sort_nuts_and_bolts",
        "nuts_bolts_sorting",
        "Sort the loose nuts into the teal bin and the bolts into the orange bin.",
        (*NUTS, *BOLTS, "nuts_sorting_bin", "bolts_sorting_bin"),
        all_of(
            in_box(NUTS, "nuts_sorting_bin", SORTING_BIN),
            in_box(BOLTS, "bolts_sorting_bin", SORTING_BIN),
        ),
        arms="dual_arm",
        tags=("sorting",),
    ),
    case(
        "sort_lego_blocks",
        "lego_blocks_sorting",
        "Sort the red, yellow and blue lego blocks into the bins of matching colour.",
        (*(b for bs in LEGO.values() for b in bs), *(f"{c}_lego_sorting_bin" for c in LEGO)),
        all_of(*(in_box(LEGO[c], f"{c}_lego_sorting_bin", LEGO_BIN) for c in LEGO)),
        arms="dual_arm",
        tags=("sorting",),
    ),
    case(
        "load_plates_into_dish_rack",
        "dishrack",
        "Load both plates into the dish rack, standing upright in its slots.",
        (*PLATES, "dish_rack"),
        all_of(in_box(PLATES, "dish_rack", DISH_RACK), standing(PLATES)),
        arms="dual_arm",
    ),
    case(
        "set_up_chess_pieces_on_board",
        "chess",
        "Six chess pieces stand on the table beside the board. Put each back on its empty "
        "home square of the standard starting position, standing up.",
        CHESS_BODIES,
        set_up("chessboard", CHESS_GOALS, CHESS_PIECES),
        arms="dual_arm",
    ),
    case(
        "build_wood_block_tower",
        "building_blocks",
        "Build a tower of at least three wooden blocks stacked on top of each other.",
        WOOD_BLOCKS,
        tower(WOOD_BLOCKS, table_z=TABLE_Z),
        arms="single_arm",
        tags=("ours_grader",),
    ),
    *(_spell_case(word) for word in SPELL_WORDS),
]
