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

"""One skills-enabled task per exported robosuite scene; filter with --tags <scene>."""

import math

import numpy as np

from dimos.evals.environments.lib.recorded_poses import last_body_transform
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.suites.mujoco_xarm import PERCEPTION_MODULES, lifted, stacked_on
from dimos.evals.types import EvalCase, Outcome, Suite, recording
from dimos.utils.data import LfsPath

GUIDANCE = (
    "Locate the objects with the wrist camera and use the robot's manipulation skills. "
    "The arm base is mounted at world z=0.912 m. Read the current robot pose; "
    "preserve its orientation for top-down moves. Keep the final result steady for "
    "at least two seconds before finishing."
)


def environment(scene: str, bodies: tuple[str, ...]) -> MujocoEnvironment:
    return MujocoEnvironment(
        blueprint=["xarm-perception-sim", "mcp-server", "observe-skill"],
        disable=PERCEPTION_MODULES,
        scene=LfsPath(f"robosuite/{scene}/scene.xml"),
        base_height=0.912,
        tracked_bodies=bodies,
    )


def opened_door(outcome: Outcome) -> float:
    with recording(outcome) as store:
        try:
            frame = last_body_transform(store, "Door_frame").rotation.to_rotation_matrix()
            panel = last_body_transform(store, "Door_door").rotation.to_rotation_matrix()
        except LookupError:
            return 0.0
    relative = frame.T @ panel
    return float(math.atan2(relative[1, 0], relative[0, 0]) >= 0.3)


def placed_can(outcome: Outcome) -> float:
    with recording(outcome) as store:
        try:
            can = last_body_transform(store, "Can_main")
            marker = last_body_transform(store, "VisualCan_main")
        except LookupError:
            return 0.0
    delta = (can.translation - marker.translation).to_numpy()
    return float(
        np.all(np.abs(delta) <= [0.05, 0.075, 0.005])
        and can.rotation.to_rotation_matrix()[2, 2] > 0.98
    )


def seated_nut(outcome: Outcome) -> float:
    with recording(outcome) as store:
        try:
            nut = last_body_transform(store, "SquareNut_main")
            peg = last_body_transform(store, "peg1")
        except LookupError:
            return 0.0
    delta = (nut.translation - peg.translation).to_numpy()
    # The seated nut's centre is 2 cm below the exported peg body's origin.
    return float(
        np.linalg.norm(delta[:2]) < 0.007
        and abs(delta[2] + 0.02) < 0.004
        and abs(nut.rotation.to_rotation_matrix()[2, 2]) > 0.98
    )


def hung_tool(outcome: Outcome) -> float:
    """Geometric assembly/hanging proxy from body poses; does not check contact or release."""
    with recording(outcome) as store:
        try:
            stand = last_body_transform(store, "stand_root")
            frame = last_body_transform(store, "frame_root")
            hole = last_body_transform(store, "tool_hole1_root")
        except LookupError:
            return 0.0
    stand_r = stand.rotation.to_rotation_matrix()
    frame_r = frame.rotation.to_rotation_matrix()
    # Local offsets from the exported frame tip, stand slot and horizontal hook.
    tip_world = frame.translation.to_numpy() + frame_r @ np.array([0.04375, 0, -0.13445])
    tip_in_stand = stand_r.T @ (tip_world - stand.translation.to_numpy())
    hole_in_frame = frame_r.T @ (hole.translation - frame.translation).to_numpy()
    return float(
        stand_r[2, 2] > 0.98
        and np.dot(stand_r[:, 2], frame_r[:, 2]) > 0.98
        and np.linalg.norm(tip_in_stand - np.array([0, 0.045, -0.07])) < 0.01
        and -0.043 < hole_in_frame[0] < 0.04375
        and math.hypot(hole_in_frame[1], hole_in_frame[2] - 0.08625) < 0.007
        and abs(np.dot(hole.rotation.to_rotation_matrix()[:, 2], frame_r[:, 0])) > 0.95
    )


SUITE: Suite = [
    EvalCase(
        id="robosuite_lift_cube",
        inputs=(
            "Pick up the red cube and hold it at least 5 cm above its resting position. "
            "The table top is at world z=0.80 m; the cube is 4 cm wide. " + GUIDANCE
        ),
        environment=environment("lift", ("cube_main",)),
        grade=lifted("cube_main", by_m=0.05),
        timeout_s=600.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "lift"}),
    ),
    EvalCase(
        id="robosuite_open_door",
        inputs=(
            "Open the door by at least 0.3 radians (about 17 degrees), then release it "
            "and leave it open. The door has a movable handle. " + GUIDANCE
        ),
        environment=environment("door", ("Door_frame", "Door_door")),
        grade=opened_door,
        timeout_s=900.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "door"}),
    ),
    EvalCase(
        id="robosuite_place_can",
        inputs=(
            "Move the soda can from the source bin to the destination compartment marked "
            "by the transparent can. Place it upright near the marker's center, release it, "
            "and move the gripper away. The bin floors are at world z=0.82 m. " + GUIDANCE
        ),
        environment=environment("pick_place", ("Can_main", "VisualCan_main")),
        grade=placed_can,
        timeout_s=900.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "pick_place"}),
    ),
    EvalCase(
        id="robosuite_stack_cubes",
        inputs=(
            "Stack the smaller red cube centrally on top of the larger green cube. "
            "Leave both cubes upright on the table, release the red cube, and move the "
            "gripper away. The table top is at world z=0.80 m. " + GUIDANCE
        ),
        environment=environment("stack", ("cubeA_main", "cubeB_main")),
        grade=stacked_on("cubeA_main", "cubeB_main", rise_m=(0.041, 0.049), band_m=0.03),
        threshold=0.5,
        timeout_s=900.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "stack"}),
    ),
    EvalCase(
        id="robosuite_hang_tool",
        inputs=(
            "Insert the hook frame into the upright stand, then hang the wrench by its "
            "larger hole on the horizontal hook. Release all pieces and move the gripper "
            "away so the wrench hangs unsupported by the robot. The table top is at "
            "world z=0.80 m. " + GUIDANCE
        ),
        environment=environment(
            "tool_hang",
            ("stand_root", "frame_root", "tool_hole1_root"),
        ),
        grade=hung_tool,
        timeout_s=1200.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "tool_hang"}),
    ),
    EvalCase(
        id="robosuite_assemble_square_nut",
        inputs=(
            "Pick up the square nut and lower its hole over the matching square peg until "
            "the nut rests flat on the table. Release it and move the gripper away. "
            "The table top is at world z=0.82 m. " + GUIDANCE
        ),
        environment=environment("nut_assembly", ("SquareNut_main", "peg1")),
        grade=seated_nut,
        timeout_s=900.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "nut_assembly"}),
    ),
]
