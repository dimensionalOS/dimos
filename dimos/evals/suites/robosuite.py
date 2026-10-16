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

from dimos.evals.constants import XARM7_GRIPPER_NOTES
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import (
    GRIPPER_BODIES,
    can_partial,
    door_partial,
    hang_partial,
    lift_partial,
    nut_partial,
    stack_partial,
)
from dimos.evals.suites.mujoco_xarm import PERCEPTION_MODULES
from dimos.evals.types import EvalCase, Suite
from dimos.utils.data import LfsPath

GUIDANCE = (
    "The robot is an xArm7 arm with a two-finger parallel gripper, mounted at the edge of the "
    "table. Positions are in the world frame, in metres: the arm's base is at (0, 0, 0.912), "
    "+x points from the base out across the table, +y to the robot's left and +z up. "
    "One camera is connected: the wrist camera, mounted on the gripper and moving with it. "
    "observe returns its image, and localize finds named objects in its images and returns "
    "their world positions. Use the robot's manipulation skills to move: move_to_pose places, "
    "and get_robot_state reports, the gripper's tool centre point (TCP). "
    f"{XARM7_GRIPPER_NOTES} "
    "Do not read the simulator's scene, asset or recording files or the source of robosuite or "
    "MuJoCo; learn about the scene only through the robot's camera and tools."
)


# robosuite's own contact solver (robosuite/models/assets/base.xml), which the scene export dropped.
ROBOSUITE_PHYSICS = {
    "MUJOCOSIMMODULE__FRICTION_CONE": "elliptic",
    "MUJOCOSIMMODULE__IMPRATIO": "20",
}


def environment(scene: str, bodies: tuple[str, ...]) -> MujocoEnvironment:
    return MujocoEnvironment(
        blueprint=[
            "xarm-perception-sim",
            "mcp-server",
            "observe-skill",
            "perception-localize",
        ],
        disable=PERCEPTION_MODULES,
        scene=LfsPath(f"robosuite/{scene}/scene.xml"),
        base_height=0.912,
        tracked_bodies=(*bodies, *GRIPPER_BODIES),  # gripper poses feed the partial graders
        agent_artifacts=(),  # sensors and skills only; the recording holds ground-truth poses
        # The wrist camera starts parked, so confirm objects from one view; robosuite's plain,
        # flat-shaded objects score about 0.3 with OWLv2, under the 0.40 real-world default.
        module_env={
            "PERCEPTIONLOCALIZE__POLICY": (
                '{"min_views": 1, "candidate_floor": 0.2, "accept_score": 0.25}'
            ),
            **ROBOSUITE_PHYSICS,
        },
    )


SUITE: Suite = [
    EvalCase(
        id="robosuite_lift_cube",
        inputs=(
            "Pick up the red cube and hold it at least 5 cm above its resting position. "
            "The cube is 4 cm wide. " + GUIDANCE
        ),
        environment=environment("lift", ("cube_main",)),
        grade=lift_partial("cube_main", by_m=0.05, half_size_m=0.02),
        timeout_s=2400.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "lift"}),
    ),
    EvalCase(
        id="robosuite_open_door",
        inputs=(
            "Open the door by at least 0.3 radians (about 17 degrees), then release it "
            "and leave it open. The door has a movable handle. " + GUIDANCE
        ),
        environment=environment("door", ("Door_frame", "Door_door", "Door_latch")),
        grade=door_partial,
        timeout_s=3600.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "door"}),
    ),
    EvalCase(
        id="robosuite_place_can",
        inputs=(
            "Move the soda can from the source bin to the destination compartment marked "
            "by the transparent can. Place it upright near the marker's center, release it, "
            "and move the gripper away. " + GUIDANCE
        ),
        environment=environment("pick_place", ("Can_main", "VisualCan_main")),
        grade=can_partial,
        timeout_s=3600.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "pick_place"}),
    ),
    EvalCase(
        id="robosuite_stack_cubes",
        inputs=(
            "Stack the smaller red cube centrally on top of the larger green cube. "
            "Leave both cubes upright on the table, release the red cube, and move the "
            "gripper away. " + GUIDANCE
        ),
        environment=environment("stack", ("cubeA_main", "cubeB_main")),
        grade=stack_partial("cubeA_main", "cubeB_main", rise_m=(0.041, 0.049), band_m=0.03),
        threshold=0.6,  # 0.2 lifted + 0.8 x the old 0.5 placement bar
        timeout_s=3600.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "stack"}),
    ),
    EvalCase(
        id="robosuite_hang_tool",
        inputs=(
            "Insert the hook frame into the upright stand, then hang the wrench by its "
            "larger hole on the horizontal hook. Release all pieces and move the gripper "
            "away so the wrench hangs unsupported by the robot. " + GUIDANCE
        ),
        environment=environment(
            "tool_hang",
            ("stand_root", "frame_root", "tool_hole1_root"),
        ),
        grade=hang_partial,
        timeout_s=4800.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "tool_hang"}),
    ),
    EvalCase(
        id="robosuite_assemble_square_nut",
        inputs=(
            "Pick up the square nut and lower its hole over the matching square peg until "
            "the nut rests flat on the table. Release it and move the gripper away. " + GUIDANCE
        ),
        environment=environment("nut_assembly", ("SquareNut_main", "peg1")),
        grade=nut_partial,
        timeout_s=3600.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "nut_assembly"}),
    ),
]
