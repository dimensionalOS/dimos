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

"""Scan and Pick on the simulated rig, by the grasp stack's own modules.

dual-openyam-grasp-sim's modules run in this process, joined in memory. The
voxel mapper is a native module: it runs as its own process, and the topics it
is on go over the stack's network transport on the test session's private bus.
"""

from collections.abc import Iterator
from pathlib import Path
import time

import numpy as np
import pytest
from pytest_mock import MockerFixture
from reactivex.scheduler import ThreadPoolScheduler

from dimos.control.coordinator import ControlCoordinator
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import ExecutionStatus, PlanStatus
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.mapping.pointclouds.box_filter import PointCloudBoxFilter
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap, RayTracingVoxelMapConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.dual_openyam._sim_test_helpers import Deployed, deployed
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import (
    DUAL_OPENYAM_TABLE_TOP_Z,
    DUAL_OPENYAM_VOXEL_SIZE,
)
from dimos.robot.manipulators.dual_openyam.blueprints.simulation import dual_openyam_grasp_sim
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_HOME_PER_ARM,
    dual_openyam_arm_joints,
)
from dimos.robot.manipulators.dual_openyam.scene_registration import (
    SelfClearedManipulationLocalize,
)
from dimos.robot.manipulators.dual_openyam.sim_connection import DualOpenYamSimConnection
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing
from dimos.utils.testing.waiting import wait_until

pytestmark = pytest.mark.self_hosted

# What the scan is asked for, and the simulated body each name stands for.
BODIES = {"cup": "cup", "bottle": "bottle", "rubiks cube": "rubiks_cube"}
# Each object's height, metres.
HEIGHT = {"cup": 0.108, "bottle": 0.17, "rubiks_cube": 0.057}
# The rig, its coordinator, the planner and the scan.
SCAN = (
    DualOpenYamSimConnection,
    ControlCoordinator,
    ManipulationModule,
    SelfClearedManipulationLocalize,
)
# The planner's map: the self filter, the voxel mapper and the box filter.
MAP = (UrdfManagerPointClearing, RayTracingVoxelMap, PointCloudBoxFilter)
PICK = (HeuristicGraspModule, PickAndPlaceModule)
# The planner's web viewer would take the port a robot run on this machine uses.
NO_VIEWER = {ManipulationModule: {"visualization": {"backend": "none"}}}
CUBE = "rubiks_cube"
# How many frames the scan's memory holds of one unchanged scene before it is
# asked: its window of 10 s at one frame a second, and one to spare. The rig is
# scanned like this, minutes after start. A scan of fewer frames joins fewer
# sightings into an object's points, and the grasps come out differently.
WINDOW_FRAMES = 11


@pytest.fixture
def models(mujoco_gl: None, mocker: MockerFixture) -> Iterator[None]:
    """What the scan's models need: a GPU, and a thread pool that ends with the test."""
    torch = pytest.importorskip("torch")
    if not torch.cuda.is_available():
        pytest.skip("the scan's segmenter needs a CUDA device")
    # The scan's memory streams otherwise start the process-wide pool, which never stops.
    pool = ThreadPoolScheduler(max_workers=4)
    mocker.patch("dimos.utils.threadpool.get_scheduler", return_value=pool)
    yield
    pool.executor.shutdown(wait=True)


@pytest.fixture
def voxel_mapper() -> None:
    """Skip where the native voxel mapper is not built."""
    executable = Path(RayTracingVoxelMapConfig().resolve_paths()[1])
    if not executable.exists():
        pytest.skip(f"the voxel mapper is not built: {executable}")


def watched(stack: Deployed, since: int = 0) -> Deployed:
    """Wait until the scan's memory holds a whole window of frames newer than its first ``since``."""
    scan = stack.modules[SelfClearedManipulationLocalize]
    wait_until(lambda: scan.state().startswith("ready"), timeout=180.0, interval=0.5)
    wait_until(lambda: scan.index.count() >= since + WINDOW_FRAMES, timeout=60.0, interval=0.5)
    return stack


@pytest.fixture
def scanning(no_rpc: None, models: None) -> Iterator[Deployed]:
    """The simulated rig with the planner and the scan, joined in memory."""
    with deployed(dual_openyam_grasp_sim, *SCAN, settings=NO_VIEWER) as stack:
        yield watched(stack)


@pytest.fixture
def grasping(no_rpc: None, models: None, voxel_mapper: None) -> Iterator[Deployed]:
    """All of dual-openyam-grasp-sim that a scan and a pick go through, with the planner's map."""
    with deployed(dual_openyam_grasp_sim, *SCAN, *MAP, *PICK, settings=NO_VIEWER) as stack:
        # The map is in place once the planner has had it for a few clouds.
        wait_until(lambda: map_cells(stack) is not None, timeout=60.0)
        yield watched(stack)


def map_cells(stack: Deployed) -> np.ndarray | None:
    """The cells of the planner's newest map, metres in the world frame; None before the first."""
    try:
        return np.asarray(stack.topics["voxel_map"].latest.points_f32())
    except AssertionError:
        return None


def truth(stack: Deployed, body: str) -> np.ndarray:
    """Where the simulation itself has a body."""
    return np.array(stack.modules[DualOpenYamSimConnection].body_position(body))


def stand_cube_square(stack: Deployed) -> None:
    """Turn the cube square to the workcell where it stands, and let the scan watch it so."""
    rig = stack.modules[DualOpenYamSimConnection]
    rig.place_body(CUBE, rig.body_position(CUBE), 0.0)
    watched(stack, since=stack.modules[SelfClearedManipulationLocalize].index.count())


def pick(stack: Deployed, name: str) -> tuple[str, float]:
    """Scan for the object and pick it with the arm on its side.

    Returns pick_object's answer and how far the simulated object rose, metres.
    """
    found = stack.modules[SelfClearedManipulationLocalize].scan(names=list(BODIES))
    [hit] = [hit for hit in found if hit["label"] == name]
    side = "left" if hit["position"][1] > 0 else "right"
    before = truth(stack, BODIES[name])
    picker = stack.modules[PickAndPlaceModule]
    # Now and then the planner's inverse kinematics finds no posture for the
    # pre-grasp. Such an attempt moves nothing, and is made again.
    for _ in range(3):
        result = picker.pick_object(hit["id"], planning_group=f"{side}_manipulator")
        if not result.metadata.get("no_path"):
            break
    return result.message, float(truth(stack, BODIES[name])[2] - before[2])


def plan_home(stack: Deployed, side: str) -> PlanStatus:
    planner = stack.modules[ManipulationModule]
    names = [f"arm/{name}" for name in dual_openyam_arm_joints(side)]
    goal = JointState(name=names, position=list(DUAL_OPENYAM_HOME_PER_ARM))
    return planner.plan_to_joints({f"{side}_manipulator": goal}).status


def test_scan_finds_the_cup_the_bottle_and_the_cube_where_they_stand(scanning: Deployed) -> None:
    scan = scanning.modules[SelfClearedManipulationLocalize]

    found = scan.scan(names=list(BODIES))

    assert sorted(hit["label"] for hit in found) == sorted(BODIES)
    for hit in found:
        body = BODIES[hit["label"]]
        seen, real = np.asarray(hit["position"]), truth(scanning, body)
        # One view from the camera's side of the table: the position is that of
        # the surface it sees, a little short of the object's own axis.
        assert np.linalg.norm(seen[:2] - real[:2]) < 0.03
        assert DUAL_OPENYAM_TABLE_TOP_Z < seen[2] < DUAL_OPENYAM_TABLE_TOP_Z + HEIGHT[body]
        assert max(hit["size"]) < HEIGHT[body] + 0.02
        assert hit["views"] == 1
    # Pick and place reads an object's points by the id the scan gave it.
    cube = next(hit for hit in found if hit["label"] == "rubiks cube")
    assert len(scan.get_pointcloud(cube["id"])) > 30


def test_planner_maps_the_table_from_the_simulated_cloud_and_plans_from_home(
    grasping: Deployed,
) -> None:
    newest = grasping.topics["voxel_map"].latest
    cells = np.asarray(newest.points_f32())
    assert newest.frame_id == "world"

    # The table is one flat layer of cells at its height.
    on_table = np.abs(cells[:, 2] - DUAL_OPENYAM_TABLE_TOP_Z) < DUAL_OPENYAM_VOXEL_SIZE / 2
    assert on_table.sum() > 500
    # Each object stands out of it where the simulation has it.
    raised = cells[cells[:, 2] > DUAL_OPENYAM_TABLE_TOP_Z + DUAL_OPENYAM_VOXEL_SIZE]
    for body in BODIES.values():
        near = np.linalg.norm(raised[:, :2] - truth(grasping, body)[:2], axis=1) < 0.06
        assert near.sum() >= 5
    # No cell is an arm: at home both stand above the table, and nothing in
    # the map is as high as their hands.
    assert cells[:, 2].max() < 0.25
    # With the map as its obstacle the planner still finds its way from home.
    planner = grasping.modules[ManipulationModule]
    for side, sign in (("left", 1.0), ("right", -1.0)):
        names = [f"arm/{name}" for name in dual_openyam_arm_joints(side)]
        reach = [0.4 * sign, 0.9, 1.3, 0.2 * sign, -0.2 * sign, 0.4 * sign]
        goal = JointState(name=names, position=reach)
        assert planner.plan_to_joints({f"{side}_manipulator": goal}).status is PlanStatus.SUCCEEDED
        assert planner.clear_planned_path().succeeded


def test_pick_lifts_the_cube_standing_square_to_the_arm_and_carries_it_home(
    grasping: Deployed,
) -> None:
    stand_cube_square(grasping)
    start = truth(grasping, CUBE)

    message, rose = pick(grasping, "rubiks cube")

    assert message == "Pick complete"
    # The pick ends with the object held at the pre-grasp height, 5 cm up.
    assert rose == pytest.approx(0.05, abs=0.01)
    planner = grasping.modules[ManipulationModule]
    held = planner.get_state().groups["right_manipulator"].gripper_position
    # The jaws stopped on the cube: 57 mm of their 98.
    assert held == pytest.approx(0.58, abs=0.05)
    # The held cube and the arm are no obstacle to the next plan, and the cube goes with it.
    assert plan_home(grasping, "right") is PlanStatus.SUCCEEDED
    assert planner.execute(blocking=True).status is ExecutionStatus.COMPLETED
    assert truth(grasping, CUBE)[2] - start[2] > 0.25


@pytest.mark.xfail(
    strict=True,
    reason=(
        "The heuristic grasp centres the jaws on the middle of the points one overhead view "
        "sees of the cube, 18 mm short of its centre, and closes them along that view's line "
        "of sight. Turned 30 degrees the cube reaches 57 mm past that point and the open jaws "
        "49 mm, so a finger comes down on the cube's far corner. The contact check lets the "
        "grasp through: the far corner is not among the cube's points. pick_object answers "
        "'Gripper position 1.00 had not settled after 5.0 s'."
    ),
)
def test_pick_lifts_the_cube_turned_as_it_stands_on_the_rig(grasping: Deployed) -> None:
    message, rose = pick(grasping, "rubiks cube")

    assert message == "Pick complete"
    assert rose == pytest.approx(0.05, abs=0.01)


@pytest.mark.xfail(
    strict=True,
    reason=(
        "A pick that does not hold leaves the hand at the object. pick_object then lets the "
        "object's cells back into the planner's map, the arm stands among them, and every "
        "plan is refused: 'Start configuration is in collision, cannot plan!'."
    ),
)
def test_arm_plans_home_after_a_pick_that_did_not_hold(grasping: Deployed) -> None:
    message, _ = pick(grasping, "rubiks cube")
    assert message != "Pick complete"
    # The map has the object's cells back a moment after pick_object returns.
    time.sleep(2.0)

    assert plan_home(grasping, "right") is PlanStatus.SUCCEEDED


@pytest.mark.xfail(
    strict=True,
    reason=(
        "The contact check drops the cup's only grasp: 'Generated 0 grasp candidates'. The "
        "heuristic closes the jaws across the camera's line of sight, on the two sides of the "
        "cup that one view sees edge on. None of the cup's points lies between the pads (the "
        "nearest is 23 mm off the closing line), so the check finds the fingers closing on "
        "nothing."
    ),
)
def test_pick_lifts_the_cup(grasping: Deployed) -> None:
    message, rose = pick(grasping, "cup")

    assert message == "Pick complete"
    assert rose == pytest.approx(0.05, abs=0.01)
