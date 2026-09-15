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

"""Graph-level checks for ``alfred-nav`` and ``alfred-sim``: composition only, no hardware."""

from __future__ import annotations

from pathlib import Path
import re
from typing import Any, cast

import pytest

from dimos.control.components import HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.robot.diy.alfred.alfred_model import (
    ALFRED_LIFT_LOWER_M,
    ALFRED_LIFT_UPPER_M,
    ALFRED_PLANAR_BASE,
    alfred_arm_joints,
    alfred_joint_names,
    alfred_model_config,
    alfred_rerun_urdf,
)
from dimos.robot.diy.alfred.blueprints.alfred_nav import (
    ALFRED_BASE_HARDWARE_ID,
    PLANNER_CLEARANCE_HEIGHT_M,
    WALL_CLEARANCE_M,
    alfred_nav,
)
from dimos.robot.diy.alfred.blueprints.alfred_sim import alfred_sim
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.robot.diy.alfred.mount_tf import AlfredMountTf, mount_transforms
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_HARDWARE_ID,
    PILLAR_LIFT_JOINT,
    PillarConnection,
)
from dimos.robot.manipulators.openarm.config import OPENARM_HARDWARE_ID
from dimos.utils.data import get_project_root
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer


def _atoms(blueprint: Blueprint, module: type) -> list[Any]:
    return [
        atom
        for atom in blueprint.blueprints
        if isinstance(atom.module, type) and issubclass(atom.module, module)
    ]


def _coordinator_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    (atom,) = _atoms(blueprint, ControlCoordinator)
    return cast("dict[str, Any]", atom.kwargs)


def _lfs_archive_available() -> bool:
    archive = get_project_root() / "data" / ".lfs" / "alfred_description.tar.gz"
    try:
        with archive.open("rb") as f:
            return not f.read(64).startswith(b"version https://git-lfs")
    except OSError:
        return False


def test_alfred_nav_leaves_the_flowbase_to_alfred_high_level() -> None:
    """The coordinator plans for the base but must never be a second Portal writer.

    Locomanipulation needs the base in the coordinator so a whole-body plan can move it,
    but AlfredHighLevel still owns the hardware: the coordinator's base is a transport
    adapter that only publishes a twist for MovementManager to mux.
    """
    assert _atoms(alfred_nav, AlfredHighLevel), "navigation base owner missing"
    hardware = {hw.hardware_id: hw for hw in _coordinator_kwargs(alfred_nav)["hardware"]}
    assert set(hardware) == {PILLAR_HARDWARE_ID, OPENARM_HARDWARE_ID, ALFRED_BASE_HARDWARE_ID}

    base = hardware[ALFRED_BASE_HARDWARE_ID]
    assert base.hardware_type is HardwareType.BASE
    # "flowbase" would open a second Portal client against the same base driver.
    assert base.adapter_type == "transport_lcm"


def test_alfred_nav_leaves_the_base_command_on_the_coordinator_topic() -> None:
    """One arbitrated base command leaves the coordinator; AlfredHighLevel drives it."""
    topics = {name: spec.args[0] for (name, _type), spec in alfred_nav.transport_map.items()}
    assert topics["cmd_vel"] == f"/{ALFRED_BASE_HARDWARE_ID}/cmd_vel"
    # Odometry feedback for the base tasks is StartRelay's pose.
    assert topics["start_pose"] == f"/{ALFRED_BASE_HARDWARE_ID}/odom"


def test_alfred_nav_splits_a_whole_body_plan_between_the_two_trajectory_tasks() -> None:
    (atom,) = _atoms(alfred_nav, ManipulationModule)
    split = atom.kwargs["trajectory_tasks"]
    assert set(split["base_trajectory"]) == set(ALFRED_PLANAR_BASE.joint_names)
    assert PILLAR_LIFT_JOINT in split["joint_trajectory"]
    assert not set(split["joint_trajectory"]) & set(ALFRED_PLANAR_BASE.joint_names)

    # The coordinator names the base joints differently from the planner; the alias keeps
    # a plan's base coordinates and the hardware's twist joints the same three numbers.
    aliases = atom.kwargs["joint_state_aliases"]
    assert set(aliases.values()) == set(ALFRED_PLANAR_BASE.joint_names)


def test_alfred_nav_tasks_cover_lift_and_both_arms() -> None:
    tasks = cast("list[TaskConfig]", _coordinator_kwargs(alfred_nav)["tasks"])
    (task,) = [t for t in tasks if t.type == "trajectory"]
    assert set(task.joint_names) == set(alfred_joint_names())
    limits = task.params["velocity_limits"]
    assert set(limits) == set(task.joint_names)
    assert limits[PILLAR_LIFT_JOINT] == 0.1


def test_alfred_nav_holds_the_arms_under_the_trajectory_task() -> None:
    """A driving base must not swing the arms, but a plan still outranks the hold."""
    tasks = cast("list[TaskConfig]", _coordinator_kwargs(alfred_nav)["tasks"])
    (hold,) = [t for t in tasks if t.type == "joint_hold"]
    (trajectory,) = [t for t in tasks if t.type == "trajectory"]

    # Arms only: the lift is a braked leadscrew and does not swing.
    assert set(hold.joint_names) == set(alfred_arm_joints())
    assert PILLAR_LIFT_JOINT not in hold.joint_names
    assert hold.priority < trajectory.priority


def test_alfred_nav_routes_teleop_into_the_coordinator() -> None:
    """Teleop is a joint claim like any other, not a mux input upstream of the base."""
    assert alfred_nav.remapping_map[("ControlCoordinator", "twist_command")] == "tele_cmd_vel"
    # A viewer click is the navigation goal; nothing relays it any more.
    assert alfred_nav.remapping_map[("rerunwebsocketserver", "clicked_point")] == "goal"


def test_alfred_nav_arbitrates_the_base_on_one_coordinator() -> None:
    """Teleop > a plan's base segment > navigation, by priority on shared joints."""
    tasks = {t.name: t for t in cast("list[TaskConfig]", _coordinator_kwargs(alfred_nav)["tasks"])}
    teleop = tasks["vel_flowbase"]
    base_plan = tasks["base_trajectory"]
    follower = tasks["holonomic_follower"]

    base_joints = {frozenset(t.joint_names) for t in (teleop, base_plan, follower)}
    assert len(base_joints) == 1, "the three base claims must contend for the same joints"
    assert teleop.priority > base_plan.priority > follower.priority

    # The follower replaces DanHolonomicTC; both in one coordinator would fight.
    assert not _atoms(alfred_nav, DanHolonomicTC)


def test_alfred_nav_runs_on_lidar_odometry() -> None:
    """Point-LIO owns odom -> mid360_link; the mount tree must hang off the lidar."""
    assert _atoms(alfred_nav, PointLio)
    assert _atoms(alfred_nav, AlfredMountTf)
    assert not any(atom.module.__name__ == "DimSlam" for atom in alfred_nav.blueprints)
    (pointlio,) = _atoms(alfred_nav, PointLio)
    assert pointlio.kwargs["frame_id"] == "odom"
    assert pointlio.kwargs["sensor_frame_id"] == "mid360_link"


def test_alfred_mount_tree_reroots_onto_the_lidar_without_losing_a_frame() -> None:
    """Point-LIO owns odom -> mid360_link, so the lidar must be the tree's only root."""
    (atom,) = _atoms(alfred_nav, AlfredMountTf)
    assert atom.kwargs["root_frame"] == "mid360_link"

    transforms = mount_transforms("mid360_link")
    edges = {t.child_frame_id: t.frame_id for t in transforms}
    assert len(edges) == len(transforms), "a frame has two parents"
    assert edges["base_link"] == "mid360_link"
    assert "mid360_link" not in edges, "Point-LIO must be the lidar frame's only parent"
    # Re-rooting flips one edge and keeps every other mount, cameras included.
    rooted_at_base = {t.child_frame_id for t in mount_transforms()}
    assert set(edges) == (rooted_at_base | {"base_link"}) - {"mid360_link"}
    for link in ("camera_link", "d455_link", "mid360_imu_link", "mast_link"):
        frame = link
        while frame in edges:
            frame = edges[frame]
        assert frame == "mid360_link", f"{link} does not reach the odometry root"


def test_alfred_nav_composes_nav_planner_pillar_and_viewer_teleop() -> None:
    assert _atoms(alfred_nav, PillarConnection)
    assert _atoms(alfred_nav, ManipulationModule)
    assert _atoms(alfred_nav, RerunWebSocketServer), "viewer teleop source missing"


def test_alfred_nav_planner_publishes_no_world_rooted_tf() -> None:
    (atom,) = _atoms(alfred_nav, ManipulationModule)
    assert atom.kwargs["model"].tf_extra_links == []


def test_alfred_model_uses_pillar_joint_convention() -> None:
    """Lift is negative below the top switch, exactly as pillar_connection reports it."""
    config = alfred_model_config()
    assert config.joint_names[0] == PILLAR_LIFT_JOINT
    assert ALFRED_LIFT_LOWER_M == -0.5
    assert ALFRED_LIFT_UPPER_M == -0.002
    groups = {group.name: group for group in config.planning_groups}
    assert groups["lift"].joint_names == (PILLAR_LIFT_JOINT,)
    assert set(groups) == {"lift", "left_manipulator", "right_manipulator"}


@pytest.mark.skipif(
    not _lfs_archive_available(), reason="alfred_description LFS archive not pulled"
)
def test_home_pose_is_inside_every_joint_limit() -> None:
    """The lift's zero is the top switch, above its reachable range; Home must not send it."""
    for wheels in (False, True):
        config = alfred_model_config(wheels=wheels)
        limits = {j.name: j for j in config.model.load().joints}
        assert config.home_joints is not None
        for name, value in zip(config.joint_names, config.home_joints, strict=True):
            joint = limits[name]
            if joint.lower is not None and joint.upper is not None:
                assert joint.lower <= value <= joint.upper, f"{name}: home {value} outside limits"


def test_alfred_sim_still_composes() -> None:
    hardware_ids = {hw.hardware_id for hw in _coordinator_kwargs(alfred_sim)["hardware"]}
    assert {PILLAR_HARDWARE_ID, OPENARM_HARDWARE_ID} <= hardware_ids


@pytest.mark.skipif(
    not _lfs_archive_available(), reason="alfred_description LFS archive not pulled"
)
def test_rerun_urdf_is_materialized_with_resolved_meshes_and_coordinator_joints() -> None:
    urdf = alfred_rerun_urdf()
    xml = urdf.read_text()
    assert "package://" not in xml
    assert 'name="pillar/lift"' in xml, "renamed lift joint must reach the rerun model"
    meshes = re.findall(r'filename="([^"]+)"', xml)
    assert meshes and all(Path(m).is_file() for m in meshes), "unresolved mesh path"


def test_the_follower_falls_back_to_the_go2_artifact_until_alfred_has_one(
    tmp_path, monkeypatch
) -> None:
    """A robot that has never been characterized must still start.

    The follower raises on a missing artifact, so alfred-nav cannot simply point at
    Alfred's and assume it is there - it resolves, and says so loudly.
    """
    from dimos.robot.diy.alfred import alfred_model

    missing = tmp_path / "alfred_posedomain.json"
    monkeypatch.setattr(alfred_model, "ALFRED_FOLLOWER_ARTIFACT", str(missing))
    assert alfred_model.alfred_follower_artifact() == alfred_model._GO2_FOLLOWER_ARTIFACT

    missing.write_text("{}")
    assert alfred_model.alfred_follower_artifact() == str(missing)


def test_alfreds_tuned_artifact_is_checked_in_where_the_follower_reads_it() -> None:
    """The artifact is config now, not a build output - losing it degrades silently.

    Nothing in this branch regenerates it, and a missing file does not fail: the
    follower quietly falls back to a quadruped's gains and overshoots on hardware.
    """
    import json

    from dimos.robot.diy.alfred.alfred_model import (
        ALFRED_FOLLOWER_ARTIFACT,
        alfred_follower_artifact,
    )

    assert Path(ALFRED_FOLLOWER_ARTIFACT).is_file(), "Alfred's tuned artifact is missing"
    assert json.loads(Path(ALFRED_FOLLOWER_ARTIFACT).read_text()), "artifact is empty"
    assert alfred_follower_artifact() == ALFRED_FOLLOWER_ARTIFACT


def test_alfreds_measured_size_constants_match_the_urdf() -> None:
    """The size constants must not fall below what the URDF says the robot is.

    These describe the robot. What the planner is *given* is a separate question
    - see test_the_planner_gets_the_real_height_and_a_footprint_under_the_robot -
    because the MLS planner models a cylinder and Alfred is a short wide base
    under a thin mast.

    Re-derived from the collision scene rather than pinned to a literal, so a
    change to the URDF fails here instead of on a wall.
    """
    import numpy as np
    import yourdfpy

    from dimos.robot.diy.alfred.alfred_model import (
        ALFRED_FOOTPRINT_RADIUS_M,
        ALFRED_HEIGHT_M,
        ALFRED_V1_MODEL,
    )

    loaded = ALFRED_V1_MODEL.load()
    urdf = yourdfpy.URDF.load(
        loaded.source_path, build_collision_scene_graph=True, load_collision_meshes=True
    )
    bounds = urdf.collision_scene.bounds
    # Worst xy corner: the circumscribed radius, which is what a circular
    # footprint needs when the robot can sit at any yaw.
    measured_radius = max(float(np.hypot(x, y)) for x in bounds[:, 0] for y in bounds[:, 1])
    measured_height = float(bounds[1, 2])

    assert ALFRED_FOOTPRINT_RADIUS_M >= measured_radius - 1e-3, (
        f"clearance {ALFRED_FOOTPRINT_RADIUS_M} is under the measured "
        f"{measured_radius:.3f} m; the planner would route Alfred through gaps it cannot fit"
    )
    assert ALFRED_HEIGHT_M >= measured_height - 1e-2, (
        f"headroom {ALFRED_HEIGHT_M} is under the measured {measured_height:.3f} m"
    )


def test_the_planner_gets_the_real_height_and_a_footprint_under_the_robot() -> None:
    """The two planner sizes are set from different evidence. Pinned separately.

    robot_height is the robot's height, because it is not a headroom nicety:
    Config::headroom_cells turns it into clearance_cells and surfaces.rs
    is_standable() uses that to decide whether a cell is floor at all. A cell
    under something lower than Alfred is not somewhere Alfred can go, so the
    honest value is the measured height. Normal ceilings clear it comfortably -
    max_overhead_m caps the map at sensor_z + 2 m and 24 cells is 1.92 m - and
    tables, shelves and sub-2 m doorways do not, which is correct.

    wall_clearance_m is a hard footprint clearance and is knowingly set BELOW
    Alfred's inscribed radius, so the planner can route it through a gap it will
    not fit at some yaws. Measured on the robot: 0.2 plans, 0.28 cannot. The
    reason the step is so sharp is spawn_floor in nodes.rs, which gates where new
    graph nodes may appear at wall_clearance_m + 0.5 * wall_buffer_m - 0.575 m
    today, so 8 cm of clearance moves a threshold that is already large. Raising
    it wants wall_buffer_m lowered in the same change, and a mapped space to test
    in; it is a commissioning decision, not a config edit.
    """
    from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
    from dimos.robot.diy.alfred.alfred_model import (
        ALFRED_FOOTPRINT_RADIUS_M,
        ALFRED_HEIGHT_M,
    )

    (planner,) = _atoms(alfred_nav, MLSPlannerNative)
    assert planner.kwargs["wall_clearance_m"] == WALL_CLEARANCE_M
    assert planner.kwargs["robot_height"] == PLANNER_CLEARANCE_HEIGHT_M
    assert PLANNER_CLEARANCE_HEIGHT_M == ALFRED_HEIGHT_M
    assert WALL_CLEARANCE_M < ALFRED_FOOTPRINT_RADIUS_M


def test_the_planner_and_its_visualization_agree_on_clearance() -> None:
    """A drawn clearance that is not the enforced one is worse than none."""
    from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative

    (planner,) = _atoms(alfred_nav, MLSPlannerNative)
    assert planner.kwargs["wall_clearance_m"] == WALL_CLEARANCE_M
