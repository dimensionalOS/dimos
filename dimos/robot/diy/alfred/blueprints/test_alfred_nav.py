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

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.diy.alfred.alfred_model import (
    ALFRED_LIFT_LOWER_M,
    ALFRED_LIFT_UPPER_M,
    alfred_joint_names,
    alfred_model_config,
    alfred_rerun_urdf,
)
from dimos.robot.diy.alfred.blueprints.alfred_nav import alfred_nav
from dimos.robot.diy.alfred.blueprints.alfred_sim import alfred_sim
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.robot.diy.alfred.mount_tf import AlfredLidarMountTf, alfred_mount_transforms
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


def test_alfred_nav_keeps_the_base_out_of_the_coordinator() -> None:
    """AlfredHighLevel owns the FlowBase (Portal + wheel odom); the coordinator must not."""
    assert _atoms(alfred_nav, AlfredHighLevel), "navigation base owner missing"
    hardware_ids = {hw.hardware_id for hw in _coordinator_kwargs(alfred_nav)["hardware"]}
    assert hardware_ids == {PILLAR_HARDWARE_ID, OPENARM_HARDWARE_ID}


def test_alfred_nav_tasks_cover_lift_and_both_arms() -> None:
    (task,) = cast("list[TaskConfig]", _coordinator_kwargs(alfred_nav)["tasks"])
    assert task.type == "trajectory"
    assert set(task.joint_names) == set(alfred_joint_names())
    limits = task.params["velocity_limits"]
    assert set(limits) == set(task.joint_names)
    assert limits[PILLAR_LIFT_JOINT] == 0.1


def test_alfred_nav_runs_on_lidar_odometry() -> None:
    """Point-LIO owns odom -> mid360_link; the mount tree must hang off the lidar."""
    assert _atoms(alfred_nav, PointLio)
    assert _atoms(alfred_nav, AlfredLidarMountTf)
    assert not any(atom.module.__name__ == "DimSlam" for atom in alfred_nav.blueprints)
    (pointlio,) = _atoms(alfred_nav, PointLio)
    assert pointlio.kwargs["frame_id"] == "odom"
    assert pointlio.kwargs["sensor_frame_id"] == "mid360_link"


@pytest.mark.skipif(
    not _lfs_archive_available(), reason="alfred_description LFS archive not pulled"
)
def test_alfred_mount_tree_is_rooted_at_the_lidar_and_leaves_moving_parts_out() -> None:
    transforms = alfred_mount_transforms()
    edges = {t.child_frame_id: t.frame_id for t in transforms}
    assert len(edges) == len(transforms), "a frame has two parents"
    assert edges["base_link"] == "mid360_link"
    assert "mid360_link" not in edges, "Point-LIO must be the lidar frame's only parent"
    assert "lift_link" not in edges and not any("openarm" in c for c in edges)
    assert "camera_front_depth_optical_frame" not in edges, "imager frames belong to the driver"
    for link in ("camera_front_link", "camera_back_link", "mid360_imu_link"):
        frame = link
        while frame in edges:
            frame = edges[frame]
        assert frame == "mid360_link", f"{link} does not reach the odometry root"


def test_alfred_nav_composes_nav_planner_pillar_and_viewer_teleop() -> None:
    assert _atoms(alfred_nav, MovementManager)
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


def test_alfred_sim_still_composes() -> None:
    hardware_ids = {hw.hardware_id for hw in _coordinator_kwargs(alfred_sim)["hardware"]}
    assert {PILLAR_HARDWARE_ID, OPENARM_HARDWARE_ID, "casters"} <= hardware_ids


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
