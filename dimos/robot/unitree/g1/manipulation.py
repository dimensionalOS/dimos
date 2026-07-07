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

"""G1 arm manipulation wiring: planning configs + coordinator trajectory tasks.

The planning side reuses the reachability registry's arm models (one G1 arm
rooted at the pelvis, base pinned at the WBC standing height), so the
ManipulationModule, the offline capability maps, and the viser overlay all
share one kinematic frame. The execution side adds one passive trajectory
task per arm to the whole-body ControlCoordinator: idle tasks emit nothing
(the servo hold keeps the arms), an executing task outranks the hold for
exactly its 7 joints, and legs/waist stay with the WBC policy throughout.
"""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import g1_arms
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.reachability.robots import robot_model_config

G1_ARM_SIDES: tuple[str, ...] = ("left", "right")

# servo_arms holds the arms at priority 10; an executing trajectory must win
# per-joint arbitration, then fall back to the hold when it completes.
G1_ARM_TRAJECTORY_PRIORITY = 20


def g1_arm_trajectory_task_name(side: str) -> str:
    return f"traj_g1_{side}_arm"


def _g1_arm_coordinator_joints(side: str) -> list[str]:
    joints = [j for j in g1_arms if j.startswith(f"g1/{side}_")]
    if len(joints) != 7:
        raise ValueError(f"expected 7 {side} arm joints, got {joints}")
    return joints


def g1_arm_model_config(side: str) -> RobotModelConfig:
    """Planning model for one G1 arm, wired to the whole-body coordinator.

    Coordinator joints are ``g1/<stem>`` while the MJCF names them
    ``<stem>_joint``; the mapping lets the module consume the coordinator's
    29-joint state and emit trajectories the coordinator understands.
    """
    base = robot_model_config(f"g1-{side}")
    mapping = {f"g1/{name.removesuffix('_joint')}": name for name in base.joint_names}
    return base.model_copy(
        update={
            "joint_name_mapping": mapping,
            "coordinator_task_name": g1_arm_trajectory_task_name(side),
            # The GR00T arm hold pose (all zeros) doubles as the home pose.
            "home_joints": [0.0] * len(base.joint_names),
        }
    )


def g1_arm_trajectory_task(side: str, priority: int = G1_ARM_TRAJECTORY_PRIORITY) -> TaskConfig:
    """Passive per-arm trajectory task for the whole-body coordinator."""
    return TaskConfig(
        name=g1_arm_trajectory_task_name(side),
        type="trajectory",
        joint_names=_g1_arm_coordinator_joints(side),
        priority=priority,
        # Without this the servo hold reclaims the arm the tick after a
        # trajectory finishes and snaps it back to the default pose.
        # reset() (or a new trajectory) releases the hold.
        params={"hold_on_complete": True},
    )
