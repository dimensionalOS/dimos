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

"""R1Pro pose solving through the standard manipulation planning backend."""

from dataclasses import replace
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.manipulation.planning.factory import create_world
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.kinematics.pink_ik import PinkIK
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.config import make_r1pro_model_config
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS

POSITION_TOLERANCE = 0.0003
ORIENTATION_TOLERANCE = 0.003


class HomeKinematics:
    """Share the SDK's URDF, planning groups and Pink solver with the MuJoCo task.

    The SDK supplies pose solutions. The caller validates complete sweeps against
    the actual MuJoCo house and objects before ControlCoordinator execution.
    """

    def __init__(
        self, model: mujoco.MjModel, data: mujoco.MjData, *, lock_lower_torso: bool = False
    ) -> None:
        self.config = make_r1pro_model_config()
        for side in ("left", "right"):
            self.config.model = self.config.model.with_fixed_frame(
                f"{side}_tcp", f"{side}_gripper_link", xyz=(0.0, 0.0, -0.085)
            )
        self.lower = np.array([model.joint(n).range[0] + 0.01 for n in self.config.joint_names])
        self.upper = np.array([model.joint(n).range[1] - 0.01 for n in self.config.joint_names])
        for name, low, high in zip(self.config.joint_names, self.lower, self.upper, strict=True):
            self.config.model = self.config.model.with_joint_position_limits(
                name.removeprefix("r1pro/"), lower=float(low), upper=float(high)
            )
        self.config.planning_groups = [
            replace(g, tip_link=g.name.replace("_arm", "_tcp"))
            if g.name.endswith("_arm")
            else (
                replace(g, joint_names=g.joint_names[1:])
                if g.name == "torso" and lock_lower_torso
                else g
            )
            for g in self.config.planning_groups
        ]
        base = data.body("base_link")
        yaw = float(np.arctan2(base.xmat[3], base.xmat[0]))
        self.orientation = Quaternion.from_euler(Vector3(0.0, 0.0, yaw))
        self.config.base_pose = PoseStamped(
            position=base.xpos.copy(), orientation=self.orientation, frame_id="world"
        )
        self.world = create_world("roboplan")
        self.world.load_model(prepare_robot_model(self.config))
        self.world.finalize()
        self.solver = PinkIK(
            PinkKinematicsConfig(
                max_iterations=500,
                posture_cost=1e-5,
                joint_limit_posture_margin=0.03,
                solver_kwargs={"eps_abs": 1e-9, "eps_rel": 1e-9},
            )
        )
        self.groups = {g.id: g for g in PlanningGroupRegistry(self.config.planning_groups).list()}
        self.qids = np.array([model.joint(n).qposadr[0] for n in R1PRO_PICK_PLACE_JOINTS])
        # Compare physical FK at the measured state; conservative planning
        # limits apply only to the subsequent IK seed, not this model check.
        seed = JointState(
            name=self.config.joint_names,
            position=[float(data.joint(n).qpos[0]) for n in self.config.joint_names],
        )
        with self.world.scratch_context() as ctx:
            self.world.set_joint_state(ctx, seed)
            for side in ("left", "right"):
                expected = data.site(f"{side}_tcp").xpos
                actual = self.world.get_link_pose(ctx, f"{side}_tcp")[:3, 3]
                if np.linalg.norm(actual - expected) > 1e-5:
                    raise RuntimeError("Manipulation SDK and simulator TCP models do not agree")

    def seed(self, data: mujoco.MjData) -> JointState:
        return JointState(
            name=self.config.joint_names,
            position=np.clip(
                [float(data.joint(n).qpos[0]) for n in self.config.joint_names],
                self.lower + 1e-6,
                self.upper - 1e-6,
            ).tolist(),
        )

    def solve(
        self,
        data: mujoco.MjData,
        targets: dict[str, NDArray[Any]],
        *,
        orientations: dict[str, Quaternion] | None = None,
        position_tolerance: float = POSITION_TOLERANCE,
        orientation_tolerance: float = ORIENTATION_TOLERANCE,
    ) -> NDArray[np.float64]:
        result = self.solver.solve_pose_targets(
            self.world,
            {
                self.groups[f"{side}_arm"]: PoseStamped(
                    position=xyz,
                    orientation=(orientations or {}).get(side, self.orientation),
                    frame_id="world",
                )
                for side, xyz in targets.items()
            },
            auxiliary_groups=[self.groups["torso"]],
            seed=self.seed(data),
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            check_collision=False,
            max_attempts=8,
        )
        if not result.is_success() or result.joint_state is None:
            raise RuntimeError(
                f"SDK pose planning failed: {result.message}; "
                f"position error={result.position_error:.6f} m, orientation error={result.orientation_error:.6f} rad"
            )
        positions = dict(zip(result.joint_state.name, result.joint_state.position, strict=True))
        return np.array(
            [
                positions.get(name, float(data.joint(name).qpos[0]))
                for name in R1PRO_PICK_PLACE_JOINTS
            ]
        )
