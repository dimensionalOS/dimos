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
from dimos.robot.galaxea.r1pro.posture_ik import R1ProPostureIK, posture_is_valid

POSITION_TOLERANCE = 0.0003
ORIENTATION_TOLERANCE = 0.003
JOINT_LIMIT_MARGIN = 0.01


def measured_joint_seed(
    positions: NDArray[np.float64], lower: NDArray[np.float64], upper: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Bound a physically valid measured state inside the conservative IK limits.

    Loaded joints can settle a little beyond the planning margin without
    crossing the mechanical stop. This projection changes only the IK seed,
    never the simulator, gripper command or physical joint limits.
    """
    if (
        not np.isfinite(positions).all()
        or np.any(positions < lower - JOINT_LIMIT_MARGIN - 1e-6)
        or np.any(positions > upper + JOINT_LIMIT_MARGIN + 1e-6)
    ):
        raise RuntimeError("Measured joint state is nonfinite or exceeds physical joint limits")
    return np.clip(positions, lower + 1e-6, upper - 1e-6)


def bounded_joint_positions(
    positions: NDArray[np.float64], lower: NDArray[np.float64], upper: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Remove sub-microradian IK rounding at a stop; reject actual limit violations."""
    if (
        not np.all(np.isfinite(positions))
        or np.any(positions < lower - 1e-6)
        or np.any(positions > upper + 1e-6)
    ):
        raise RuntimeError("SDK pose solution exceeds conservative joint limits")
    return np.clip(positions, lower, upper)


class HomeKinematics:
    """Share the SDK's URDF, planning groups and Pink solver with the MuJoCo task.

    The SDK supplies pose solutions. The caller validates complete sweeps against
    the actual MuJoCo house and objects before ControlCoordinator execution.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        lock_lower_torso: bool = False,
        natural_posture: bool = False,
    ) -> None:
        self.config = make_r1pro_model_config()
        for side in ("left", "right"):
            self.config.model = self.config.model.with_fixed_frame(
                f"{side}_tcp", f"{side}_gripper_link", xyz=(0.0, 0.0, -0.085)
            )
        self.lower = np.array(
            [model.joint(n).range[0] + JOINT_LIMIT_MARGIN for n in self.config.joint_names]
        )
        self.upper = np.array(
            [model.joint(n).range[1] - JOINT_LIMIT_MARGIN for n in self.config.joint_names]
        )
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
        # Targets are re-expressed relative to this build pose, so one world serves any stance.
        self.reference_position = base.xpos.copy()
        self.reference_rotation = base.xmat.reshape(3, 3).copy()
        yaw = float(np.arctan2(base.xmat[3], base.xmat[0]))
        self.orientation = Quaternion.from_euler(Vector3(0.0, 0.0, yaw))
        self.config.base_pose = PoseStamped(
            position=base.xpos.copy(), orientation=self.orientation, frame_id="world"
        )
        self.world = create_world("roboplan")
        self.world.load_model(prepare_robot_model(self.config))
        self.world.finalize()
        self.natural_posture = natural_posture
        solver_type = R1ProPostureIK if natural_posture else PinkIK
        self.solver = solver_type(
            PinkKinematicsConfig(
                max_iterations=500,
                position_cost=40.0 if natural_posture else 1.0,
                orientation_cost=10.0 if natural_posture else 1.0,
                posture_cost=0.2 if natural_posture else 1e-5,
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
            position=measured_joint_seed(
                np.array([float(data.joint(n).qpos[0]) for n in self.config.joint_names]),
                self.lower,
                self.upper,
            ).tolist(),
        )

    def solve(
        self,
        data: mujoco.MjData,
        targets: dict[str, NDArray[Any]],
        *,
        orientations: dict[str, Quaternion] | None = None,
        allow_torso: bool = True,
        torso_yaw_only: bool = False,
        position_tolerance: float = POSITION_TOLERANCE,
        orientation_tolerance: float = ORIENTATION_TOLERANCE,
        max_attempts: int = 8,
        seed: JointState | None = None,
    ) -> NDArray[np.float64]:
        torso = self.groups["torso"]
        if torso_yaw_only:
            torso = replace(torso, joint_names=torso.joint_names[-1:])
        measured_seed = self.seed(data)
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
            auxiliary_groups=[torso] if allow_torso else [],
            seed=measured_seed if seed is None else seed,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            check_collision=False,
            max_attempts=max_attempts,
        )
        if not result.is_success() or result.joint_state is None:
            raise RuntimeError(
                f"SDK pose planning failed: {result.message}; "
                f"position error={result.position_error:.6f} m, orientation error={result.orientation_error:.6f} rad"
            )
        # The SDK seed already accounts for small measured deviations beyond
        # its conservative limits. Retain those bounded values for joints not
        # solved by this request instead of reintroducing the raw measurements.
        positions = dict(zip(measured_seed.name, measured_seed.position, strict=True))
        columns = [self.config.joint_names.index(name) for name in result.joint_state.name]
        bounded = bounded_joint_positions(
            np.asarray(result.joint_state.position, dtype=np.float64),
            self.lower[columns],
            self.upper[columns],
        )
        positions.update(zip(result.joint_state.name, bounded, strict=True))
        solved = np.array(
            [
                positions.get(name, float(data.joint(name).qpos[0]))
                for name in R1PRO_PICK_PLACE_JOINTS
            ]
        )
        if self.natural_posture and not posture_is_valid(solved):
            raise RuntimeError("SDK pose solution violates the R1Pro posture envelope")
        return solved
