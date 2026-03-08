# Copyright 2025-2026 Dimensional Inc.
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

"""KinematicsSpec-compatible wrapper around PinocchioIK.

Uses Pinocchio for FK/Jacobian (no Drake lock contention) — dramatically
faster than JacobianIK which re-acquires the Drake scratch_context lock
for every iteration.

Collision checking still uses the DrakeWorld WorldSpec (the only
collision-capable backend available).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.manipulation.planning.kinematics.pinocchio_ik import PinocchioIK, PinocchioIKConfig
from dimos.manipulation.planning.spec import IKResult, IKStatus, WorldRobotID, WorldSpec
from dimos.msgs.sensor_msgs import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs import PoseStamped

logger = setup_logger()


def _create_failure(status: IKStatus, message: str) -> IKResult:
    return IKResult(
        status=status, joint_state=None, position_error=float("inf"), iterations=0, message=message
    )


class PinocchioKinematicsSolver:
    """KinematicsSpec wrapper around PinocchioIK.

    Implements the same interface as JacobianIK (random restarts, collision
    checking via WorldSpec) but uses Pinocchio for FK/Jacobian instead of Drake.
    This avoids Drake lock contention and is 10-50x faster per iteration.
    """

    def __init__(
        self,
        max_iter: int = 200,
        max_attempts: int = 10,
        eps: float = 1e-4,
        damp: float = 1e-2,
    ) -> None:
        self._max_iter = max_iter
        self._max_attempts = max_attempts
        self._eps = eps
        self._damp = damp
        # Cache: robot_id → (ik, frame_id, q_neutral_full, q_indices, lower_arm, upper_arm)
        self._cache: dict[
            WorldRobotID, tuple[PinocchioIK, int, NDArray, list[int], NDArray, NDArray]
        ] = {}

    def _get_or_build(
        self, world: WorldSpec, robot_id: WorldRobotID
    ) -> tuple[PinocchioIK, int, NDArray, NDArray] | None:
        """Lazily build and cache the Pinocchio model for a robot."""
        if robot_id in self._cache:
            return self._cache[robot_id]

        import pinocchio  # type: ignore[import-untyped]

        config = world.get_robot_config(robot_id)

        # Resolve URDF (process xacro if needed)
        try:
            from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake

            urdf_path = prepare_urdf_for_drake(
                urdf_path=config.urdf_path.resolve(),
                package_paths=config.package_paths,
                xacro_args=config.xacro_args,
                convert_meshes=False,  # not needed for Pinocchio
            )
        except Exception as e:
            logger.warning(f"PinocchioIK: failed to resolve URDF: {e}")
            return None

        try:
            model = pinocchio.buildModelFromUrdf(str(urdf_path))
        except Exception as e:
            logger.warning(f"PinocchioIK: failed to load model: {e}")
            return None

        data = model.createData()

        # Find EE frame ID
        ee_link = config.end_effector_link
        frame_id = model.getFrameId(ee_link)
        if frame_id >= model.nframes:
            logger.warning(
                f"PinocchioIK: EE frame '{ee_link}' not found (id={frame_id} >= nframes={model.nframes})"
            )
            return None

        # Build index mapping: arm joint names → q-vector indices in the full model
        # This lets us pass/extract only the 7 arm joints while keeping gripper at neutral.
        arm_joint_names = config.joint_names
        q_indices: list[int] = []
        for jname in arm_joint_names:
            jid = model.getJointId(jname)
            if jid >= model.njoints:
                logger.warning(f"PinocchioIK: joint '{jname}' not found in model")
                return None
            idx_q = model.joints[jid].idx_q
            nq_j = model.joints[jid].nq
            q_indices.extend(range(idx_q, idx_q + nq_j))

        q_neutral = pinocchio.neutral(model)
        lower_arm = q_neutral[q_indices].copy()
        upper_arm = q_neutral[q_indices].copy()
        for i, jname in enumerate(arm_joint_names):
            jid = model.getJointId(jname)
            idx_q = model.joints[jid].idx_q
            nq_j = model.joints[jid].nq
            lower_arm[i : i + nq_j] = model.lowerPositionLimit[idx_q : idx_q + nq_j]
            upper_arm[i : i + nq_j] = model.upperPositionLimit[idx_q : idx_q + nq_j]

        ik_config = PinocchioIKConfig(max_iter=self._max_iter, eps=self._eps, damp=self._damp)
        ik = PinocchioIK(
            model, data, ee_joint_id=model.frames[frame_id].parentJoint, config=ik_config
        )

        logger.info(
            f"PinocchioIK: built model for '{robot_id}' "
            f"(ee='{ee_link}', frame_id={frame_id}, full_dof={model.nq}, arm_dof={len(q_indices)})"
        )
        entry = (ik, frame_id, q_neutral, q_indices, lower_arm, upper_arm)
        self._cache[robot_id] = entry
        return entry

    def _frame_fk(self, ik: PinocchioIK, frame_id: int, q: NDArray) -> Any:
        """FK to a specific frame (handles fixed joints like link_tcp)."""
        import pinocchio  # type: ignore[import-untyped]

        pinocchio.forwardKinematics(ik.model, ik._data, q)
        pinocchio.updateFramePlacements(ik.model, ik._data)
        return ik._data.oMf[frame_id]

    def _solve_one(
        self,
        ik: PinocchioIK,
        frame_id: int,
        target_se3: Any,
        q_init: NDArray,
        q_indices: list[int],
        lower_arm: NDArray,
        upper_arm: NDArray,
    ) -> tuple[NDArray, bool, float]:
        """Single IK solve attempt using frame-level FK."""
        import pinocchio  # type: ignore[import-untyped]

        cfg = ik._config
        q = q_init.copy()

        for _ in range(cfg.max_iter):
            pinocchio.forwardKinematics(ik.model, ik._data, q)
            pinocchio.updateFramePlacements(ik.model, ik._data)
            current = ik._data.oMf[frame_id]

            iMd = current.actInv(target_se3)
            err = pinocchio.log(iMd).vector
            err_norm = float(np.linalg.norm(err))

            if err_norm < cfg.eps:
                return q, True, err_norm

            # Full Jacobian at frame
            J = pinocchio.computeFrameJacobian(ik.model, ik._data, q, frame_id, pinocchio.LOCAL)
            J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
            v = -J.T.dot(np.linalg.solve(J.dot(J.T) + cfg.damp * np.eye(6), err))

            v_norm = float(np.linalg.norm(v))
            if v_norm > cfg.max_velocity:
                v = v * (cfg.max_velocity / v_norm)

            q = pinocchio.integrate(ik.model, q, v * cfg.dt)
            # Clamp arm joints to their limits to keep solution valid
            q[q_indices] = np.clip(q[q_indices], lower_arm, upper_arm)

        # Return best result even if not converged
        pinocchio.forwardKinematics(ik.model, ik._data, q)
        pinocchio.updateFramePlacements(ik.model, ik._data)
        final = ik._data.oMf[frame_id]
        iMd_final = final.actInv(target_se3)
        final_err = float(np.linalg.norm(pinocchio.log(iMd_final).vector))
        return q, False, final_err

    def solve(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        target_pose: PoseStamped,
        seed: JointState | None = None,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
        check_collision: bool = True,
        max_attempts: int | None = None,
    ) -> IKResult:
        import pinocchio  # type: ignore[import-untyped]

        entry = self._get_or_build(world, robot_id)
        if entry is None:
            return _create_failure(IKStatus.NO_SOLUTION, "Failed to build Pinocchio model")

        ik, frame_id, q_neutral_full, q_indices, lower_arm, upper_arm = entry

        # Build target SE3
        p = target_pose.position
        q_quat = target_pose.orientation
        rot = pinocchio.Quaternion(q_quat.w, q_quat.x, q_quat.y, q_quat.z).toRotationMatrix()
        target_se3 = pinocchio.SE3(rot, np.array([p.x, p.y, p.z]))

        # Seed arm joints (7 DOF)
        if seed is not None:
            q_arm_seed = np.array(seed.position, dtype=np.float64)
        else:
            with world.scratch_context() as ctx:
                js = world.get_joint_state(ctx, robot_id)
            q_arm_seed = np.array(js.position, dtype=np.float64)

        joint_names = (
            seed.name if seed is not None else world.get_robot_config(robot_id).joint_names
        )

        n_attempts = max_attempts if max_attempts is not None else self._max_attempts
        best_result: IKResult | None = None
        best_error = float("inf")

        for attempt in range(n_attempts):
            # Build full q (13 DOF): arm joints set, gripper at neutral
            q_full = q_neutral_full.copy()
            q_arm_init = q_arm_seed if attempt == 0 else np.random.uniform(lower_arm, upper_arm)
            q_full[q_indices] = q_arm_init

            q_full_sol, converged, err = self._solve_one(
                ik, frame_id, target_se3, q_full, q_indices, lower_arm, upper_arm
            )

            if not converged:
                continue

            if err > position_tolerance * 10:
                continue

            # Extract arm joints only
            q_arm_sol = q_full_sol[q_indices]
            joint_state = JointState(name=joint_names, position=q_arm_sol.tolist())

            if check_collision and not world.check_config_collision_free(robot_id, joint_state):
                continue

            result = IKResult(
                status=IKStatus.SUCCESS,
                joint_state=joint_state,
                position_error=err,
                iterations=attempt * self._max_iter,
                message=f"Pinocchio IK solved (attempt {attempt + 1})",
            )

            if err < best_error:
                best_error = err
                best_result = result

            if err < position_tolerance:
                break

        if best_result is not None:
            return best_result

        return _create_failure(
            IKStatus.NO_SOLUTION, f"Pinocchio IK: no solution in {n_attempts} attempts"
        )
