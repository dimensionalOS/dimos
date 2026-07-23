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

"""Pink-based differential IK for the teleop control loop.

Same call surface as PinocchioIK so TeleopIKTask can select either solver.
Each solve runs a few velocity-QP steps: joint position and velocity limits
are hard constraints inside the QP (no post-hoc clamping), position versus
orientation is an explicit cost trade-off, and a posture task regularizes
the null space so the elbow does not drift.

Pink ships in the manipulation extra; import errors surface to the caller,
which falls back to the DLS solver.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
import pinocchio

if TYPE_CHECKING:
    from numpy.typing import NDArray


@dataclass
class PinkTeleopIKConfig:
    """Configuration for the Pink teleop IK solver.

    Attributes:
        max_iter: QP integration steps per solve call
        dt: Integration step per QP solve (seconds of virtual time)
        eps: Convergence threshold (SE3 log-error norm)
        damping: QP Levenberg damping
        position_cost: FrameTask position cost (per meter of error)
        orientation_cost: FrameTask orientation cost (per radian of error);
            lower than position_cost makes position win when both cannot be met
        posture_cost: PostureTask cost pulling toward the seed configuration
        qp_solver: qpsolvers backend name
    """

    max_iter: int = 8
    dt: float = 0.05
    eps: float = 1e-3
    damping: float = 1e-10
    position_cost: float = 1.0
    orientation_cost: float = 0.5
    posture_cost: float = 1e-2
    qp_solver: str = "proxqp"


class PinkTeleopIK:
    """Differential IK on pink with the PinocchioIK call surface."""

    def __init__(
        self,
        model: pinocchio.Model,
        data: pinocchio.Data,
        ee_joint_id: int,
        config: PinkTeleopIKConfig | None = None,
    ) -> None:
        from pink.tasks import FrameTask, PostureTask

        self._model = model
        self._data = data
        self._ee_joint_id = ee_joint_id
        self._config = config or PinkTeleopIKConfig()
        self._frame_name = model.names[ee_joint_id]
        self._frame_task = FrameTask(
            self._frame_name,
            position_cost=self._config.position_cost,
            orientation_cost=self._config.orientation_cost,
        )
        self._posture_task = PostureTask(cost=self._config.posture_cost)

    @classmethod
    def from_model_path(
        cls,
        model_path: str | Path,
        ee_joint_id: int,
        config: PinkTeleopIKConfig | None = None,
    ) -> PinkTeleopIK:
        path = Path(str(model_path))
        if not path.exists():
            raise FileNotFoundError(f"Model file not found: {path}")
        if path.suffix == ".xml":
            model = pinocchio.buildModelFromMJCF(str(path))
        else:
            model = pinocchio.buildModelFromUrdf(str(path))
        return cls(model, model.createData(), ee_joint_id, config)

    @property
    def model(self) -> pinocchio.Model:
        return self._model

    @property
    def nq(self) -> int:
        return int(self._model.nq)

    @property
    def ee_joint_id(self) -> int:
        return self._ee_joint_id

    def solve(
        self,
        target_pose: pinocchio.SE3,
        q_init: NDArray[np.floating[Any]],
        config: PinkTeleopIKConfig | None = None,
    ) -> tuple[NDArray[np.floating[Any]], bool, float]:
        import pink
        from pink.exceptions import NoSolutionFound

        cfg = config or self._config
        # Seed strictly inside the limits: the QP is degenerate from a seed on
        # or past a bound (the folded home pose sits exactly on two bounds).
        eps_q = 1e-5
        q_seed = np.clip(
            q_init,
            self._model.lowerPositionLimit + eps_q,
            self._model.upperPositionLimit - eps_q,
        )
        configuration = pink.Configuration(self._model, self._data, q_seed)
        self._frame_task.set_target(target_pose)
        self._posture_task.set_target(q_seed)
        tasks = (self._frame_task, self._posture_task)

        final_err = float("inf")
        for _ in range(cfg.max_iter):
            final_err = self._error(configuration.q, target_pose)
            if final_err < cfg.eps:
                return configuration.q.copy(), True, final_err
            try:
                velocity = pink.solve_ik(
                    configuration,
                    tasks,
                    cfg.dt,
                    solver=cfg.qp_solver,
                    damping=cfg.damping,
                    safety_break=False,
                )
            except NoSolutionFound:
                # Infeasible at this window; return progress so far and let the
                # caller's backoff shrink the target.
                break
            configuration.integrate_inplace(velocity, cfg.dt)
        return configuration.q.copy(), False, self._error(configuration.q, target_pose)

    def _error(self, q: NDArray[np.floating[Any]], target_pose: pinocchio.SE3) -> float:
        pinocchio.forwardKinematics(self._model, self._data, q)
        return float(
            np.linalg.norm(
                pinocchio.log(self._data.oMi[self._ee_joint_id].actInv(target_pose)).vector
            )
        )

    def forward_kinematics(self, joint_positions: NDArray[np.floating[Any]]) -> pinocchio.SE3:
        pinocchio.forwardKinematics(self._model, self._data, joint_positions)
        return self._data.oMi[self._ee_joint_id].copy()
