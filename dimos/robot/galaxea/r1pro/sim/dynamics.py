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

"""WIM-shaped upper-body trajectories and URDF gravity for simulation.

Reference: recovered WIM d2594a1, robots/r1pro/r1pro.yaml and control/loop.cpp.
This models its outer position controller, not the unidentified motor firmware.
"""

from typing import Any

import numpy as np
from numpy.typing import NDArray
import pinocchio as pin
from ruckig import InputParameter, OutputParameter, Result, Ruckig  # type: ignore[import-not-found]

from dimos.robot.assets.model import LoadedRobotModel
from dimos.robot.galaxea.r1pro.sim.config import CONTROL_PERIOD_NS, UPPER_JOINTS, UPPER_RATES


def coupled_wrist_bounds(joint6: float, *, right: bool) -> tuple[float, float]:
    """Vendor J6/J7 envelope used by WIM, intersected with URDF bounds by caller.

    The polynomial applies only to negative J6. WIM tabulates it at 129 points
    with a 0.0006 rad inward interpolation margin; evaluate it directly here.
    """
    if joint6 > 0:
        return -np.inf, np.inf
    p = float(
        np.polynomial.polynomial.polyval(
            joint6, (4.260748, 17.752459, 43.403286, 58.502658, 40.209266, 11.012791)
        )
    )
    q = float(
        np.polynomial.polynomial.polyval(joint6, (-2.2953, -3.9766, -3.333, 0.605, 2.5598, 1.0522))
    )
    return (0.8 * q, 0.8 * p) if right else (-0.8 * p, -0.8 * q)


class UpperBodyServo:
    """Independent time-synchronized torso/left/right Ruckig groups at 250 Hz."""

    def __init__(self, model: LoadedRobotModel) -> None:
        full = pin.buildModelFromXML(model.xml)
        keep = set(UPPER_JOINTS)
        self.model = pin.buildReducedModel(
            full,
            [i for i, name in enumerate(full.names) if i and name not in keep],
            pin.neutral(full),
        )
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        self.q_indices = [self.model.joints[self.model.getJointId(n)].idx_q for n in UPPER_JOINTS]
        self.v_indices = [self.model.joints[self.model.getJointId(n)].idx_v for n in UPPER_JOINTS]
        joints = {j.name: j for j in model.joints}
        self.lower = np.array([joints[n].lower for n in UPPER_JOINTS], dtype=float)
        self.upper = np.array([joints[n].upper for n in UPPER_JOINTS], dtype=float)
        self.groups: list[tuple[slice, Any, Any, Any]] = []
        for start, count in ((0, 4), (4, 7), (11, 7)):
            group = slice(start, start + count)
            inp = InputParameter(count)
            # WIM treats requested speed as a ceiling and uses acceleration=1.5*v.
            velocity_limits = []
            for name in UPPER_JOINTS[group]:
                velocity = joints[name].velocity
                assert velocity is not None and velocity > 0
                velocity_limits.append(velocity)
            rates = np.minimum(UPPER_RATES[group], velocity_limits)
            inp.max_velocity = rates.tolist()
            inp.max_acceleration = (1.5 * rates).tolist()
            inp.max_jerk = [1e4] * count
            self.groups.append(
                (group, Ruckig(count, CONTROL_PERIOD_NS / 1e9), inp, OutputParameter(count))
            )
        self.initialized = False

    def bound(self, positions: NDArray[np.float64]) -> NDArray[np.float64]:
        result = np.clip(positions, self.lower, self.upper)
        for start, right in ((4, False), (11, True)):
            lo, hi = coupled_wrist_bounds(float(result[start + 5]), right=right)
            result[start + 6] = np.clip(
                result[start + 6], max(lo, self.lower[start + 6]), min(hi, self.upper[start + 6])
            )
        return result

    def step(
        self,
        target: NDArray[np.float64],
        measured: NDArray[np.float64],
        *,
        dt: float = CONTROL_PERIOD_NS / 1e9,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("servo time step must be finite and positive")
        target = self.bound(target)
        positions, velocities = np.zeros(18), np.zeros(18)
        for group, otg, inp, out in self.groups:
            if not self.initialized:
                inp.current_position = measured[group].tolist()
                inp.current_velocity = [0.0] * len(inp.current_position)
                inp.current_acceleration = [0.0] * len(inp.current_position)
            inp.target_position = target[group].tolist()
            # Finished trajectories retain floating-point residuals. A subsequent
            # sub-picoradian retarget can make Ruckig's synchronization solver
            # fail with t_sync=0. Normalize only states already at rest.
            if (
                np.max(np.abs(target[group] - inp.current_position)) < 1e-10
                and max(map(abs, inp.current_velocity)) < 1e-8
                and max(map(abs, inp.current_acceleration)) < 1e-8
            ):
                positions[group] = target[group]
                inp.current_position = target[group].tolist()
                inp.current_velocity = [0.0] * len(inp.current_position)
                inp.current_acceleration = [0.0] * len(inp.current_position)
                otg.reset()
                continue
            otg.delta_time = dt
            result = otg.update(inp, out)
            if result not in (Result.Working, Result.Finished):
                raise RuntimeError(f"R1 Pro trajectory failed: {result}")
            positions[group], velocities[group] = out.new_position, out.new_velocity
            out.pass_to_input(inp)
        self.initialized = True
        bounded = self.bound(positions)
        if not np.array_equal(bounded, positions):
            # A moving coupled limit or reversal near a stop can trim a trajectory.
            # Restart from the bounded command rather than accumulating overshoot.
            velocities[bounded != positions] = 0
            for group, _, inp, _ in self.groups:
                inp.current_position = bounded[group].tolist()
                inp.current_velocity = velocities[group].tolist()
                inp.current_acceleration = [0.0] * len(inp.current_position)
        return bounded, velocities

    def gravity(
        self, commanded: NDArray[np.float64], measured: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        # WIM: measured torso + commanded arms. The sim also compensates torso
        # gravity to approximate its hidden firmware position loop; this is not
        # a claim that WIM transmits torso feedforward torque (it does not).
        self.q[self.q_indices] = np.concatenate((measured[:4], commanded[4:]))
        return np.asarray(pin.computeGeneralizedGravity(self.model, self.data, self.q))[
            self.v_indices
        ].copy()
