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

"""Check actual planner/controller travel and fingertip FK against the pinned MJCF."""

import json

import mujoco
import numpy as np
import pinocchio as pin

from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_ARM_JOINTS,
    DUAL_OPENYAM_HOME_JOINTS,
    dual_openyam_sim_hardware,
)
from dimos.robot.manipulators.dual_openyam.sim import (
    DUAL_OPENYAM_SCENE_PATH,
    DUAL_OPENYAM_SIM_BASE_XYZ,
    dual_openyam_sim_model_config,
)


def main() -> None:
    mj_model = mujoco.MjModel.from_xml_path(str(DUAL_OPENYAM_SCENE_PATH))
    mj_data = mujoco.MjData(mj_model)
    config = dual_openyam_sim_model_config()
    model = pin.buildModelFromXML(config.model.load().xml)
    data = model.createData()
    hardware = dual_openyam_sim_hardware()
    if hardware.limits is None:
        raise RuntimeError("Simulation controller has no joint limits")
    for index, name in enumerate(DUAL_OPENYAM_ARM_JOINTS):
        joint = model.joints[model.getJointId(name)]
        lower = model.lowerPositionLimit[joint.idx_q]
        upper = model.upperPositionLimit[joint.idx_q]
        control_lower = hardware.limits.position_lower[index]
        control_upper = hardware.limits.position_upper[index]
        if control_lower is None or control_upper is None:
            raise RuntimeError(f"Controller travel is unbounded for {name}")
        np.testing.assert_allclose(
            [lower, upper],
            [control_lower, control_upper],
            atol=1e-8,
        )
        mj_lower, mj_upper = mj_model.joint(name).range
        if lower < mj_lower - 1e-8 or upper > mj_upper + 1e-8:
            raise RuntimeError(f"Planner permits travel outside MuJoCo limits for {name}")
    postures = [
        np.zeros(12),
        np.asarray(DUAL_OPENYAM_HOME_JOINTS),
        np.asarray(DUAL_OPENYAM_HOME_JOINTS) + [0.3, 0.2, -0.1, 0.2, -0.2, 0.1] * 2,
    ]
    errors = []
    for posture in postures:
        q = pin.neutral(model)
        for name, value in zip(DUAL_OPENYAM_ARM_JOINTS, posture, strict=True):
            joint = model.joints[model.getJointId(name)]
            q[joint.idx_q] = value
            mj_data.qpos[mj_model.joint(name).qposadr[0]] = value
        mujoco.mj_forward(mj_model, mj_data)
        pin.framesForwardKinematics(model, data, q)
        for side in ("left", "right"):
            fingertips = [
                mj_data.geom_xpos[index]
                for index in range(mj_model.ngeom)
                if mj_model.geom_type[index] == mujoco.mjtGeom.mjGEOM_SPHERE
                and mj_model.body(mj_model.geom_bodyid[index]).name
                in {f"{side}_lf_down", f"{side}_rf_down"}
            ]
            if not fingertips:
                raise RuntimeError(f"No fingertip contact spheres for {side}")
            expected = np.mean(fingertips, axis=0)
            actual = (
                data.oMf[model.getFrameId(f"{side}_tcp")].translation + DUAL_OPENYAM_SIM_BASE_XYZ
            )
            error = float(np.linalg.norm(expected - actual))
            errors.append(error)
            if error > 0.003:
                raise RuntimeError(f"{side} TCP mismatch: {error:.6f} m")
    print(
        json.dumps(
            {"joint_limits": "matched", "tcp_samples": len(errors), "max_tcp_error_m": max(errors)}
        )
    )


if __name__ == "__main__":
    main()
