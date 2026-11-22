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

"""Native run45 plant; assets are supplied as a separate browser-demo bundle."""

import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS

DT = 0.02
SUBSTEPS = 10
CHECKPOINT = "r1pro_run46_upperdr_8000u.onnx"
XML = "r1pro-holonomic-run45.xml"
XML_SHA256 = "650388ef5e711d473ac896a07ccffcba0fdbc34c7b9bb5561e75e037e5ed0796"


def pose(position, rotation):
    return Pose(Vector3(*position), Quaternion(*Rotation.from_matrix(rotation).as_quat()))


class R1ProPlant:
    def __init__(self, bundle_dir: str) -> None:
        import mujoco as mj

        assets = Path(bundle_dir).expanduser() / "assets"
        if not (assets / XML).is_file():
            raise FileNotFoundError(
                f"Set R1PRO_SIM_BUNDLE to the demo checkout containing assets/{XML}"
            )
        if hashlib.sha256((assets / XML).read_bytes()).hexdigest() != XML_SHA256:
            raise ValueError(f"{XML} is not the pinned run45 plant")
        bundle = json.loads((assets / "BUNDLE.json").read_text())
        spec = mj.MjSpec.from_file(str(assets / XML))
        # Add instrumentation only; sites/sensors do not affect plant inertia/contact.
        for name in ("imu_chassis_link", "imu_torso_link"):
            spec.body(name).add_site(name=name, size=[0.001] * 3)
            for suffix, kind in (
                ("gyro", mj.mjtSensor.mjSENS_GYRO),
                ("accel", mj.mjtSensor.mjSENS_ACCELEROMETER),
            ):
                spec.add_sensor(
                    name=f"{name}_{suffix}", type=kind, objtype=mj.mjtObj.mjOBJ_SITE, objname=name
                )
        self.mj = mj
        self.model = m = spec.compile()
        if (m.nq, m.nv, m.nu) != (25, 24, 21) or not np.isclose(m.opt.timestep, DT / SUBSTEPS):
            raise ValueError("Unexpected R1 Pro run45 plant dimensions/timebase")
        if (
            m.jnt_type[0] != mj.mjtJoint.mjJNT_FREE
            or m.joint(0).name != "floating_base"
            or m.opt.integrator != mj.mjtIntegrator.mjINT_IMPLICITFAST
            or not np.isclose(m.body_mass.sum(), 126.0, atol=1e-6, rtol=0)
            or tuple(m.joint(i).name for i in range(1, 19)) != UPPER_BODY_JOINTS
            or not np.array_equal(m.actuator_trnid[:18, 0], np.arange(1, 19))
            or not np.all(m.actuator_trnid[18:, 0] == m.site("base_drive").id)
            or not np.all(m.actuator_trntype[18:] == mj.mjtTrn.mjTRN_SITE)
            or not np.all(m.actuator_forcelimited[18:])
            or np.any(m.actuator_ctrllimited)
            or not np.array_equal(
                m.actuator_forcerange[18:], np.array([315, 252, 16.87])[:, None] * [-1, 1]
            )
        ):
            raise ValueError("Expected 126 kg run45 with a free root and native body drive")
        kp, kd = (np.asarray(bundle["controller"][key]) for key in ("kp", "kd"))
        m.actuator_gainprm[:18, 0] = kp
        m.actuator_biasprm[:18, 1] = -kp
        m.actuator_biasprm[:18, 2] = -kd
        self.data = d = mj.MjData(m)
        d.qpos[7:25] = d.ctrl[:18] = bundle["home"]
        mj.mj_forward(m, d)
        self.upper_bodies = m.jnt_bodyid[1:19]

    def step(self) -> None:
        m, d = self.model, self.data
        self.mj.mj_forward(m, d)
        b = self.upper_bodies
        torque = np.cross(
            d.subtree_com[b] - d.xanchor[1:19], m.body_subtreemass[b, None] * m.opt.gravity
        )
        m.actuator_biasprm[:18, 0] = -np.sum(d.xaxis[1:19] * torque, axis=1)
        elapsed_from = d.time
        self.mj.mj_step(m, d, nstep=SUBSTEPS)
        self.mj.mj_forward(m, d)
        if (
            not np.isclose(d.time - elapsed_from, DT, atol=1e-9, rtol=0)
            or not np.isfinite(d.qpos).all()
            or not np.isfinite(d.qvel).all()
        ):
            raise RuntimeError("Non-finite R1 Pro simulation state")

    def wrists(self) -> list[Pose]:
        return [
            pose(self.data.site(name).xpos, self.data.site(name).xmat.reshape(3, 3))
            for name in ("left_wrist", "right_wrist")
        ]
