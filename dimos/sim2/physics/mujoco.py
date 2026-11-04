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

"""MuJoCo compilation and dynamics. The only dynamics engine in this prototype.

Imports the shared, materialized URDF; no parallel MJCF robot asset. Native
indices are bound once. Fixed links are retained so logical frame IDs agree
with UrdfManager even when the model includes massless sensor frames.
"""

from dimos_lcm.control_msgs import BodyWrench, JointTarget
from dimos_lcm.geometry_msgs import Point, Pose, Quaternion
from dimos_lcm.sim_msgs import BodyMotion, BodyPose, CborExtension, JointTruth
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.sim2.description import SceneConfig
from dimos.sim2.geometry.mujoco import compile_scene


class MujocoBackend:
    def load(self, scene: SceneConfig, dt_ns: int) -> None:
        self.model = compile_scene(scene, dt_ns)
        self.data = mujoco.MjData(self.model)
        self._initial = {d.joint: d.initial for i in scene.instances for d in i.drives}
        self.body_ids = tuple(self.model.body(i).name for i in range(1, self.model.nbody))
        self.joint_ids = tuple(
            self.model.joint(i).name
            for i in range(self.model.njnt)
            if self.model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE
        )
        self.driven_joint_ids = tuple(self.model.actuator(i).name for i in range(self.model.nu))
        self.root_ids = tuple(i.description.root_link for i in scene.instances)
        self._drives = {n: self.model.actuator(n).id for n in self.driven_joint_ids}
        self._gain = self.model.actuator_gainprm.copy()
        self._bias = self.model.actuator_biasprm.copy()
        self.reset()

    def set_joint_targets(self, targets: list[JointTarget]) -> None:
        names = [t.joint_id for t in targets]
        if not names or len(set(names)) != len(names):
            raise ValueError("targets require unique, nonempty joint IDs")
        values = np.asarray([[t.position, t.velocity, t.kp, t.kd, t.feedforward] for t in targets])
        if not np.isfinite(values).all() or np.any(values[:, 2:4] < 0):
            raise ValueError("targets must be finite with nonnegative gains")
        ids = []
        for target in targets:
            if target.joint_id not in self._drives:
                raise ValueError(f"not a driven joint: {target.joint_id}")
            joint = self.model.joint(target.joint_id)
            if (
                target.kp
                and self.model.jnt_limited[joint.id]
                and not joint.range[0] <= target.position <= joint.range[1]
            ):
                raise ValueError(f"target outside joint limits: {target.joint_id}")
            aid = self._drives[target.joint_id]
            if (
                target.kp == target.kd == 0
                and not self.model.actuator_forcerange[aid, 0]
                <= target.feedforward
                <= self.model.actuator_forcerange[aid, 1]
            ):
                raise ValueError(f"effort outside limits: {target.joint_id}")
            ids.append(aid)
        position, velocity, kp, kd, feedforward = values.T
        self.model.actuator_gainprm[ids] = 0
        self.model.actuator_gainprm[ids, 0] = 1
        self.model.actuator_biasprm[ids] = 0
        self.model.actuator_biasprm[ids, 1] = -kp
        self.model.actuator_biasprm[ids, 2] = -kd
        self.data.ctrl[ids] = kp * position + kd * velocity + feedforward

    def hold(self, joints: tuple[str, ...]) -> None:
        for name in joints:
            aid = self._drives[name]
            self.model.actuator_gainprm[aid] = self._gain[aid]
            self.model.actuator_biasprm[aid] = self._bias[aid]
            self.data.ctrl[self._drives[name]] = self.data.qpos[self.model.joint(name).qposadr[0]]

        mujoco.mj_forward(self.model, self.data)

    def validate_wrench(self, wrench: BodyWrench) -> None:
        if wrench.body_id not in self.body_ids:
            raise ValueError(f"unknown wrench body: {wrench.body_id}")
        if not np.isfinite(
            (*wrench.force_world, *wrench.torque_world, *(wrench.point_world))
        ).all():
            raise ValueError("nonfinite wrench")

    def step(self, wrenches: tuple[BodyWrench, ...]) -> None:
        self.data.qfrc_applied[:] = 0
        for wrench in wrenches:
            body = self.model.body(wrench.body_id).id
            point = np.asarray(wrench.point_world)
            mujoco.mj_applyFT(
                self.model,
                self.data,
                np.asarray(wrench.force_world),
                np.asarray(wrench.torque_world),
                point,
                body,
                self.data.qfrc_applied,
            )
        mujoco.mj_step(self.model, self.data)
        # mj_step leaves derived positions/sensors at the previous integration boundary.
        # Refresh for coherent pose, joint and accelerometer samples at the new time.
        mujoco.mj_forward(self.model, self.data)

    def _pose(self, body: int) -> Pose:
        return Pose(
            position=Point(*self.data.xpos[body]),
            orientation=Quaternion(*self.data.xquat[body, [1, 2, 3, 0]]),
        )

    def read_body_poses(self) -> list[BodyPose]:
        return [
            BodyPose(body_id=name, pose_world=self._pose(self.model.body(name).id))
            for name in self.body_ids
        ]

    def read_joints(self, joint_ids: tuple[str, ...]) -> JointTruth:
        ids = np.asarray([self.model.joint(n).id for n in joint_ids], dtype=int)
        if any(n not in self.joint_ids for n in joint_ids):
            raise ValueError("only scalar joint coordinates are supported")
        q, v = self.model.jnt_qposadr[ids], self.model.jnt_dofadr[ids]
        count = len(joint_ids)
        return JointTruth(
            name_length=count,
            position_length=count,
            velocity_length=count,
            effort_length=count,
            name=list(joint_ids),
            position=self.data.qpos[q].tolist(),
            velocity=self.data.qvel[v].tolist(),
            effort=self.data.qfrc_actuator[v].tolist(),
        )

    def read_extensions(self) -> list[CborExtension]:
        return []

    def read_motion(self, frame_id: str) -> BodyMotion:
        body = self.model.body(frame_id).id
        velocity, acceleration = np.empty(6), np.empty(6)
        mujoco.mj_objectVelocity(
            self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY, body, velocity, 0
        )
        # Native cacc uses gravity-subtracted spatial acceleration; populate it
        # explicitly even when the model contains no MuJoCo sensors.
        mujoco.mj_rnePostConstraint(self.model, self.data)
        mujoco.mj_objectAcceleration(
            self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY, body, acceleration, 0
        )
        result = BodyMotion(
            pose_world=self._pose(body),
            linear_velocity_world=velocity[3:].tolist(),
            angular_velocity_world=velocity[:3].tolist(),
            linear_acceleration_world=(acceleration[3:] + self.model.opt.gravity).tolist(),
            gravity_world=self.model.opt.gravity.tolist(),
        )
        result.frame_id = frame_id
        return result

    def close(self) -> None:
        pass

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        for name, position in self._initial.items():
            self.data.qpos[self.model.joint(name).qposadr[0]] = position
        mujoco.mj_forward(self.model, self.data)
        self.hold(self.driven_joint_ids)

    def place(self, body: str, pose: tuple[float, ...]) -> None:
        if body not in self.root_ids or len(pose) != 7 or not np.isfinite(pose).all():
            raise ValueError("place requires a free root and finite xyz/xyzw pose")
        b = self.model.body(body)
        jid = int(b.jntadr[0])
        if b.jntnum[0] != 1 or self.model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError("only floating roots can be placed")
        quat = Rotation.from_quat(pose[3:]).as_quat(scalar_first=True)
        q, v = self.model.jnt_qposadr[jid], self.model.jnt_dofadr[jid]
        self.data.qpos[q : q + 7] = (*pose[:3], *quat)
        self.data.qvel[v : v + 6] = 0
        self.data.time = 0.0
        self.data.qfrc_applied[:] = 0
        self.data.qacc_warmstart[:] = 0
        mujoco.mj_forward(self.model, self.data)
