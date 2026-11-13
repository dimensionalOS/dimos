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

Loads the composed USD through MuJoCo's native importer. Native indices are bound
once. Fixed bodies remain addressable by UsdManager's frame IDs, including
massless sensor frames.
"""

from dimos_lcm.control_msgs import CborCommand, JointCommand
from dimos_lcm.geometry_msgs import Accel, Point, Pose, Quaternion, Twist, Vector3
from dimos_lcm.sensor_msgs import JointState
from dimos_lcm.sim_msgs import BodyState, CborObservation
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.drives import usd_drives
from dimos.sim2.geometry.mujoco import compile_scene


class MujocoBackend:
    def load(
        self, scene: UsdModel, dt_ns: int, gravity_world: tuple[float, float, float] = (0, 0, -9.81)
    ) -> None:
        self.model = compile_scene(scene, dt_ns, gravity_world=gravity_world)
        self.data = mujoco.MjData(self.model)
        root = scene.source_path.parent if scene.source_path else None
        self._initial = {n: values[2] for n, values in usd_drives(scene.usd, root).items()}
        self.body_ids = tuple(self.model.body(i).name for i in range(1, self.model.nbody))
        self.joint_ids = tuple(
            self.model.joint(i).name
            for i in range(self.model.njnt)
            if self.model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE
        )
        self.driven_joint_ids = tuple(self.model.actuator(i).name for i in range(self.model.nu))
        self.root_ids = tuple(
            self.model.body(int(self.model.jnt_bodyid[i])).name
            for i in range(self.model.njnt)
            if self.model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE
        )
        self._drives = {n: self.model.actuator(n).id for n in self.driven_joint_ids}
        self._gain = self.model.actuator_gainprm.copy()
        self._bias = self.model.actuator_biasprm.copy()
        self.reset()

    def apply_action(
        self, joint_commands: list[JointCommand], cbor_commands: list[CborCommand]
    ) -> None:
        """Validate before mutation. This backend implements joint targets only."""
        if cbor_commands:
            command = cbor_commands[0]
            raise ValueError(f"unsupported CBOR command: {command.key}")
        if not joint_commands:
            return
        # ROS JointCommand carries one interface across named joints. Sim2's
        # motor profile supplies all five interfaces in one atomic action.
        interfaces = ("position", "velocity", "kp", "kd", "effort")
        rows: dict[str, dict[str, float]] = {}
        for command in joint_commands:
            names = list(command.joint_names)
            if (
                command.joint_names_length != len(names)
                or command.values_length != len(command.values)
                or len(names) != len(command.values)
                or not names
                or not all(names)
                or len(set(names)) != len(names)
            ):
                raise ValueError(
                    "interface requires matching values and unique, nonempty joint IDs"
                )
            if command.interface_name not in interfaces or command.interface_name in rows:
                raise ValueError("unknown or duplicate motor interface")
            rows[command.interface_name] = dict(zip(names, command.values, strict=True))
        if set(rows) != set(interfaces):
            raise ValueError("motor action requires position, velocity, kp, kd and effort")
        names = list(rows["position"])
        if any(set(row) != set(names) for row in rows.values()):
            raise ValueError("motor interfaces must address the same joint set")
        values = np.asarray([[rows[interface][name] for interface in interfaces] for name in names])
        if not np.isfinite(values).all() or np.any(values[:, 2:4] < 0):
            raise ValueError("targets must be finite with nonnegative gains")
        ids = []
        for name, row in zip(names, values, strict=True):
            position, _velocity, kp, kd, effort = row
            if name not in self._drives:
                raise ValueError(f"not a driven joint: {name}")
            joint = self.model.joint(name)
            if kp and self.model.jnt_limited[joint.id]:
                bounded = np.clip(position, *joint.range)
                # USD degrees -> URDF radians -> decimal text can round a
                # boundary slightly beyond the native importer's limit.
                if abs(position - bounded) > 1e-10:
                    raise ValueError(f"target outside joint limits: {name}")
                row[0] = bounded
            aid = self._drives[name]
            if (
                kp == kd == 0
                and not self.model.actuator_forcerange[aid, 0]
                <= effort
                <= self.model.actuator_forcerange[aid, 1]
            ):
                raise ValueError(f"effort outside limits: {name}")
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

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)
        # mj_step leaves derived positions/sensors at the previous integration boundary.
        # Refresh for coherent pose, joint and accelerometer samples at the new time.
        mujoco.mj_forward(self.model, self.data)

    def _pose(self, body: int) -> Pose:
        return Pose(
            position=Point(*self.data.xpos[body]),
            orientation=Quaternion(*self.data.xquat[body, [1, 2, 3, 0]]),
        )

    def read_bodies(self) -> list[BodyState]:
        # Populate all constraint-aware accelerations once for this acquisition.
        # MuJoCo cacc is gravity-subtracted; export physical acceleration instead.
        mujoco.mj_rnePostConstraint(self.model, self.data)
        velocity, acceleration = np.empty(6), np.empty(6)
        bodies = []
        for body, name in enumerate(self.body_ids, start=1):
            mujoco.mj_objectVelocity(
                self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY, body, velocity, 0
            )
            mujoco.mj_objectAcceleration(
                self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY, body, acceleration, 0
            )
            bodies.append(
                BodyState(
                    body_id=name,
                    pose=self._pose(body),
                    twist=Twist(linear=Vector3(*velocity[3:]), angular=Vector3(*velocity[:3])),
                    acceleration=Accel(
                        linear=Vector3(*(acceleration[3:] + self.model.opt.gravity)),
                        angular=Vector3(*acceleration[:3]),
                    ),
                )
            )
        return bodies

    def read_joints(self, joint_ids: tuple[str, ...]) -> JointState:
        ids = np.asarray([self.model.joint(n).id for n in joint_ids], dtype=int)
        if any(n not in self.joint_ids for n in joint_ids):
            raise ValueError("only scalar joint coordinates are supported")
        q, v = self.model.jnt_qposadr[ids], self.model.jnt_dofadr[ids]
        count = len(joint_ids)
        return JointState(
            name_length=count,
            position_length=count,
            velocity_length=count,
            effort_length=count,
            name=list(joint_ids),
            position=self.data.qpos[q].tolist(),
            velocity=self.data.qvel[v].tolist(),
            effort=self.data.qfrc_actuator[v].tolist(),
        )

    def read_cbor_observations(self) -> list[CborObservation]:
        return []

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
        self.data.qfrc_applied[:] = 0
        self.data.qacc_warmstart[:] = 0
        mujoco.mj_forward(self.model, self.data)
