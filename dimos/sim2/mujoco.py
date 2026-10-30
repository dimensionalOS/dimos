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

import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.sim2.types import (
    Array,
    Camera,
    Frame,
    JointEfforts,
    JointTargets,
    Layout,
    World,
    WorldState,
    Wrench,
)


def compile_world(world: World) -> mujoco.MjModel:
    if not np.isfinite(world.dt) or not 0 < world.dt <= 0.02:
        raise ValueError("dt must be in (0, .02] seconds")
    spec = mujoco.MjSpec()
    spec.option.timestep = world.dt
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.compiler.fusestatic = False
    spec.worldbody.add_geom(
        name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[4, 4, 0.05], rgba=[0.25, 0.3, 0.35, 1]
    )
    spec.worldbody.add_light(
        pos=[0, -3, 5],
        dir=[0, 0, -1],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        diffuse=[0.8, 0.8, 0.8],
    )
    spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
    spec.visual.headlight.diffuse = [0.45, 0.45, 0.45]
    spec.add_texture(
        name="sky",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        width=256,
        height=1536,
        rgb1=[0.65, 0.73, 0.8],
        rgb2=[0.85, 0.88, 0.9],
    )
    ids: set[str] = set()
    mounts: dict[str, tuple[str, Array]] = {}
    for instance in world.instances:
        if instance.id in ids:
            raise ValueError(f"duplicate instance {instance.id}")
        ids.add(instance.id)
        model = instance.model.load()
        links = {model.root_link, *(j.child_link for j in model.joints)}
        for sensor in world.sensors:
            if sensor.frame in links:
                mounts[sensor.frame] = model.resolve_frame(sensor.frame)
        if any(
            j.type not in {"fixed", "prismatic", "revolute", "continuous"} or j.mimic
            for j in model.joints
        ):
            raise ValueError("prototype supports fixed and independent scalar URDF joints")
        # Import flags must be present before parsing (setting discardvisual afterwards is too late).
        xml = ET.fromstring(model.xml)
        for extension in xml.findall("mujoco"):
            xml.remove(extension)
        extension = ET.SubElement(xml, "mujoco")
        ET.SubElement(extension, "compiler", fusestatic="false", discardvisual="false")
        for link in xml.findall("link"):
            for shape_kind in ("visual", "collision"):
                for index, shape in enumerate(link.findall(shape_kind)):
                    shape.set("name", f"{instance.id}/{link.attrib['name']}/{shape_kind}/{index}")
        child = mujoco.MjSpec.from_string(ET.tostring(xml, encoding="unicode"))
        if instance.floating:
            child.body(model.root_link).add_freejoint(name=model.root_link + "/free")
        for drive in instance.drives:
            joint = model.get_joint(drive.joint)
            if joint is None or joint.type == "fixed" or joint.effort is None:
                raise ValueError(f"drive requires a scalar joint with effort limit: {drive.joint}")
            if (
                not np.isfinite(
                    [drive.kp, drive.kd, drive.initial, drive.armature, drive.friction]
                ).all()
                or drive.kp <= 0
                or min(drive.kd, drive.armature, drive.friction) < 0
            ):
                raise ValueError("invalid servo gains")
            if (joint.lower is not None and drive.initial < joint.lower) or (
                joint.upper is not None and drive.initial > joint.upper
            ):
                raise ValueError(f"initial position outside joint limits: {drive.joint}")
            child.joint(drive.joint).armature = drive.armature
            child.joint(drive.joint).frictionloss = drive.friction
            child.add_actuator(
                name=drive.joint,
                target=drive.joint,
                trntype=mujoco.mjtTrn.mjTRN_JOINT,
                gainprm=[drive.kp] + [0] * 9,
                biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                biasprm=[0, -drive.kp, -drive.kd] + [0] * 7,
                forcelimited=True,
                forcerange=[-joint.effort, joint.effort],
            )
        # MuJoCo's URDF importer retains colors but drops diffuse texture references.
        # Restore those from the same shared URDF; do not author a second visual asset.
        for visual in xml.findall(".//visual"):
            texture = visual.find("material/texture")
            if texture is None:
                continue
            name = visual.get("name")
            if not name:
                raise ValueError("textured URDF visuals must have names")
            child.add_texture(
                name=name + "/texture",
                type=mujoco.mjtTexture.mjTEXTURE_2D,
                file=texture.attrib["filename"],
            )
            textures = [""] * int(mujoco.mjtTextureRole.mjNTEXROLE)
            textures[int(mujoco.mjtTextureRole.mjTEXROLE_RGB)] = name + "/texture"
            child.add_material(name=name + "/material", textures=textures)
            child.geom(name).material = name + "/material"
        spec.attach(
            child,
            prefix="",
            frame=spec.worldbody.add_frame(
                pos=instance.xyz,
                quat=Rotation.from_euler("xyz", instance.rpy).as_quat(scalar_first=True),
            ),
        )
    frames = {body.name for body in spec.bodies}
    for sensor in world.sensors:
        if sensor.frame not in frames:
            raise ValueError(f"unknown sensor mount {sensor.frame}")
        ancestor, transform = mounts[sensor.frame]
        body = spec.body(ancestor)
        position = transform[:3, 3]
        rotation = transform[:3, :3]
        if isinstance(sensor, Camera):
            if sensor.width <= 0 or sensor.height <= 0 or not 0 < sensor.fovy < 180:
                raise ValueError("invalid camera profile")
            body.add_camera(
                name=sensor.frame,
                pos=position,
                quat=Rotation.from_matrix(rotation @ np.diag([1.0, -1.0, -1.0])).as_quat(
                    scalar_first=True
                ),
                fovy=sensor.fovy,
            )
        elif sensor.kind == "imu":
            body.add_site(
                name=sensor.frame,
                size=[0.005, 0.005, 0.005],
                pos=position,
                quat=Rotation.from_matrix(rotation).as_quat(scalar_first=True),
            )
            for suffix, kind in (
                ("acc", mujoco.mjtSensor.mjSENS_ACCELEROMETER),
                ("gyro", mujoco.mjtSensor.mjSENS_GYRO),
            ):
                spec.add_sensor(
                    name=sensor.frame + "/" + suffix,
                    type=kind,
                    objtype=mujoco.mjtObj.mjOBJ_SITE,
                    objname=sensor.frame,
                )
        elif sensor.kind != "lidar":
            raise ValueError(f"unsupported sensor kind {sensor.kind}")
    return spec.compile()


class MujocoBackend:
    def __init__(self, world: World) -> None:
        self.model = compile_world(world)
        self._initial = {d.joint: d.initial for i in world.instances for d in i.drives}
        self.data = mujoco.MjData(self.model)
        self.dt = world.dt
        self.layout = Layout(
            bodies=tuple(self.model.body(i).name for i in range(1, self.model.nbody)),
            joints=tuple(
                self.model.joint(i).name
                for i in range(self.model.njnt)
                if self.model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE
            ),
            drives=tuple(self.model.actuator(i).name for i in range(self.model.nu)),
            roots=tuple(i.model.load().root_link for i in world.instances),
            imu_frames=tuple(s.frame for s in world.sensors if s.kind == "imu"),
            revision=world.revision,
        )
        self._jids = np.array([self.model.joint(n).id for n in self.layout.joints], dtype=int)
        self._q = self.model.jnt_qposadr[self._jids]
        self._v = self.model.jnt_dofadr[self._jids]
        self._drives = {name: self.model.actuator(name).id for name in self.layout.drives}
        self._gain = self.model.actuator_gainprm.copy()
        self._bias = self.model.actuator_biasprm.copy()
        self.reset()

    def targets(self, command: JointTargets) -> None:
        if not command.joints or len(set(command.joints)) != len(command.joints):
            raise ValueError("targets require unique, nonempty joint names")
        if (
            len(command.joints) != len(command.positions)
            or not np.isfinite(command.positions).all()
        ):
            raise ValueError("invalid target vector")
        ids = []
        for name, position in zip(command.joints, command.positions, strict=True):
            if name not in self._drives:
                raise ValueError(f"not a driven joint: {name}")
            joint = self.model.joint(name)
            if (
                self.model.jnt_limited[joint.id]
                and not joint.range[0] <= position <= joint.range[1]
            ):
                raise ValueError(f"target outside joint limits: {name}")
            ids.append(self._drives[name])
        count = len(ids)
        for values in (command.velocities, command.kp, command.kd, command.feedforward):
            if values and (len(values) != count or not np.isfinite(values).all()):
                raise ValueError("invalid servo vector")
        kp = np.asarray(command.kp) if command.kp else self._gain[ids, 0]
        kd = np.asarray(command.kd) if command.kd else -self._bias[ids, 2]
        if np.any(kp < 0) or np.any(kd < 0):
            raise ValueError("servo gains must be nonnegative")
        velocity = np.asarray(command.velocities) if command.velocities else np.zeros(count)
        feedforward = np.asarray(command.feedforward) if command.feedforward else np.zeros(count)
        # Affine native actuator: evaluate PD at every integration step, including
        # MuJoCo's implicit damping treatment. No Python policy in the physics loop.
        self.model.actuator_gainprm[ids] = 0
        self.model.actuator_gainprm[ids, 0] = 1
        self.model.actuator_biasprm[ids] = 0
        self.model.actuator_biasprm[ids, 1] = -kp
        self.model.actuator_biasprm[ids, 2] = -kd
        self.data.ctrl[ids] = kp * command.positions + kd * velocity + feedforward

    def efforts(self, command: JointEfforts) -> None:
        if (
            not command.joints
            or len(set(command.joints)) != len(command.joints)
            or len(command.joints) != len(command.efforts)
            or not np.isfinite(command.efforts).all()
        ):
            raise ValueError("invalid effort vector")
        ids = []
        for name, effort in zip(command.joints, command.efforts, strict=True):
            if name not in self._drives:
                raise ValueError(f"not a driven joint: {name}")
            aid = self._drives[name]
            if (
                not self.model.actuator_forcerange[aid, 0]
                <= effort
                <= self.model.actuator_forcerange[aid, 1]
            ):
                raise ValueError(f"effort outside limits: {name}")
            ids.append(aid)
        self.model.actuator_gainprm[ids] = 0
        self.model.actuator_gainprm[ids, 0] = 1
        self.model.actuator_biasprm[ids] = 0
        self.data.ctrl[ids] = command.efforts

    def hold(self, joints: tuple[str, ...]) -> None:
        for name in joints:
            aid = self._drives[name]
            self.model.actuator_gainprm[aid] = self._gain[aid]
            self.model.actuator_biasprm[aid] = self._bias[aid]
            self.data.ctrl[self._drives[name]] = self.data.qpos[self.model.joint(name).qposadr[0]]

        mujoco.mj_forward(self.model, self.data)

    def validate_wrench(self, wrench: Wrench) -> None:
        if wrench.body not in self.layout.bodies:
            raise ValueError(f"unknown wrench body: {wrench.body}")
        if not np.isfinite((*wrench.force, *wrench.torque, *(wrench.point or ()))).all():
            raise ValueError("nonfinite wrench")

    def step(self, wrenches: tuple[Wrench, ...]) -> None:
        self.data.qfrc_applied[:] = 0
        for wrench in wrenches:
            body = self.model.body(wrench.body).id
            point = self.data.xpos[body] if wrench.point is None else np.asarray(wrench.point)
            mujoco.mj_applyFT(
                self.model,
                self.data,
                np.asarray(wrench.force),
                np.asarray(wrench.torque),
                point,
                body,
                self.data.qfrc_applied,
            )
        mujoco.mj_step(self.model, self.data)
        # mj_step leaves derived positions/sensors at the previous integration boundary.
        # Refresh for coherent pose, joint and accelerometer samples at the new time.
        mujoco.mj_forward(self.model, self.data)

    def snapshot(self, epoch: str, tick: int) -> Frame:
        poses = np.column_stack((self.data.xpos[1:], self.data.xquat[1:, [1, 2, 3, 0]]))
        twists = np.empty((self.model.nbody - 1, 6))
        velocity = np.empty(6)
        for body in range(1, self.model.nbody):
            mujoco.mj_objectVelocity(
                self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY, body, velocity, 0
            )
            twists[body - 1] = np.roll(velocity, 3)
        imu = np.array(
            [
                np.r_[self.data.sensor(n + "/acc").data, self.data.sensor(n + "/gyro").data]
                for n in self.layout.imu_frames
            ]
        ).reshape((-1, 6))
        arrays = (
            poses,
            twists,
            self.data.qpos[self._q].copy(),
            self.data.qvel[self._v].copy(),
            self.data.qfrc_actuator[self._v].copy(),
            imu,
        )
        for array in arrays:
            array.flags.writeable = False
        return Frame(
            WorldState(
                epoch, tick, tick * self.dt, time.time(), *arrays[:-1], self.layout.revision
            ),
            imu,
        )

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        for name, position in self._initial.items():
            self.data.qpos[self.model.joint(name).qposadr[0]] = position
        mujoco.mj_forward(self.model, self.data)
        self.hold(self.layout.drives)

    def place(self, body: str, pose: tuple[float, ...]) -> None:
        if body not in self.layout.roots or len(pose) != 7 or not np.isfinite(pose).all():
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
