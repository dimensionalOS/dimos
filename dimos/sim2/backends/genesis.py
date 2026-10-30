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

"""Genesis rigid-body integration of the same authored scene and robot assets.

MJCF compilation is asset import only. Genesis alone advances the world. Its
native arrays and handles never cross the public physics contract.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import TYPE_CHECKING, Any, Literal

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.sim2.messages import FrameMetadata, StateFrame
from dimos.sim2.physics import (
    BackendStep,
    BodyDescription,
    BodyState,
    BodyWrench,
    InertialState,
    JointDescription,
    JointDrive,
    JointState,
    KinematicTarget,
    PhysicsCommand,
    Quat,
    RobotState,
    Vec3,
    WorldDescription,
    WorldState,
)
from dimos.sim2.spec import RobotInstanceSpec
from dimos.sim2.world.scene import describe_scene, scene_path

if TYPE_CHECKING:
    from dimos.sim2.visuals import VisualScene


def _vec3(value: Any) -> Vec3:
    return float(value[0]), float(value[1]), float(value[2])


def _quat(value: Any) -> Quat:
    return float(value[0]), float(value[1]), float(value[2]), float(value[3])


@dataclass(frozen=True)
class JointBinding:
    native: Any
    model_id: int
    actuator: int | None
    position_servo: bool


class GenesisBackend:
    """Own one headless Genesis scene and its engine initialization in a worker."""

    name = "genesis"
    done = False

    def __init__(
        self,
        robots: dict[str, RobotInstanceSpec],
        scene: Path | None = None,
        sensors: tuple[Any, ...] = (),
        physics_dt: float | None = None,
    ) -> None:
        if sensors:
            raise NotImplementedError(
                "Genesis camera/lidar sensor workers are not implemented; use neutral world state or Unreal"
            )
        self.robots = robots
        self.scene_path = scene
        self.physics_dt = (
            min((robot.timestep for robot in robots.values()), default=0.005)
            if physics_dt is None
            else physics_dt
        )
        if not np.isfinite(self.physics_dt) or self.physics_dt <= 0:
            raise ValueError("physics timestep must be finite and positive")
        self.step_seconds = self.physics_dt
        self.sim_time = 0.0
        self.description: WorldDescription
        self._gs: Any = None
        self._scene: Any = None
        self._entity: Any = None
        self._model: Any = None
        self._body_names: dict[int, str] = {}
        self._public_name: Callable[[str], str]
        self._directory: TemporaryDirectory[str] | None = None
        self._bodies: dict[str, Any] = {}
        self._joints: dict[str, JointBinding] = {}
        self._robot_imus: dict[str, tuple[str, Any, Any]] = {}
        self._initial: Any = None
        self._applied_effort: Any = None
        self._initial_acceleration: tuple[Any, Any] | None = None
        self._stepped = False

    def build(self) -> None:
        # Both engines are optional. MuJoCo is used by Genesis's MJCF importer,
        # and here for authored asset metadata; no MuJoCo simulation is stepped.
        import genesis as gs
        import mujoco

        from dimos.sim2.arena import AuthoredArena
        from dimos.sim2.backends.assets import assemble_mjcf, lower_genesis_contacts
        from dimos.sim2.model import create_model

        if self._scene is not None:
            raise RuntimeError("Genesis world is already built")
        default_scene = next(
            (robot.default_scene for robot in self.robots.values()), "logistics.xml"
        )
        path = self.scene_path or scene_path(None, default_scene)
        self._directory = TemporaryDirectory(prefix="dimos-genesis-")
        scene_description = describe_scene(path)
        models = {name: create_model(robot, path) for name, robot in self.robots.items()}
        task = assemble_mjcf(
            AuthoredArena(path), models, hidden_geom_groups=scene_description.hidden_geom_groups
        )
        xml = task.get_xml()
        self._model = model = mujoco.MjModel.from_xml_string(xml)
        source = Path(self._directory.name) / "world.xml"
        lower_genesis_contacts(task, model)
        source.write_text(task.get_xml())
        prefixes = {name: robot.naming_prefix for name, robot in models.items()}

        aliases = {value.body: key for key, value in scene_description.entities.items()}
        aliases.update({value.joint: key for key, value in scene_description.joints.items()})

        def public(name: str) -> str:
            for robot_id, prefix in prefixes.items():
                if name.startswith(prefix):
                    return f"{robot_id}/{name.removeprefix(prefix)}"
            return aliases.get(name, name)

        self._public_name = public
        gs.init(backend=gs.cpu, logging_level="warning")
        self._gs = gs
        self._scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=self.physics_dt, gravity=_vec3(model.opt.gravity)),
            rigid_options=gs.options.RigidOptions(
                integrator=gs.integrator.implicitfast,
                enable_mujoco_compatibility=True,
                enable_neutral_collision=True,
                friction_cone=gs.friction_cone.elliptic
                if int(model.opt.cone) == 1
                else gs.friction_cone.pyramidal,
            ),
            show_viewer=False,
        )
        self._entity = entity = self._scene.add_entity(
            gs.morphs.MJCF(file=str(source), decimate=False, default_armature=None)
        )
        self._scene.build()
        native_masses = entity.get_links_mass().cpu().numpy()
        native_inertias = entity.get_links_inertia().cpu().numpy()
        bodies: dict[str, BodyDescription] = {}
        joints: dict[str, JointDescription] = {}
        for link in entity.links:
            name = public(link.name)
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, link.name)
            if body_id < 0:
                # Genesis names the MJCF world link even when the original has
                # no body name; it is a static frame, not an extra entity.
                if link.parent_idx != -1:
                    raise ValueError(f"Genesis imported an unknown body {link.name!r}")
                body_id = 0
            current = body_id
            dynamic = False
            while current:
                dynamic |= bool(model.body_jntnum[current])
                current = int(model.body_parentid[current])
            if dynamic and (
                not np.isclose(native_masses[link.idx_local], model.body_mass[body_id], rtol=1e-5)
                or not np.allclose(
                    np.linalg.eigvalsh(native_inertias[link.idx_local]),
                    np.sort(model.body_inertia[body_id]),
                    rtol=1e-4,
                    atol=1e-8,
                )
            ):
                raise ValueError(f"Genesis changed authored mass or inertia for {link.name!r}")
            if int(model.body_mocapid[body_id]) >= 0:
                raise NotImplementedError("Genesis adapter does not support MJCF mocap bodies")
            owner = next((r for r, p in prefixes.items() if link.name.startswith(p)), "scene")
            bodies[name] = BodyDescription(owner, "dynamic" if dynamic else "static")
            self._bodies[name] = link
            self._body_names[body_id] = name
        if set(self._body_names) != set(range(model.nbody)):
            raise NotImplementedError("Genesis merged authored bodies without a world-pose mapping")
        for native in entity.joints:
            if native.n_dofs == 6:
                continue  # Floating roots are represented as body poses/twists.
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, native.name)
            if native.n_dofs == 0:
                continue  # Imported fixed links have no authored scalar joint.
            if joint_id < 0:
                raise ValueError(f"Genesis imported an unknown joint {native.name!r}")
            name = public(native.name)
            actuator_ids = np.flatnonzero(model.actuator_trnid[:, 0] == joint_id)
            if len(actuator_ids) > 1:
                raise NotImplementedError(
                    "multiple actuators per joint require a transmission adapter"
                )
            actuator = int(actuator_ids[0]) if len(actuator_ids) else None
            servo = (
                actuator is not None
                and model.actuator_biastype[actuator] == mujoco.mjtBias.mjBIAS_AFFINE
            )
            if actuator is not None:
                if model.actuator_trntype[actuator] != mujoco.mjtTrn.mjTRN_JOINT:
                    raise NotImplementedError("Genesis motor binding requires joint transmissions")
                if native.n_dofs != 1:
                    raise NotImplementedError("motor drives currently require scalar joints")
            kind: Literal["revolute", "prismatic", "spherical"] = "revolute"
            if int(model.jnt_type[joint_id]) == 1:
                kind = "spherical"
            elif int(model.jnt_type[joint_id]) == 2:
                kind = "prismatic"
            child = native.link
            parent = entity.solver.links[child.parent_idx] if child.parent_idx >= 0 else None
            joints[name] = JointDescription(
                public(parent.name) if parent is not None else "world",
                public(child.name),
                kind,
                actuator is not None,
            )
            self._joints[name] = JointBinding(native, joint_id, actuator, servo)
        roots = {name: public(robot.root_body) for name, robot in models.items()}
        robot_joints: dict[str, tuple[str, ...]] = {}
        for name, robot in models.items():
            config = self.robots[name]
            physical = tuple(
                public(model.joint(int(model.actuator_trnid[model.actuator(a).id, 0])).name)
                for a in robot.actuators
            )
            robot_joints[name] = physical
            if len(physical) != len(config.joint_names):
                raise ValueError(f"{name}: public motor count differs from imported actuators")
            scalar = [j for j in robot.joints if model.joint(j).type in (2, 3)]
            initial = robot.init_qpos
            if len(scalar) != len(initial):
                raise ValueError(f"{name}: initial pose differs from model joint count")
            for joint, value in zip(scalar, initial, strict=True):
                model.qpos0[model.joint(joint).qposadr[0]] = value
            if config.control_imu is not None:
                site = model.site(robot.correct_naming(config.control_imu))
                body = public(model.body(site.bodyid[0]).name)
                self._robot_imus[name] = (
                    body,
                    site.pos.copy(),
                    Rotation.from_quat(np.roll(site.quat, -1)),
                )
        self.description = WorldDescription(
            scene_description,
            bodies,
            joints,
            roots,
            robot_joints,
            frozenset({"joint_drive", "body_wrench", "robot_imu", "world_state"}),
        )
        # Resolve authored initial state into Genesis's own coordinate layout.
        self._apply_initial_scene(model)
        qpos = entity.get_qpos().cpu().numpy().copy()
        for joint in entity.joints:
            if not joint.n_qs:
                continue
            mj_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint.name)
            if mj_joint < 0 and joint.n_qs == 7:
                # An authored <freejoint/> may be unnamed; Genesis names it
                # after its body. Resolve that floating root by ownership.
                candidate = int(model.body(joint.link.name).jntadr[0])
                if candidate >= 0 and model.jnt_type[candidate] == mujoco.mjtJoint.mjJNT_FREE:
                    mj_joint = candidate
            if mj_joint < 0:
                raise ValueError(f"initial pose has no imported joint {joint.name!r}")
            address = int(model.jnt_qposadr[mj_joint])
            qpos[joint.qs_idx_local] = model.qpos0[address : address + joint.n_qs]
        entity.set_qpos(qpos)
        entity.set_dofs_velocity(np.zeros(entity.n_dofs))
        self._initial = self._scene.get_state()
        self._applied_effort = np.zeros(entity.n_dofs)
        self._initial_acceleration = (
            entity.get_links_acc().cpu().numpy().copy(),
            entity.get_links_acc_ang().cpu().numpy().copy(),
        )
        self._stepped = False
        self.sim_time = 0.0

    def _apply_initial_scene(self, model: Any) -> None:
        description = self.description.scene
        for name, pose in description.initial.poses.items():
            body = model.body(description.entities[name].body)
            joint = int(model.body_jntadr[body.id])
            if joint < 0 or int(model.jnt_type[joint]) != 0:
                raise ValueError(f"{name}: initial body pose requires a free root")
            address = int(model.jnt_qposadr[joint])
            model.qpos0[address : address + 7] = (
                *pose.position.to_tuple(),
                *np.roll(pose.orientation.to_tuple(), 1),
            )
        for name, position in description.initial.joints.items():
            joint = model.joint(description.joints[name].joint)
            model.qpos0[joint.qposadr[0]] = position

    def reset(self) -> BackendStep:
        self._require_built()
        self._scene.reset(state=self._initial)
        self._applied_effort = np.zeros(self._entity.n_dofs)
        self._stepped = False
        self.sim_time = 0.0
        return BackendStep()

    def validate(self, commands: tuple[PhysicsCommand, ...]) -> None:
        seen: set[str] = set()
        for command in commands:
            if isinstance(command, JointDrive):
                if command.joint in seen:
                    raise ValueError(f"duplicate joint drive {command.joint}")
                seen.add(command.joint)
                binding = self._joints.get(command.joint)
                if binding is None or binding.actuator is None:
                    raise ValueError(f"unknown or unactuated joint {command.joint!r}")
                if not np.isfinite(
                    (command.position, command.velocity, command.kp, command.kd, command.effort)
                ).all():
                    raise ValueError("joint drive values must be finite")
                if command.kp < 0 or command.kd < 0:
                    raise ValueError("joint drive gains must be nonnegative")
                if binding.position_servo and any(
                    (command.velocity, command.kp, command.kd, command.effort)
                ):
                    raise ValueError("authored position servos accept position targets only")
            elif isinstance(command, BodyWrench):
                body = self.description.bodies.get(command.body)
                if body is None or body.motion != "dynamic":
                    raise ValueError(f"wrench requires a dynamic body: {command.body!r}")
                if (
                    len(command.force) != 3
                    or len(command.torque) != 3
                    or (command.point is not None and len(command.point) != 3)
                ):
                    raise ValueError("body wrench vectors must have three components")
                if command.frame not in ("world", "body"):
                    raise ValueError("wrench frame must be world or body")
                if not np.isfinite((*command.force, *command.torque, *(command.point or ()))).all():
                    raise ValueError("body wrench values must be finite")
            elif isinstance(command, KinematicTarget):
                raise NotImplementedError("Genesis kinematic targets are not implemented")
            else:
                raise TypeError(f"unsupported physics command {type(command).__name__}")

    def step(self, commands: tuple[PhysicsCommand, ...]) -> BackendStep:
        self._require_built()
        self.validate(commands)
        entity = self._entity
        self._refresh_cartesian()
        # Native commands persist in Genesis. Clear all motors for the next
        # interval, then apply only commands active in this backend step.
        positions = entity.get_dofs_position().cpu().numpy()
        velocities = entity.get_dofs_velocity().cpu().numpy()
        efforts = np.zeros(entity.n_dofs)
        servo_dofs: list[int] = []
        servo_targets: list[float] = []
        for command in commands:
            if isinstance(command, JointDrive):
                binding = self._joints[command.joint]
                dof = binding.native.dofs_idx_local[0]
                if binding.position_servo:
                    servo_dofs.append(dof)
                    servo_targets.append(command.position)
                else:
                    efforts[dof] = (
                        command.kp * (command.position - positions[dof])
                        + command.kd * (command.velocity - velocities[dof])
                        + command.effort
                    )
            elif isinstance(command, BodyWrench):
                link = self._bodies[command.body]
                force = np.asarray(command.force)
                torque = np.asarray(command.torque)
                point = command.point
                if command.frame == "body":
                    rotation = Rotation.from_quat(np.roll(link.get_quat().cpu().numpy(), -1))
                    force, torque = rotation.apply(force), rotation.apply(torque)
                    if point is not None:
                        point = _vec3(link.get_pos().cpu().numpy() + rotation.apply(point))
                # Genesis's local CoM frame uses inertia principal axes. The
                # public body frame is the authored link frame, so convert first.
                link.apply_external_wrench(
                    force=force,
                    torque=torque,
                    ref=self._gs.link_ref_frame.link_COM,
                    local=False,
                    pos=point,
                )
        entity.control_dofs_force(efforts)
        if servo_dofs:
            entity.control_dofs_position(servo_targets, dofs_idx_local=servo_dofs)
        self._applied_effort = entity.get_dofs_control_force().cpu().numpy().copy()
        self._scene.step()
        self._stepped = True
        self.sim_time += self.step_seconds
        return BackendStep()

    def _refresh_cartesian(self) -> None:
        # Genesis's MuJoCo-compatible stepping leaves Cartesian caches at the
        # start of the interval. Refresh from the unchanged generalized state.
        self._entity.solver.update_forward_pos()
        if self._entity.n_dofs:
            self._entity.solver.set_dofs_velocity(self._entity.get_dofs_velocity())

    def read_state(self, metadata: FrameMetadata) -> WorldState:
        self._require_built()
        entity = self._entity
        self._refresh_cartesian()
        positions = entity.get_links_pos().cpu().numpy()
        orientations = entity.get_links_quat().cpu().numpy()
        velocities = entity.get_links_vel().cpu().numpy()
        angular = entity.get_links_ang().cpu().numpy()
        accelerations = entity.get_links_acc().cpu().numpy()
        angular_accelerations = entity.get_links_acc_ang().cpu().numpy()
        if not self._stepped:
            assert self._initial_acceleration is not None
            accelerations, angular_accelerations = self._initial_acceleration
        bodies = {
            name: BodyState(
                _vec3(positions[link.idx_local]),
                _quat(np.roll(orientations[link.idx_local], -1)),
                _vec3(velocities[link.idx_local]),
                _vec3(angular[link.idx_local]),
            )
            for name, link in self._bodies.items()
        }
        q = entity.get_qpos().cpu().numpy()
        dq = entity.get_dofs_velocity().cpu().numpy()
        effort = self._applied_effort
        joints = {}
        for name, binding in self._joints.items():
            native = binding.native
            value = q[native.qs_idx_local]
            if native.n_qs == 4:
                value = np.roll(value, -1)
            joints[name] = JointState(
                tuple(value.tolist()),
                tuple(dq[native.dofs_idx_local].tolist()),
                tuple(effort[native.dofs_idx_local].tolist()),
            )
        robots = {}
        for name, config in self.robots.items():
            ids = self.description.robot_joints[name]
            scale = np.asarray(config.position_scale or (1.0,) * len(ids))
            offset = np.asarray(config.position_offset or (0.0,) * len(ids))
            imu = None
            if name in self._robot_imus:
                body_name, local_pos, local_rotation = self._robot_imus[name]
                body = bodies[body_name]
                link = self._bodies[body_name]
                body_rotation = Rotation.from_quat(body.orientation)
                rotation = body_rotation * local_rotation
                displacement = body_rotation.apply(local_pos)
                omega = np.asarray(body.angular_velocity)
                velocity = np.asarray(body.linear_velocity) + np.cross(omega, displacement)
                acceleration = (
                    accelerations[link.idx_local]
                    + np.cross(angular_accelerations[link.idx_local], displacement)
                    + np.cross(omega, np.cross(omega, displacement))
                )
                imu = InertialState(
                    _quat(rotation.as_quat()),
                    _vec3(rotation.inv().apply(omega)),
                    _vec3(rotation.inv().apply(velocity)),
                    _vec3(rotation.inv().apply(acceleration - self._model.opt.gravity)),
                )
            robots[name] = RobotState(
                config.joint_names,
                tuple(
                    ((np.asarray([joints[j].position[0] for j in ids]) - offset) / scale).tolist()
                ),
                tuple((np.asarray([joints[j].velocity[0] for j in ids]) / scale).tolist()),
                tuple(
                    (
                        np.asarray([effort[self._joints[j].native.dofs_idx_local[0]] for j in ids])
                    ).tolist()
                ),
                bodies[self.description.robot_roots[name]],
                imu,
            )
        return WorldState(metadata, bodies, joints, robots)

    def set_task_action(self, action: list[float]) -> None:
        raise NotImplementedError("Genesis does not run robosuite task actions")

    def describe_visuals(self) -> VisualScene:
        # MJCF visual metadata is parsed only when a renderer requests it.
        from dimos.sim2.backends.mjcf_visuals import load_visuals

        self._require_built()
        return load_visuals(self._model, self._body_names, self._public_name, 0)

    def sensor_description(self) -> dict[str, Any]:
        return {"backend": self.name, "capabilities": sorted(self.description.capabilities)}

    def checkpoint(self, metadata: FrameMetadata, wall_time: float) -> StateFrame | None:
        return None

    def _require_built(self) -> None:
        if self._scene is None or self._initial is None:
            raise RuntimeError("Genesis world has not been built")

    def close(self) -> None:
        if self._scene is not None:
            self._scene.destroy()
            self._scene = None
        if self._gs is not None:
            self._gs.destroy()
            self._gs = None
        self._initial = None
        if self._directory is not None:
            self._directory.cleanup()
            self._directory = None
