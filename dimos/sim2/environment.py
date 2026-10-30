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

"""A robosuite environment for externally controlled motor devices.

The model factories return ordinary RobotModel instances. Task owns composition,
MujocoEnv owns stepping/reset/observables, and MotorController implements the
hardware's PD command semantics. Stock robosuite task environments can also run
through SimulationModule directly, using their own actions and observations.
"""

from __future__ import annotations

from collections.abc import Callable
from copy import deepcopy
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from numpy.typing import NDArray
from robosuite.models.robots.robot_model import RobotModel
from robosuite.models.tasks.task import Task
from robosuite.utils.mjcf_utils import array_to_string
from robosuite.utils.observables import Observable
from scipy.spatial.transform import Rotation

from dimos.sim2.arena import AuthoredArena
from dimos.sim2.control import MotorController
from dimos.sim2.rendering import NativeRendering
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.composition import Sensor, mounted_sensors
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.observations import create_observables
from dimos.sim2.sensors.spec import Mount
from dimos.sim2.world.scene import describe_scene


def attach_sensors(model: Any, sensors: tuple[Sensor, ...], prefix: str) -> None:
    """Attach driver-declared devices to existing upstream model bodies or sites."""
    for device in mounted_sensors(sensors):
        attachment = device.camera if isinstance(device, Camera) else device.site
        tag = "camera" if isinstance(device, Camera) else "site"
        name = prefix + device.model_name
        if isinstance(attachment, Mount):
            parent = (
                model.worldbody
                if not prefix and attachment.link == "world"
                else model.worldbody.find(f".//body[@name='{prefix + attachment.link}']")
            )
            xyz = np.asarray(attachment.xyz)
            rotation = Rotation.from_euler("xyz", attachment.rpy)
            if parent is None:
                site = model.worldbody.find(f".//site[@name='{prefix + attachment.link}']")
                if site is None:
                    raise ValueError(f"{device.name}: unknown attachment {attachment.link!r}")
                parent = next(node for node in model.worldbody.iter() if site in node)
                site_rot = Rotation.from_quat(
                    np.roll(np.fromstring(site.get("quat", "1 0 0 0"), sep=" "), -1)
                )
                xyz = np.fromstring(site.get("pos", "0 0 0"), sep=" ") + site_rot.apply(xyz)
                rotation = site_rot * rotation
            attrs = dict(
                name=name,
                pos=array_to_string(xyz),
                quat=array_to_string(np.roll(rotation.as_quat(), 1)),
            )
            if isinstance(device, Camera):
                attrs["fovy"] = str(device.fovy if device.fovy is not None else 60)
            else:
                attrs.update(size=".001", rgba="0 0 0 0")
            ET.SubElement(parent, tag, **attrs)
        elif model.worldbody.find(f".//{tag}[@name='{name}']") is None:
            raise ValueError(f"{device.name}: unknown {tag} {attachment!r}")
        if isinstance(device, Imu):
            stem = prefix + f"sensor/{device.name}/"
            ET.SubElement(model.sensor, "gyro", name=stem + "gyro", site=name)
            ET.SubElement(model.sensor, "accelerometer", name=stem + "accel", site=name)
            ET.SubElement(
                model.sensor, "framequat", name=stem + "quat", objtype="site", objname=name
            )


class MotorScene(NativeRendering):
    """Upstream scene lifecycle with a full PD action for each controlled motor.

    One environment action is one physics tick. The external DimOS policy may
    update less frequently; the bridge holds its latest command between updates.
    This environment has no benchmark task or reward. To run an existing task,
    supply that task's factory to SimulationModule instead.
    """

    def __init__(
        self,
        scene: Path,
        models: dict[str, Callable[[], RobotModel]],
        sensors: tuple[tuple[str | None, Sensor], ...] = (),
        timestep: float = 0.005,
        *,
        hard_reset: bool = False,
        seed: int | None = None,
    ) -> None:
        if not np.isfinite(timestep) or timestep <= 0:
            raise ValueError("physics timestep must be finite and positive")
        self.scene = scene
        self.model_factories = models
        self.sensor_bindings = sensors
        self.physics_timestep = timestep
        self.robot_models: dict[str, RobotModel] = {}
        self.controllers: dict[str, MotorController] = {}
        self.description = describe_scene(scene)
        super().__init__(
            control_freq=1 / timestep,
            hard_reset=hard_reset,
            has_renderer=False,
            has_offscreen_renderer=any(isinstance(s, Camera) for _, s in sensors),
            renderer="mujoco",
            render_camera=None,
            lite_physics=False,
            ignore_done=True,
            seed=seed,
        )

    def _load_model(self) -> None:
        arena = AuthoredArena(self.scene)
        self.robot_models = {name: factory() for name, factory in self.model_factories.items()}
        for name, model in self.robot_models.items():
            attach_sensors(
                model,
                tuple(s for owner, s in self.sensor_bindings if owner == name),
                model.naming_prefix,
            )
        attach_sensors(arena, tuple(s for owner, s in self.sensor_bindings if owner is None), "")
        self.model = Task(arena, list(self.robot_models.values()))
        # Task merges physical model sections but drops authored custom metadata.
        authored_custom = arena.root.find("custom")
        if authored_custom is not None:
            custom = self.model.root.find("custom")
            if custom is None:
                self.model.root.append(deepcopy(authored_custom))
            else:
                custom.extend(deepcopy(list(authored_custom)))
        option = self.model.root.find("option")
        if option is None:
            option = ET.SubElement(self.model.root, "option")
        authored_option = arena.root.find("option")
        if authored_option is not None:
            option.attrib.update(authored_option.attrib)
        option.set("timestep", str(self.physics_timestep))
        option.set("integrator", "implicitfast")
        # Task uses its standard world defaults; retain the authored visual setup.
        visual = self.model.root.find("visual")
        if visual is not None:
            self.model.root.remove(visual)
        authored_visual = arena.root.find("visual")
        visual = deepcopy(authored_visual) if authored_visual is not None else ET.Element("visual")
        self.model.root.append(visual)
        cameras = [s for _, s in self.sensor_bindings if isinstance(s, Camera)]
        if cameras:
            settings = visual.find("global")
            if settings is None:
                settings = ET.SubElement(visual, "global")
            settings.set("offwidth", str(max(640, *(s.width for s in cameras))))
            settings.set("offheight", str(max(480, *(s.height for s in cameras))))
        for geom in self.model.worldbody.iter("geom"):
            if int(geom.get("group", "0")) in self.description.hidden_geom_groups:
                rgba = np.fromstring(geom.get("rgba", ".5 .5 .5 1"), sep=" ")
                rgba[3] = 0
                geom.set("rgba", array_to_string(rgba))

    def initialize_time(self, control_freq: float) -> None:
        super().initialize_time(control_freq)
        # Upstream defaults to a global timestep; this environment reads its model.
        self.model_timestep = float(self.sim.model.opt.timestep)
        self.control_timestep = self.model_timestep

    def _setup_references(self) -> None:
        super()._setup_references()
        # The environment owns one controller per compiled robot. The DimOS
        # runtime configures its public motor units without replacing it.
        self.controllers = {
            name: MotorController(self.sim, model) for name, model in self.robot_models.items()
        }

    def _reset_internal(self) -> None:
        super()._reset_internal()
        for name, controller in self.controllers.items():
            model = self.robot_models[name]
            home = np.asarray(model.init_qpos)
            scalar_joints = [
                joint
                for joint in model.joints
                if self.sim.model.jnt_type[self.sim.model.joint_name2id(joint)]
                in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)
            ]
            if home.shape != (len(scalar_joints),):
                raise ValueError(f"{name}: initial joint configuration must match model joints")
            for joint, position in zip(scalar_joints, home, strict=True):
                self.sim.data.set_joint_qpos(joint, position)
            controller.reset_goal()
            self.sim.data.ctrl[controller.actuators] = controller.run_controller()
        for key, pose in self.description.initial.poses.items():
            body = self.description.entities[key].body
            body_id = self.sim.model.body_name2id(body)
            quaternion = (
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            )
            mocap_id = int(self.sim.model.body_mocapid[body_id])
            if mocap_id >= 0:
                self.sim.data.mocap_pos[mocap_id] = pose.position.to_tuple()
                self.sim.data.mocap_quat[mocap_id] = quaternion
            else:
                joint = int(self.sim.model.body_jntadr[body_id])
                if joint < 0 or self.sim.model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE:
                    raise ValueError(f"{key!r} has no movable root")
                qpos = int(self.sim.model.jnt_qposadr[joint])
                dof = int(self.sim.model.jnt_dofadr[joint])
                self.sim.data.qpos[qpos : qpos + 7] = (*pose.position.to_tuple(), *quaternion)
                self.sim.data.qvel[dof : dof + 6] = 0
        for key, value in self.description.initial.joints.items():
            name = self.description.joints[key].joint
            joint = self.sim.model.joint_name2id(name)
            if self.sim.model.jnt_type[joint] not in (
                mujoco.mjtJoint.mjJNT_HINGE,
                mujoco.mjtJoint.mjJNT_SLIDE,
            ):
                raise ValueError(f"{key!r} must be a scalar fixture joint")
            lower, upper = self.sim.model.jnt_range[joint]
            if self.sim.model.jnt_limited[joint] and not lower <= value <= upper:
                raise ValueError(f"{key!r} initial position is outside its model limits")
            self.sim.data.set_joint_qpos(name, value)
        self.sim.forward()

    def _setup_observables(self) -> dict[str, Observable]:
        observables: dict[str, Observable] = dict(super()._setup_observables())
        for owner, device in self.sensor_bindings:
            if isinstance(device, (Camera, Imu)):
                prefix = self.robot_models[owner].naming_prefix if owner is not None else ""
                observables.update(create_observables(self.sim, device, prefix, owner))
        return observables

    def _update_observables(self, force: bool = False) -> None:
        # Refresh post-integration poses/sensors before labelling observations.
        self.sim.forward()
        super()._update_observables(force)

    def _pre_action(self, action: NDArray[np.float64], policy_step: bool = False) -> None:
        offset = 0
        for controller in self.controllers.values():
            size = 5 * len(controller.actuators)
            if policy_step:
                controller.set_goal(action[offset : offset + size])
            self.sim.data.ctrl[controller.actuators] = controller.run_controller()
            offset += size

    @property
    def action_dim(self) -> int:
        return sum(5 * len(controller.actuators) for controller in self.controllers.values())

    @property
    def action_spec(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return np.full(self.action_dim, -np.inf), np.full(self.action_dim, np.inf)

    def reward(self, action: Any = None) -> float:
        return 0.0

    def _check_success(self) -> bool:
        return False
