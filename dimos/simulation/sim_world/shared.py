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


"""One MuJoCo world with several robots in it, stepped once for all of them.

`Sim` is the single-robot shape: it owns the model, the robot and the control loop together,
which is right when there is one robot and the policy runs in the same process. A world with
three G1s in it cannot work that way -- they share one MjData, so only one thing may call
`mj_step`, and the actors live in the robots' own modules. So this holds the physics and the
sensing, takes each robot's joint targets from outside, and hands back what each one's lidar
saw.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.sim_msgs.Contacts import Contact
from dimos.simulation.scenes.mjcf import SCENE_GROUP, geom_name
from dimos.simulation.scenes.procedural import Scene
from dimos.simulation.sensors.lidar import SimLidar
from dimos.simulation.sensors.mujoco_raycaster import NO_BODY_EXCLUDED, MujocoRaycaster
from dimos.simulation.sim_world.body import Chassis, Rig, Sensor
from dimos.simulation.sim_world.world import (
    FRAME_DT,
    GroundTruthLio,
    LidarFrame,
    add_housings,
    with_scene,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class Placement:
    """One robot in a shared world: what it is, what its names are prefixed with, where it stands."""

    prefix: str
    rig: Rig
    at: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if self.rig.chassis is None:
            raise ValueError(
                f"the rig placed at {self.prefix!r} has no chassis, so it cannot share a world: "
                f"give its Rig a chassis= that takes (model, data, prefix)"
            )


@dataclass
class Occupant:
    """One robot inside a shared world: its chassis, its lidars and its own deskew."""

    placement: Placement
    chassis: Chassis
    lidars: list[SimLidar] = field(default_factory=list)

    @property
    def prefix(self) -> str:
        return self.placement.prefix

    @property
    def rig(self) -> Rig:
        return self.placement.rig

    def sensor_pose(self, sensor: Sensor) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        position, rotation = self.chassis.base_pose()
        return position + rotation @ sensor.mount_xyz, rotation @ sensor.mount_r

    def odom_pose(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """The pose odometry reports, of the rig's one sensor or of its base."""
        if self.rig.cloud_frame == "base":
            return self.chassis.base_pose()
        return self.sensor_pose(self.rig.sensors[0])

    def odom_velocity(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Linear velocity in the world frame, angular velocity in the odometry child frame."""
        linear, angular = self.chassis.base_velocity()
        if self.rig.cloud_frame == "base":
            return linear, angular
        sensor = self.rig.sensors[0]
        _, rotation = self.chassis.base_pose()
        return linear + rotation @ np.cross(angular, sensor.mount_xyz), sensor.mount_r.T @ angular


def build_shared_model(scene: Scene, placements: list[Placement]) -> mujoco.MjModel:
    """The scene with every robot attached into it under its own prefix.

    The single-robot `build_model` grows the scene onto the robot's own spec, which has nowhere
    to put a second robot. Here the scene is the parent and each robot is attached to it, the
    way `MjSpec.attach` composes any two specs.
    """
    if not placements:
        raise ValueError("a world needs at least one robot in it")
    spec = with_scene(mujoco.MjSpec(), scene)
    # Taken from the first robot and taken before any attach, which keeps the parent's values
    # and warns about every difference. A scene carries MuJoCo's defaults; a robot's vendor
    # scene is usually tuned, and it is the robot's contacts that the solver is solving.
    first = placements[0].rig.spec()
    spec.option.iterations = first.option.iterations
    spec.option.ls_iterations = first.option.ls_iterations
    spec.option.gravity = first.option.gravity
    for placement in placements:
        x, y, yaw = placement.at
        frame = spec.worldbody.add_frame()
        frame.pos = [x, y, 0.0]
        frame.quat = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        spec.attach(placement.rig.spec(), prefix=placement.prefix, frame=frame)
        add_housings(spec, placement.rig, placement.prefix)
    model = spec.compile()
    for placement in placements:
        if placement.rig.tune is not None:
            placement.rig.tune(model)
    return model


class SharedWorld:
    """One compiled scene with several robots in it, stepped once for all of them.

    Nothing here decides what a robot does. Targets come in through `apply` and the lidar
    frames go out of `tick`, so each robot's actor can live wherever its module does.
    """

    def __init__(self, scene: Scene, seed: int, placements: list[Placement]) -> None:
        self.scene = scene
        self.placements = list(placements)
        prefixes = [placement.prefix for placement in self.placements]
        if len(set(prefixes)) != len(prefixes):
            raise ValueError(f"two robots share a prefix, so their names would collide: {prefixes}")
        self.model = build_shared_model(scene, self.placements)
        self.data = mujoco.MjData(self.model)
        self.occupants = {
            placement.prefix: self._occupy(placement, seed + i)
            for i, placement in enumerate(self.placements)
        }
        self.lio = {prefix: GroundTruthLio() for prefix in self.occupants}
        substeps = {occupant.chassis.substeps for occupant in self.occupants.values()}
        if len(substeps) != 1:
            raise ValueError(f"the robots disagree on how many substeps a tick is: {substeps}")
        self.substeps = substeps.pop()
        self.dt = next(iter(self.occupants.values())).chassis.dt
        self.ticks_per_frame = round(FRAME_DT / self.dt)
        self.t = 0.0
        self._tick = 0
        geom = mujoco.mjtObj.mjOBJ_GEOM
        self._kind = {
            mujoco.mj_name2id(self.model, geom, geom_name(i, box)): box.kind
            for i, box in enumerate(scene.boxes)
        }

    def _occupy(self, placement: Placement, seed: int) -> Occupant:
        assert placement.rig.chassis is not None  # Placement refuses one without
        chassis = placement.rig.chassis(self.model, self.data, placement.prefix)
        groups = np.isin(np.arange(6), (SCENE_GROUP, placement.rig.lidar_group)).astype(np.uint8)
        lidars = []
        for i, sensor in enumerate(placement.rig.sensors):
            shell = f"{placement.prefix}{sensor.housing_body or placement.rig.base_body}"
            excluded = (
                NO_BODY_EXCLUDED
                if sensor.sees_base
                else mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, shell)
            )
            if excluded == -1:
                raise ValueError(
                    f"{placement.prefix}'s lidar looks out of {shell}, which is not a body"
                )
            lidars.append(
                sensor.lidar(MujocoRaycaster(self.model, self.data, groups, excluded), seed + i)
            )
        return Occupant(placement, chassis, lidars)

    def reset(self) -> None:
        """Every robot back on its own spot, and the world's own state cleared with them."""
        mujoco.mj_resetData(self.model, self.data)  # type: ignore[attr-defined]
        for occupant in self.occupants.values():
            x, y, yaw = occupant.placement.at
            occupant.chassis.reset(x, y, 0.0, yaw)
        for lio in self.lio.values():
            lio.clear()
        self.t = 0.0
        self._tick = 0

    def apply(self, prefix: str, target: NDArray[np.float64]) -> None:
        """Hand one robot's actuators its joint target, to be held until the next one lands."""
        self.occupants[prefix].chassis.apply(target)

    def contacts(self, prefix: str) -> list[Contact]:
        """Every (part, scene kind) pair this robot is touching right now, sorted."""
        occupant = self.occupants[prefix]
        found: set[Contact] = set()
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            for scene_geom, robot_geom in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                kind = self._kind.get(scene_geom)
                name = self.model.geom(int(robot_geom)).name or ""
                if kind is not None and name.startswith(prefix):
                    found.add(Contact(occupant.chassis.part(int(robot_geom)), kind))
        return sorted(found)

    def tick(self) -> dict[str, LidarFrame | None]:
        """Step one control tick on the targets already applied, and report the frames that ended.

        Every robot is stepped by this one call, which is the whole point: one MjData, one
        `mj_step`, and whatever each robot's actuators were last told stays in force.
        """
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
            self._cast()
        self._tick += 1
        self.t = self._tick * self.dt
        if self._tick % self.ticks_per_frame:
            return dict.fromkeys(self.occupants, None)
        return {
            prefix: self.lio[prefix].frame(self.t, *occupant.odom_pose())
            for prefix, occupant in self.occupants.items()
        }

    def _cast(self) -> None:
        """One substep's worth of returns for every lidar in the world, into its own deskew."""
        for prefix, occupant in self.occupants.items():
            for sensor, lidar in zip(occupant.rig.sensors, occupant.lidars, strict=True):
                n = int(lidar.pattern.point_rate * self.model.opt.timestep)
                position, rotation = occupant.sensor_pose(sensor)
                points = lidar.cast(position, rotation, n, sensor.lio_keep_every)
                kept = points[np.linalg.norm(points, axis=1) >= sensor.lio_blind_range]
                self.lio[prefix].add(kept, position, rotation)
