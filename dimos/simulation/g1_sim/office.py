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


"""The G1 standing in office_555, so its lidar sees a real room rather than generated boxes."""

from __future__ import annotations

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.simulation.g1_sim.body import LeggedG1, g1_spec
from dimos.simulation.g1_sim.policy import G1Policy
from dimos.simulation.g1_sim.world import G1_RIG, MID360
from dimos.simulation.scenes.mjcf import SCENE_GROUP
from dimos.simulation.scenes.office_555 import CLEAR_FLOOR, SCENE
from dimos.simulation.sensors.lidar import SimLidar
from dimos.simulation.sensors.mujoco_raycaster import MujocoRaycaster
from dimos.simulation.sim_world.body import ROBOT_VISUAL_GROUP
from dimos.simulation.sim_world.world import add_housings, light_indoors
from dimos.simulation.usd_spec import load_usd_spec

# the prefix the G1's elements take once attached, so nothing collides with the room's names
PREFIX = "g1_"
# The converted room spreads its geometry over three MuJoCo groups, where a generated scene uses
# one. Group 0 is the building: the floor and roof slabs, 234 boxes of wall and partition, and
# two stairwells. Group 1 is the furniture, one visual mesh each -- chairs, desks, shelves,
# crates. Group 3 is the convex pieces those decompose into for contact. A lidar has to see the
# first two and not the third, which is the distinction SCENE_GROUP draws by itself in a
# generated scene, so the mask names the groups rather than taking the rig's single one.
FURNITURE_GROUP = 1
LIDAR_GROUPS: NDArray[np.uint8] = np.isin(
    np.arange(6), (SCENE_GROUP, FURNITURE_GROUP, ROBOT_VISUAL_GROUP)
).astype(np.uint8)


def office_model(at: tuple[float, float] = CLEAR_FLOOR) -> mujoco.MjSpec:
    """office_555 with a G1 standing in it, as one spec.

    Both halves are USD: the room is the committed scene layer and the robot is its own, and
    MjSpec.attach composes them the way an MJCF `<include>` would. The alternative -- the
    procedural box scene the other sims use -- is generated geometry, so a lidar pointed at it
    tells you nothing about how the real room scans.
    """
    office = load_usd_spec(SCENE.resolve())
    # The layer carries no lights -- the converter writes none and the MJCF had none either, so
    # the room arrived lit by whatever the viewer defaulted to.
    light_indoors(office)
    robot = g1_spec()
    # attach keeps the parent's solver settings and warns where they differ, and the parent here
    # is a room that never moves: every contact in the model is the robot's, so its own are the
    # ones to keep. Set before attaching, or there is nothing left to disagree with. The vendor's
    # G1 scene runs 3 iterations and 5 line searches; the office layer says nothing about either
    # and so carries MuJoCo's defaults of 100 and 50.
    office.option.iterations = robot.option.iterations
    office.option.ls_iterations = robot.option.ls_iterations
    frame = office.worldbody.add_frame()
    frame.pos = [at[0], at[1], 0.0]
    office.attach(robot, prefix=PREFIX, frame=frame)
    return office


def standing(at: tuple[float, float] = CLEAR_FLOOR) -> tuple[LeggedG1, mujoco.MjModel]:
    """A G1 reset into its default pose on the office floor, with its lidar housing in place."""
    spec = office_model(at)
    add_housings(spec, G1_RIG, PREFIX)
    model = spec.compile()
    body = LeggedG1(model, mujoco.MjData(model), G1Policy.load(LeggedG1.dt), prefix=PREFIX)
    body.reset(at[0], at[1], 0.0, 0.0)
    return body, model


def sensor_pose(body: LeggedG1) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Where the Mid-360 is, in the world: the base pose with the mount on top of it."""
    position, rotation = body.base_pose()
    return position + rotation @ MID360.mount_xyz, rotation @ MID360.mount_r


def office_lidar(body: LeggedG1, seed: int = 0) -> SimLidar:
    """The G1's Mid-360 casting against the room, seeing the shell and the furniture.

    The same cast SimWorld sets up, with the attached robot's prefix on the one body name it
    needs: the shell the lens looks out of (see HOUSING_BODY). Without it the robot scans the
    inside of its own head and every frame comes back empty.
    """
    housing = mujoco.mj_name2id(
        body.model, mujoco.mjtObj.mjOBJ_BODY, f"{PREFIX}{MID360.housing_body}"
    )
    assert housing >= 0, f"no body named {PREFIX}{MID360.housing_body}"
    return MID360.lidar(MujocoRaycaster(body.model, body.data, LIDAR_GROUPS, housing), seed)
