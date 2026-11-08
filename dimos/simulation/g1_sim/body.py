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

"""The vendored G1 USD in MuJoCo, walking under its locomotion policy."""

from __future__ import annotations

from collections.abc import Callable

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.sim_msgs.Contacts import Part
from dimos.robot.unitree.g1.model import G1_USD
from dimos.simulation.g1_sim.policy import G1Policy, G1Proprioception
from dimos.simulation.go2_legged.robot import (
    BASE_ANGULAR_VELOCITY,
    BASE_LINEAR_VELOCITY,
    BASE_QUAT,
    projected_gravity,
    yaw_of,
)
from dimos.simulation.usd_spec import load_usd_spec

# the policy acts at 50 Hz
CONTROL_DT = 0.02
# the vendor MJCF's collision geom group; MuJoCo's convention in every menagerie model
COLLISION_GROUP = 3
BASE_BODY = "pelvis"
# the feet, which are what the contact report cares about
FOOT_BODIES = ("left_ankle_roll_link", "right_ankle_roll_link")
# the sole boxes, the only geoms the vendor MJCF lets touch the ground
FOOT_GEOMS = ("left_foot", "right_foot")


def g1_spec() -> mujoco.MjSpec:
    """An editable spec of the G1, read from the one USD its model is, with no ground under it."""
    spec = load_usd_spec(G1_USD)
    world = spec.worldbody
    for element in [*world.geoms, *world.lights]:
        spec.delete(element)
    _give_it_a_body_that_collides(spec)
    return spec


def _give_it_a_body_that_collides(spec: mujoco.MjSpec) -> None:
    """Switch collision on for the geoms the vendor drew as its collision volumes.

    The vendor MJCF turns collision off for the whole robot -- `contype="0" conaffinity="0"` on
    its default class, because the walking policy does not need self-collision -- and then names
    two explicit `<pair>` elements, left_foot/floor and right_foot/floor, to put the soles back
    in contact with the ground. MuJoCo's USD converter says outright that `pairs are not
    supported`, so they do not survive, and the robot falls through the floor: 0.86 m to -77 m
    in four seconds, which is 0.5 * 9.81 * 16 and nothing else.

    A pair cannot be restored here anyway: it names a floor geom that only exists once a scene
    has been built around the robot. Switching the collision geoms on instead says the same
    thing without naming anything outside the robot.

    The soles and nothing else, which is what the vendor's two pairs said. Switching on every
    group-3 geom instead was tried and measured, and it is worse: the thigh and shin capsules
    catch on each other and the gait locks up, so the robot walks **-0.12 m** in four seconds on
    a bare floor rather than 2.76 m. It survives the furniture only by being unable to move.

    The cost of leaving it at the soles is that the G1 has no body to bump into things with: in
    the procedural office scene it walks through a desk until a sole catches an edge and trips,
    about five seconds in. That is a real limitation of the vendor's collision model rather than
    of the conversion, and fixing it means collision volumes the walking policy can live with.
    """
    collidable = [geom for geom in spec.geoms if geom.group == COLLISION_GROUP]
    soles = [geom for geom in collidable if geom.name.rsplit("/", 1)[-1] in FOOT_GEOMS]
    if len(soles) != len(FOOT_GEOMS):
        raise ValueError(f"expected the G1 to have soles named {FOOT_GEOMS}, found {len(soles)}")
    for sole in soles:
        sole.contype = 1
        sole.conaffinity = 1


class LeggedG1:
    """A G1 in a compiled model, stepped one policy tick at a time from a velocity command."""

    dt = CONTROL_DT

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, policy: G1Policy) -> None:
        self.model = model
        self.data = data
        self.policy = policy
        self.trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY)
        foot_bodies = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in FOOT_BODIES
        ]
        self.feet = tuple(g for g in range(model.ngeom) if model.geom_bodyid[g] in foot_bodies)
        self.substeps = max(1, round(CONTROL_DT / model.opt.timestep))
        # The actor was trained on the model's own joint order, which is what qpos[7:] is, so
        # the order is taken from the model rather than written out and kept in step by hand.
        self._joint_names = tuple(
            model.joint(j).name
            for j in range(model.njnt)
            if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE
        )
        joints = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in self._joint_names]
        self._qpos_adr = np.array([model.jnt_qposadr[j] for j in joints])
        self._dof_adr = np.array([model.jnt_dofadr[j] for j in joints])
        self._actuator_ids = np.array(
            [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in self._joint_names]
        )
        self._target = policy.default_pose.copy()

    def reset(self, x: float, y: float, z_feet: float, yaw: float) -> None:
        """Stand in the policy's default pose with the soles on z_feet, at rest."""
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)  # type: ignore[attr-defined]
        d.qpos[0:2] = [x, y]
        d.qpos[BASE_QUAT] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        d.qpos[self._qpos_adr] = self.policy.default_pose
        mujoco.mj_forward(m, d)
        lowest = min(float(d.geom_xpos[g][2] - m.geom_rbound[g]) for g in self.feet)
        d.qpos[2] += z_feet - lowest
        mujoco.mj_forward(m, d)
        self._target = self.policy.default_pose.copy()
        self.policy.reset()

    def tick(
        self, command: NDArray[np.float64], on_substep: Callable[[], None] | None = None
    ) -> None:
        """Advance one policy tick toward a velocity command."""
        self._target = self.policy.act(self._observe(), command)
        d = self.data
        for _ in range(self.substeps):
            # The G1's actuators are position servos, so the target goes straight to ctrl --
            # unlike the M20, whose torque motors need the PD loop written out here.
            d.ctrl[self._actuator_ids] = self._target
            mujoco.mj_step(self.model, d)
            if on_substep is not None:
                on_substep()

    def _observe(self) -> G1Proprioception:
        d = self.data
        # The actor wants linear velocity in the base frame; qvel holds it in the world frame.
        rotation = d.xmat[self.trunk].reshape(3, 3)
        return G1Proprioception(
            linear_velocity=rotation.T @ d.qvel[BASE_LINEAR_VELOCITY],
            angular_velocity=d.qvel[BASE_ANGULAR_VELOCITY].copy(),
            gravity=projected_gravity(d.qpos[BASE_QUAT]),
            joint_position=d.qpos[self._qpos_adr].copy(),
            joint_velocity=d.qvel[self._dof_adr].copy(),
        )

    def base_pose(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return self.data.xpos[self.trunk].copy(), self.data.xmat[self.trunk].reshape(3, 3).copy()

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self._joint_names

    def joint_state(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Position and velocity of every joint, in the model's own order."""
        return self.data.qpos[self._qpos_adr].copy(), self.data.qvel[self._dof_adr].copy()

    def base_velocity(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Linear velocity in the world frame and angular velocity in the base frame."""
        return (
            self.data.qvel[BASE_LINEAR_VELOCITY].copy(),
            self.data.qvel[BASE_ANGULAR_VELOCITY].copy(),
        )

    def part(self, geom: int) -> Part:
        if geom in self.feet:
            return "foot"
        if self.model.geom_bodyid[geom] == self.trunk:
            return "trunk"
        return "limb"

    def yaw(self) -> float:
        return yaw_of(self.data.qpos[BASE_QUAT])

    def upright(self) -> float:
        """z of the trunk's up axis, 1 when level."""
        return float(self.data.xmat[self.trunk][8])
