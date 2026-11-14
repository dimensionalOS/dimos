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

"""The workcell model on its own: stepped by hand, so every check is deterministic."""

import math

import mujoco
import numpy as np
from numpy.typing import NDArray
import pinocchio
import pytest

from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM,
    DUAL_OPENYAM_HOME_JOINTS,
    DUAL_OPENYAM_SIDES,
    dual_openyam_arm_joints,
)
from dimos.robot.manipulators.dual_openyam.model import DUAL_OPENYAM_MODEL_PATH
from dimos.robot.manipulators.dual_openyam.sim_scene import (
    CAMERA_FOVY,
    DUAL_OPENYAM_TABLE_TOP_Z,
    FINGER_TRAVEL,
    OVERHEAD_CAMERA,
    WORKCELL_OBJECTS,
    WRIST_CAMERAS,
    dual_openyam_workcell,
)

pytestmark = pytest.mark.self_hosted

# How high each object's body frame rests above the table top, metres.
REST_HEIGHT = {"cup": 0.05, "bottle": 0.0, "rubiks_cube": 0.0285}
# How wide each object is where the jaws close on it, metres.
WIDTH = {"cup": 0.08, "rubiks_cube": 0.057}
# The MIT torque range of each Damiao motor type, Nm.
MOTOR_TORQUE = {"DM4340": 28.0, "DM4310": 10.0}
HOME = np.array(DUAL_OPENYAM_HOME_JOINTS[:6])
IMAGE_WIDTH, IMAGE_HEIGHT = 640, 480


class Workcell:
    """A compiled workcell at the home posture, with what the checks keep looking up."""

    def __init__(self, spec: mujoco.MjSpec) -> None:
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.arm = {
            side: [self.model.joint(name) for name in dual_openyam_arm_joints(side)]
            for side in DUAL_OPENYAM_SIDES
        }
        for side in DUAL_OPENYAM_SIDES:
            for joint, position in zip(self.arm[side], HOME, strict=True):
                self.data.qpos[joint.qposadr[0]] = position
            self.command_arm(side, HOME)
        mujoco.mj_forward(self.model, self.data)

    def arm_positions(self, side: str) -> NDArray[np.float64]:
        return np.array([self.data.qpos[joint.qposadr[0]] for joint in self.arm[side]])

    def command_arm(self, side: str, positions: NDArray[np.float64]) -> None:
        for joint, position in zip(self.arm[side], positions, strict=True):
            self.data.ctrl[self.model.actuator(joint.name).id] = position

    def command_jaw(self, side: str, opening: float) -> None:
        """0 closes the jaw and 1 opens it fully."""
        self.data.ctrl[self.model.actuator(f"{side}_gripper").id] = -FINGER_TRAVEL * opening

    def jaw_gap(self, side: str) -> float:
        """How far apart the two fingers' gripping faces are, metres."""
        slides = [self.model.joint(f"{side}_tip_{jaw}_slide") for jaw in ("left", "right")]
        return -float(sum(self.data.qpos[slide.qposadr[0]] for slide in slides))

    def step(self, seconds: float) -> None:
        for _ in range(round(seconds / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)

    def move_arm(self, side: str, goal: NDArray[np.float64], seconds: float) -> None:
        """Command a smooth joint-space move and step through it."""
        start = self.arm_positions(side)
        steps = round(seconds / self.model.opt.timestep)
        for step in range(1, steps + 1):
            blend = step / steps
            self.command_arm(side, start + (goal - start) * blend * blend * (3 - 2 * blend))
            mujoco.mj_step(self.model, self.data)
        self.step(0.5)

    def contacts(self) -> set[frozenset[str]]:
        """The pairs of bodies in contact now."""
        return {
            frozenset(
                self.model.body(self.model.geom_bodyid[geom]).name
                for geom in (contact.geom1, contact.geom2)
            )
            for contact in self.data.contact[: self.data.ncon]
        }

    def fingers_down_over(
        self,
        side: str,
        xy: NDArray[np.float64],
        height: float,
        yaw: float,
        seed: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Arm joints that hold the grasp frame ``height`` above the table at ``xy``, fingers down.

        Damped least squares from ``seed``. ``yaw`` turns the gripper about
        the vertical; at pi the jaws close along the world's y.
        """
        model, scratch = self.model, mujoco.MjData(self.model)
        frame = model.body(f"{side}_grasp_frame").id
        qpos = [joint.qposadr[0] for joint in self.arm[side]]
        scratch.qpos[qpos] = seed
        dofs = [joint.dofadr[0] for joint in self.arm[side]]
        limits = np.array([joint.range for joint in self.arm[side]])
        goal = np.array([*xy, DUAL_OPENYAM_TABLE_TOP_Z + height])
        upright = np.array(
            [[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0, 0, 1]]
        )
        jac_pos, jac_rot = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        quat, turn = np.zeros(4), np.zeros(3)
        for _ in range(500):
            mujoco.mj_kinematics(model, scratch)
            mujoco.mj_comPos(model, scratch)
            offset = goal - scratch.xpos[frame]
            mujoco.mju_mat2Quat(quat, (upright @ scratch.xmat[frame].reshape(3, 3).T).ravel())
            mujoco.mju_quat2Vel(turn, quat, 1.0)
            if np.linalg.norm(offset) < 1e-4 and np.linalg.norm(turn) < 1e-3:
                return scratch.qpos[qpos].copy()
            mujoco.mj_jacBody(model, scratch, jac_pos, jac_rot, frame)
            jacobian = np.vstack([jac_pos[:, dofs], jac_rot[:, dofs]])
            step = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + 1e-4 * np.eye(6), np.concatenate([offset, turn])
            )
            scratch.qpos[qpos] = np.clip(
                scratch.qpos[qpos] + np.clip(step, -0.2, 0.2), limits[:, 0], limits[:, 1]
            )
        raise AssertionError(f"the {side} arm cannot reach {goal} with its fingers down")


@pytest.fixture
def workcell() -> Workcell:
    return Workcell(dual_openyam_workcell(DUAL_OPENYAM))


def test_actuators_are_the_robots_joints_with_its_gains_and_torque_ranges(
    workcell: Workcell,
) -> None:
    model = workcell.model
    assert [model.actuator(i).name for i in range(model.nu)] == DUAL_OPENYAM.joint_names
    for bus in DUAL_OPENYAM.buses:
        for motor, kp, kd in zip(bus.arm, bus.kp, bus.kd, strict=True):
            actuator = model.actuator(motor.name)
            assert actuator.gainprm[0] == kp
            assert list(actuator.biasprm[:3]) == [0.0, -kp, -kd]
            assert list(actuator.ctrlrange) == list(model.joint(motor.name).range)
            assert list(actuator.forcerange) == [
                -MOTOR_TORQUE[motor.type],
                MOTOR_TORQUE[motor.type],
            ]


def test_grasp_frames_are_where_the_planning_model_puts_them(workcell: Workcell) -> None:
    planning = pinocchio.buildModelFromUrdf(str(DUAL_OPENYAM_MODEL_PATH))
    placements = planning.createData()
    pinocchio.framesForwardKinematics(planning, placements, np.array(DUAL_OPENYAM_HOME_JOINTS))

    for side in DUAL_OPENYAM_SIDES:
        planned = placements.oMf[planning.getFrameId(f"{side}_grasp_frame")]
        simulated = workcell.data.body(f"{side}_grasp_frame")
        assert simulated.xpos == pytest.approx(planned.translation, abs=1e-6)
        assert simulated.xmat.reshape(3, 3) == pytest.approx(planned.rotation, abs=1e-6)


def test_home_posture_touches_nothing_and_holds(workcell: Workcell) -> None:
    # The table is part of the world body; nothing else may be in contact.
    resting = {frozenset({"world", name}) for name in WORKCELL_OBJECTS}

    assert workcell.contacts() <= resting
    workcell.step(2.0)

    assert workcell.contacts() == resting
    for side in DUAL_OPENYAM_SIDES:
        assert workcell.arm_positions(side) == pytest.approx(HOME, abs=1e-4)


def test_no_two_links_of_an_arm_overlap_at_home() -> None:
    # The workcell does not collide an arm with itself, so check its links here:
    # let every link hull collide with every other and look at the home posture.
    spec = dual_openyam_workcell(DUAL_OPENYAM)
    for body in spec.bodies:
        for geom in body.geoms:
            if body.name.startswith(DUAL_OPENYAM_SIDES) and geom.meshname and geom.contype:
                geom.contype = geom.conaffinity = 1
    everything_collides = Workcell(spec)

    touching = {
        pair for pair in everything_collides.contacts() if not pair & {"world", *WORKCELL_OBJECTS}
    }

    # MuJoCo skips a link and its parent, except under a fixed base: link1 sits
    # on its base at every posture.
    assert touching == {frozenset({f"{side}_base", f"{side}_link1"}) for side in DUAL_OPENYAM_SIDES}


def test_objects_rest_on_the_table_in_the_overhead_cameras_view(workcell: Workcell) -> None:
    model, data = workcell.model, workcell.data
    start = {name: data.body(name).xpos.copy() for name in WORKCELL_OBJECTS}

    workcell.step(5.0)

    camera = model.camera(OVERHEAD_CAMERA).id
    focal = IMAGE_HEIGHT / 2 / math.tan(math.radians(CAMERA_FOVY) / 2)
    for name in WORKCELL_OBJECTS:
        position = data.body(name).xpos
        assert position[2] == pytest.approx(DUAL_OPENYAM_TABLE_TOP_Z + REST_HEIGHT[name], abs=5e-4)
        assert position[:2] == pytest.approx(start[name][:2], abs=5e-4)
        assert np.linalg.norm(data.body(name).cvel) < 0.01
        # In the camera's frame x is right, y is up and it looks along -z.
        seen = (position - data.cam_xpos[camera]) @ data.cam_xmat[camera].reshape(3, 3)
        column = IMAGE_WIDTH / 2 + focal * seen[0] / -seen[2]
        row = IMAGE_HEIGHT / 2 - focal * seen[1] / -seen[2]
        assert 40 < column < IMAGE_WIDTH - 40
        assert 40 < row < IMAGE_HEIGHT - 40


def test_wrist_cameras_ride_their_grippers(workcell: Workcell) -> None:
    model, data = workcell.model, workcell.data
    for side in DUAL_OPENYAM_SIDES:
        camera = model.camera(WRIST_CAMERAS[side])
        assert model.body(camera.bodyid[0]).name == f"{side}_gripper"
        # It looks along the fingers: the grasp frame is ahead of the lens.
        ahead = data.body(f"{side}_grasp_frame").xpos - data.cam_xpos[camera.id]
        assert ahead @ -data.cam_xmat[camera.id].reshape(3, 3)[:, 2] > 0.03


@pytest.mark.parametrize("side", DUAL_OPENYAM_SIDES)
def test_jaw_opens_to_the_rigs_width_and_closes(workcell: Workcell, side: str) -> None:
    assert workcell.jaw_gap(side) == pytest.approx(0.0, abs=1e-4)

    workcell.command_jaw(side, 1.0)
    workcell.step(1.5)
    assert workcell.jaw_gap(side) == pytest.approx(0.098, abs=5e-4)

    workcell.command_jaw(side, 0.0)
    workcell.step(1.5)
    assert workcell.jaw_gap(side) == pytest.approx(0.0, abs=5e-4)


@pytest.mark.parametrize(
    ("side", "name", "yaw", "grasp_height"),
    [
        # The cube stands turned 30 degrees, so the jaws close across two of its faces.
        ("right", "rubiks_cube", math.radians(210.0), 0.060),
        ("left", "cup", math.pi, 0.105),
    ],
)
def test_arm_picks_an_object_from_above(
    workcell: Workcell, side: str, name: str, yaw: float, grasp_height: float
) -> None:
    data = workcell.data
    start = data.body(name).xpos.copy()
    above = workcell.fingers_down_over(side, start[:2], 0.16, yaw, seed=HOME)
    around = workcell.fingers_down_over(side, start[:2], grasp_height, yaw, seed=above)

    workcell.command_jaw(side, 1.0)
    workcell.move_arm(side, above, 2.0)
    workcell.move_arm(side, around, 1.5)
    assert data.body(name).xpos == pytest.approx(start, abs=1e-3)
    workcell.command_jaw(side, 0.0)
    workcell.step(1.0)
    # The jaws stop on the object instead of closing through it.
    assert workcell.jaw_gap(side) == pytest.approx(WIDTH[name], abs=1.5e-3)
    workcell.move_arm(side, above, 1.5)
    lifted = data.body(name).xpos.copy()
    workcell.step(5.0)

    assert lifted[2] - start[2] > 0.9 * (0.16 - grasp_height)
    assert data.body(name).xpos == pytest.approx(lifted, abs=1e-3)
