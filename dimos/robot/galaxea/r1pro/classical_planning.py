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

"""Collision-checked whole-body reachability for actual GraspGen TCP poses.

The same DimOS IK is used for candidate assessment and Cartesian execution.
All kinematic object attachments here live only in the planning snapshot.
"""

from collections import Counter
from dataclasses import dataclass, replace
from itertools import islice, pairwise
import time
from typing import Any, cast

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation, Slerp

from dimos.manipulation.planning.groups.models import PlanningGroupSelection
from dimos.manipulation.planning.planners.rrt_planner import RRTConnectPlanner
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.home_kinematics import HomeKinematics, bounded_joint_positions
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.object_primitive_state import PrimitiveSceneState
from dimos.robot.galaxea.r1pro.object_primitives import ARMS, Arm, active_indices
from dimos.robot.galaxea.r1pro.object_reachability import ObjectReachability, stance_candidates
from dimos.robot.galaxea.r1pro.posture_ik import (
    NOMINAL_POSTURE,
    POSTURE_RANK_WEIGHT,
    posture_error,
    posture_is_valid,
)
from dimos.utils.logging_config import setup_logger

GRASP_FEASIBLE_STANCES = 3
CARRY_PLANNING_SECONDS = 20.0
CARRY_HOME_TOLERANCE = 0.03
CARRY_MIN_ELBOW_FLEXION = np.deg2rad(45)
logger = setup_logger()


def preserves_cargo_tilt(initial_up_z: float, current_up_z: float) -> bool:
    """Keep a verified upright hold within 2 degrees of its initial tilt, capped at 15."""
    initial_tilt = float(np.arccos(np.clip(initial_up_z, -1, 1)))
    limit = min(np.deg2rad(15), max(np.deg2rad(5), initial_tilt + np.deg2rad(2)))
    return bool(current_up_z >= np.cos(limit))


def carry_ready_joints(joints: NDArray[Any], home: NDArray[Any], loaded: set[Arm]) -> bool:
    """Ready means home torso/empty arms and bent loaded elbows, not just reachable."""
    if not posture_is_valid(joints):
        return False
    columns = list(range(4))
    for side in ARMS:
        arm_columns = list(active_indices(side))[:-1]
        if side not in loaded:
            columns.extend(arm_columns)
        elif joints[arm_columns[3]] > -CARRY_MIN_ELBOW_FLEXION:
            return False
    return bool(np.max(np.abs(joints[columns] - home[columns])) <= CARRY_HOME_TOLERANCE)


def carry_yaw(initial: NDArray[Any], home: NDArray[Any]) -> float:
    """Nearest home wrist heading obtainable by rotating only about gravity."""
    relative = home @ initial.T
    return float(np.arctan2(relative[1, 0] - relative[0, 1], relative[0, 0] + relative[1, 1]))


@dataclass(frozen=True)
class ClassicalGraspPlan:
    arm: Arm
    index: int
    base_pose: list[float]
    source_position: list[float]
    tcp: list[list[float]]
    pregrasp: list[list[float]]
    ready_joints: list[float]
    grasp_score: float
    joint_margin: float
    manipulability: float
    cost: float
    refinement: str = "graspgenx"
    posture_error: float = 0.0


class ClassicalGraspPlanner(ObjectReachability):
    allow_target_contact: bool = False
    sweep_error: str = ""

    def __init__(
        self, scene: PrimitiveSceneState, *, kinematics: HomeKinematics | None = None
    ) -> None:
        kinematics = kinematics or HomeKinematics(scene.model, scene.data, natural_posture=True)
        if not kinematics.natural_posture:
            raise ValueError("Classical grasp planning requires the R1Pro posture policy")
        super().__init__(scene, kinematics=kinematics)

    def _joint_margin(self, arm: Arm, joints: NDArray[Any]) -> float:
        margins = []
        for column in active_indices(arm)[:-1]:
            lower, upper = self.model.joint(R1PRO_PICK_PLACE_JOINTS[column]).range
            margins.append(min(joints[column] - lower, upper - joints[column]) / (upper - lower))
        return float(min(margins))

    def _joint_clearance(self, arm: Arm, joints: NDArray[Any]) -> float:
        """Minimum distance to an arm stop, in radians rather than range percentage."""
        return min(
            float(min(joints[column] - limits[0], limits[1] - joints[column]))
            for column in active_indices(arm)[:-1]
            for limits in [self.model.joint(R1PRO_PICK_PLACE_JOINTS[column]).range]
        )

    def _body_poses(self, target: NDArray[Any], arm: Arm) -> list[NDArray[np.float64]]:
        """Try the current stance, then center the target in the arm's working area."""
        current = self.transport.start
        candidates = stance_candidates(target, current, arm)
        preferred = np.array([0.42, 0.32 if arm == "left" else -0.32])

        def cost(pose: NDArray[Any]) -> float:
            c, s = np.cos(pose[2]), np.sin(pose[2])
            local = np.array([[c, s], [-s, c]]) @ (target[:2] - pose[:2])
            return float(
                np.linalg.norm(local - preferred)
                + 0.15 * np.linalg.norm(pose[:2] - current[:2])
                + 0.03 * abs(pose[2] - current[2])
            )

        result = [current.copy()]
        for pose in sorted(candidates, key=cost):
            if any(
                np.linalg.norm(pose[:2] - old[:2]) < 0.02 and abs(pose[2] - old[2]) < 0.02
                for old in result
            ):
                continue
            result.append(pose)
        # A target across the body from the free hand is a slow, awkward reach
        # from where the robot stands. Reposition first; keep that stance last.
        c, s = np.cos(current[2]), np.sin(current[2])
        lateral = float(-s * (target[0] - current[0]) + c * (target[1] - current[1]))
        if (arm == "left" and lateral < -0.05) or (arm == "right" and lateral > 0.05):
            result = result[1:] + result[:1]
        return result

    def _collisions(
        self, *, selected: int, arm: Arm, allow_selected_contact: bool = False
    ) -> list[str]:
        return super()._collisions(
            selected=selected,
            arm=arm,
            allow_selected_contact=allow_selected_contact or self.allow_target_contact,
        )

    def _sweep(self, goal: NDArray[Any], selected: int, arm: Arm) -> bool:
        """Preserve existing cargo tilt; do not reject an unchanged, verified hold."""
        start = self.probe.qpos[self.qids].copy()
        count = max(2, int(np.ceil(np.max(np.abs(goal - start)) / 0.025)))
        for t in np.linspace(0, 1, count + 1):
            self.probe.qpos[self.qids] = start + t * (goal - start)
            if not posture_is_valid(self.probe.qpos[self.qids]):
                self.sweep_error = "R1Pro posture envelope violated"
                return False
            self._forward()
            collisions = self._collisions(selected=selected, arm=arm)
            if collisions:
                self.sweep_error = f"contacts {collisions}"
                return False
            for index in self.attachments:
                name = self.scene.layout.objects[index].name
                if not preserves_cargo_tilt(
                    self.initial.body(name).xmat[8], self.probe.body(name).xmat[8]
                ):
                    self.sweep_error = f"cargo tilt changed for {name}"
                    return False
        return True

    def solve_pose(
        self,
        arm: Arm,
        tcp: NDArray[Any],
        *,
        torso: bool = False,
        preserve_other: bool = False,
        neutral_seed: bool = False,
    ) -> NDArray[np.float64]:
        base = self.probe.body("base_link")
        transform = self.reference_rotation @ base.xmat.reshape(3, 3).T
        targets: dict[str, NDArray[Any]] = {
            arm: self.reference_position + transform @ (tcp[:3, 3] - base.xpos)
        }
        orientations: dict[str, Quaternion] = {
            arm: Quaternion.from_rotation_matrix(transform @ tcp[:3, :3])
        }
        other: Arm = "right" if arm == "left" else "left"
        if torso and (preserve_other or any(row["held_by"] == other for row in self.rows)):
            site = self.probe.site(f"{other}_tcp")
            targets[other] = self.reference_position + transform @ (site.xpos - base.xpos)
            orientations[other] = Quaternion.from_rotation_matrix(
                transform @ site.xmat.reshape(3, 3)
            )
        seed = None
        if neutral_seed:
            # Start with the bounded measured seed. Raw measurements in an
            # already-loaded wrist can sit just outside the conservative IK
            # margin and reject every candidate for the other hand.
            measured_seed = self.kinematics.seed(self.probe)
            positions = dict(zip(measured_seed.name, measured_seed.position, strict=True))
            for column in active_indices(arm)[:-1]:
                positions[R1PRO_PICK_PLACE_JOINTS[column]] = float(NOMINAL_POSTURE[column])
            seed = JointState(
                name=measured_seed.name,
                position=[positions[name] for name in measured_seed.name],
            )
        result = self.kinematics.solve(
            self.probe,
            targets,
            orientations=orientations,
            allow_torso=torso,
            position_tolerance=0.0002,
            orientation_tolerance=0.008,
            max_attempts=1,
            seed=seed,
        )
        result[-2:] = self.probe.qpos[self.qids[-2:]]
        return result

    def segment(self, arm: Arm, index: int, target: NDArray[Any]) -> list[list[float]]:
        """Solve and sweep a fixed-orientation line from the measured TCP, including its start."""
        site = self.probe.site(f"{arm}_tcp")
        start = site.xpos.copy()
        if np.linalg.norm(site.xmat.reshape(3, 3) - target[:3, :3]) > 0.03:
            raise RuntimeError("Cartesian approach requires the staged grasp orientation")
        # Plan collisions at measured finger positions, but retain the actuator
        # closure targets in execution. Commanding the measured contact width
        # would remove the grip force as soon as a new arm segment starts.
        points = [self._arm_command(self.probe.qpos[self.qids], arm)]
        whole_body = False
        steps = max(2, int(np.ceil(np.linalg.norm(target[:3, 3] - start) / 0.004)))
        for t in np.linspace(0, 1, steps + 1)[1:]:
            pose = target.copy()
            pose[:3, 3] = start + t * (target[:3, 3] - start)
            before = self.probe.qpos[self.qids].copy()
            try:
                goal = self.solve_pose(arm, pose)
                if np.max(np.abs(goal - before)) > 0.20:
                    raise RuntimeError("Arm-only Cartesian IK changes posture discontinuously")
            except RuntimeError:
                # A converged arm-only solution can jump to a different joint
                # posture near a singularity. Try torso assistance for that
                # case as well as nonconvergence, preserving the other hand.
                goal = self.solve_pose(arm, pose, torso=True, preserve_other=True)
                whole_body = True
            if np.max(np.abs(goal - before)) > 0.20:
                raise RuntimeError("Cartesian sweep has discontinuous IK")
            if not self._sweep(goal, index, arm):
                raise RuntimeError(f"Cartesian sweep rejected: {self.sweep_error}")
            points.append(self._arm_command(goal, arm, whole_body=whole_body))
        return points

    def _arm_command(
        self, joints: NDArray[Any], arm: Arm, *, whole_body: bool = False
    ) -> list[float]:
        """Keep static load compensation on moving joints and targets on stationary joints."""
        command = [
            float(self.scene.data.actuator(name).ctrl[0]) for name in R1PRO_PICK_PLACE_JOINTS
        ]
        columns = range(18) if whole_body else active_indices(arm)[:-1]
        for column in columns:
            measured = float(self.scene.data.qpos[self.qids[column]])
            bias = command[column] - measured
            if abs(bias) > 0.03:
                raise RuntimeError(
                    "Arm has not settled enough to estimate static load compensation"
                )
            command[column] = float(joints[column]) + bias
        return command

    def align_pose(self, index: int, arm: Arm, target: NDArray[Any]) -> list[list[float]]:
        """Correct measured TCP error while retaining each joint's static load compensation."""
        self.initialize_local_probe(index, arm)
        before = self.probe.qpos[self.qids].copy()
        try:
            goal = self.solve_pose(arm, target)
        except RuntimeError:
            goal = self.solve_pose(arm, target, torso=True, preserve_other=True)
        if np.max(np.abs(goal - before)) > 0.20 or not self._sweep(goal, index, arm):
            raise RuntimeError("Staging correction exceeds the local collision-checked range")
        command = np.array(
            [float(self.scene.data.actuator(name).ctrl[0]) for name in R1PRO_PICK_PLACE_JOINTS]
        )
        corrected = command.copy()
        corrected[:18] += (goal - before)[:18]
        return [command.tolist(), corrected.tolist()]

    def _check_closure(self, index: int, arm: Arm) -> None:
        """Require opposed finger-pad contacts at one attainable jaw opening."""
        pads = self.scene.arms[arm].pad_ids
        body = self.model.body(self.scene.layout.objects[index].name).id
        driver = self.model.joint(f"r1pro/{arm}_gripper").qposadr[0]
        for opening in np.linspace(0.05, 0, 101):
            self.probe.qpos[driver] = opening
            self._forward()
            touching = set()
            for contact in self.probe.contact:
                if not -0.002 <= contact.dist <= 0.0005:
                    continue
                a, b = map(int, contact.geom)
                if a in pads and self.model.geom_bodyid[b] == body:
                    touching.add(a)
                if b in pads and self.model.geom_bodyid[a] == body:
                    touching.add(b)
            if touching == pads and not self._collisions(selected=index, arm=arm):
                return
        raise RuntimeError("Generated grasp cannot establish two-pad contact at a valid opening")

    def initialize_probe(self, pose: NDArray[Any]) -> None:
        self.allow_target_contact = False
        if not self.transport.clear_pose_segment(pose, pose):
            raise RuntimeError("Candidate body pose is obstructed")
        self.probe.qpos[:] = self.transport.probe.qpos
        mujoco.mj_forward(self.model, self.probe)
        self.attachments.clear()
        for i, row in enumerate(self.rows):
            if row["held_by"]:
                self._attach(i, row["held_by"])

    def initialize_local_probe(self, index: int, arm: Arm) -> None:
        """Preserve measured supported grasps during the first lift, without base motion."""
        self.probe.qpos[:] = self.initial.qpos
        mujoco.mj_forward(self.model, self.probe)
        self.attachments.clear()
        for i, row in enumerate(self.rows):
            if row["held_by"]:
                self._attach(i, row["held_by"])
            elif i == index and row["grasped"] and arm in row["contacting_arms"]:
                # Before lift the table still supports the object. Two-pad
                # contact, rather than unsupported ownership, authorizes this
                # planning attachment. Live physics must prove the actual lift.
                self._attach(i, arm)
        # A generic free-arm move may use an index held by the other hand.
        # Do not replace that measured contact owner with the moving hand.
        self.allow_target_contact = self.rows[index]["held_by"] in (None, arm)
        if self._collisions(selected=index, arm=arm):
            raise RuntimeError("Measured local manipulation start is obstructed")

    def evaluate_grasp(
        self, index: int, arm: Arm, tcp: NDArray[Any], pose: NDArray[Any], confidence: float
    ) -> ClassicalGraspPlan:
        if self.rows[index]["held_by"] or any(row["held_by"] == arm for row in self.rows):
            raise ValueError("A pick requires a free hand and an unheld object")
        if np.linalg.norm(np.asarray(self.rows[index]["position"])[:2] - pose[:2]) > 0.85:
            raise RuntimeError("Target outside bounded arm reach search")
        pregrasp = tcp.copy()
        pregrasp[:3, 3] += 0.10 * tcp[:3, 2]
        failures = []
        for torso in (False, True):
            self.initialize_probe(pose)
            try:
                ready = self.solve_pose(arm, pregrasp, torso=torso, neutral_seed=True)
                self.probe.qpos[self.qids] = ready
                self._forward()
                if self._collisions(selected=index, arm=arm):
                    raise RuntimeError("Pregrasp is obstructed")
                # Only finger pads may touch the selected object during approach.
                self.allow_target_contact = True
                self.segment(arm, index, tcp)
                self._check_closure(index, arm)
                posture = max(
                    posture_error(ready, arm), posture_error(self.probe.qpos[self.qids], arm)
                )
                columns = list(active_indices(arm))[:-1]
                joints = self.probe.qpos[self.qids[columns]]
                lower = np.array(
                    [
                        self.model.joint(self.kinematics.config.joint_names[i]).range[0]
                        for i in columns
                    ]
                )
                upper = np.array(
                    [
                        self.model.joint(self.kinematics.config.joint_names[i]).range[1]
                        for i in columns
                    ]
                )
                margin = float(np.min(np.minimum(joints - lower, upper - joints) / (upper - lower)))
                jac = np.zeros((3, self.model.nv))
                mujoco.mj_jacSite(  # type: ignore[attr-defined]
                    self.model, self.probe, jac, None, self.model.site(f"{arm}_tcp").id
                )
                dofs = [
                    self.model.joint(self.kinematics.config.joint_names[i]).dofadr[0]
                    for i in columns
                ]
                manipulability = float(np.prod(np.linalg.svd(jac[:, dofs], compute_uv=False)))
                self._attach(index, arm)
                lifted = tcp.copy()
                lifted[:3, 3] += [0, 0, 0.12]
                self.segment(arm, index, lifted)
                posture = max(posture, posture_error(self.probe.qpos[self.qids], arm))
                distance = float(np.linalg.norm(pose[:2] - self.transport.start[:2]))
                yaw = abs(float(pose[2] - self.transport.start[2]))
                cost = (
                    distance
                    + 0.15 * yaw
                    + 0.3 * (1 - confidence)
                    - 0.5 * margin
                    - manipulability
                    + POSTURE_RANK_WEIGHT * posture
                )
                return ClassicalGraspPlan(
                    arm,
                    index,
                    pose.tolist(),
                    list(self.rows[index]["position"]),
                    tcp.tolist(),
                    pregrasp.tolist(),
                    ready.tolist(),
                    confidence,
                    margin,
                    manipulability,
                    cost,
                    posture_error=posture,
                )
            except RuntimeError as exc:
                failures.append(str(exc))
        raise RuntimeError("; ".join(failures))

    def evaluate_place(
        self,
        index: int,
        arm: Arm,
        target: NDArray[Any],
        pose: NDArray[Any],
        *,
        yaw_offset: float = 0,
    ) -> dict[str, Any]:
        """Prove approach, support contact, finger opening and retreat at a candidate stance."""
        if self.rows[index]["held_by"] != arm:
            raise ValueError("Placement must use the holding hand")
        if np.linalg.norm(target[:2] - pose[:2]) > 0.85:
            raise RuntimeError("Placement outside bounded arm reach search")
        errors = []
        for torso, neutral_seed in ((False, False), (True, False), (True, True)):
            self.initialize_probe(pose)
            site = self.probe.site(f"{arm}_tcp")
            c, s = np.cos(yaw_offset), np.sin(yaw_offset)
            turn = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            tcp = np.eye(4)
            tcp[:3, :3] = turn @ site.xmat.reshape(3, 3)
            tcp[:3, 3] = target + turn @ (
                site.xpos - self.probe.body(self.scene.layout.objects[index].name).xpos
            )
            above = tcp.copy()
            above[2, 3] = max(
                tcp[2, 3] + 0.10,
                target[2] + self.scene.layout.objects[index].half_size[2] + 0.05,
            )
            try:
                ready = self.solve_pose(arm, above, torso=torso, neutral_seed=neutral_seed)
                margin = self._joint_margin(arm, ready)
                clearance = self._joint_clearance(arm, ready)
                # The joint ranges differ substantially. Requiring 4% of each
                # range rejected poses with >10 degrees of available motion.
                # Keep at least 0.06 rad away from the physical stop (twice the
                # maximum accepted load-compensation bias) on every arm joint.
                if clearance < 0.06:
                    raise RuntimeError("Placement posture leaves insufficient joint-limit margin")
                self.probe.qpos[self.qids] = ready
                self._forward()
                self.allow_target_contact = True
                if self._collisions(selected=index, arm=arm):
                    raise RuntimeError("Preplace is obstructed")
                self.segment(arm, index, tcp)
                self.attachments.pop(index)
                self.probe.qpos[self.qids[18 if arm == "left" else 19]] = 0.05
                self._forward()
                if self._collisions(selected=index, arm=arm):
                    raise RuntimeError("The hand cannot open at this supported spot")
                self.segment(arm, index, above)
                cost = float(
                    np.linalg.norm(pose[:2] - self.transport.start[:2])
                    + 0.15 * abs(pose[2] - self.transport.start[2])
                    - 0.5 * margin
                    + 0.02 * abs(yaw_offset)
                )
                return dict(
                    index=index,
                    arm=arm,
                    target=target.tolist(),
                    base_pose=pose.tolist(),
                    tcp=tcp.tolist(),
                    preplace=above.tolist(),
                    ready_joints=ready.tolist(),
                    joint_margin=margin,
                    joint_clearance_rad=clearance,
                    yaw_offset=yaw_offset,
                    cost=cost,
                )
            except RuntimeError as exc:
                errors.append(str(exc))
        raise RuntimeError("; ".join(errors))

    def rank_places(
        self, index: int, arm: Arm, targets: list[NDArray[Any]], *, seconds: float = 60
    ) -> list[dict[str, Any]]:
        deadline = time.monotonic() + seconds
        results: list[dict[str, Any]] = []
        # Prefer a nearby empty spot, but examine multiple support locations and
        # body poses before deciding the region is unreachable.
        ordered = sorted(
            targets,
            key=lambda p: float(np.linalg.norm(p[:2] - self.initial.site(f"{arm}_tcp").xpos[:2])),
        )
        for target in ordered[:24]:
            # Nearby ideal reach poses can be inside a counter or cabinet.
            # Obstructed poses must not consume the finite IK search budget.
            clear_poses = (
                pose
                for pose in self._body_poses(target, arm)
                if self.transport.clear_pose_segment(pose, pose)
            )
            for pose in islice(clear_poses, 8):
                # Placement may turn an upright item about gravity. A fixed
                # wrist heading can otherwise force a joint against its stop.
                for yaw in (0.0, -np.pi / 2, np.pi / 2, np.pi):
                    if time.monotonic() > deadline:
                        if not results:
                            raise RuntimeError(
                                f"Placement search exhausted its {seconds:g}s budget; "
                                "the region has not been proved unreachable"
                            )
                        return sorted(results, key=lambda row: row["cost"])
                    try:
                        candidate = self.evaluate_place(index, arm, target, pose, yaw_offset=yaw)
                        # A reachable preplace pose can still require an
                        # impossible upright transfer from the current grasp.
                        # Check that transfer before selecting this body pose.
                        self.initialize_probe(pose)
                        staged = self.scene.snapshot()
                        staged.data.qpos[:] = self.probe.qpos
                        mujoco.mj_forward(self.model, staged.data)
                        transfer = ClassicalGraspPlanner(staged).transfer_path(
                            index, arm, np.asarray(candidate["preplace"])
                        )
                        candidate["staging_waypoints"] = len(transfer)
                        results.append(candidate)
                    except (ValueError, RuntimeError):
                        continue
                    if len(results) >= 3:
                        return sorted(results, key=lambda row: row["cost"])
        return sorted(results, key=lambda row: row["cost"])

    def posture_path(
        self,
        index: int,
        arm: Arm,
        positions: list[float],
        *,
        allow_target_contact: bool = True,
    ) -> list[list[float]]:
        """Use the SDK RRT with actual apartment/cargo collisions during its search."""
        self.initialize_local_probe(index, arm)
        self.allow_target_contact = allow_target_contact
        kin = self.kinematics
        start = kin.seed(self.probe)
        mapping = dict(zip(R1PRO_PICK_PLACE_JOINTS, positions, strict=True))
        goal = JointState(
            name=start.name,
            position=bounded_joint_positions(
                np.array([mapping[name] for name in start.name]), kin.lower, kin.upper
            ).tolist(),
        )
        selection = PlanningGroupSelection.from_groups(
            tuple(kin.groups[name] for name in ("torso", "left_arm", "right_arm"))
        )
        checked_world = _ApartmentCollisionWorld(self, index, arm)
        result = RRTConnectPlanner().plan_selected_joint_path(
            cast("WorldSpec", checked_world),
            selection,
            start,
            goal,
            timeout=20.0,
            max_iterations=5000,
        )
        if not result.is_success() or not result.path:
            raise RuntimeError(
                f"DimOS apartment posture planning failed: {result.status}: {result.message}"
            )
        grippers = [
            float(self.scene.data.actuator(f"r1pro/{side}_gripper").ctrl[0])
            for side in ("left", "right")
        ]
        points = []
        for state in result.path:
            values = dict(zip(state.name, state.position, strict=True))
            points.append([values[name] for name in R1PRO_PICK_PLACE_JOINTS[:18]] + grippers)
        return points

    def transfer_path(
        self,
        index: int,
        arm: Arm,
        target: NDArray[Any],
        *,
        allow_target_contact: bool = True,
    ) -> list[list[float]]:
        """Stage held cargo with Cartesian IK, preserving its attitude and the other hand.

        An unconstrained joint-space search struggles with the narrow set of
        upright carrying configurations. Try direct and raised TCP corridors;
        every joint edge still passes apartment, self and cargo collision checks.
        """
        failures = []
        for clearance in (0.0, 0.06, 0.12):
            for torso in (False, True):
                self.initialize_local_probe(index, arm)
                self.allow_target_contact = allow_target_contact and self.rows[index][
                    "held_by"
                ] in (None, arm)
                site = self.probe.site(f"{arm}_tcp")
                start = np.eye(4)
                start[:3, :3] = site.xmat.reshape(3, 3)
                start[:3, 3] = site.xpos
                command = np.array(
                    [float(self.scene.data.actuator(n).ctrl[0]) for n in R1PRO_PICK_PLACE_JOINTS]
                )
                bias = command[:18] - self.initial.qpos[self.qids[:18]]
                if np.max(np.abs(bias)) > 0.03:
                    raise RuntimeError("Cargo must settle before planning a transfer")
                points = [command.tolist()]
                raised_start, raised_target = start.copy(), target.copy()
                height = max(start[2, 3], target[2, 3]) + clearance
                raised_start[2, 3] = raised_target[2, 3] = height
                corridor = (
                    [start, target]
                    if not clearance
                    else [start, raised_start, raised_target, target]
                )
                try:
                    for a, b in pairwise(corridor):
                        rotations = Slerp([0, 1], Rotation.from_matrix([a[:3, :3], b[:3, :3]]))
                        angle = (
                            Rotation.from_matrix(a[:3, :3]).inv() * Rotation.from_matrix(b[:3, :3])
                        ).magnitude()
                        count = max(
                            2,
                            int(np.ceil(np.linalg.norm(b[:3, 3] - a[:3, 3]) / 0.01)),
                            int(np.ceil(angle / 0.05)),
                        )
                        for t in np.linspace(0, 1, count + 1)[1:]:
                            pose = np.eye(4)
                            pose[:3, :3] = rotations(t).as_matrix()
                            pose[:3, 3] = a[:3, 3] + t * (b[:3, 3] - a[:3, 3])
                            before = self.probe.qpos[self.qids].copy()
                            goal = self.solve_pose(arm, pose, torso=torso, preserve_other=True)
                            if np.max(np.abs(goal - before)) > 0.20:
                                raise RuntimeError("Transfer IK is discontinuous")
                            if not self._sweep(goal, index, arm):
                                raise RuntimeError(f"Transfer sweep rejected: {self.sweep_error}")
                            command[:18] = goal[:18] + bias
                            points.append(command.tolist())
                    return points
                except RuntimeError as exc:
                    failures.append(str(exc))
        raise RuntimeError("No clear upright transfer corridor: " + "; ".join(failures))

    def move_arm(self, arm: Arm, target: NDArray[Any], *, linear: bool) -> list[list[float]]:
        """Plan an explicit world-frame TCP move while preserving every held object.

        This does not acquire or release objects. A linear move preserves the
        current wrist orientation; a pose move may use raised transfer corridors.
        Both use the existing posture, collision and cargo-tilt policy.
        """
        if arm not in ARMS:
            raise ValueError("Choose left or right")
        target = np.asarray(target, dtype=float)
        if (
            target.shape != (4, 4)
            or not np.isfinite(target).all()
            or not np.allclose(target[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(target[:3, :3].T @ target[:3, :3], np.eye(3), atol=1e-6, rtol=0)
            or abs(float(np.linalg.det(target[:3, :3])) - 1) > 1e-6
        ):
            raise ValueError("TCP target must be a finite rigid 4x4 world-frame transform")
        for row in self.rows:
            if not row["held_by"] and (row["grasped"] or row["contacting_arms"]):
                raise RuntimeError(
                    f"Cannot move while {row['object']} has an unverified or supported grasp"
                )
        index = next((i for i, row in enumerate(self.rows) if row["held_by"] == arm), 0)
        measured = self.initial.qpos[self.qids].copy()
        command = np.array(
            [float(self.scene.data.actuator(n).ctrl[0]) for n in R1PRO_PICK_PLACE_JOINTS]
        )
        bias = command[:18] - measured[:18]
        if not np.isfinite(command).all() or not np.isfinite(measured).all():
            raise RuntimeError("Measured robot state and commands must be finite")
        if np.max(np.abs(bias)) > 0.03:
            raise RuntimeError("Robot must settle before planning an arm move")
        self.initialize_local_probe(index, arm)
        self.allow_target_contact = False
        points = (
            self.segment(arm, index, target)
            if linear
            else self.transfer_path(index, arm, target, allow_target_contact=False)
        )
        # Transfer planning also serves pick/place, which permits selected
        # contact. Explicit motion skills must not gain that permission.
        self.initialize_local_probe(index, arm)
        self.allow_target_contact = False
        for point in points:
            goal = np.asarray(point, dtype=float).copy()
            goal[:18] -= bias
            goal[18:] = measured[18:]
            if not self._sweep(goal, index, arm):
                raise RuntimeError(f"Arm move sweep rejected: {self.sweep_error}")
            point[18:] = command[18:].tolist()
        return points

    def init_posture(self) -> list[list[float]]:
        """Plan to the recorded startup joints, without resetting the scene or base.

        Both gripper commands remain unchanged. Unlike carry preparation, this
        has one fixed joint-space goal: an unsafe loaded-hand endpoint is an
        explicit failure, never silently replaced by a different carry posture.
        """
        for row in self.rows:
            if not row["held_by"] and (row["grasped"] or row["contacting_arms"]):
                raise RuntimeError(
                    f"Cannot return to init while {row['object']} has an unverified or "
                    "supported grasp; finish the pick or release it safely first"
                )
        held = [
            (i, cast("Arm", row["held_by"])) for i, row in enumerate(self.rows) if row["held_by"]
        ]
        index, arm = held[0] if held else (0, "right")
        measured = self.initial.qpos[self.qids].copy()
        command = np.array(
            [float(self.scene.data.actuator(n).ctrl[0]) for n in R1PRO_PICK_PLACE_JOINTS]
        )
        bias = command[:18] - measured[:18]
        if np.max(np.abs(bias)) > 0.03:
            raise RuntimeError("Robot must settle before planning a return to init")
        if not posture_is_valid(measured):
            raise RuntimeError("Measured init-move start violates the R1Pro posture envelope")
        home = np.asarray(self.scene.arms[arm].home, dtype=float)[:20].copy()
        if home.shape != (20,) or not np.isfinite(home).all():
            raise RuntimeError("The recorded startup joint position is unavailable")
        if not posture_is_valid(home):
            raise RuntimeError("The recorded startup position violates the R1Pro posture envelope")

        self.initialize_local_probe(index, arm)
        self.allow_target_contact = False
        self.probe.qpos[self.qids[:18]] = home[:18]
        self._forward()
        for cargo, _ in held:
            name = self.scene.layout.objects[cargo].name
            initial_up_z = self.initial.body(name).xmat[8]
            goal_up_z = self.probe.body(name).xmat[8]
            if not preserves_cargo_tilt(initial_up_z, goal_up_z):
                tilt = float(np.rad2deg(np.arccos(np.clip(goal_up_z, -1, 1))))
                raise RuntimeError(
                    f"Exact init position would tip held {name} to {tilt:.1f} degrees; "
                    "place it safely before returning to init"
                )
        collisions = self._collisions(selected=index, arm=arm)
        if collisions:
            raise RuntimeError(f"The fixed init position is obstructed: {collisions}")
        if np.max(np.abs(measured[:18] - home[:18])) <= 0.005:
            return [command.tolist()]

        points = self.posture_path(index, arm, home.tolist(), allow_target_contact=False)
        # Plan in measured joint space, then preserve the small steady-state
        # actuator bias so loaded joints actually settle at the home target.
        for point in points:
            point[:18] = (np.asarray(point[:18]) + bias).tolist()
            point[18:] = command[18:].tolist()
        points[0] = command.tolist()
        return points

    def carry_posture(self, *, seconds: float = CARRY_PLANNING_SECONDS) -> list[list[float]]:
        """Return to the ready posture without opening hands or tipping their cargo.

        Empty arms and the torso follow their recorded home joints. Loaded
        hands follow Cartesian corridors to their home TCPs: changing yaw
        about gravity is safe, but pitch and roll retain the measured grasp.
        A joint-space RRT cannot reliably find this narrow upright manifold.
        """
        held = [
            (i, cast("Arm", row["held_by"])) for i, row in enumerate(self.rows) if row["held_by"]
        ]
        index, arm = held[0] if held else (0, "right")
        if not posture_is_valid(self.initial.qpos[self.qids]):
            raise RuntimeError("Measured carrying start violates the R1Pro posture envelope")
        command = np.array(
            [float(self.scene.data.actuator(n).ctrl[0]) for n in R1PRO_PICK_PLACE_JOINTS]
        )
        if np.max(np.abs(command[:18] - self.initial.qpos[self.qids[:18]])) > 0.03:
            raise RuntimeError("Cargo must settle before planning a return to ready")
        home = np.asarray(self.scene.arms[arm].home).copy()
        home_data = mujoco.MjData(self.model)
        home_data.qpos[:] = self.initial.qpos
        home_data.qpos[self.qids[:18]] = home[:18]
        mujoco.mj_forward(self.model, home_data)
        home_targets = {side: home_data.site(f"{side}_tcp").xpos.copy() for _, side in held}
        self.initialize_local_probe(index, arm)
        base_rotation = self.probe.body("base_link").xmat.reshape(3, 3)
        compact = carry_ready_joints(self.initial.qpos[self.qids], home, {s for _, s in held})
        for _, side in held:
            offset = base_rotation.T @ (self.probe.site(f"{side}_tcp").xpos - home_targets[side])
            compact = compact and bool(
                -0.02 <= offset[0] <= 0.10 and abs(offset[1]) <= 0.04 and abs(offset[2]) <= 0.02
            )
        if compact:
            return [command.tolist()]
        if not held:
            return self.posture_path(index, arm, home.tolist(), allow_target_contact=False)

        # Small outward alternatives leave space for wide cargo; do not force
        # every hold up to an arbitrary height or preserve an empty hand's pose.
        failures: list[str] = []
        deadline = time.monotonic() + seconds
        for forward, lateral in ((0.0, 0.0), (0.04, 0.0), (0.08, 0.02)):
            for clearance in (0.0, 0.06, 0.12):
                for align_yaw in (True, False):
                    if time.monotonic() >= deadline:
                        last_failure = failures[-1] if failures else "no corridor evaluated"
                        raise RuntimeError(
                            f"Return-to-ready search exhausted its {seconds:g}s "
                            f"budget; last failure: {last_failure}"
                        )
                    try:
                        return self._carry_corridor(
                            held,
                            home,
                            home_data,
                            home_targets,
                            forward=forward,
                            lateral=lateral,
                            clearance=clearance,
                            align_yaw=align_yaw,
                            deadline=deadline,
                        )
                    except RuntimeError as exc:
                        failures.append(str(exc))
        raise RuntimeError("No clear compact carrying posture: " + "; ".join(failures))

    def _carry_corridor(
        self,
        held: list[tuple[int, Arm]],
        home: NDArray[Any],
        home_data: mujoco.MjData,
        home_targets: dict[Arm, NDArray[Any]],
        *,
        forward: float,
        lateral: float,
        clearance: float,
        align_yaw: bool,
        deadline: float,
    ) -> list[list[float]]:
        index, arm = held[0]
        self.initialize_local_probe(index, arm)
        start = self.probe.qpos[self.qids].copy()
        command = np.array(
            [float(self.scene.data.actuator(name).ctrl[0]) for name in R1PRO_PICK_PLACE_JOINTS]
        )
        bias = command[:18] - start[:18]
        if np.max(np.abs(bias)) > 0.03:
            raise RuntimeError("Cargo must settle before planning a return to ready")
        base = self.probe.body("base_link")
        base_rotation, origin = base.xmat.reshape(3, 3).copy(), base.xpos.copy()
        transform = self.reference_rotation @ base_rotation.T
        starts, rotations, goals, yaws = {}, {}, {}, {}
        for _, side in held:
            site = self.probe.site(f"{side}_tcp")
            starts[side] = site.xpos.copy()
            rotations[side] = site.xmat.reshape(3, 3).copy()
            goals[side] = home_targets[side] + base_rotation @ np.array(
                [forward, lateral if side == "left" else -lateral, 0.0]
            )
            yaws[side] = (
                carry_yaw(rotations[side], home_data.site(f"{side}_tcp").xmat.reshape(3, 3))
                if align_yaw
                else 0.0
            )
        loaded = {side for _, side in held}
        home_columns = list(range(4))
        for side in ARMS:
            if side not in loaded:
                home_columns.extend(active_indices(side)[:-1])
        count = max(
            2,
            int(np.ceil(np.max(np.abs(home[home_columns] - start[home_columns])) / 0.025)),
            *(
                int(np.ceil((np.linalg.norm(goals[s] - starts[s]) + 2 * clearance) / 0.006))
                for s in loaded
            ),
            *(int(np.ceil(abs(yaws[s]) / 0.04)) for s in loaded),
        )
        points = [command.tolist()]
        for t in np.linspace(0, 1, count + 1)[1:]:
            if time.monotonic() >= deadline:
                raise RuntimeError("Return-to-ready corridor exceeded the planning budget")
            before = self.probe.qpos[self.qids].copy()
            self.probe.qpos[self.qids[home_columns]] = start[home_columns] + t * (
                home[home_columns] - start[home_columns]
            )
            mujoco.mj_forward(self.model, self.probe)
            targets: dict[str, NDArray[Any]] = {}
            orientations: dict[str, Quaternion] = {}
            for side in loaded:
                xyz = starts[side] + t * (goals[side] - starts[side])
                xyz[2] += clearance * np.sin(np.pi * t)
                targets[side] = self.reference_position + transform @ (xyz - origin)
                rotation = Rotation.from_rotvec([0, 0, t * yaws[side]]).as_matrix()
                orientations[side] = Quaternion.from_rotation_matrix(
                    transform @ rotation @ rotations[side]
                )
            goal = self.kinematics.solve(
                self.probe,
                targets,
                orientations=orientations,
                allow_torso=False,
                position_tolerance=0.0002,
                orientation_tolerance=0.008,
                max_attempts=1,
            )
            self.probe.qpos[self.qids] = before
            self._forward()
            if np.max(np.abs(goal - before)) > 0.20:
                raise RuntimeError("Return-to-ready IK is discontinuous")
            if not self._sweep(goal, index, arm):
                raise RuntimeError(f"Return-to-ready sweep rejected: {self.sweep_error}")
            command[:18] = goal[:18] + bias
            points.append(command.tolist())
        if not carry_ready_joints(self.probe.qpos[self.qids], home, loaded):
            raise RuntimeError("Return-to-ready endpoint has not reached a compact posture")
        return points

    def rank(
        self,
        index: int,
        poses: NDArray[Any],
        scores: NDArray[Any],
        *,
        arm: str = "auto",
        seconds_per_arm: float = 25,
        max_results: int = 3,
    ) -> list[ClassicalGraspPlan]:
        if arm not in (*ARMS, "auto"):
            raise ValueError("Choose auto, left or right")
        hands = [
            side
            for side in ARMS
            if arm in ("auto", side) and not any(r["held_by"] == side for r in self.rows)
        ]
        results = []
        attempted = 0
        feasible = 0
        rejections: Counter[str] = Counter()
        target = np.asarray(self.rows[index]["position"])
        for side in hands:
            deadline = time.monotonic() + seconds_per_arm
            side_results = []
            feasible_stances = 0
            current_rotation = self.initial.site(f"{side}_tcp").xmat.reshape(3, 3)
            proposals = []
            for matrix, score in zip(poses, scores, strict=True):
                for symmetry in (np.eye(3), np.diag([-1.0, -1.0, 1.0])):
                    tcp = matrix.copy()
                    tcp[:3, :3] = matrix[:3, :3] @ symmetry
                    angle = float(
                        np.arccos(
                            np.clip((np.trace(current_rotation.T @ tcp[:3, :3]) - 1) / 2, -1, 1)
                        )
                    )
                    # Retain the generated approach/orientation, and also test
                    # a central body grasp for these symmetric demo objects.
                    # This changes the executed TCP goal, never the live object.
                    centered = tcp.copy()
                    offset = target - centered[:3, 3]
                    centered[:3, 3] += sum(
                        centered[:3, column] * np.dot(offset, centered[:3, column])
                        for column in (1, 2)
                    )
                    for pose, refinement in ((centered, "body_centered"), (tcp, "graspgenx")):
                        depth = abs(float(np.dot(target - pose[:3, 3], pose[:3, 2])))
                        proposals.append(
                            (
                                1 - float(score) + 0.3 * angle + 2 * depth,
                                pose,
                                float(score),
                                refinement,
                            )
                        )
            proposals.sort(key=lambda item: item[0])
            diverse: list[tuple[float, NDArray[Any], float, str]] = []
            for proposal in proposals:
                matrix = proposal[1]
                if any(
                    np.linalg.norm(matrix[:3, :3] - existing[1][:3, :3]) < 0.3
                    and np.linalg.norm(matrix[:3, 3] - existing[1][:3, 3]) < 0.02
                    for existing in diverse
                ):
                    continue
                diverse.append(proposal)
                if len(diverse) >= 24:
                    break
            for base in self._body_poses(target, side):
                found_at_base = 0
                # Bound each body pose's allocation so a bad dock does not
                # exhaust the entire reachability search on hundreds of grasps.
                for _, tcp, score, refinement in diverse[:12]:
                    if time.monotonic() > deadline:
                        break
                    attempted += 1
                    try:
                        result = self.evaluate_grasp(index, side, tcp, base, score)
                    except (RuntimeError, ValueError) as exc:
                        rejections[str(exc)] += 1
                        continue
                    result = replace(result, refinement=refinement)
                    side_results.append(result)
                    feasible += 1
                    found_at_base += 1
                    if found_at_base >= max_results:
                        break
                feasible_stances += bool(found_at_base)
                if time.monotonic() > deadline or feasible_stances >= GRASP_FEASIBLE_STANCES:
                    break
            results.extend(sorted(side_results, key=lambda result: result.cost)[:max_results])

        logger.info(
            "Grasp approach-and-lift assessment finished",
            object_index=index,
            hands=hands,
            source_candidates=len(poses),
            attempted=attempted,
            feasible=feasible,
            returned=len(results),
            rejection_reasons=dict(rejections.most_common(5)),
        )
        # A verified lift is a pick. Carrying, exact home and placement are
        # separate situation-dependent motions, not universal grasp filters.
        return sorted(results, key=lambda result: result.cost)


class _ApartmentCollisionWorld:
    """Delegate SDK world operations, adding frozen simulator collision checks.

    This adapter is local to one RRT request. It never mutates the shared SDK
    world or live simulator, and both robot self-collisions and scene contacts
    remain enabled while the planner searches (not only after it returns).
    """

    def __init__(self, planner: ClassicalGraspPlanner, index: int, arm: Arm) -> None:
        self.planner, self.index, self.arm = planner, index, arm

    def __getattr__(self, name: str) -> Any:
        return getattr(self.planner.kinematics.world, name)

    def check_edge_collision_free(
        self, start: JointState, end: JointState, step_size: float = 0.05
    ) -> bool:
        """Keep RRT edges in this policy/collision world, not the delegated SDK world."""
        if step_size <= 0:
            raise ValueError("Collision step size must be positive")
        space = self.planner.kinematics.world.get_prepared_model().joint_space.select(
            tuple(start.name)
        )
        end_values = dict(zip(end.name, end.position, strict=True))
        q_start = np.asarray(start.position)
        q_end = np.array([end_values[name] for name in start.name])
        steps = max(1, int(np.ceil(np.max(np.abs(space.delta(q_start, q_end))) / step_size)))
        return all(
            self.check_config_collision_free(
                JointState(
                    name=start.name, position=space.interpolate(q_start, q_end, float(t)).tolist()
                )
            )
            for t in np.linspace(0, 1, steps + 1)
        )

    def check_config_collision_free(self, state: JointState) -> bool:
        planner = self.planner
        for name, value in zip(state.name, state.position, strict=True):
            planner.probe.joint(name).qpos[0] = value
        if not posture_is_valid(planner.probe.qpos[planner.qids]):
            return False
        planner._forward()
        if planner._collisions(selected=self.index, arm=self.arm):
            return False
        return all(
            preserves_cargo_tilt(
                planner.initial.body(planner.scene.layout.objects[i].name).xmat[8],
                planner.probe.body(planner.scene.layout.objects[i].name).xmat[8],
            )
            for i in planner.attachments
        )
