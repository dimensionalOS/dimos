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

"""Check grasp candidates against the robot's own shape, with the fingers open and as they close.

The check runs in a planning world of its own, built from the robot config with
the gripper's finger joints added so it can move them. For each grasp, the arm's
inverse kinematics puts the tool tip on the grasp (no solution rejects it), and
the world then says whether the robot is collision free:

- Open, nothing may touch the object or the scene: fingers already touching the
  object would have to push it aside to get there.
- At each closing step, only the pad links (``GripperConfig.pad_links``) may
  touch the object, and nothing may touch the scene. The fingers stop at the
  first step where a pad touches the object; later steps are not checked.

The object and the scene are octree obstacles. Each point fills the octree cell
it falls in, and the cells sit on a fixed grid, so an obstacle reaches between 0
and one cell edge past its points. A cell edge of twice a margin puts it about
one margin out, on average.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import replace
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import cKDTree

from dimos.manipulation.planning.factory import WorldBackend, create_kinematics, create_world
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle, PlanningGroupID
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.JointState import JointState

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# How far (metres) beyond the object's points scene points are kept: farther
# than any part of the gripper reaches from a grasp on the object.
_REACH = 0.25
# Scene points this close (metres) to a gripper's tool tip are that gripper: the
# wrist camera sees its own fingers.
_GRIPPER_SELF_RADIUS = 0.15
# Scene points this close (metres) to the object's points are the object's own
# edge left outside its mask.
_TARGET_EDGE = 0.01
_TARGET = "grasp-contact/target"
_SCENE = "grasp-contact/scene"


class GraspContactChecker:
    """Says which grasps a robot's gripper can close on without touching what it should not.

    Args:
        robot: The robot. ``robot.gripper`` names the pad links and finger joints.
        group: Planning group whose tool tip the grasps place.
        world_backend: Planning world backend; it must support ``Obstacle.touch_links``.

    Raises ValueError when the robot has no gripper config or no such group.
    """

    def __init__(
        self,
        robot: RobotModelConfig,
        group: PlanningGroupID,
        world_backend: WorldBackend = "roboplan",
    ) -> None:
        if robot.gripper is None:
            raise ValueError("The robot config has no gripper config")
        self._gripper = robot.gripper
        fingers = [name for name in self._gripper.open_joints if name not in robot.joint_names]
        model = robot.model_copy(update={"joint_names": [*robot.joint_names, *fingers]})
        self._world = create_world(backend=world_backend)
        self._world.load_model(prepare_robot_model(model))
        self._world.finalize()
        self._kinematics = create_kinematics()
        self._joint_names = model.joint_names
        found = [g for g in self._world.get_prepared_model().planning_groups if g.id == group]
        if not found:
            raise ValueError(f"The robot has no planning group {group!r}")
        self._group = found[0]
        # Joints a grasp does not set: the home pose, fingers open.
        home = robot.home_joints or [0.0] * len(robot.joint_names)
        self._rest = {
            **dict(zip(robot.joint_names, home, strict=True)),
            **self._gripper.open_joints,
        }

    def keep_clear(
        self,
        candidates: GraspCandidateArray,
        target: PointCloud2,
        scene: PointCloud2,
        tips: Sequence[Vector3],
        contact_margin: float,
        obstacle_margin: float,
        seed: JointState | None = None,
    ) -> GraspCandidateArray:
        """The grasps whose gripper touches the object only with its pads and clears the scene.

        Args:
            candidates: Grasps (tool tip poses), in the planning world's frame.
            target: The object's points, metres, same frame.
            scene: Camera points with the object masked out, metres, same frame.
            tips: Where each gripper's tool tip is now, metres, same frame; scene
                points near them are the robot's own gripper and are ignored.
            contact_margin: About this close (metres) to the object is a touch.
            obstacle_margin: About this close (metres) to a scene point is a touch.
            seed: The robot's joint positions now (radians, metres for a sliding
                joint), where the IK starts; joints left out start at home.
        """
        clear = self.check(
            [candidate.pose for candidate in candidates.candidates],
            _xyz(target),
            scene_obstacles(scene, target, tips),
            candidates.header.frame_id,
            contact_margin,
            obstacle_margin,
            seed,
        )
        kept = [c for c, ok in zip(candidates.candidates, clear, strict=True) if ok]
        return GraspCandidateArray(candidates.header, kept)

    def check(
        self,
        grasps: Sequence[Pose],
        target: NDArray[np.float64],
        scene: NDArray[np.float64],
        frame_id: str,
        contact_margin: float,
        obstacle_margin: float,
        seed: JointState | None = None,
    ) -> list[bool]:
        """Say, for each grasp, whether the gripper closes on it touching only what it may.

        Args:
            grasps: Tool tip poses, in ``frame_id``.
            target: Points of the object being grasped, shape (M, 3), metres.
            scene: Other points the robot must not touch, shape (K, 3), metres.
            frame_id: Frame of the grasps and points: the planning world's frame.
            contact_margin: About this close (metres) to the object is a touch.
            obstacle_margin: About this close (metres) to a scene point is a touch.
            seed: The robot's joint positions now (radians, metres for a sliding
                joint), where the IK starts; joints left out start at home.
        """
        world = self._world
        world.clear_obstacles()
        object_ = _octree(_TARGET, target, 2.0 * contact_margin, frame_id)
        for obstacle in (object_, _octree(_SCENE, scene, 2.0 * obstacle_margin, frame_id)):
            if obstacle is not None:
                world.add_obstacle(obstacle)
        start = {
            **self._rest,
            **({} if seed is None else dict(zip(seed.name, seed.position, strict=True))),
        }
        clear: list[bool] = []
        # The first closing step where anything touches anything, by grasp.
        first_touch: dict[int, JointState] = {}
        for index, grasp in enumerate(grasps):
            arm = self._solve(grasp, frame_id, start)
            if arm is None:
                clear.append(False)
                continue
            steps = self._gripper.joint_positions()
            poses = [self._state({**start, **arm, **fingers}) for fingers in steps]
            clear.append(world.check_config_collision_free(poses[0]))
            if clear[-1]:
                touching = (
                    pose for pose in poses[1:] if not world.check_config_collision_free(pose)
                )
                if (pose := next(touching, None)) is not None:
                    first_touch[index] = pose
        # At that step only the pads may touch the object: let them, and ask again.
        # Every earlier step touched nothing, so it passes either way.
        if first_touch and object_ is not None:
            world.update_obstacle(replace(object_, touch_links=self._gripper.pad_links))
        for index, pose in first_touch.items():
            clear[index] = world.check_config_collision_free(pose)
        return clear

    def _solve(
        self, grasp: Pose, frame_id: str, start: dict[str, float]
    ) -> dict[str, float] | None:
        """Arm joint positions that put the tool tip on ``grasp``, or None when the IK finds none.

        One attempt from ``start``: random restarts would make the verdict change
        from run to run, and slow the check.
        """
        target = PoseStamped(
            frame_id=frame_id, position=grasp.position, orientation=grasp.orientation
        )
        result = self._kinematics.solve_pose_targets(
            self._world,
            {self._group: target},
            seed=self._state(start),
            check_collision=False,
            max_attempts=1,
        )
        if not result.is_success() or result.joint_state is None:
            return None
        return dict(zip(result.joint_state.name, result.joint_state.position, strict=True))

    def _state(self, positions: dict[str, float]) -> JointState:
        return JointState(
            name=self._joint_names, position=[positions[name] for name in self._joint_names]
        )


def scene_obstacles(
    scene: PointCloud2, target: PointCloud2, tips: Sequence[Vector3]
) -> NDArray[np.float64]:
    """The scene points around an object that a gripper grasping it must keep clear of.

    Args:
        scene: Camera points with the object masked out, metres, in the object's frame.
        target: The object's points, metres.
        tips: Where each gripper's tool tip is, metres, same frame; points near
            them are the robot's own gripper and are dropped.

    Returns the points within reach of the object, less the robot's grippers
    and the object's own edge, shape (N, 3), metres.
    """
    points = _xyz(scene)
    own = _xyz(target)
    keep = np.zeros(len(points), dtype=np.bool_)
    if len(own):
        low, high = own.min(axis=0) - _REACH, own.max(axis=0) + _REACH
        keep = np.all((points >= low) & (points <= high), axis=1)
    for tip in tips:
        keep &= np.linalg.norm(points - tip.to_numpy(), axis=1) > _GRIPPER_SELF_RADIUS
    points = points[keep]
    if len(own) and len(points):
        distance, _ = cKDTree(own).query(points)
        points = points[distance > _TARGET_EDGE]
    return points


def _octree(name: str, points: NDArray[np.float64], cell: float, frame_id: str) -> Obstacle | None:
    """Points as an octree obstacle with cells of edge ``cell`` (metres); None for no points."""
    finite = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    finite = finite[np.all(np.isfinite(finite), axis=1)]
    if not len(finite):
        return None
    return Obstacle(
        name=name,
        obstacle_type=ObstacleType.OCTREE,
        pose=PoseStamped(frame_id=frame_id),
        points=tuple((x, y, z) for x, y, z in finite.tolist()),
        octree_resolution=cell,
    )


def _xyz(cloud: PointCloud2) -> NDArray[np.float64]:
    points = np.asarray(cloud.points_f32(), dtype=np.float64).reshape((-1, 3))
    return points[np.all(np.isfinite(points), axis=1)]
