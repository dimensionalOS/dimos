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

"""Check grasp candidates against the gripper's own shape, open and while it closes.

A grasp passes when, at the open pose and at each closing step until a contact
patch presses on the object, only contact patches come near the object and no
part of the gripper overlaps anything else. At the open pose a contact patch may
come near the object but not press into it: the fingers would have to push the
object aside to get there.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import trimesh

from dimos.manipulation.planning.spec.gripper import GripperConfig
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.robot.assets.model import (
    CollisionDescription,
    JointDescription,
    LinkDescription,
    LoadedRobotModel,
)
from dimos.utils.transform_utils import pose_to_matrix

if TYPE_CHECKING:
    import open3d as o3d  # type: ignore[import-untyped]

    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# Object points are thinned to one per cube of this edge (metres) before the
# check: fine enough for a margin of a few millimetres, and it bounds the work.
_TARGET_CELL = 0.0025
# Rays cast to decide whether a point is inside a link's shape; an odd count, so
# a ray grazing an edge is outvoted.
_INSIDE_RAYS = 3
# How far (metres) beyond the object's points scene points are kept: farther
# than any part of the gripper reaches from a grasp on the object.
_REACH = 0.25
# Scene points this close (metres) to a gripper's tool tip are that gripper: the
# wrist camera sees its own fingers.
_GRIPPER_SELF_RADIUS = 0.15
# Scene points this close (metres) to the object's points are the object's own
# edge left outside its mask.
_TARGET_EDGE = 0.01


@dataclass(frozen=True)
class _Body:
    """One piece of a link's collision shape.

    Args:
        scene: Ray-casting scene holding the piece, in the link's frame.
        low: Lower corner of its bounding box in the link's frame, metres.
        high: Upper corner of that box, metres.
        closed: Whether the piece is a closed surface. Only a closed piece
            says whether a point is inside it; an open one gives only how far a
            point is from its surface.
    """

    scene: o3d.t.geometry.RaycastingScene
    low: NDArray[np.float64]
    high: NDArray[np.float64]
    closed: bool


@dataclass(frozen=True)
class _LinkShape:
    """A gripper link's collision shape, as pieces.

    Args:
        link: URDF link name.
        bodies: The pieces; a point is inside the link when it is inside any.
        low: Lower corner of a box around every piece, link frame, metres.
        high: Upper corner of that box, metres.
    """

    link: str
    bodies: tuple[_Body, ...]
    low: NDArray[np.float64]
    high: NDArray[np.float64]


class GripperContactModel:
    """A gripper's link shapes, posed relative to its tool tip at each step of a close.

    Args:
        description: The robot's URDF, loaded.
        gripper: Contact patches and finger joint positions of the gripper.
        tip_link: Link whose pose a grasp candidate gives (the tool tip).

    Raises ValueError when a link or joint the gripper config names is not in the URDF.
    """

    def __init__(
        self, description: LoadedRobotModel, gripper: GripperConfig, tip_link: str
    ) -> None:
        links = {link.name: link for link in description.links}
        parent_joint = {joint.child_link: joint for joint in description.joints}
        patch_links = [patch.link for patch in gripper.contact_patches]
        for name in [tip_link, *patch_links, *([gripper.base_link] if gripper.base_link else [])]:
            if name not in links:
                raise ValueError(f"Gripper link {name!r} is not in the robot's URDF")
        for name in gripper.open_joints:
            if description.get_joint(name) is None:
                raise ValueError(f"Gripper joint {name!r} is not in the robot's URDF")
        base = gripper.base_link or _common_ancestor(parent_joint, [tip_link, *patch_links])
        gripper_links = [link for link in links if base in _ancestors(parent_joint, link)]
        self._shapes = [
            shape for link in gripper_links if (shape := _shape(links[link])) is not None
        ]
        self._patches = {
            link: [
                (
                    np.subtract(p.center, np.divide(p.size, 2.0)),
                    np.add(p.center, np.divide(p.size, 2.0)),
                )
                for p in gripper.contact_patches
                if p.link == link
            ]
            for link in gripper_links
        }
        # For each pose of the close, each link's pose in the tool tip's frame.
        self._tip_from_link: list[dict[str, NDArray[np.float64]]] = []
        for positions in gripper.joint_positions():
            world = _link_poses(
                description.joints, parent_joint, [tip_link, *gripper_links], positions
            )
            tip_inverse = np.linalg.inv(world[tip_link])
            self._tip_from_link.append({link: tip_inverse @ world[link] for link in gripper_links})
        corners = [
            (pose @ np.append(corner, 1.0))[:3]
            for poses in self._tip_from_link
            for shape in self._shapes
            for corner in _box_corners(shape.low, shape.high)
            for pose in [poses[shape.link]]
        ]
        if not corners:
            raise ValueError(f"No collision shapes on the gripper links below {base}")
        low, high = np.min(corners, axis=0), np.max(corners, axis=0)
        # A sphere, in the tip frame, holding every shape at every pose of the close.
        self._center = (low + high) / 2.0
        self._radius = float(np.linalg.norm(high - low) / 2.0)

    @property
    def links(self) -> list[str]:
        """Gripper links that have collision shapes."""
        return sorted({shape.link for shape in self._shapes})

    def tip_from_link(self, step: int, link: str) -> NDArray[np.float64]:
        """A link's pose in the tool tip's frame at one pose of the close (0 is open), 4x4."""
        return self._tip_from_link[step][link]

    def keep_clear(
        self,
        candidates: GraspCandidateArray,
        target: PointCloud2,
        scene: PointCloud2,
        tips: Sequence[Vector3],
        contact_margin: float,
        obstacle_margin: float,
    ) -> GraspCandidateArray:
        """The grasps whose gripper touches the object only at its contact patches and clears the scene.

        Args:
            candidates: Grasps, in the frame of the points.
            target: The object's points, metres.
            scene: Camera points with the object masked out, metres, same frame.
            tips: Where each gripper's tool tip is now, metres, same frame; scene
                points near them are the robot's own gripper and are ignored.
            contact_margin: Closer than this (metres) to the object is a touch.
            obstacle_margin: Closer than this (metres) to a scene point is an overlap.
        """
        poses = np.asarray([pose_to_matrix(c.pose) for c in candidates.candidates])
        clear = self.check(
            poses.reshape((-1, 4, 4)),
            target.points_f32(),
            scene_obstacles(scene, target, tips),
            contact_margin,
            obstacle_margin,
        )
        kept = [c for c, ok in zip(candidates.candidates, clear, strict=True) if ok]
        return GraspCandidateArray(candidates.header, kept)

    def check(
        self,
        grasps: NDArray[np.float64],
        target: NDArray[np.float64],
        obstacles: NDArray[np.float64],
        contact_margin: float,
        obstacle_margin: float,
    ) -> NDArray[np.bool_]:
        """Say, for each grasp, whether its gripper keeps clear of what it should not touch.

        Args:
            grasps: Tool tip poses, shape (N, 4, 4), in the frame of the points.
            target: Points of the object being grasped, shape (M, 3), metres.
            obstacles: Other scene points the gripper must not overlap, shape
                (K, 3), metres.
            contact_margin: A gripper part closer than this (metres) to a target
                point touches it; allowed only inside a contact patch.
            obstacle_margin: A gripper part closer than this (metres) to an
                obstacle point overlaps it.

        Returns shape (N,): True for each grasp that passes.
        """
        grasps = np.asarray(grasps, dtype=np.float64).reshape((-1, 4, 4))
        steps = len(self._tip_from_link)
        target = _thin(_finite(target), _TARGET_CELL)
        off_patch, pressed = self._touches(grasps, target, contact_margin, patches=True)
        hit, _ = self._touches(grasps, _finite(obstacles), obstacle_margin, patches=False)
        # The fingers stop once a contact patch presses into the object, so the
        # poses from there on are never reached. A patch already pressing at the
        # open pose leaves only the open pose to check, and is itself a touch.
        off_patch[:, 0] |= pressed[:, 0]
        reached = pressed.any(axis=1)
        first = np.argmax(pressed, axis=1)
        last = np.where(reached, np.maximum(first - 1, 0), steps - 1)
        checked = np.arange(steps)[None, :] <= last[:, None]
        return ~((off_patch | hit) & checked).any(axis=1)  # type: ignore[return-value]

    def _touches(
        self,
        grasps: NDArray[np.float64],
        points: NDArray[np.float64],
        margin: float,
        *,
        patches: bool,
    ) -> tuple[NDArray[np.bool_], NDArray[np.bool_]]:
        """Which grasps bring a gripper part within ``margin`` of any point, at each pose.

        Returns two (grasps, poses) masks: a point near the gripper outside every
        contact patch, and a point inside a contact patch and inside the link's
        shape, which the patch presses on (all False unless ``patches``).
        """
        steps = len(self._tip_from_link)
        outside = np.zeros((len(grasps), steps), dtype=bool)
        inside = np.zeros((len(grasps), steps), dtype=bool)
        if not len(grasps) or not len(points):
            return outside, inside
        owner, local = self._near_points(grasps, points, self._radius + margin)
        if not len(owner):
            return outside, inside
        for step, tip_from_link in enumerate(self._tip_from_link):
            for shape in self._shapes:
                pose = tip_from_link[shape.link]
                # Each point in the link's frame: R^T (p - t), written for row vectors.
                in_link = (local - pose[:3, 3]) @ pose[:3, :3]
                kept = np.flatnonzero(_in_box(in_link, shape.low - margin, shape.high + margin))
                if not len(kept):
                    continue
                in_link = in_link[kept]
                distance = _distance(shape, in_link, margin)
                near = distance < margin
                if not near.any():
                    continue
                allowed = np.zeros(int(near.sum()), dtype=bool)
                if patches:
                    for low, high in self._patches.get(shape.link, ()):
                        allowed |= _in_box(in_link[near], low, high)
                owners = owner[kept[near]]
                outside[np.unique(owners[~allowed]), step] = True
                inside[np.unique(owners[allowed & (distance[near] <= 0.0)]), step] = True
        return outside, inside

    def _near_points(
        self, grasps: NDArray[np.float64], points: NDArray[np.float64], radius: float
    ) -> tuple[NDArray[np.int64], NDArray[np.float64]]:
        """The points within reach of each grasp, in that grasp's tool tip frame.

        Returns which grasp each kept point belongs to, and the point in that
        grasp's tip frame (metres); a point near two grasps appears twice.
        """
        rotations, translations = grasps[:, :3, :3], grasps[:, :3, 3]
        centers = np.einsum("nij,j->ni", rotations, self._center) + translations
        found = cKDTree(points).query_ball_point(centers, radius)
        counts = np.fromiter((len(indices) for indices in found), dtype=np.int64, count=len(found))
        if not counts.sum():
            return np.empty(0, dtype=np.int64), np.empty((0, 3))
        owner = np.repeat(np.arange(len(grasps)), counts)
        indices = np.concatenate([np.asarray(i, dtype=np.int64) for i in found])
        # R^T (p - t) for each point and the grasp it belongs to.
        local = np.einsum("kji,kj->ki", rotations[owner], points[indices] - translations[owner])
        return owner, local


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


def _xyz(cloud: PointCloud2) -> NDArray[np.float64]:
    points = np.asarray(cloud.points_f32(), dtype=np.float64).reshape((-1, 3))
    return points[np.all(np.isfinite(points), axis=1)]


def _ancestors(parent_joint: Mapping[str, JointDescription], link: str) -> list[str]:
    """The link, then its parent, up to the URDF root."""
    chain = [link]
    while (joint := parent_joint.get(chain[-1])) is not None:
        chain.append(joint.parent_link)
    return chain


def _common_ancestor(parent_joint: Mapping[str, JointDescription], links: Sequence[str]) -> str:
    """The deepest link that every one of ``links`` is at or below."""
    chains = [_ancestors(parent_joint, link) for link in links]
    shared = set(chains[0]).intersection(*chains[1:])
    return next(link for link in chains[0] if link in shared)


def _link_poses(
    joints: Sequence[JointDescription],
    parent_joint: Mapping[str, JointDescription],
    links: Sequence[str],
    positions: Mapping[str, float],
) -> dict[str, NDArray[np.float64]]:
    """Each of ``links``' pose in the URDF root's frame, 4x4.

    Joints in ``positions`` (radians or metres) take those values, joints that
    mimic one follow it, and every other joint stays at zero.
    """
    by_name = {joint.name: joint for joint in joints}
    poses: dict[str, NDArray[np.float64]] = {}

    def value(joint: JointDescription) -> float:
        if joint.name in positions:
            return float(positions[joint.name])
        leader = None if joint.mimic is None else by_name.get(joint.mimic.joint)
        if joint.mimic is None or leader is None:
            return 0.0
        return joint.mimic.multiplier * value(leader) + joint.mimic.offset

    def pose(link: str) -> NDArray[np.float64]:
        if link not in poses:
            joint = parent_joint.get(link)
            poses[link] = (
                np.eye(4)
                if joint is None
                else pose(joint.parent_link)
                @ _origin(joint.origin_xyz, joint.origin_rpy)
                @ _motion(joint, value(joint))
            )
        return poses[link]

    return {link: pose(link) for link in links}


def _origin(
    xyz: tuple[float, float, float], rpy: tuple[float, float, float]
) -> NDArray[np.float64]:
    """A URDF origin as a 4x4 transform."""
    transform = np.eye(4)
    # URDF roll-pitch-yaw turns about the fixed x, then y, then z axes.
    transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    transform[:3, 3] = xyz
    return transform


def _motion(joint: JointDescription, value: float) -> NDArray[np.float64]:
    """How a joint at ``value`` (radians or metres) moves its child, as a 4x4 transform."""
    transform = np.eye(4)
    if joint.type == "fixed" or value == 0.0:
        return transform
    axis = np.asarray(joint.axis) / np.linalg.norm(joint.axis)
    if joint.type in ("revolute", "continuous"):
        transform[:3, :3] = Rotation.from_rotvec(axis * value).as_matrix()
    elif joint.type == "prismatic":
        transform[:3, 3] = axis * value
    return transform


def _shape(link: LinkDescription) -> _LinkShape | None:
    """The link's collision shape in its own frame, or None when it has none."""
    bodies = []
    for collision in link.collisions:
        mesh = _collision_mesh(collision)
        if mesh is None:
            continue
        mesh.apply_transform(_origin(collision.origin_xyz, collision.origin_rpy))
        bodies += _bodies(mesh)
    if not bodies:
        return None
    low = np.min([body.low for body in bodies], axis=0)
    high = np.max([body.high for body in bodies], axis=0)
    return _LinkShape(link.name, tuple(bodies), low, high)


def _collision_mesh(collision: CollisionDescription) -> trimesh.Trimesh | None:
    """A collision shape as a triangle mesh, metres; None for a shape it cannot build."""
    if collision.geometry == "box":
        return trimesh.creation.box(extents=collision.size)  # type: ignore[no-any-return]
    if collision.geometry == "sphere":
        return trimesh.creation.icosphere(radius=collision.radius)  # type: ignore[no-any-return]
    if collision.geometry == "cylinder":
        return trimesh.creation.cylinder(  # type: ignore[no-any-return]
            radius=collision.radius, height=collision.length
        )
    if collision.geometry != "mesh":
        return None
    mesh = trimesh.load_mesh(collision.filename.removeprefix("file://"), force="mesh")
    mesh.apply_scale(np.asarray(collision.scale))  # type: ignore[no-untyped-call]
    return mesh


def _bodies(mesh: trimesh.Trimesh) -> list[_Body]:
    """A collision mesh as pieces to measure distances to.

    CAD exports often hold several overlapping closed shells; each is kept
    apart, since counting surface crossings through overlapping shells gets
    inside and outside wrong. A mesh with any open shell stays whole and open:
    its convex hull would fill recesses with material that is not there.
    """
    pieces = list(mesh.split(only_watertight=False))
    if pieces and all(piece.is_watertight for piece in pieces):
        return [_body(piece, closed=True) for piece in pieces]
    return [_body(mesh, closed=False)]


def _body(mesh: trimesh.Trimesh, *, closed: bool) -> _Body:
    import open3d as o3d  # type: ignore[import-untyped]

    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(
        o3d.core.Tensor(np.asarray(mesh.vertices, dtype=np.float32)),
        o3d.core.Tensor(np.asarray(mesh.faces, dtype=np.uint32)),
    )
    low, high = np.asarray(mesh.bounds, dtype=np.float64)
    return _Body(scene, low, high, closed)


def _distance(shape: _LinkShape, points: NDArray[np.float64], margin: float) -> NDArray[np.float64]:
    """Distance (metres) from each point to the link's surface, negative inside a closed piece.

    Points farther than ``margin`` from every piece's box may get any value of
    at least ``margin``.
    """
    import open3d as o3d  # type: ignore[import-untyped]

    distance = np.full(len(points), np.inf)
    for body in shape.bodies:
        kept = np.flatnonzero(_in_box(points, body.low - margin, body.high + margin))
        if not len(kept):
            continue
        query = o3d.core.Tensor(np.asarray(points[kept], dtype=np.float32))
        found = (
            body.scene.compute_signed_distance(query, nsamples=_INSIDE_RAYS)
            if body.closed
            else body.scene.compute_distance(query)
        ).numpy()
        distance[kept] = np.minimum(distance[kept], found)
    return distance


def _in_box(
    points: NDArray[np.float64], low: NDArray[np.float64], high: NDArray[np.float64]
) -> NDArray[np.bool_]:
    return np.all((points >= low) & (points <= high), axis=1)  # type: ignore[no-any-return]


def _box_corners(low: NDArray[np.float64], high: NDArray[np.float64]) -> list[NDArray[np.float64]]:
    return [
        np.array([x, y, z])
        for x in (low[0], high[0])
        for y in (low[1], high[1])
        for z in (low[2], high[2])
    ]


def _finite(points: NDArray[np.float64]) -> NDArray[np.float64]:
    array = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    return array[np.all(np.isfinite(array), axis=1)]


def _thin(points: NDArray[np.float64], cell: float) -> NDArray[np.float64]:
    """One point per cube of edge ``cell`` (metres), the first found in each."""
    if not len(points):
        return points
    _, first = np.unique(np.floor(points / cell).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(first)]
