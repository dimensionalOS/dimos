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

from collections.abc import Collection, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import NDArray
import open3d as o3d  # type: ignore[import-untyped]
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import trimesh

from dimos.manipulation.planning.spec.gripper import ApproachOffsets, GraspContact, GripperSpec
from dimos.robot.assets.model import LoadedRobotModel

# Name GraspContact.touches gives the object being grasped.
TARGET = "target"
# Object points are thinned to one per cube of this edge (metres) before the
# check: fine enough for a margin of a few millimetres, and it bounds the work.
_TARGET_CELL = 0.0025
# Rays cast to decide whether a point is inside a link's shape; an odd count, so
# a ray grazing an edge is outvoted.
_INSIDE_RAYS = 3


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
        spec: Contact patches and finger joint positions of the gripper.
        tip_link: Link whose pose a grasp candidate gives (the tool tip).

    Raises ValueError when a link or joint the spec names is not in the URDF.
    """

    def __init__(self, description: LoadedRobotModel, spec: GripperSpec, tip_link: str) -> None:
        tree = _UrdfTree(description)
        for name in [tip_link, *(patch.link for patch in spec.contact_patches)]:
            tree.require_link(name)
        for name in spec.open_joints:
            tree.require_joint(name)
        base = spec.base_link or tree.common_ancestor(
            [tip_link, *(patch.link for patch in spec.contact_patches)]
        )
        tree.require_link(base)
        links = tree.subtree(base)
        self._shapes = [shape for link in links if (shape := tree.shape(link)) is not None]
        self._patches = {
            link: [
                (
                    np.subtract(p.center, np.divide(p.size, 2.0)),
                    np.add(p.center, np.divide(p.size, 2.0)),
                )
                for p in spec.contact_patches
                if p.link == link
            ]
            for link in links
        }
        # For each pose of the close, each link's pose in the tool tip's frame.
        self._tip_from_link: list[dict[str, NDArray[np.float64]]] = []
        for joints in spec.joint_positions():
            world = tree.forward_kinematics(joints)
            tip_inverse = np.linalg.inv(world[tip_link])
            self._tip_from_link.append({link: tip_inverse @ world[link] for link in links})
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

    @property
    def poses(self) -> int:
        """How many poses of the close the model holds: open first, fully closed last."""
        return len(self._tip_from_link)

    def tip_from_link(self, step: int, link: str) -> NDArray[np.float64]:
        """A link's pose in the tool tip's frame at one pose of the close (0 is open), 4x4."""
        return self._tip_from_link[step][link]

    def reach(self, step: int, links: Collection[str] | None = None) -> float:
        """How far (metres) gripper links reach past the tool tip along its z axis.

        Args:
            step: Pose of the close, 0 for open.
            links: Links to measure; None for every link with a shape.

        Negative when they all stay behind the tool tip. Measured to the corners
        of each link shape's bounding box, so a link turned against the tool tip
        counts a little long.
        """
        corners = [
            (self._tip_from_link[step][shape.link] @ np.append(corner, 1.0))[2]
            for shape in self._shapes
            if links is None or shape.link in links
            for corner in _box_corners(shape.low, shape.high)
        ]
        return float(max(corners))

    def check(
        self,
        grasps: NDArray[np.float64],
        target: NDArray[np.float64],
        obstacles: Mapping[str, NDArray[np.float64]],
        contact_margin: float,
        obstacle_margin: float,
    ) -> list[GraspContact]:
        """Say, for each grasp, what its gripper would touch that it should not.

        Args:
            grasps: Tool tip poses, shape (N, 4, 4), in the frame of the points.
            target: Points of the object being grasped, shape (M, 3), metres.
            obstacles: Other point sets the gripper must not overlap, by name (for
                example "table" and "objects"), each shape (K, 3), metres.
            contact_margin: A gripper part closer than this (metres) to a target
                point touches it; allowed only inside a contact patch.
            obstacle_margin: A gripper part closer than this (metres) to an
                obstacle point overlaps it.

        Returns one GraspContact per grasp, in order.
        """
        grasps = np.asarray(grasps, dtype=np.float64).reshape((-1, 4, 4))
        steps = len(self._tip_from_link)
        target = _thin(_finite(target), _TARGET_CELL)
        off_patch, pressed = self._touches(grasps, target, contact_margin, patches=True)
        hits = {
            name: self._touches(grasps, _finite(points), obstacle_margin, patches=False)[0]
            for name, points in obstacles.items()
        }
        # The fingers stop once a contact patch presses into the object, so the
        # poses from there on are never reached. A patch already pressing at the
        # open pose leaves only the open pose to check, and is itself a touch.
        off_patch[:, 0] |= pressed[:, 0]
        reached = pressed.any(axis=1)
        first = np.argmax(pressed, axis=1)
        last = np.where(reached, np.maximum(first - 1, 0), steps - 1)
        checked = np.arange(steps)[None, :] <= last[:, None]
        results = []
        for index in range(len(grasps)):
            touches = [TARGET] if (off_patch[index] & checked[index]).any() else []
            touches += [name for name, hit in hits.items() if (hit[index] & checked[index]).any()]
            at_open = off_patch[index, 0] or any(hit[index, 0] for hit in hits.values())
            results.append(GraspContact(tuple(touches), bool(touches) and not at_open))
        return results

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


def approach_offsets(
    description: LoadedRobotModel, spec: GripperSpec, tip_link: str, closing_steps: int = 40
) -> ApproachOffsets:
    """Measure where a gripper's finger pads, fingertips and palm sit, from its URDF.

    Args:
        description: The robot's URDF, loaded.
        spec: The gripper's contact patches (its finger pads) and finger joint positions.
        tip_link: The tool tip link a grasp pose gives.
        closing_steps: Poses between open and closed searched for the fingertips'
            farthest reach. Too few can miss a reach that peaks mid-close.

    Raises ValueError when a link or joint the spec names is not in the URDF.
    """
    model = GripperContactModel(
        description, spec.model_copy(update={"closing_steps": closing_steps}), tip_link
    )
    closed = model.poses - 1
    centers = [
        (model.tip_from_link(closed, patch.link) @ np.append(patch.center, 1.0))[2]
        for patch in spec.contact_patches
    ]
    fingers = {patch.link for patch in spec.contact_patches}
    return ApproachOffsets(
        pad_behind_tcp=-float(np.mean(centers)),
        fingertip_reach=max(model.reach(step) for step in range(model.poses)),
        palm_behind_tcp=-model.reach(0, [link for link in model.links if link not in fingers]),
    )


class _UrdfTree:
    """The links, joints and collision shapes of a URDF, enough for gripper kinematics."""

    def __init__(self, description: LoadedRobotModel) -> None:
        self._root = ET.fromstring(description.xml)
        self._source_dir = Path(description.source_path).parent
        self._links = {link.get("name", ""): link for link in self._root.findall("link")}
        self._joints = {joint.get("name", ""): joint for joint in self._root.findall("joint")}
        self._parent_joint = {
            _child(joint): joint for joint in self._joints.values() if _child(joint) in self._links
        }

    def require_link(self, name: str) -> None:
        if name not in self._links:
            raise ValueError(f"Gripper link {name!r} is not in the robot's URDF")

    def require_joint(self, name: str) -> None:
        if name not in self._joints:
            raise ValueError(f"Gripper joint {name!r} is not in the robot's URDF")

    def ancestors(self, link: str) -> list[str]:
        """The link, then its parent, up to the URDF root."""
        chain = [link]
        while (joint := self._parent_joint.get(chain[-1])) is not None:
            chain.append(_parent(joint))
        return chain

    def common_ancestor(self, links: Sequence[str]) -> str:
        """The deepest link that every one of ``links`` is at or below."""
        chains = [self.ancestors(link) for link in links]
        shared = set(chains[0]).intersection(*chains[1:])
        return next(link for link in chains[0] if link in shared)

    def subtree(self, base: str) -> list[str]:
        """``base`` and every link below it."""
        return [link for link in self._links if base in self.ancestors(link)]

    def forward_kinematics(self, positions: Mapping[str, float]) -> dict[str, NDArray[np.float64]]:
        """Every link's pose in the URDF root's frame, 4x4.

        Joints in ``positions`` (radians or metres) take those values, joints that
        mimic one follow it, and every other joint stays at zero.
        """
        poses: dict[str, NDArray[np.float64]] = {}

        def pose(link: str) -> NDArray[np.float64]:
            if link in poses:
                return poses[link]
            joint = self._parent_joint.get(link)
            if joint is None:
                poses[link] = np.eye(4)
                return poses[link]
            value = self._joint_value(joint, positions)
            poses[link] = pose(_parent(joint)) @ _origin(joint) @ _motion(joint, value)
            return poses[link]

        for link in self._links:
            pose(link)
        return poses

    def _joint_value(self, joint: ET.Element, positions: Mapping[str, float]) -> float:
        name = joint.get("name", "")
        if name in positions:
            return float(positions[name])
        mimic = joint.find("mimic")
        leader = None if mimic is None else self._joints.get(mimic.get("joint", ""))
        if mimic is None or leader is None:
            return 0.0
        multiplier = float(mimic.get("multiplier", "1"))
        offset = float(mimic.get("offset", "0"))
        return multiplier * self._joint_value(leader, positions) + offset

    def shape(self, link: str) -> _LinkShape | None:
        """The link's collision shape in its own frame, or None when it has none."""
        bodies = []
        for collision in self._links[link].findall("collision"):
            mesh = self._collision_mesh(collision)
            if mesh is None:
                continue
            mesh.apply_transform(_origin(collision))
            bodies += _bodies(mesh)
        if not bodies:
            return None
        low = np.min([body.low for body in bodies], axis=0)
        high = np.max([body.high for body in bodies], axis=0)
        return _LinkShape(link, tuple(bodies), low, high)

    def _collision_mesh(self, collision: ET.Element) -> trimesh.Trimesh | None:
        geometry = collision.find("geometry")
        if geometry is None:
            return None
        if (box := geometry.find("box")) is not None:
            return trimesh.creation.box(extents=_floats(box.get("size")))
        if (sphere := geometry.find("sphere")) is not None:
            return trimesh.creation.icosphere(radius=float(sphere.get("radius", "0")))
        if (cylinder := geometry.find("cylinder")) is not None:
            return trimesh.creation.cylinder(
                radius=float(cylinder.get("radius", "0")),
                height=float(cylinder.get("length", "0")),
            )
        mesh = geometry.find("mesh")
        if mesh is None:
            return None
        filename = mesh.get("filename", "").removeprefix("file://")
        path = Path(filename)
        if not path.is_absolute():
            path = self._source_dir / path
        loaded = trimesh.load_mesh(path, force="mesh")
        if mesh.get("scale"):
            loaded.apply_scale(np.asarray(_floats(mesh.get("scale"))))
        return loaded


def _child(joint: ET.Element) -> str:
    element = joint.find("child")
    return "" if element is None else element.get("link", "")


def _parent(joint: ET.Element) -> str:
    element = joint.find("parent")
    return "" if element is None else element.get("link", "")


def _floats(text: str | None) -> tuple[float, ...]:
    return tuple(float(value) for value in (text or "").split())


def _origin(element: ET.Element) -> NDArray[np.float64]:
    """An element's <origin> as a 4x4 transform; identity when it has none."""
    origin = element.find("origin")
    transform = np.eye(4)
    if origin is None:
        return transform
    # URDF roll-pitch-yaw turns about the fixed x, then y, then z axes.
    transform[:3, :3] = Rotation.from_euler(
        "xyz", _floats(origin.get("rpy")) or (0, 0, 0)
    ).as_matrix()
    transform[:3, 3] = _floats(origin.get("xyz")) or (0.0, 0.0, 0.0)
    return transform


def _motion(joint: ET.Element, value: float) -> NDArray[np.float64]:
    """How a joint at ``value`` (radians or metres) moves its child, as a 4x4 transform."""
    transform = np.eye(4)
    kind = joint.get("type", "fixed")
    if kind == "fixed" or value == 0.0:
        return transform
    axis_element = joint.find("axis")
    axis = np.asarray(
        _floats(None if axis_element is None else axis_element.get("xyz")) or (1.0, 0.0, 0.0)
    )
    axis = axis / np.linalg.norm(axis)
    if kind in ("revolute", "continuous"):
        transform[:3, :3] = Rotation.from_rotvec(axis * value).as_matrix()
    elif kind == "prismatic":
        transform[:3, 3] = axis * value
    return transform


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
    return np.all((points >= low) & (points <= high), axis=1)


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
