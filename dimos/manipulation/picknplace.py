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

"""Request-driven perception interface for the pick-and-place workflow."""

import math
import threading
import time
from typing import Literal

import numpy as np
from pydantic import AliasChoices, Field

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT, CAP_PERCEPTION
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.manipulation.candidate_filter_spec import GraspCandidateFilterSpec
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.obstacle_world_spec import ObstacleWorldSpec
from dimos.manipulation.pick_execution_spec import PickExecutionSpec
from dimos.manipulation.visualization.layers import (
    LineSetElement,
    MeshElement,
    PointCloudElement,
    VisualizationLayer,
)
from dimos.manipulation.visualization.pose_overlay import draw_pose_axes
from dimos.manipulation.visualization_spec import ManipulationVisualizationSpec
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.experimental.object import (
    Object as DetObject,
    to_detection3d_array,
)
from dimos.perception.experimental.object_scene_registration_spec import ObjectSceneRegistrationSpec


def _estimate_table_surface(points: np.ndarray) -> dict[str, float] | None:
    """Fit the dominant horizontal support plane and return a conservative footprint."""
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 30:
        return None
    import open3d as o3d  # type: ignore[import-untyped]

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    plane, inliers = cloud.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    normal = np.asarray(plane[:3], dtype=np.float64)
    normal /= np.linalg.norm(normal)
    if abs(normal[2]) < 0.98 or len(inliers) < 30:
        return None
    surface = points[np.asarray(inliers)]
    x_low, y_low = np.quantile(surface[:, :2], 0.02, axis=0)
    x_high, y_high = np.quantile(surface[:, :2], 0.98, axis=0)
    # Extend the observed tabletop patch so collision protection includes its edges.
    margin = 0.10
    return {
        "center_x": float((x_low + x_high) / 2),
        "center_y": float((y_low + y_high) / 2),
        "tabletop_z": float(np.median(surface[:, 2])),
        "width": float(max(x_high - x_low + 2 * margin, 0.20)),
        "depth": float(max(y_high - y_low + 2 * margin, 0.20)),
        "inlier_count": float(len(inliers)),
    }


def _table_midpoint_grasp_z(
    points: np.ndarray, tabletop_z: float | None, fallback_z: float
) -> float:
    """Return the midpoint from the physical table plane to an object's observed top surface."""
    if tabletop_z is None or points.ndim != 2 or points.shape[1] != 3 or len(points) < 10:
        return fallback_z
    top_z = float(np.quantile(points[:, 2], 0.95))
    if top_z <= tabletop_z:
        return fallback_z
    return tabletop_z + (top_z - tabletop_z) / 2.0


def _primitive_mesh(
    shape: Literal["box", "sphere", "cylinder"],
    center: Vector3,
    dimensions: tuple[float, ...],
    orientation: Quaternion,
) -> tuple[np.ndarray, np.ndarray]:
    """Create a display mesh for one planner primitive."""
    if shape == "box":
        x, y, z = (dimension / 2.0 for dimension in dimensions)
        vertices = np.asarray(
            [
                [-x, -y, -z],
                [x, -y, -z],
                [x, y, -z],
                [-x, y, -z],
                [-x, -y, z],
                [x, -y, z],
                [x, y, z],
                [-x, y, z],
            ]
        )
        triangles = np.asarray(
            [
                [0, 1, 2],
                [0, 2, 3],
                [4, 6, 5],
                [4, 7, 6],
                [0, 4, 5],
                [0, 5, 1],
                [1, 5, 6],
                [1, 6, 2],
                [2, 6, 7],
                [2, 7, 3],
                [3, 7, 4],
                [3, 4, 0],
            ]
        )
    else:
        segments = 16
        angles = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
        radius = dimensions[0]
        if shape == "cylinder":
            half_height = dimensions[1] / 2.0
            vertices = np.vstack(
                (
                    np.column_stack(
                        (radius * np.cos(angles), radius * np.sin(angles), -half_height)
                    ),
                    np.column_stack(
                        (radius * np.cos(angles), radius * np.sin(angles), half_height)
                    ),
                    [[0.0, 0.0, -half_height], [0.0, 0.0, half_height]],
                )
            )
            bottom_center, top_center = 2 * segments, 2 * segments + 1
            triangles = np.asarray(
                [
                    triangle
                    for index in range(segments)
                    for triangle in (
                        [index, (index + 1) % segments, segments + index],
                        [
                            (index + 1) % segments,
                            segments + (index + 1) % segments,
                            segments + index,
                        ],
                        [bottom_center, (index + 1) % segments, index],
                        [top_center, segments + index, segments + (index + 1) % segments],
                    )
                ]
            )
        else:
            rings = 8
            phi = np.linspace(0.0, math.pi, rings + 1)
            vertices = np.asarray(
                [
                    [
                        radius * math.sin(p) * math.cos(a),
                        radius * math.sin(p) * math.sin(a),
                        radius * math.cos(p),
                    ]
                    for p in phi
                    for a in angles
                ]
            )
            triangles = np.asarray(
                [
                    triangle
                    for ring in range(rings)
                    for index in range(segments)
                    for triangle in (
                        [
                            ring * segments + index,
                            ring * segments + (index + 1) % segments,
                            (ring + 1) * segments + index,
                        ],
                        [
                            ring * segments + (index + 1) % segments,
                            (ring + 1) * segments + (index + 1) % segments,
                            (ring + 1) * segments + index,
                        ],
                    )
                ]
            )
    transformed = vertices @ orientation.to_rotation_matrix().T
    return transformed + np.asarray(center.as_tuple), triangles


class PickNPlaceConfig(ModuleConfig):
    """Configuration for PickNPlaceModule."""

    align_grasp_yaw: bool = False
    grasp: Literal["obb_center", "graspgenx"] = Field(
        default="obb_center", validation_alias=AliasChoices("grasp", "grasp_strategy")
    )
    graspgenx_pregrasp_offset: float = 0.10
    graspgenx_ik_filter_limit: int = 10
    grasp_empty_closed_threshold: float = 0.01
    grasp_feedback_delay: float = 0.5


class PickNPlaceModule(Module):
    """Provide request-driven perception and target selection for pick and place."""

    config: PickNPlaceConfig
    _scene: ObjectSceneRegistrationSpec
    _grasp_generator: GraspGenSpec | None
    _grasp_filter: GraspCandidateFilterSpec
    _pick_execution: PickExecutionSpec
    _obstacle_world: ObstacleWorldSpec
    _visualization: ManipulationVisualizationSpec
    objects: In[list[DetObject]]
    camera_info: In[CameraInfo]
    basic_grasp_overlay: Out[Image]
    graspgenx_candidates: Out[GraspCandidateArray]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._objects_condition = threading.Condition()
        self._latest_objects: tuple[DetObject, ...] = ()
        self._objects_version = 0
        self._camera_info: CameraInfo | None = None
        self._goal_pose: PoseStamped | None = None
        self._pre_grasp_pose: PoseStamped | None = None
        self._grasp_candidates: GraspCandidateArray | None = None
        self._selected_object: DetObject | None = None
        self._held_object_size: Vector3 | None = None
        self._tabletop_z: float | None = None
        self._open_box: dict[str, float] | None = None
        self._scene_geometry_ids: set[str] = set()

    @rpc
    def start(self) -> None:
        super().start()
        self.objects.subscribe(self._on_objects)
        self.camera_info.subscribe(self._on_camera_info)

    def _on_objects(self, objects: list[DetObject]) -> None:
        with self._objects_condition:
            self._latest_objects = tuple(objects)
            self._objects_version += 1
            self._objects_condition.notify_all()

    def _on_camera_info(self, camera_info: CameraInfo) -> None:
        with self._objects_condition:
            self._camera_info = camera_info

    @rpc
    def scan_scene(
        self, prompt: str | None = None, prompts: list[str] | None = None
    ) -> Detection3DArray:
        """Run one RGB-D detection pass, optionally targeting one or more text prompts."""
        if prompt is not None and prompts is not None:
            raise ValueError("Specify either prompt or prompts, not both")
        with self._objects_condition:
            objects_version = self._objects_version
        if prompts is not None:
            self._scene.set_prompts(prompts)
        elif prompt:
            self._scene.set_prompts([prompt])
        detections = self._scene.scan_scene()
        with self._objects_condition:
            received_result = self._objects_condition.wait_for(
                lambda: self._objects_version > objects_version,
                timeout=5.0,
            )
            objects = self._latest_objects
        if received_result:
            # Stream delivery crosses process boundaries and can lag the OSR RPC response.
            # Return the same snapshot used by the object/grasp APIs, not the prior response.
            return to_detection3d_array(
                list(objects),
                frame_id=objects[0].frame_id if objects else detections.frame_id,
                ts=objects[0].ts if objects else detections.ts,
            )
        return detections

    @skill(uses=[CAP_PERCEPTION])
    def scan(self, prompt: str) -> SkillResult:
        """Detect a prompted object from one RGB-D frame without moving the robot.

        Returns numbered objects. Use a returned number with ``select_object`` to create a grasp target
        or ``get_object_geometry`` to inspect a container target.
        """
        if not prompt.strip():
            return SkillResult.fail("INVALID_INPUT", "A nonempty object prompt is required")
        try:
            detections = self.scan_scene(prompt)
        except RuntimeError as exc:
            return SkillResult.fail("PERCEPTION_FAILED", str(exc))
        self._publish_scene_objects()
        return SkillResult.ok(
            f"Detected {detections.detections_length} object(s)", objects=self.get_scene_info()
        )

    @skill(uses=[CAP_PERCEPTION])
    def scan_objects(self, object_names: list[str]) -> SkillResult:
        """Detect instances of simple object names from one RGB-D frame.

        Pass one short noun phrase per item, for example ``["wooden block", "white box"]``. Each name is
        an independent Moondream query and can return multiple instances. Do not pass instructions,
        exclusions, counting requests, or full sentences as object names.
        """
        names = [name.strip() for name in object_names if name.strip()]
        if not names:
            return SkillResult.fail("INVALID_INPUT", "At least one simple object name is required")
        try:
            detections = self.scan_scene(prompts=names)
        except RuntimeError as exc:
            return SkillResult.fail("PERCEPTION_FAILED", str(exc))
        self._publish_scene_objects()
        return SkillResult.ok(
            f"Detected {detections.detections_length} object(s)",
            queried_names=names,
            objects=self.get_scene_info(),
        )

    @rpc
    def get_scene_info(self) -> list[dict[str, object]]:
        """Return the number, name, and confidence for current detections."""
        with self._objects_condition:
            objects = self._latest_objects
        return [
            {
                "number": number,
                "name": obj.name,
                "confidence": obj.confidence,
            }
            for number, obj in enumerate(objects, 1)
        ]

    @skill
    def describe_scene(self, question: str = "What objects are visible on the table?") -> str:
        """Answer an open-ended question about the latest camera image without moving the robot.

        Requires ``osr.det=moondream`` and is descriptive only; use ``scan`` for numbered 3D objects.
        """
        return self._scene.describe_scene(question)

    @skill
    def get_object_geometry(self, number: int) -> dict[str, object] | None:
        """Return a scanned object's center and OBB size without moving the robot.

        ``number`` must come from the latest ``scan`` result. ``center`` and ``size`` are ``[x, y, z]``
        lists in meters in the returned planning frame; use this to derive a container placement target.
        """
        with self._objects_condition:
            if number < 1 or number > len(self._latest_objects):
                return None
            obj = self._latest_objects[number - 1]
        return {
            "number": number,
            "name": obj.name,
            "frame_id": obj.frame_id,
            "center": [obj.center.x, obj.center.y, obj.center.z],
            "size": [obj.size.x, obj.size.y, obj.size.z],
        }

    @skill
    def install_object_obstacle(
        self, number: int, shape: Literal["box", "sphere", "cylinder"] = "box"
    ) -> SkillResult:
        """Install one measured object as a planner obstacle and render the same primitive in Viser.

        ``number`` must come from the latest ``scan`` result. Choose ``box`` for rectangular objects,
        ``cylinder`` for upright round objects, and ``sphere`` only for near-spherical objects.
        """
        obj = self._object_for_number(number)
        if obj is None:
            return SkillResult.fail("INVALID_INPUT", f"No detected object numbered {number}")
        orientation = self._upright_orientation(obj)
        if shape == "box":
            dimensions = (obj.size.x, obj.size.y, obj.size.z)
        elif shape == "sphere":
            dimensions = (max(obj.size.x, obj.size.y, obj.size.z) / 2.0,)
        else:
            dimensions = (max(obj.size.x, obj.size.y) / 2.0, obj.size.z)
        name = f"scene-object-{number}"
        center = Vector3(obj.center)
        if not self._install_geometry(name, center, orientation, shape, dimensions):
            return SkillResult.fail("EXECUTION_FAILED", f"Failed to install obstacle '{name}'")
        return SkillResult.ok(
            "Obstacle installed",
            name=name,
            shape=shape,
            center=[center.x, center.y, center.z],
            dimensions=list(dimensions),
        )

    @skill
    def install_open_box(self, number: int, wall_thickness: float = 0.01) -> SkillResult:
        """Measure an open rectangular box and render it as a display-only solid box in Viser.

        Call ``estimate_table`` first. The result describes the free opening for top-down placement, but
        does not add box walls to the planning world.
        """
        if wall_thickness <= 0.0:
            return SkillResult.fail("INVALID_INPUT", "wall_thickness must be positive")
        if self._tabletop_z is None:
            return SkillResult.fail(
                "INVALID_STATE", "Estimate the table before modeling an open box"
            )
        obj = self._object_for_number(number)
        if obj is None:
            return SkillResult.fail("INVALID_INPUT", f"No detected object numbered {number}")
        width, depth = obj.size.x, obj.size.y
        if width <= 2.0 * wall_thickness or depth <= 2.0 * wall_thickness:
            return SkillResult.fail(
                "INVALID_INPUT", "Box opening is smaller than twice wall_thickness"
            )
        points = obj.pointcloud.points_f32()
        rim_z = (
            float(np.quantile(points[:, 2], 0.95)) if len(points) else obj.center.z + obj.size.z / 2
        )
        height = rim_z - self._tabletop_z
        if height <= 0.0:
            return SkillResult.fail(
                "PERCEPTION_FAILED", "Box rim is not above the estimated tabletop"
            )
        center = Vector3(obj.center.x, obj.center.y, self._tabletop_z + height / 2.0)
        orientation = self._upright_orientation(obj)

        vertices, triangles = _primitive_mesh("box", center, (width, depth, height), orientation)
        self._visualization.set_visualization_layer(
            VisualizationLayer(
                "picknplace/open-box",
                "world",
                (
                    MeshElement(
                        "box-envelope",
                        vertices,
                        triangles,
                        color=np.asarray([230, 230, 230]),
                        opacity=0.25,
                    ),
                ),
            )
        )
        self._open_box = {
            "center_x": center.x,
            "center_y": center.y,
            "tabletop_z": self._tabletop_z,
            "rim_z": rim_z,
            "opening_width": width - 2.0 * wall_thickness,
            "opening_depth": depth - 2.0 * wall_thickness,
        }
        return SkillResult.ok(
            "Open box measured and displayed",
            center=[center.x, center.y],
            rim_z=rim_z,
            opening_width=width - 2.0 * wall_thickness,
            opening_depth=depth - 2.0 * wall_thickness,
        )

    @skill
    def clear_scene_geometry(self) -> SkillResult:
        """Remove temporary scene obstacles and the display-only open-box marker."""
        removed = [
            geometry_id
            for geometry_id in tuple(self._scene_geometry_ids)
            if self._obstacle_world.remove_obstacle(geometry_id)
        ]
        self._scene_geometry_ids.difference_update(removed)
        self._visualization.set_visualization_layer(
            VisualizationLayer("picknplace/open-box", "world", ())
        )
        self._open_box = None
        return SkillResult.ok("Temporary scene geometry cleared", removed=removed)

    @rpc
    def get_goal_pose(self, number: int) -> PoseStamped | None:
        """Select an object and return its downward-facing, floor-clamped grasp goal."""
        selection = self._basic_grasp(number)
        if selection is None:
            return None
        grasp, obj = selection
        if self.config.grasp == "graspgenx":
            if self._grasp_generator is None:
                raise RuntimeError("GraspGenX is not configured for this pick-and-place blueprint")
            candidates = self._grasp_generator.propose_grasps(obj.pointcloud)
            self._selected_object = obj
            self._grasp_candidates = self._filter_graspgenx_candidates(candidates)
            if not self._grasp_candidates.candidates:
                self.graspgenx_candidates.publish(self._grasp_candidates)
                return None
            return self._select_graspgenx_candidate(0)
        yaw = self._grasp_yaw(obj) if self.config.align_grasp_yaw else 0.0
        pick_execution = getattr(self, "_pick_execution", None)
        current_pose = pick_execution.get_ee_pose() if pick_execution is not None else None
        if current_pose is not None:
            yaw = self._closest_parallel_jaw_yaw(yaw, current_pose.orientation.to_euler().z)
        self._grasp_candidates = None
        self._selected_object = obj
        self.graspgenx_candidates.publish(GraspCandidateArray())
        grasp_z = _table_midpoint_grasp_z(
            obj.pointcloud.points_f32(), self._tabletop_z, grasp.position.z
        )
        self._goal_pose = PoseStamped(
            ts=grasp.ts,
            frame_id=grasp.frame_id,
            position=Vector3(grasp.position.x, grasp.position.y, max(grasp_z, 0.100)),
            orientation=Quaternion.from_euler(Vector3(-math.pi, 0.0, yaw)),
        )
        self._pre_grasp_pose = None
        return self._goal_pose

    @skill
    def select_object(self, number: int) -> SkillResult:
        """Select a scanned object and return grasp and pre-grasp targets without moving the robot.

        ``number`` must come from the latest ``scan`` result. Returned target values are XYZ in meters and
        roll/pitch/yaw in radians. Move to ``pre_grasp`` first, then move to ``goal`` for gripper contact;
        ``pre_grasp`` is 100 mm above the object and is not a grasp pose.
        """
        goal = self.get_goal_pose(number)
        if goal is None:
            return SkillResult.fail("INVALID_INPUT", f"No selectable object numbered {number}")
        pre_grasp = self.get_pre_grasp_pose()
        if pre_grasp is None:
            return SkillResult.fail("INVALID_STATE", "Selected object has no pre-grasp target")

        def pose_target(pose: PoseStamped) -> dict[str, float]:
            euler = pose.orientation.to_euler()
            return {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "roll": euler.x,
                "pitch": euler.y,
                "yaw": euler.z,
            }

        return SkillResult.ok(
            "Object selected. Move to pre_grasp, then goal for gripper contact before closing the gripper.",
            goal=pose_target(goal),
            pre_grasp=pose_target(pre_grasp),
        )

    @skill(uses=[CAP_MOVEMENT])
    def pick_selected(self, robot_name: str | None = None) -> SkillResult:
        """Pick the object most recently selected with ``select_object``.

        Executes the full pre-grasp, contact-grasp, close, feedback verification, and retreat sequence.
        A gripper position at or below the empty-closed threshold means no object was picked up and returns
        ``GRASP_VERIFICATION_FAILED``. Call ``select_object`` before this tool; do not manually recreate
        the grasp sequence with individual motion and gripper tools.
        """
        goal = self._goal_pose
        pre_grasp = self._pre_grasp_pose
        if goal is None or pre_grasp is None:
            return SkillResult.fail("INVALID_STATE", "Select an object before starting a pick")

        def move(pose: PoseStamped) -> SkillResult:
            euler = pose.orientation.to_euler()
            return self._pick_execution.move_to_pose(
                pose.position.x,
                pose.position.y,
                pose.position.z,
                euler.x,
                euler.y,
                euler.z,
                robot_name,
            )

        opened = self._pick_execution.open_gripper(robot_name)
        if not opened.is_success():
            return opened
        approach = move(pre_grasp)
        if not approach.is_success():
            return approach
        contact = move(goal)
        if not contact.is_success():
            return contact
        closed = self._pick_execution.close_gripper(robot_name)
        if not closed.is_success():
            return closed

        time.sleep(self.config.grasp_feedback_delay)
        gripper_position = self._pick_execution.get_gripper(robot_name)
        if gripper_position is None:
            return SkillResult.fail(
                "GRIPPER_FAILED", "Cannot verify pickup: gripper feedback unavailable"
            )
        if gripper_position <= self.config.grasp_empty_closed_threshold:
            self._pick_execution.open_gripper(robot_name)
            recovery = move(pre_grasp)
            result = SkillResult.fail(
                "GRASP_VERIFICATION_FAILED",
                "Pickup failed: gripper reached the empty-closed position; rescan and select before retrying",
            )
            result.metadata = {
                "gripper_position": gripper_position,
                "rescan_required": True,
                "recovered_to_pre_grasp": recovery.is_success(),
            }
            return result

        retreat = move(pre_grasp)
        if not retreat.is_success():
            return retreat
        selected_object = self._selected_object
        if selected_object is None:
            return SkillResult.fail(
                "INVALID_STATE", "Selected object details are unavailable after grasp"
            )
        self._held_object_size = Vector3(selected_object.size)
        return SkillResult.ok(
            "Pick complete: grasp verified and object retreated from the table",
            gripper_position=gripper_position,
        )

    @skill(uses=[CAP_MOVEMENT])
    def place_selected(self, robot_name: str | None = None) -> SkillResult:
        """Drop the verified held object into the most recently measured open box.

        Call ``install_open_box`` for the destination and complete ``pick_selected`` first. This tool moves
        above the remembered opening and releases above the rim. It first lifts the held object for transit,
        then lowers only at the box center. It never lowers the end effector into the box. The box remains
        display-only and does not add planner collisions.
        """
        box = self._open_box
        held_size = self._held_object_size
        if box is None:
            return SkillResult.fail(
                "INVALID_STATE", "Measure the destination with install_open_box first"
            )
        if held_size is None:
            return SkillResult.fail(
                "INVALID_STATE", "No verified held object is available to place"
            )
        if held_size.x > box["opening_width"] or held_size.y > box["opening_depth"]:
            return SkillResult.fail(
                "INVALID_INPUT", "Held object does not fit inside the measured box opening"
            )

        def move(x: float, y: float, z: float) -> SkillResult:
            return self._pick_execution.move_to_pose(x, y, z, robot_name=robot_name)

        # The held object's bottom remains above the rim throughout lateral travel.
        drop_z = box["rim_z"] + held_size.z / 2.0 + 0.02
        transit_z = drop_z + 0.10
        current_pose = self._pick_execution.get_ee_pose(robot_name)
        if current_pose is not None and current_pose.position.z < transit_z:
            lift = move(current_pose.position.x, current_pose.position.y, transit_z)
            if not lift.is_success():
                return lift
        approach = move(box["center_x"], box["center_y"], transit_z)
        if not approach.is_success():
            return approach
        lower = move(box["center_x"], box["center_y"], drop_z)
        if not lower.is_success():
            return lower
        opened = self._pick_execution.open_gripper(robot_name)
        if not opened.is_success():
            return opened
        self._held_object_size = None
        return SkillResult.ok(
            "Drop complete: object released above the measured box opening",
            drop_z=drop_z,
            object_bottom_clearance=0.02,
        )

    @rpc
    def select_grasp_candidate(self, rank: int) -> PoseStamped | None:
        """Select one ranked GraspGenX proposal as the goal and Rerun highlight."""
        return self._select_graspgenx_candidate(rank)

    def _select_graspgenx_candidate(self, rank: int) -> PoseStamped | None:
        candidates = self._grasp_candidates
        if candidates is None or rank < 0 or rank >= len(candidates.candidates):
            return None
        candidates.selected_index = rank
        self.graspgenx_candidates.publish(candidates)
        candidate = candidates.candidates[rank]
        self._goal_pose = PoseStamped(
            ts=candidates.header.timestamp,
            frame_id=candidates.header.frame_id,
            position=candidate.pose.position,
            orientation=candidate.pose.orientation,
        )
        self._pre_grasp_pose = None
        return self._goal_pose

    def _filter_graspgenx_candidates(self, candidates: GraspCandidateArray) -> GraspCandidateArray:
        """Keep only top-ranked proposals whose TCP IK is collision-free in the live world."""
        accepted = []
        for candidate in candidates.candidates[: self.config.graspgenx_ik_filter_limit]:
            result = self._grasp_filter.inverse_kinematics_single(
                candidate.pose, "arm", check_collision=True
            )
            if result.is_success():
                accepted.append(candidate)
        return GraspCandidateArray(candidates.header, accepted)

    @rpc
    def get_pre_grasp_pose(self) -> PoseStamped | None:
        """Return the selected goal offset 100 mm opposite its final approach direction."""
        if self._goal_pose is None:
            return None
        if self.config.grasp == "graspgenx":
            offset = self._goal_pose.orientation.rotate_vector(
                # GraspGenX local +Z points in the direction of the final
                # approach. A pre-grasp retreats along the opposite axis.
                Vector3(0.0, 0.0, -self.config.graspgenx_pregrasp_offset)
            )
        else:
            offset = Vector3(0.0, 0.0, 0.100)
        self._pre_grasp_pose = PoseStamped(
            ts=self._goal_pose.ts,
            frame_id=self._goal_pose.frame_id,
            position=Vector3(
                self._goal_pose.position.x + offset.x,
                self._goal_pose.position.y + offset.y,
                self._goal_pose.position.z + offset.z,
            ),
            orientation=self._goal_pose.orientation,
        )
        self._publish_viser_selection()
        return self._pre_grasp_pose

    def _publish_viser_selection(self) -> None:
        """Show the selected object and TCP targets without mutating the planning scene."""
        obj = self._selected_object
        goal = self._goal_pose
        pre_grasp = self._pre_grasp_pose
        if obj is None or goal is None or pre_grasp is None:
            return
        points = obj.pointcloud.points_f32()
        if len(points) == 0:
            return
        cloud_colors = np.repeat(np.array([[255, 190, 70]], dtype=np.uint8), len(points), axis=0)
        vertices: list[np.ndarray] = []
        edges: list[list[int]] = []
        colors: list[list[int]] = []
        for pose, color in ((goal, [255, 70, 70]), (pre_grasp, [70, 255, 120])):
            start = len(vertices)
            origin = np.asarray(pose.position.as_tuple, dtype=np.float32)
            axes = pose.orientation.to_rotation_matrix().astype(np.float32) * 0.06
            vertices.extend((origin, origin + axes[:, 0], origin + axes[:, 1], origin + axes[:, 2]))
            edges.extend(((start, start + 1), (start, start + 2), (start, start + 3)))
            colors.extend((color, color, color))
        self._visualization.set_visualization_layer(
            VisualizationLayer(
                "picknplace/selection",
                "world",
                (
                    PointCloudElement("object", points, cloud_colors, point_size=0.003),
                    LineSetElement(
                        "tcp-targets",
                        np.asarray(vertices),
                        np.asarray(edges),
                        np.asarray(colors),
                    ),
                ),
            )
        )

    def _publish_scene_objects(self) -> None:
        """Display the latest measured object envelopes without affecting planning."""
        visualization = getattr(self, "_visualization", None)
        if visualization is None:
            return
        with self._objects_condition:
            objects = tuple(self._latest_objects)
        colors = ([255, 180, 70], [80, 180, 255], [130, 230, 130], [230, 150, 230])
        elements: list[MeshElement] = []
        for number, obj in enumerate(objects, 1):
            dimensions = (obj.size.x, obj.size.y, obj.size.z)
            if any(dimension <= 0.0 for dimension in dimensions):
                continue
            vertices, triangles = _primitive_mesh(
                "box", Vector3(obj.center), dimensions, self._upright_orientation(obj)
            )
            elements.append(
                MeshElement(
                    f"object-{number}",
                    vertices,
                    triangles,
                    color=np.asarray(colors[(number - 1) % len(colors)]),
                    opacity=0.30,
                )
            )
        visualization.set_visualization_layer(
            VisualizationLayer("picknplace/scene-objects", "world", tuple(elements))
        )

    @rpc
    def get_grasp_candidates(self) -> GraspCandidateArray:
        """Return the GraspGenX proposals generated for the selected object."""
        return self._grasp_candidates or GraspCandidateArray()

    @rpc
    def estimate_table_surface(self) -> dict[str, float] | None:
        """Estimate a horizontal tabletop from the latest full RGB-D scene cloud."""
        scene = self._scene.get_full_scene_pointcloud(voxel_size=0.01)
        if scene is None:
            return None
        estimate = _estimate_table_surface(scene.points_f32())
        if estimate is None:
            return None
        self._tabletop_z = estimate["tabletop_z"]
        z = estimate["tabletop_z"]
        half_width = estimate["width"] / 2
        half_depth = estimate["depth"] / 2
        x = estimate["center_x"]
        y = estimate["center_y"]
        vertices = np.asarray(
            [
                [x - half_width, y - half_depth, z],
                [x + half_width, y - half_depth, z],
                [x + half_width, y + half_depth, z],
                [x - half_width, y + half_depth, z],
            ]
        )
        self._visualization.set_visualization_layer(
            VisualizationLayer(
                "picknplace/table-estimate",
                "world",
                (
                    MeshElement(
                        "tabletop-fill",
                        vertices,
                        np.asarray([[0, 1, 2], [0, 2, 3]]),
                        color=np.asarray([80, 180, 255]),
                        opacity=1.0,
                    ),
                    LineSetElement(
                        "tabletop",
                        vertices,
                        np.asarray([[0, 1], [1, 2], [2, 3], [3, 0]]),
                        colors=np.asarray([[80, 180, 255]] * 4),
                        line_width=2.0,
                    ),
                ),
            )
        )
        return estimate

    @skill(uses=[CAP_PERCEPTION])
    def estimate_table(self) -> SkillResult:
        """Run a fresh RGB-D scan, then estimate the tabletop without moving the robot.

        Pass the returned ``center_x``, ``center_y``, ``tabletop_z``, ``width``, and ``depth`` directly to
        ``set_table_collision`` before requesting motion near the table. Do not call this concurrently with
        ``scan``; both tools exclusively use the perception pipeline.
        """
        try:
            self.scan_scene()
        except RuntimeError as exc:
            return SkillResult.fail("PERCEPTION_FAILED", str(exc))
        estimate = self.estimate_table_surface()
        if estimate is None:
            return SkillResult.fail(
                "PERCEPTION_FAILED", "No horizontal tabletop estimate is available"
            )
        return SkillResult.ok("Table estimated", **estimate)

    def _basic_grasp(self, number: int) -> tuple[PoseStamped, DetObject] | None:
        """Return the selected cloud's OBB-center grasp frame and object geometry."""
        with self._objects_condition:
            if number < 1 or number > len(self._latest_objects):
                return None
            obj = self._latest_objects[number - 1]
            camera_info = self._camera_info
        grasp = PoseStamped(
            ts=obj.ts,
            frame_id=obj.frame_id,
            position=obj.center,
            orientation=obj.pose.orientation,
        )
        if camera_info is not None and obj.camera_transform is not None and obj.image is not None:
            if overlay := draw_pose_axes(
                obj.image, grasp, obj.camera_transform.inverse(), camera_info
            ):
                self.basic_grasp_overlay.publish(overlay)
        return grasp, obj

    def _object_for_number(self, number: int) -> DetObject | None:
        with self._objects_condition:
            if number < 1 or number > len(self._latest_objects):
                return None
            return self._latest_objects[number - 1]

    @staticmethod
    def _upright_orientation(obj: DetObject) -> Quaternion:
        """Keep measured horizontal yaw while constraining scene primitives upright."""
        return Quaternion.from_euler(Vector3(0.0, 0.0, obj.pose.orientation.to_euler().z))

    def _install_geometry(
        self,
        name: str,
        center: Vector3,
        orientation: Quaternion,
        shape: Literal["box", "sphere", "cylinder"],
        dimensions: tuple[float, ...],
    ) -> bool:
        pose = Pose(center, orientation)
        if self._obstacle_world.update_obstacle(name, pose, shape, list(dimensions)):
            self._scene_geometry_ids.add(name)
            return True
        obstacle_id = self._obstacle_world.add_obstacle(name, pose, shape, list(dimensions))
        if obstacle_id:
            self._scene_geometry_ids.add(obstacle_id)
            return True
        return False

    @staticmethod
    def _grasp_yaw(obj: DetObject) -> float:
        """Align the gripper's local Y closing axis with the narrowest horizontal OBB axis."""
        rotation = obj.pose.orientation.to_rotation_matrix()
        extents = (obj.size.x, obj.size.y, obj.size.z)
        horizontal_axes = sorted(range(3), key=lambda axis: abs(rotation[2, axis]))[:2]
        narrow_axis = min(horizontal_axes, key=lambda axis: extents[axis])
        return math.atan2(rotation[1, narrow_axis], rotation[0, narrow_axis]) - math.pi / 2

    @staticmethod
    def _closest_parallel_jaw_yaw(target_yaw: float, current_yaw: float) -> float:
        """Choose the equivalent parallel-jaw yaw requiring the smallest wrist rotation."""
        return target_yaw + math.pi * round((current_yaw - target_yaw) / math.pi)
