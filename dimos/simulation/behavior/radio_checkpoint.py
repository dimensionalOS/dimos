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

"""Radio-only development scene binding and checked SDK grasp/lift dispatch.

Oracle scene/contact observations are operator diagnostics, not policy sensors.
The scene is deliberately limited to wxnicr's 14 convex parts and its table.
"""

from collections.abc import Callable, Generator, Mapping, Sequence
from contextlib import contextmanager, nullcontext
import copy
import hashlib
import itertools
import json
import math
from pathlib import Path
import time
from typing import Any

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from dimos.manipulation.manipulation_spec import ExecutionResult, ExecutionStatus, PlanResult
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.manipulation.sdk import Arm, MotionError
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.simulation.behavior.radio_collision import temporary_carried_radio_geometry
from dimos.simulation.behavior.radio_contact_evidence import (
    AssistedRadioRetention,
    radio_interaction_evidence,
    radio_stage_displacement,
)
from dimos.simulation.behavior.radio_motion import (
    check_development_start,
    trajectory_digest,
    validate_development_trajectory,
)
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

DUAL_PHASES = {"press_approach", "press_contact"}
COORDINATED_PHASES = {"reposition", *DUAL_PHASES}
TABLETOP_BEFORE_PRESS = {"reorient", "lower", "place", "retract", "table_approach"}
HELD_PHASES = {"departure", "lift", "reorient", "lower", "place", *COORDINATED_PHASES}
PHASES = {"pregrasp", "grasp", "retract", "table_approach", "table_press", *HELD_PHASES}


class RadioEpisodeTerminalError(RuntimeError):
    """Explicit evaluator/runtime outcome, never a motion-arrival success."""

    def __init__(self, outcome: Mapping[str, Any]) -> None:
        self.outcome = dict(outcome)
        super().__init__(f"Radio episode outcome: {self.outcome['kind']}")


def radio_episode_terminal(
    status: Mapping[str, Any], expected_episode: str
) -> dict[str, Any] | None:
    """Only a same-episode authoritative terminal event can establish task success."""
    episode = status.get("episode") or {}
    common = {"episode": episode.get("id"), "step": episode.get("step"), "motion_arrived": False}
    if episode.get("id") != expected_episode:
        return {**common, "kind": "RUNTIME_FAULT", "reason": "Episode changed"}
    if status.get("state") in ("error", "stopped") or status.get("error"):
        return {**common, "kind": "RUNTIME_FAULT", "reason": status.get("error") or status["state"]}
    if status.get("state") != "finished":
        return None
    done = (episode.get("info") or {}).get("done") or {}
    goals = done.get("goal_status") or {}
    verified = (
        episode.get("terminated") is True
        and episode.get("truncated") is False
        and episode.get("success") is True
        and done.get("success") is True
        and isinstance(goals.get("satisfied"), list)
        and bool(goals["satisfied"])
        and goals.get("unsatisfied") == []
    )
    if verified:
        return {**common, "kind": "TASK_GOAL_MET", "goal_status": copy.deepcopy(goals)}
    if (
        episode.get("success")
        or done.get("success")
        or not (episode.get("terminated") is True or episode.get("truncated") is True)
    ):
        return {**common, "kind": "RUNTIME_FAULT", "reason": "Inconsistent terminal evidence"}
    return {**common, "kind": "EPISODE_ENDED", "goal_status": copy.deepcopy(goals)}


def rigid(value: Any) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if (
        matrix.shape != (4, 4)
        or not np.isfinite(matrix).all()
        or not np.allclose(matrix[3], [0, 0, 0, 1])
        or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-7)
        or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-7)
    ):
        raise ValueError("Use a finite rigid radio transform")
    return matrix.copy()


def stamped(matrix: np.ndarray) -> PoseStamped:
    pose = matrix_to_pose(matrix)
    return PoseStamped(frame_id="world", position=pose.position, orientation=pose.orientation)


class RadioCheckpointScene:
    """Exclusive, task-local geometry owner used identically on both sides of RPC."""

    def __init__(self, world: Any, description: Mapping[str, Any]) -> None:
        self.world = world
        self.description = copy.deepcopy(dict(description))
        if description.get("model") != "wxnicr" or not description.get("provenance"):
            raise ValueError("Declare wxnicr geometry provenance")
        meshes = description["meshes"]
        if len(meshes) != 14:
            raise ValueError("All 14 convex radio parts are required")
        vertices_list = []
        for mesh in meshes:
            path = Path(mesh["path"])
            data = path.read_bytes()
            if hashlib.sha256(data).hexdigest() != mesh["sha256"]:
                raise ValueError("Radio convex mesh checksum changed")
            vertices = np.asarray(
                [
                    [float(v) for v in line.split()[1:]]
                    for line in data.decode().splitlines()
                    if line.startswith("v ")
                ]
            )
            if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all():
                raise ValueError("Invalid radio collision vertices")
            vertices_list.append(vertices)
        self.vertices = np.vstack(vertices_list)
        self.part_vertices = vertices_list
        self.signature = hashlib.sha256(
            json.dumps(self.description, sort_keys=True).encode()
        ).hexdigest()
        self.names = tuple(f"radio_part_{i}" for i in range(14))
        table = description["table"]
        self.table = rigid(table["pose"])
        self.table_extent = np.asarray(table["extent"], dtype=float)
        if (
            self.table_extent.shape != (3,)
            or not np.isfinite(self.table_extent).all()
            or np.any(self.table_extent <= 0)
        ):
            raise ValueError("Declare positive table extents")
        existing = {obj.name for obj in world.get_obstacles()}
        if existing & {*self.names, "radio", "table"}:
            raise ValueError("Checkpoint obstacle names already have another owner")
        added = []
        try:
            for name, mesh in zip(self.names, meshes, strict=True):
                if (
                    world.add_obstacle(
                        Obstacle(
                            name=name,
                            obstacle_type=ObstacleType.MESH,
                            pose=stamped(np.eye(4)),
                            mesh_path=mesh["path"],
                        )
                    )
                    != name
                ):
                    raise RuntimeError("Checkpoint radio part registration failed")
                added.append(name)
            if (
                world.add_obstacle(
                    Obstacle(
                        name="table",
                        obstacle_type=ObstacleType.BOX,
                        pose=stamped(self.table),
                        dimensions=tuple(self.table_extent),
                    )
                )
                != "table"
            ):
                raise RuntimeError("Checkpoint table registration failed")
            added.append("table")
            for a, b in itertools.combinations(self.names, 2):
                world._require_scene().setCollisions(a, b, False)  # one rigid body
        except BaseException:
            for name in reversed(added):
                world.remove_obstacle(name)
            raise

    @contextmanager
    def context(self, request: Mapping[str, Any]) -> Generator[None, None, None]:
        phase = request["phase"]
        if phase not in PHASES or request["signature"] != self.signature:
            raise ValueError("Checkpoint phase/geometry identity changed")
        radio = rigid(request["radio_pose"])
        if phase in ("retract", "table_approach", "table_press"):
            table_radio = np.linalg.inv(self.table) @ radio
            vertices = self.vertices @ table_radio[:3, :3].T + table_radio[:3, 3]
            height = float(vertices[:, 2].min() - self.table_extent[2] / 2)
            if not -0.002 <= height <= 0.005 or np.any(
                np.max(np.abs(vertices[:, :2]), axis=0) > self.table_extent[:2] / 2
            ):
                raise RuntimeError("Tabletop radio is outside bounded stationary support")
        held = phase in HELD_PHASES
        local = rigid(request["gripper_from_radio"]) if held else None
        if phase in DUAL_PHASES:
            rigid(request["holding_pose"])
        if phase in ("press_contact", "table_press"):
            self._contact_points(request)
        native = self.world._require_scene()
        with self.world._lock:
            for name in self.names:
                if not self.world.update_obstacle_pose(name, stamped(radio)):
                    raise RuntimeError("Checkpoint collision part disappeared")
            try:
                native.setCollisions(
                    "radio_part_0",
                    "table",
                    phase
                    not in (
                        "pregrasp",
                        "grasp",
                        "departure",
                        "place",
                        "retract",
                        "table_approach",
                        "table_press",
                    ),
                )
                for finger in (1, 2):
                    native.setCollisions(
                        f"right_gripper_finger_link{finger}", "radio_part_6", not held
                    )
                    if phase == "press_contact" and finger == 1:
                        native.setCollisions(
                            f"left_gripper_finger_link{finger}", "radio_part_0", False
                        )
                    if phase == "table_press" and finger == 2:
                        native.setCollisions("right_gripper_finger_link2", "radio_part_0", False)
                attachment: Any = nullcontext()
                if held:
                    assert local is not None
                    attachment = temporary_carried_radio_geometry(
                        self.world, "right_gripper_link", {name: local for name in self.names}
                    )
                with attachment:
                    yield
            finally:
                # Restore conservative pairs; internal compound-body pairs remain excluded.
                try:
                    native.setCollisions("radio_part_0", "table", True)
                    for finger in (1, 2):
                        native.setCollisions(
                            f"right_gripper_finger_link{finger}", "radio_part_6", True
                        )
                        if phase == "press_contact" and finger == 1:
                            native.setCollisions(
                                f"left_gripper_finger_link{finger}", "radio_part_0", True
                            )
                        if phase == "table_press" and finger == 2:
                            native.setCollisions("right_gripper_finger_link2", "radio_part_0", True)
                except Exception:
                    self.world._usable = False
                    raise

    def validate(
        self,
        state: JointState,
        trajectory: Any,
        selected: Sequence[str],
        request: Mapping[str, Any],
    ) -> dict[str, Any]:
        check_development_start(trajectory, dict(zip(state.name, state.position, strict=True)))
        with self.context(request):
            result = validate_development_trajectory(self.world, state, trajectory, selected)
            if request["phase"] == "departure":
                result["support_departure"] = self._support_departure(state, trajectory, request)
            if request["phase"] == "place":
                result["support_placement"] = self._support_placement(state, trajectory, request)
            if request["phase"] in ("reorient", "lower", "place", "retract", "table_approach"):
                result["trigger_avoidance"] = self._tabletop_path(state, trajectory, request)
            if request["phase"] == "table_press":
                result["single_press"] = self._tabletop_path(state, trajectory, request)
            if request["phase"] in DUAL_PHASES:
                result["holding_constraint"] = self._holding_path(state, trajectory, request)
            if request["phase"] == "reposition":
                values = dict(zip(state.name, state.position, strict=True))
                values.update(
                    zip(trajectory.joint_names, trajectory.points[-1].positions, strict=True)
                )
                end = JointState(name=state.name, position=[values[n] for n in state.name])
                result["endpoint_tcp_errors"] = self.reposition_endpoint(end, request)
        return {**result, "geometry_signature": self.signature, "phase": request["phase"]}

    def _tabletop_path(
        self, state: JointState, trajectory: Any, request: Mapping[str, Any]
    ) -> dict[str, Any]:
        """Conservative finger envelope before pressing; explicit contact corridor after."""
        values = dict(zip(state.name, state.position, strict=True))
        previous = np.asarray([values[n] for n in trajectory.joint_names])
        minimum_gap = math.inf
        samples = 0
        held = request["phase"] in HELD_PHASES
        for point in trajectory.points:
            target = np.asarray(point.positions)
            count = max(1, math.ceil(float(np.max(np.abs(target - previous))) / 0.01))
            for fraction in np.linspace(0, 1, count + 1):
                candidate = dict(values)
                candidate.update(
                    zip(
                        trajectory.joint_names,
                        previous + fraction * (target - previous),
                        strict=True,
                    )
                )
                q = JointState(name=state.name, position=[candidate[n] for n in state.name])
                with self.world.scratch_context() as ctx:
                    self.world.set_joint_state(ctx, q)
                    right = self.world.get_link_pose(ctx, "right_gripper_link")
                radio = (
                    right @ rigid(request["gripper_from_radio"])
                    if held
                    else rigid(request["radio_pose"])
                )
                marker = (radio @ np.array([0.0446848528, 0.0420822057, -0.0124619396, 1]))[:3]
                if request["phase"] == "table_press":
                    surface, pad = self._contact_points(request)
                    gap = (np.linalg.inv(radio) @ right @ np.append(pad, 1))[:3] - surface
                    normal = np.asarray(
                        request["contact"].get("outward_normal_in_radio", [1, 0, 0])
                    )
                    signed = float(gap @ normal)
                    if (
                        not -0.002 <= signed <= 0.060
                        or np.linalg.norm(gap - signed * normal) > 0.015
                    ):
                        raise RuntimeError("Single tabletop press exceeded its contact corridor")
                else:
                    if request["phase"] != "table_approach":
                        bounds = self.description.get("finger_bounds")
                        if not bounds or not self.description.get("finger_geometry_provenance"):
                            raise RuntimeError(
                                "Certify actual finger collision bounds before turning/placing"
                            )
                        if set(bounds) != {
                            f"{side}_gripper_finger_link{i}"
                            for side in ("left", "right")
                            for i in (1, 2)
                        }:
                            raise ValueError("Certify all four finger collision bounds")
                        with self.world.scratch_context() as ctx:
                            self.world.set_joint_state(ctx, q)
                            for name, bound in bounds.items():
                                lo, hi = np.asarray(bound["minimum"]), np.asarray(bound["maximum"])
                                if (
                                    lo.shape != (3,)
                                    or hi.shape != (3,)
                                    or not np.isfinite([lo, hi]).all()
                                    or np.any(hi <= lo)
                                ):
                                    raise ValueError("Use finite positive finger collision bounds")
                                finger = self.world.get_link_pose(ctx, name)
                                local_marker = (np.linalg.inv(finger) @ np.append(marker, 1))[:3]
                                gap = float(
                                    np.linalg.norm(local_marker - np.clip(local_marker, lo, hi))
                                    - 0.02235804685
                                )
                                minimum_gap = min(minimum_gap, gap)
                                if gap <= 0:
                                    raise RuntimeError(
                                        "Tabletop path may unintentionally enter radio trigger"
                                    )
                samples += 1
            previous = target
        return {
            "samples": samples,
            "minimum_conservative_gap_m": minimum_gap if math.isfinite(minimum_gap) else None,
        }

    def _support_placement(
        self, state: JointState, trajectory: Any, request: Mapping[str, Any]
    ) -> dict[str, Any]:
        values = dict(zip(state.name, state.position, strict=True))
        previous = np.asarray([values[n] for n in trajectory.joint_names])
        heights = []
        centers = []
        table_inverse = np.linalg.inv(self.table)
        for point in trajectory.points:
            target = np.asarray(point.positions)
            count = max(1, math.ceil(float(np.max(np.abs(target - previous))) / 0.01))
            for fraction in np.linspace(0, 1, count + 1):
                candidate = dict(values)
                candidate.update(
                    zip(
                        trajectory.joint_names,
                        previous + fraction * (target - previous),
                        strict=True,
                    )
                )
                q = JointState(name=state.name, position=[candidate[n] for n in state.name])
                with self.world.scratch_context() as ctx:
                    self.world.set_joint_state(ctx, q)
                    transform = (
                        table_inverse
                        @ self.world.get_link_pose(ctx, "right_gripper_link")
                        @ rigid(request["gripper_from_radio"])
                    )
                vertices = self.vertices @ transform[:3, :3].T + transform[:3, 3]
                if np.any(np.max(np.abs(vertices[:, :2]), axis=0) > self.table_extent[:2] / 2):
                    raise RuntimeError("Placement left table support footprint")
                heights.append(float(vertices[:, 2].min() - self.table_extent[2] / 2))
                centers.append(transform[:3, 3])
            previous = target
        if (
            not heights
            or heights[0] < 0.005
            or min(heights) < -0.002
            or not -0.002 <= heights[-1] <= 0.003
            or any(b > a + 0.0002 for a, b in itertools.pairwise(heights))
            or max(np.linalg.norm(c[:2] - centers[0][:2]) for c in centers) > 0.002
        ):
            raise RuntimeError("Placement violates bounded vertical support contact")
        return {
            "samples": len(heights),
            "initial_height": heights[0],
            "final_height": heights[-1],
            "support_pair_only": ["radio_part_0", "table"],
        }

    def reposition_endpoint(self, state: JointState, request: Mapping[str, Any]) -> dict[str, Any]:
        """Reposition moves both TCPs; its endpoint retains the strict pose contract."""
        errors = {}
        with self.world.scratch_context() as ctx:
            self.world.set_joint_state(ctx, state)
            for side in ("right", "left"):
                goal = rigid(request[f"{side}_target_pose"])
                actual = self.world.get_link_pose(ctx, f"{side}_gripper_link")
                delta = np.linalg.inv(goal) @ actual
                position = float(np.linalg.norm(delta[:3, 3]))
                rotation = float(Rotation.from_matrix(delta[:3, :3]).magnitude())
                if position > 0.002 or rotation > 0.005:
                    raise RuntimeError(f"Reposition {side} endpoint exceeds 2 mm / 0.005 rad")
                errors[side] = {"position_m": position, "rotation_rad": rotation}
        return errors

    def _contact_points(self, request: Mapping[str, Any]) -> tuple[np.ndarray, np.ndarray]:
        contact = request["contact"]
        surface = np.asarray(contact["surface_in_radio"], dtype=float)
        pad = np.asarray(
            contact.get("pad_in_gripper", contact.get("pad_in_left_gripper")), dtype=float
        )
        if (
            surface.shape != (3,)
            or pad.shape != (3,)
            or not np.isfinite(surface).all()
            or not np.isfinite(pad).all()
            or not 0.03 <= np.linalg.norm(pad) <= 0.12
        ):
            raise ValueError("Declare calibrated wxnicr press surface and fingertip")
        if contact.get("source") == "caller_sensor_intent":
            normal = np.asarray(contact.get("outward_normal_in_radio"), dtype=float)
            if (
                normal.shape != (3,)
                or not np.isfinite(normal).all()
                or not np.isclose(np.linalg.norm(normal), 1, atol=1e-6)
                or np.any(surface < self.vertices.min(axis=0) - 0.005)
                or np.any(surface > self.vertices.max(axis=0) + 0.005)
            ):
                raise ValueError("Sensor contact intent is outside physical radio geometry")
            # Match a real face of the only radio part admitted for finger-two
            # contact. An arbitrary plane through the object's interior is unsafe.
            hull = ConvexHull(self.part_vertices[0])
            distances = hull.equations[:, :3] @ surface + hull.equations[:, 3]
            nearest = int(np.argmax(distances))
            if abs(distances[nearest]) > 0.005 or hull.equations[nearest, :3] @ normal < 0.9:
                raise ValueError("Sensor contact must match an outward physical body surface")
        elif np.linalg.norm(surface - [0.0446848528, 0.0420822057, -0.0124619396]) > 0.003:
            raise ValueError("Declare calibrated wxnicr press surface and fingertip")
        return surface, pad

    def holding_sample(self, state: JointState, request: Mapping[str, Any]) -> dict[str, float]:
        """Check measured or interpolated TCPs, including bounded local press contact."""
        with self.world.scratch_context() as ctx:
            self.world.set_joint_state(ctx, state)
            right = self.world.get_link_pose(ctx, "right_gripper_link")
            left = self.world.get_link_pose(ctx, "left_gripper_link")
        delta = np.linalg.inv(rigid(request["holding_pose"])) @ right
        position_error = float(np.linalg.norm(delta[:3, 3]))
        rotation_error = float(Rotation.from_matrix(delta[:3, :3]).magnitude())
        if position_error > 0.002 or rotation_error > 0.005:
            raise RuntimeError("Coordinated press moved holding TCP beyond its constraint")
        result = {"position_error_m": position_error, "rotation_error_rad": rotation_error}
        if request["phase"] == "press_contact":
            surface, pad = self._contact_points(request)
            radio = right @ rigid(request["gripper_from_radio"])
            radio_from_pad = np.linalg.inv(radio) @ left @ np.append(pad, 1)
            displacement = radio_from_pad[:3] - surface
            if not -0.002 <= displacement[0] <= 0.020 or np.linalg.norm(displacement[1:]) > 0.015:
                raise RuntimeError("Left press exceeded 2 mm depth or local contact corridor")
            result["signed_contact_clearance_m"] = float(displacement[0])
        return result

    def _holding_path(
        self, state: JointState, trajectory: Any, request: Mapping[str, Any]
    ) -> dict[str, Any]:
        positions = dict(zip(state.name, state.position, strict=True))
        previous = np.array([positions[n] for n in trajectory.joint_names])
        samples = []
        for point in trajectory.points:
            target = np.asarray(point.positions)
            steps = max(1, math.ceil(float(np.max(np.abs(target - previous))) / 0.01))
            for fraction in np.linspace(0, 1, steps + 1):
                values = dict(positions)
                values.update(
                    zip(
                        trajectory.joint_names,
                        previous + fraction * (target - previous),
                        strict=True,
                    )
                )
                candidate = JointState(name=state.name, position=[values[n] for n in state.name])
                samples.append(self.holding_sample(candidate, request))
            previous = target
        if not samples:
            raise ValueError("Coordinated press requires a nonempty trajectory")
        return {
            "samples": len(samples),
            "max_position_error_m": max(s["position_error_m"] for s in samples),
            "max_rotation_error_rad": max(s["rotation_error_rad"] for s in samples),
        }

    def _support_departure(
        self, state: JointState, trajectory: Any, request: Mapping[str, Any]
    ) -> dict[str, Any]:
        # Check the same effective interpolation, not only nominal Cartesian waypoints.
        positions = dict(zip(state.name, state.position, strict=True))
        local = rigid(request["gripper_from_radio"])
        table_inverse = np.linalg.inv(self.table)
        heights, centers = [], []
        previous = np.array([positions[n] for n in trajectory.joint_names])
        for point in trajectory.points:
            target = np.asarray(point.positions)
            steps = max(1, math.ceil(float(np.max(np.abs(target - previous))) / 0.01))
            for fraction in np.linspace(0, 1, steps + 1):
                values = dict(positions)
                values.update(
                    zip(
                        trajectory.joint_names,
                        previous + fraction * (target - previous),
                        strict=True,
                    )
                )
                candidate = JointState(name=state.name, position=[values[n] for n in state.name])
                with self.world.scratch_context() as ctx:
                    self.world.set_joint_state(ctx, candidate)
                    table_from_radio = (
                        table_inverse @ self.world.get_link_pose(ctx, "right_gripper_link") @ local
                    )
                vertices = self.vertices @ table_from_radio[:3, :3].T + table_from_radio[:3, 3]
                if np.any(np.max(np.abs(vertices[:, :2]), axis=0) > self.table_extent[:2] / 2):
                    raise RuntimeError("Radio departure left the declared support footprint")
                heights.append(float(vertices[:, 2].min() - self.table_extent[2] / 2))
                centers.append(table_from_radio[:3, 3])
            previous = target
        if (
            not heights
            or heights[0] < -0.002
            or min(heights) < heights[0] - 0.0002
            or any(b < a - 0.0002 for a, b in itertools.pairwise(heights))
            or heights[-1] < 0.005
            or max(np.linalg.norm(c[:2] - centers[0][:2]) for c in centers) > 0.002
        ):
            raise RuntimeError("Radio departure violates bounded initial support contact")
        return {
            "samples": len(heights),
            "initial_height": heights[0],
            "final_height": heights[-1],
            "initial_support_pair_only": ["radio_part_0", "table"],
            "oracle_assisted": True,
        }


class RadioGraspCheckpoint:
    """One development operator; same stored SDK plan and effective JTT command."""

    def __init__(
        self,
        arm: Arm,
        coordinator: Any,
        scene: RadioCheckpointScene,
        full_state: Callable[[], JointState],
        observe: Callable[[], Mapping[str, Any]],
        guard: Callable[[str, Mapping[str, Any]], None],
        evidence: list[dict[str, Any]],
        state_from_observation: Callable[[Mapping[str, Any]], JointState] | None = None,
        episode_status: Callable[[], Mapping[str, Any]] | None = None,
        expected_episode: str | None = None,
    ) -> None:
        if arm.info.id != "right_arm":
            raise ValueError("The checkpoint uses the right holding arm")
        self.arm, self.coordinator, self.scene = arm, coordinator, scene
        self.checkpoint_rpc: Any = arm.rpc
        self.full_state, self.observe, self.guard, self.evidence = (
            full_state,
            observe,
            guard,
            evidence,
        )
        self.attachment: np.ndarray | None = None
        self.episode: str | None = expected_episode
        self.episode_status = episode_status
        self.state_from_observation = state_from_observation or (lambda _: self.full_state())

    def _observation(self, phase: str) -> Mapping[str, Any]:
        outcome = self._terminal_outcome()
        if outcome is not None:
            if phase in TABLETOP_BEFORE_PRESS and outcome["kind"] == "TASK_GOAL_MET":
                outcome = {
                    **outcome,
                    "kind": "RUNTIME_FAULT",
                    "reason": "Unintended activation before ordered tabletop press",
                }
            raise RadioEpisodeTerminalError(outcome)
        value = dict(self.observe())
        if self.evidence and self.evidence[-1]["phase"] == phase:
            record = self.evidence[-1]
            value.update(
                radio_interaction_evidence(
                    record["before"], value, record["request"].get("contact")
                )
            )
        if not 0 <= time.monotonic() - value["observed_at_monotonic"] <= 1:
            raise RuntimeError("Checkpoint oracle observation is stale")
        if self.episode is not None and value["episode"] != self.episode:
            raise RuntimeError("Checkpoint episode changed")
        self.episode = value["episode"]
        table_delta = np.linalg.inv(self.scene.table) @ rigid(value["table_pose"])
        if (
            np.linalg.norm(table_delta[:3, 3]) > 0.002
            or Rotation.from_matrix(table_delta[:3, :3]).magnitude() > 0.01
        ):
            raise RuntimeError("Checkpoint supporting table changed")
        try:
            if phase in TABLETOP_BEFORE_PRESS and (
                (value.get("evaluator_toggle_region") or {}).get("finger_contact_steps", 0) > 0
                or (value.get("goal_status") or {}).get("satisfied")
            ):
                raise RuntimeError("Radio trigger activated before ordered tabletop press")
            self.guard(phase, value)
        except RuntimeError as error:
            # Preserve the first contact-loss sample even when the guard stops motion.
            if self.evidence and self.evidence[-1]["phase"] == phase:
                self.evidence[-1]["rejected_observation"] = copy.deepcopy(dict(value))
                self.evidence[-1]["observation_guard_error"] = str(error)
            raise
        return value

    def move(
        self,
        target: PoseStamped,
        phase: str,
        *,
        timeout: float = 20,
        auxiliary_torso: bool = False,
        left_target: PoseStamped | None = None,
        contact: Mapping[str, Any] | None = None,
        dispatch: Callable[[str, float], ExecutionResult] | None = None,
        cancelled: Callable[[], bool] | None = None,
    ) -> None:
        if phase not in PHASES or not 0 < timeout <= 30:
            raise ValueError("Use a bounded checkpoint phase")
        dual = phase in COORDINATED_PHASES
        if dual != (left_target is not None):
            raise ValueError("Dual phases require a left target; single-hand phases omit it")
        if (phase in ("press_contact", "table_press")) != (contact is not None):
            raise ValueError("Declare contact geometry only for press phases")
        if cancelled is not None and cancelled():
            raise RuntimeError("Cancelled before checkpoint planning")
        deadline = time.monotonic() + timeout
        observation = self._observation(phase)
        initial = self.state_from_observation(observation)
        self.scene.world.sync_from_joint_state(initial)
        radio = rigid(observation["radio_pose"])
        if phase in HELD_PHASES:
            with self.scene.world.scratch_context() as ctx:
                self.scene.world.set_joint_state(ctx, initial)
                eef = self.scene.world.get_link_pose(ctx, "right_gripper_link")
            measured_attachment = np.linalg.inv(eef) @ radio
            if self.attachment is None:
                self.attachment = measured_attachment
            self._retained(measured_attachment)
        request = {
            "phase": phase,
            "signature": self.scene.signature,
            "radio_pose": radio.tolist(),
            "gripper_from_radio": self.attachment.tolist() if self.attachment is not None else None,
        }
        if dual:
            assert left_target is not None
            if phase == "reposition":
                request["right_target_pose"] = pose_to_matrix(target).tolist()
                request["left_target_pose"] = pose_to_matrix(left_target).tolist()
            else:
                request["holding_pose"] = pose_to_matrix(target).tolist()
            if contact is not None:
                request["contact"] = copy.deepcopy(dict(contact))
        elif contact is not None:
            request["contact"] = copy.deepcopy(dict(contact))
        record: dict[str, Any] = {
            "phase": phase,
            "before": copy.deepcopy(dict(observation)),
            "measured_start_joints": dict(zip(initial.name, initial.position, strict=True)),
            "target_pose": pose_to_matrix(target).tolist(),
            "request": request,
            "development_only": True,
            "oracle_assisted": True,
            "auxiliary_torso": auxiliary_torso,
        }
        self.evidence.append(record)
        if dual:
            assert left_target is not None
            plan: PlanResult = self.checkpoint_rpc.plan_radio_checkpoint(
                target, request, initial, auxiliary_torso, left_target
            )
            record["left_target_pose"] = pose_to_matrix(left_target).tolist()
        else:
            plan = self.checkpoint_rpc.plan_radio_checkpoint(
                target, request, initial, auxiliary_torso
            )
        record["planning_result"] = {"status": plan.status.name, "message": plan.message}
        if not plan.succeeded or plan.plan is None:
            raise MotionError("plan_radio_checkpoint", plan)
        selected = list(self.arm.info.joint_names)
        if dual:
            left = next(g for g in self.arm.rpc.list_planning_groups() if g.id == "left_arm")
            selected.extend(left.joint_names)
        if auxiliary_torso:
            torso = next(g for g in self.arm.rpc.list_planning_groups() if g.id == "torso")
            selected.extend(torso.joint_names)
        frozen = {
            n: v for n, v in zip(initial.name, initial.position, strict=True) if n not in selected
        }
        record["initial_selected_joint_margin"] = self._measured_joint_margin(initial, selected)
        effective = self.coordinator.prepare_development_trajectory(plan.plan.trajectory)
        record["validation"] = self.scene.validate(initial, effective, selected, request)
        fresh = self.full_state()
        record["dispatch_selected_joint_margin"] = self._measured_joint_margin(fresh, selected)
        before = dict(zip(initial.name, initial.position, strict=True))
        if any(
            n not in before or abs(v - before[n]) > 0.002
            for n, v in zip(fresh.name, fresh.position, strict=True)
        ):
            raise RuntimeError("Checkpoint measured start changed during validation")
        current = self._observation(phase)
        if not np.allclose(rigid(current["radio_pose"]), radio, atol=0.002, rtol=0):
            raise RuntimeError("Checkpoint radio changed before dispatch")
        remaining = deadline - time.monotonic()
        duration = effective.points[-1].time_from_start
        record["dispatch_budget"] = {"remaining_s": remaining, "effective_duration_s": duration}
        if remaining <= 0:
            raise TimeoutError("Checkpoint planning/validation deadline elapsed")
        if duration + 1.0 > remaining:
            raise TimeoutError("Prepared trajectory exceeds remaining checkpoint deadline")
        self.coordinator.authorize_development_trajectory(
            trajectory_digest(plan.plan.trajectory), trajectory_digest(effective)
        )
        try:
            if cancelled is not None and cancelled():
                raise RuntimeError("Cancelled before checkpoint dispatch")
            result = (
                dispatch(plan.plan.plan_id, remaining)
                if dispatch is not None
                else self.arm.rpc.execute(
                    blocking=False, timeout=remaining, plan_id=plan.plan.plan_id
                )
            )
            while result.status in (
                ExecutionStatus.ACCEPTED,
                ExecutionStatus.EXECUTING,
                ExecutionStatus.TIMED_OUT,
            ):
                if cancelled is not None and cancelled():
                    raise RuntimeError("Checkpoint execution cancelled")
                observed = dict(self._observation(phase))
                observed["radio_stage_displacement"] = radio_stage_displacement(
                    radio, rigid(observed["radio_pose"])
                )
                record["during_latest"] = copy.deepcopy(dict(observed))
                record.setdefault("timeline", []).append(copy.deepcopy(dict(observed)))
                state = self.state_from_observation(observed)
                record["timeline"][-1]["measured_joints"] = dict(
                    zip(state.name, state.position, strict=True)
                )
                record["timeline"][-1]["selected_joint_margin"] = self._measured_joint_margin(
                    state, selected
                )
                self._frozen(state, frozen)
                if phase in ("pregrasp", "grasp") and not np.allclose(
                    rigid(observed["radio_pose"]), radio, atol=0.002, rtol=0
                ):
                    raise RuntimeError("Stationary radio displaced during grasp approach")
                if phase in DUAL_PHASES:
                    self.scene.holding_sample(state, request)
                if self.attachment is not None and phase in HELD_PHASES:
                    with self.scene.world.scratch_context() as ctx:
                        self.scene.world.set_joint_state(ctx, state)
                        eef = self.scene.world.get_link_pose(ctx, "right_gripper_link")
                    record["timeline"][-1]["sdk_measured_fk"] = eef.tolist()
                    self._retained(np.linalg.inv(eef) @ rigid(observed["radio_pose"]))
                if time.monotonic() >= deadline:
                    raise TimeoutError("Checkpoint execution deadline elapsed")
                result = self.arm.rpc.wait_for_execution(
                    timeout=min(0.1, deadline - time.monotonic())
                )
            if result.status is not ExecutionStatus.COMPLETED:
                raise MotionError("execute_radio_checkpoint", result)
            # Command-clock completion is not measured arrival.
            while True:
                if cancelled is not None and cancelled():
                    raise RuntimeError("Checkpoint arrival wait cancelled")
                observed = dict(self._observation(phase))
                endstate = self.state_from_observation(observed)
                record["arrival_selected_joint_margin"] = self._measured_joint_margin(
                    endstate, selected
                )
                self._frozen(endstate, frozen)
                with self.scene.world.scratch_context() as ctx:
                    self.scene.world.set_joint_state(ctx, endstate)
                    endpose = self.scene.world.get_link_pose(ctx, "right_gripper_link")
                observed["radio_stage_displacement"] = radio_stage_displacement(
                    radio, rigid(observed["radio_pose"])
                )
                if phase in DUAL_PHASES:
                    self.scene.holding_sample(endstate, request)
                if phase in HELD_PHASES:
                    self._retained(np.linalg.inv(endpose) @ rigid(observed["radio_pose"]))
                delta = np.linalg.inv(pose_to_matrix(target)) @ endpose
                left_arrived = True
                if left_target is not None:
                    with self.scene.world.scratch_context() as ctx:
                        self.scene.world.set_joint_state(ctx, endstate)
                        left_pose = self.scene.world.get_link_pose(ctx, "left_gripper_link")
                    left_delta = np.linalg.inv(pose_to_matrix(left_target)) @ left_pose
                    left_arrived = bool(
                        np.linalg.norm(left_delta[:3, 3])
                        <= (0.002 if phase in COORDINATED_PHASES else 0.005)
                        and Rotation.from_matrix(left_delta[:3, :3]).magnitude()
                        <= (0.005 if phase in COORDINATED_PHASES else 0.01)
                    )
                if (
                    left_arrived
                    and np.linalg.norm(delta[:3, 3])
                    <= (0.002 if phase in COORDINATED_PHASES else 0.005)
                    and Rotation.from_matrix(delta[:3, :3]).magnitude()
                    <= (0.005 if phase in COORDINATED_PHASES else 0.01)
                ):
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError("SDK clock completed without measured target arrival")
                time.sleep(0.02)
            record["after"] = copy.deepcopy(dict(observed))
            record["measured_end_joints"] = dict(zip(endstate.name, endstate.position, strict=True))
            record["measured_arrival_fk"] = endpose.tolist()
            record["coordinator_dispatch"] = self.coordinator.get_development_dispatch_evidence()
            record["execution_status"] = result.status.name
        except BaseException as error:
            terminal = error.outcome if isinstance(error, RadioEpisodeTerminalError) else None
            if terminal is None:
                try:
                    terminal = self._terminal_outcome()
                except Exception as status_error:
                    record["terminal_status_error"] = repr(status_error)
            try:
                record["coordinator_dispatch"] = (
                    self.coordinator.get_development_dispatch_evidence()
                )
            except Exception as capture_error:
                record["dispatch_evidence_error"] = repr(capture_error)
            try:
                cancel_result = self.arm.rpc.cancel()
                record["cancel_result"] = cancel_result
                record["cancel_stop_confirmed"] = cancel_result.status in (
                    ExecutionStatus.ABORTED,
                    ExecutionStatus.NO_EXECUTION,
                    ExecutionStatus.COMPLETED,
                )
            except Exception as cancel_error:
                record["cancel_error"] = str(cancel_error)
            if terminal is not None:
                record["episode_terminal"] = copy.deepcopy(terminal)
                record["motion_arrived"] = False
                expected_terminal_error = isinstance(error, RadioEpisodeTerminalError) or (
                    isinstance(error, RuntimeError)
                    and str(error)
                    in (
                        "Development grasp truth is stale or episode changed",
                        "Checkpoint oracle observation is stale",
                    )
                )
                record["outcome"] = (
                    terminal["kind"]
                    if record.get("cancel_stop_confirmed") and expected_terminal_error
                    else "RUNTIME_FAULT"
                )
                if expected_terminal_error:
                    raise RadioEpisodeTerminalError(
                        {
                            **terminal,
                            "kind": record["outcome"],
                            "stop_confirmed": record.get("cancel_stop_confirmed", False),
                        }
                    ) from error
            else:
                record["outcome"] = (
                    "MOTION_CANCELLED"
                    if isinstance(error, MotionError)
                    and getattr(getattr(error, "result", None), "status", None)
                    is ExecutionStatus.ABORTED
                    else "RUNTIME_FAULT"
                )
            raise

    def _terminal_outcome(self) -> dict[str, Any] | None:
        if self.episode_status is None or self.episode is None:
            return None
        return radio_episode_terminal(self.episode_status(), self.episode)

    def _retained(self, measured: np.ndarray) -> None:
        assert self.attachment is not None
        delta = np.linalg.inv(self.attachment) @ measured
        if (
            np.linalg.norm(delta[:3, 3]) > 0.004
            or Rotation.from_matrix(delta[:3, :3]).magnitude() > 0.03
        ):
            raise RuntimeError("Radio slipped relative to measured holding FK")

    def _measured_joint_margin(self, state: JointState, selected: Sequence[str]) -> dict[str, Any]:
        prepared = self.scene.world.get_prepared_model()
        names = list(prepared.config.joint_names)
        lower, upper = prepared.joint_space.position_limits()
        measured = dict(zip(state.name, state.position, strict=True))
        margins = {}
        for name in selected:
            index = names.index(name)
            value = measured[name]
            if not math.isfinite(value) or not lower[index] <= value <= upper[index]:
                raise RuntimeError(f"Measured joint {name} exceeds actual position limits")
            margins[name] = float(min(value - lower[index], upper[index] - value))
        nearest = min(margins, key=lambda name: margins[name])
        return {"joint": nearest, "margin": margins[nearest]}

    @staticmethod
    def _frozen(state: JointState, reference: Mapping[str, float]) -> None:
        actual = dict(zip(state.name, state.position, strict=True))
        if any(n not in actual or abs(actual[n] - value) > 0.005 for n, value in reference.items()):
            raise RuntimeError("Checkpoint frozen base/opposite arm/torso/grippers changed")


def make_radio_grasp_checkpoint(
    app: Any, sim: Any, description: Mapping[str, Any], report: dict[str, Any]
) -> RadioGraspCheckpoint:
    """Bind existing SDK/runtime with explicitly privileged development feedback.

    description also declares radio/table task keys, physical-to-SDK calibration,
    and the table's source pose/local center. No scene state is set by this factory.
    """
    from dimos.manipulation.planning.factory import create_planning_stack
    from dimos.robot.galaxea.r1pro.config import R1PRO_PLANAR_BASE
    from dimos.robot.galaxea.r1pro.joints import coordinator_name
    from dimos.simulation.behavior.r1pro_model import simulation_model_config

    runtime = app.get_module("ManipulationModule")
    arm = Arm.from_app(app, group="right_arm")
    model = simulation_model_config()
    model.planning_groups = [
        g for g in model.planning_groups if g.name in ("right_arm", "left_arm", "torso")
    ]
    world, _, _ = create_planning_stack(model)
    geometry = dict(description)
    scene = RadioCheckpointScene(world, geometry)
    signature = runtime.configure_radio_checkpoint(geometry)
    if signature != scene.signature:
        raise RuntimeError("Client/runtime checkpoint geometry identity mismatch")
    calibration = np.asarray(description["physical_to_sdk_fk_translation"], dtype=float)
    radio_key, table_key = description["radio_key"], description["table_key"]
    table_local_center = np.asarray(description["table_local_center"], dtype=float)
    if calibration.shape != (3,) or not np.isfinite(calibration).all():
        raise ValueError("Declare finite physical-to-SDK frame calibration")
    lower, upper = world.get_prepared_model().joint_space.position_limits()
    finger_index = model.joint_names.index("r1pro/right_gripper_finger_joint1")
    finger_lower, finger_upper = float(lower[finger_index]), float(upper[finger_index])
    episode = sim.get_status().episode.id

    def truth() -> Mapping[str, Any]:
        value: Mapping[str, Any] = sim.get_ground_truth()
        status = sim.get_status()
        if (
            status.episode.id != episode
            or value.get("episode") != episode
            or not 0 <= time.monotonic() - value.get("observed_at_monotonic", -math.inf) <= 1
            or not 0 <= status.episode.step - value.get("step", -999) <= 2
        ):
            raise RuntimeError("Development grasp truth is stale or episode changed")
        radio, table = value["objects"][radio_key], value["objects"][table_key]
        if (
            radio["name"] != description["radio_name"]
            or radio["model"] != "wxnicr"
            or table["name"] != description["table_name"]
        ):
            raise RuntimeError("Checkpoint task object identity changed")
        if not np.allclose(radio["scale"], [1, 1, 1], atol=1e-4, rtol=0):
            raise RuntimeError("Scaled radio requires revalidated convex collision geometry")
        return value

    def measured_snapshot(value: Mapping[str, Any]) -> JointState:
        agent = value["objects"]["agent.n.01_1"]
        yaw = Rotation.from_quat(agent["orientation"]).as_euler("xyz")[2]
        positions = dict(
            zip(R1PRO_PLANAR_BASE.joint_names, [*agent["position"][:2], float(yaw)], strict=True)
        )
        positions.update(
            {coordinator_name(n): v for n, v in value["measured_joint_positions"].items()}
        )
        return JointState(
            name=model.joint_names, position=[positions[n] for n in model.joint_names]
        )

    def full_state() -> JointState:
        return measured_snapshot(truth())

    def paired_state(value: Mapping[str, Any]) -> JointState:
        if value["joint_snapshot_step"] != value["step"]:
            raise RuntimeError("Checkpoint poses and encoders are from different steps")
        positions = value["measured_joints"]
        return JointState(
            name=model.joint_names, position=[positions[n] for n in model.joint_names]
        )

    def observe() -> Mapping[str, Any]:
        value = truth()
        radio = value["objects"][radio_key]
        table = value["objects"][table_key]
        rotation = Rotation.from_quat(table["orientation"])
        measured = measured_snapshot(value)
        return {
            "episode": episode,
            "step": value["step"],
            "observed_at_monotonic": value["observed_at_monotonic"],
            "joint_snapshot_step": value["step"],
            "measured_joints": dict(zip(measured.name, measured.position, strict=True)),
            "joint_snapshot_source": "Measured robot encoders from the same owner snapshot",
            "radio_pose": pose_to_matrix(
                PoseStamped(
                    frame_id="world",
                    position=np.asarray(radio["position"]) + calibration,
                    orientation=radio["orientation"],
                )
            ).tolist(),
            "table_pose": pose_to_matrix(
                PoseStamped(
                    frame_id="world",
                    position=np.asarray(table["position"])
                    + rotation.apply(table_local_center)
                    + calibration,
                    orientation=table["orientation"],
                )
            ).tolist(),
            "finger_contacts": radio.get("development_finger_contacts"),
            "contact_pairs": copy.deepcopy(radio.get("development_contact_pairs")),
            "all_radio_contact_pairs": copy.deepcopy(radio.get("development_all_contact_pairs")),
            "sleep_aware_radio_contact_pairs": copy.deepcopy(
                radio.get("development_sleep_aware_contact_pairs")
            ),
            "evaluator_radio_body": copy.deepcopy(radio.get("development_body_diagnostics")),
            "evaluator_assisted_grasp": copy.deepcopy(radio.get("development_assisted_grasp")),
            "evaluator_toggle_region": copy.deepcopy(radio.get("toggle_region")),
            # Preserve full floating-base orientation to diagnose planar-FK
            # disagreement without hiding roll/pitch in the yaw-only model.
            "evaluator_robot_base": copy.deepcopy(value["objects"]["agent.n.01_1"]),
            "evaluator_finger_poses": {
                name: pose_to_matrix(
                    PoseStamped(
                        frame_id="world",
                        position=np.asarray(link["position"]) + calibration,
                        orientation=link["orientation"],
                    )
                ).tolist()
                for name, link in value["links"].items()
                if "gripper_finger" in name
            },
            "evaluator_left_gripper_pose": pose_to_matrix(
                PoseStamped(
                    frame_id="world",
                    position=np.asarray(value["links"]["left_gripper_link"]["position"])
                    + calibration,
                    orientation=value["links"]["left_gripper_link"]["orientation"],
                )
            ).tolist(),
            "evaluator_gripper_pose": pose_to_matrix(
                PoseStamped(
                    frame_id="world",
                    position=np.asarray(value["links"]["right_gripper_link"]["position"])
                    + calibration,
                    orientation=value["links"]["right_gripper_link"]["orientation"],
                )
            ).tolist(),
            # Same encoder and normalization as the SDK scalar, without pairing
            # the scene snapshot with a later gripper RPC observation or clamping.
            "measured_gripper": (measured.position[finger_index] - finger_lower)
            / (finger_upper - finger_lower),
            "goal_status": copy.deepcopy(value["goal_status"]),
            "source": "evaluator-only development diagnostics; not agent observations",
        }

    retention = AssistedRadioRetention()

    def guard(phase: str, value: Mapping[str, Any]) -> None:
        if phase in HELD_PHASES:
            retention.check(value)
        elif phase in ("retract", "table_approach", "table_press"):
            assistance = (value.get("evaluator_assisted_grasp") or {}).get("right", {})
            if (
                assistance.get("candidate_in_hand") is not False
                or assistance.get("constraint_valid") is not False
                or assistance.get("release_counter") is not None
            ):
                raise RuntimeError("Release official assisted attachment before tabletop action")
            if phase != "table_press" and any((value.get("finger_contacts") or {}).values()):
                raise RuntimeError("Clear physical finger contact before tabletop approach")

    world.sync_from_joint_state(full_state())
    return RadioGraspCheckpoint(
        arm,
        app.get_module("ControlCoordinator"),
        scene,
        full_state,
        observe,
        guard,
        report.setdefault("checkpoint_stages", []),
        state_from_observation=paired_state,
        episode_status=lambda: sim.get_status().model_dump(mode="json"),
        expected_episode=episode,
    )
