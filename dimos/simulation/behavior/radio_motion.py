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

"""Development-only radio path checks; oracle scene geometry is never a policy sensor."""

from collections.abc import Callable, Mapping, Sequence
import copy
import hashlib
import json
import math
import time
from typing import Any

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.core.core import rpc
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import (
    ExecutionResult,
    ExecutionStatus,
    PlanResult,
    PlanStatus,
)
from dimos.manipulation.planning.planners.config import CartesianPathConfig
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.sdk import Arm, MotionError
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.simulation.behavior.r1pro_bridge import BehaviorCoordinator


def trajectory_payload(trajectory: JointTrajectory) -> dict[str, Any]:
    """Hash command content independent of transport column ordering/timestamp."""
    names = trajectory.joint_names
    if not names or len(set(names)) != len(names) or not trajectory.points:
        raise ValueError("Trajectory requires unique joints and points")
    columns = sorted(range(len(names)), key=names.__getitem__)
    rows = []
    previous = -math.inf
    for point in trajectory.points:
        if len(point.positions) != len(names) or len(point.velocities) != len(names):
            raise ValueError("Incomplete trajectory point")
        if not all(
            math.isfinite(v) for v in (*point.positions, *point.velocities, point.time_from_start)
        ):
            raise ValueError("Nonfinite trajectory")
        if point.time_from_start < 0 or point.time_from_start <= previous:
            raise ValueError("Trajectory times must strictly increase")
        previous = point.time_from_start
        rows.append(
            [
                float(point.time_from_start),
                [float(point.positions[i]) for i in columns],
                [float(point.velocities[i]) for i in columns],
            ]
        )
    return {"joint_names": [names[i] for i in columns], "points": rows}


def trajectory_digest(trajectory: JointTrajectory) -> str:
    return hashlib.sha256(
        json.dumps(trajectory_payload(trajectory), separators=(",", ":"), sort_keys=True).encode()
    ).hexdigest()


def trajectory_payload_diff(
    expected: Mapping[str, Any], actual: Mapping[str, Any]
) -> dict[str, Any]:
    if expected["joint_names"] != actual["joint_names"] or len(expected["points"]) != len(
        actual["points"]
    ):
        return {"shape_changed": True}
    changes = []
    for index, (before, after) in enumerate(zip(expected["points"], actual["points"], strict=True)):
        if before != after:
            changes.append(
                {
                    "point": index,
                    "before": before,
                    "after": after,
                    "position_delta": [b - a for a, b in zip(before[1], after[1], strict=True)],
                }
            )
    return {"shape_changed": False, "changed_points": changes}


def anchored_trajectory(
    trajectory: JointTrajectory, anchors: Mapping[str, float]
) -> JointTrajectory:
    """Mirror JTT: only cached commands replace the planned first waypoint.

    Fresh measured feedback initializes execution; it does not rewrite an
    uncached joint's stored command. Snapshot every point independently.
    """
    if len(trajectory.points) < 2:
        raise ValueError("Radio motion requires a multi-point trajectory")
    snapshot = copy.deepcopy(trajectory)
    snapshot.points[0].positions = [
        anchors.get(n, snapshot.points[0].positions[i]) for i, n in enumerate(snapshot.joint_names)
    ]
    return snapshot


DEVELOPMENT_START_TOLERANCE = 0.002


def check_development_start(
    trajectory: JointTrajectory, current: Mapping[str, float]
) -> dict[str, float]:
    """Bound initial encoder drift without changing the stored command path."""
    errors = {
        n: abs(current.get(n, math.nan) - p)
        for n, p in zip(trajectory.joint_names, trajectory.points[0].positions, strict=True)
    }
    if any(not math.isfinite(v) or v > DEVELOPMENT_START_TOLERANCE for v in errors.values()):
        raise RuntimeError("Radio measured start exceeds development tolerance")
    return errors


class RadioCoordinator(BehaviorCoordinator):
    """Task-local admission check for the effective JTT command, under its lock."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._radio_ticket: tuple[str, str, float] | None = None
        self._radio_validation_required = False
        self._radio_evidence: dict[str, Any] = {}

    @rpc
    def prepare_development_trajectory(self, trajectory: JointTrajectory) -> JointTrajectory:
        current = self.get_joint_positions()
        with self._task_lock:
            if self._trajectory_task is None:
                raise RuntimeError("No trajectory task")
            self._radio_validation_required = True
            self._radio_ticket = None
            anchors = dict(self._trajectory_task._commanded_positions)
            effective = anchored_trajectory(trajectory, anchors)
            errors = check_development_start(effective, current)
            self._radio_evidence = {
                "prepared": {
                    "original": trajectory_payload(trajectory),
                    "effective": trajectory_payload(effective),
                    "cached_commands": anchors,
                    "measured": copy.deepcopy(current),
                    "start_errors": errors,
                }
            }
            return effective

    @rpc
    def authorize_development_trajectory(self, original_digest: str, effective_digest: str) -> None:
        with self._task_lock:
            if not self._radio_validation_required:
                raise RuntimeError("Prepare the development trajectory first")
            self._radio_ticket = (original_digest, effective_digest, time.monotonic())

    @rpc
    def get_development_dispatch_evidence(self) -> dict[str, Any]:
        with self._task_lock:
            return copy.deepcopy(self._radio_evidence)

    @rpc
    def task_invoke(self, task_name: str, method: str, kwargs: dict[str, Any] | None = None) -> Any:
        if (
            task_name != "joint_trajectory"
            or method != "execute"
            or not self._radio_validation_required
        ):
            return super().task_invoke(task_name, method, kwargs)
        current = self.get_joint_positions()
        with self._task_lock:
            ticket, self._radio_ticket = self._radio_ticket, None
            task = self._trajectory_task
            if ticket is None or task is None or time.monotonic() - ticket[2] > 1.0:
                return TrajectoryExecutionResult(
                    TrajectoryExecutionStatus.INVALID_TRAJECTORY, "Missing/expired radio validation"
                )
            trajectory = (kwargs or {}).get("trajectory")
            if not isinstance(trajectory, JointTrajectory):
                return TrajectoryExecutionResult(
                    TrajectoryExecutionStatus.INVALID_TRAJECTORY, "Missing trajectory"
                )
            anchors = dict(task._commanded_positions)
            effective = anchored_trajectory(trajectory, anchors)
            original_payload, effective_payload = (
                trajectory_payload(trajectory),
                trajectory_payload(effective),
            )
            self._radio_evidence["dispatch"] = {
                "original": original_payload,
                "effective": effective_payload,
                "cached_commands": anchors,
                "measured": copy.deepcopy(current),
                "original_diff": trajectory_payload_diff(
                    self._radio_evidence["prepared"]["original"], original_payload
                ),
                "effective_diff": trajectory_payload_diff(
                    self._radio_evidence["prepared"]["effective"], effective_payload
                ),
            }
            if (trajectory_digest(trajectory), trajectory_digest(effective)) != ticket[:2]:
                self._radio_evidence["rejection"] = "path_or_cached_anchor_changed"
                return TrajectoryExecutionResult(
                    TrajectoryExecutionStatus.INVALID_TRAJECTORY,
                    "Radio path/cached anchor changed after validation",
                )
            try:
                self._radio_evidence["dispatch"]["start_errors"] = check_development_start(
                    effective, current
                )
            except RuntimeError as error:
                self._radio_evidence["rejection"] = "measured_start_out_of_tolerance"
                return TrajectoryExecutionResult(
                    TrajectoryExecutionStatus.START_STATE_MISMATCH, str(error)
                )
            # Same lock protects JTT execution and anchor lookup. Its first-point
            # adjustment is exactly the effective trajectory checked above.
            return task.execute(trajectory, current)


class RadioManipulationModule(ManipulationModule):
    """Split development Cartesian planning from execution for external checks."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._development_scene: dict[str, Any] | None = None
        self._development_scene_world: WorldSpec | None = None

    @rpc
    def get_development_collision_scene(self) -> dict[str, Any] | None:
        """Owner-only development metadata; never a policy observation."""
        with self._lock:
            if (
                self._world_monitor is None
                or self._world_monitor.world is not self._development_scene_world
            ):
                return None
            return copy.deepcopy(self._development_scene)

    @rpc
    def configure_development_collision_scene(
        self,
        objects: Sequence[Mapping[str, Any]],
        source_definition: Mapping[str, Any] | None = None,
        scene_reference: Mapping[str, Any] | None = None,
    ) -> None:
        """Register once; repeated owners must supply identical declared geometry.

        The first owner retains the collision boxes and measured scene reference.
        A subsequent factory reuses those boxes after checking live scene drift;
        it must not move boxes implicitly or adopt conflicting geometry.
        """
        from dimos.manipulation.planning.spec.enums import ObstacleType
        from dimos.manipulation.planning.spec.models import Obstacle
        from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld

        if len(objects) != 2 or {obj["name"] for obj in objects} != {"radio", "table"}:
            raise ValueError("Only the development radio/support geometry is supported")
        for obj in objects:
            for key, size in (("position", 3), ("orientation", 4), ("extent", 3)):
                values = obj[key]
                if len(values) != size or not all(math.isfinite(v) for v in values):
                    raise ValueError("Development geometry requires finite coordinates")
            if any(v <= 0 for v in obj["extent"]) or not any(obj["orientation"]):
                raise ValueError("Development geometry requires positive dimensions/rotation")
        record: dict[str, Any] = copy.deepcopy(
            {
                "objects": sorted(objects, key=lambda obj: obj["name"]),
                "source_definition": source_definition,
                "scene_reference": scene_reference,
            }
        )
        with self._lock:
            if self._world_monitor is None or not isinstance(
                self._world_monitor.world, RoboPlanWorld
            ):
                raise RuntimeError("Radio runtime planning world unavailable")
            world = self._world_monitor.world
            if world is self._development_scene_world and self._development_scene is not None:
                if record != self._development_scene:
                    raise ValueError("Conflicting development collision geometry or ownership")
                actual_boxes = {obstacle.name: obstacle for obstacle in world.get_obstacles()}
                for obj in record["objects"]:
                    actual = actual_boxes.get(obj["name"])
                    if (
                        actual is None
                        or actual.obstacle_type is not ObstacleType.BOX
                        or tuple(actual.dimensions) != tuple(obj["extent"])
                        or actual.pose.frame_id != "world"
                        or actual.pose.position.to_tuple() != tuple(obj["position"])
                        or actual.pose.orientation.to_tuple() != tuple(obj["orientation"])
                    ):
                        raise ValueError("Registered development collision geometry was changed")
                return
            existing_names = {obstacle.name for obstacle in world.get_obstacles()}
            if existing_names & {"radio", "table"}:
                raise ValueError("Development obstacle names already belong to another owner")
            added_names: list[str] = []
            try:
                for obj in record["objects"]:
                    added = self._world_monitor.add_obstacle(
                        Obstacle(
                            name=obj["name"],
                            obstacle_type=ObstacleType.BOX,
                            pose=PoseStamped(
                                frame_id="world",
                                position=obj["position"],
                                orientation=obj["orientation"],
                            ),
                            dimensions=tuple(obj["extent"]),
                        )
                    )
                    if added != obj["name"]:
                        raise RuntimeError("Development collision geometry was not registered")
                    added_names.append(added)
                world._require_scene().setCollisions("radio", "table", False)
            except BaseException:
                # Roll back only geometry admitted by this registration attempt.
                for name in reversed(added_names):
                    self._world_monitor.remove_obstacle(name)
                raise
            self._development_scene_world = world
            self._development_scene = record

    @rpc
    def set_development_contact_pairs(self, side: str, enabled: bool) -> None:
        from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld

        if (
            side not in ("left", "right")
            or self._world_monitor is None
            or not isinstance(self._world_monitor.world, RoboPlanWorld)
        ):
            raise ValueError("Invalid development contact configuration")
        for index in (1, 2):
            self._world_monitor.world._require_scene().setCollisions(
                f"{side}_gripper_finger_link{index}", "radio", not enabled
            )

    @rpc
    def plan_development_linear(
        self, group: str, delta: Sequence[float], auxiliary_groups: Sequence[str] = ()
    ) -> PlanResult:
        if len(delta) != 3 or not all(math.isfinite(v) for v in delta):
            raise ValueError("Use three finite displacement coordinates")
        plan = self.generate_cartesian_plan(
            {group: (Transform.identity(), Transform(translation=Vector3(*delta)))},
            CartesianPathConfig(),
            auxiliary_groups=auxiliary_groups,
            speed_scale=0.1,
            check_collision=True,
        )
        return (
            PlanResult(PlanStatus.SUCCEEDED, plan.message, plan)
            if plan
            else PlanResult(PlanStatus.FAILED, self._error_message)
        )


def validate_development_trajectory(
    world: WorldSpec,
    full_state: JointState,
    trajectory: JointTrajectory,
    selected_joints: Sequence[str],
) -> dict[str, Any]:
    """Check the actual linearly interpolated command, including start transition.

    RoboPlan samples edges at 0.01 in configuration space. This is a discrete
    model check, not a proof of continuous collision clearance or tracking.
    """
    digest = trajectory_digest(trajectory)
    if set(trajectory.joint_names) != set(selected_joints):
        raise ValueError("Trajectory changed selected joints or includes frozen joints")
    positions = dict(zip(full_state.name, full_state.position, strict=True))
    if not set(selected_joints) <= positions.keys():
        raise ValueError("Incomplete measured robot state")
    world.sync_from_joint_state(full_state)
    previous = full_state
    for point in trajectory.points:
        values = dict(positions)
        values.update(zip(trajectory.joint_names, point.positions, strict=True))
        state = JointState(name=full_state.name, position=[values[n] for n in full_state.name])
        if not world.check_config_collision_free(state) or not world.check_edge_collision_free(
            previous, state, step_size=0.01
        ):
            raise RuntimeError("Radio trajectory intersects the development collision scene")
        previous = state
    return {
        "effective_digest": digest,
        "waypoints": len(trajectory.points),
        "edge_step": 0.01,
        "development_only": True,
    }


def assert_scene_unchanged(reference: Mapping[str, Any], current: Mapping[str, Any]) -> None:
    """Reject a moved/replaced radio or support, rather than silently retarget."""
    for key, expected in reference.items():
        actual = current.get(key)
        if (
            actual is None
            or actual.get("name") != expected.get("name")
            or ("model" in expected and actual.get("model") != expected.get("model"))
        ):
            raise RuntimeError("Development collision scene object changed")
        if math.dist(actual["position"], expected["position"]) > 0.002:
            raise RuntimeError("Development collision scene position is stale")
        a, b = actual["orientation"], expected["orientation"]
        cosine = abs(sum(x * y for x, y in zip(a, b, strict=True))) / math.sqrt(
            sum(x * x for x in a) * sum(x * x for x in b)
        )
        if 2 * math.acos(min(1.0, cosine)) > 0.01 or (
            "scale" in expected and actual.get("scale") != expected.get("scale")
        ):
            raise RuntimeError("Development collision scene rotation/scale is stale")


class DevelopmentRadioMotion:
    """Single-client, development-only checked planning and same-ID execution.

    Scene pose is checked again immediately before authorization. It can still
    change during physical execution; stage diagnostics observe that limitation.
    """

    def __init__(
        self,
        arm: Arm,
        coordinator: Any,
        linear_rpc: Any,
        world: WorldSpec,
        full_state: Callable[[], JointState],
        scene: Callable[[], Mapping[str, Any]],
        reference: Mapping[str, Any],
        selected_joints: Sequence[str],
        contact_pairs: Callable[[bool], None],
        evidence: list[dict[str, Any]],
        auxiliary_groups: Sequence[str],
    ) -> None:
        self.arm, self.coordinator, self.linear_rpc, self.world = (
            arm,
            coordinator,
            linear_rpc,
            world,
        )
        self.full_state, self.scene, self.reference = full_state, scene, reference
        self.selected_joints, self.contact_pairs, self.evidence = (
            selected_joints,
            contact_pairs,
            evidence,
        )
        self.auxiliary_groups = auxiliary_groups
        initial = full_state()
        self.frozen = {
            n: v
            for n, v in zip(initial.name, initial.position, strict=True)
            if n not in selected_joints
        }

    def move(
        self,
        position: Sequence[float],
        orientation: Sequence[float] | None,
        timeout: float,
        contact: bool,
        executor: Callable[[str, float], ExecutionResult] | None = None,
    ) -> None:
        deadline = time.monotonic() + timeout
        state = self.full_state()
        self._check_frozen(state)
        assert_scene_unchanged(self.reference, self.scene())
        self.contact_pairs(contact)
        if contact:
            xyz = self.arm.pose().position.to_tuple()
            plan = self.linear_rpc.plan_development_linear(
                self.arm.info.id,
                [b - a for a, b in zip(xyz, position, strict=True)],
                self.auxiliary_groups,
            )
        else:
            target = PoseStamped(
                frame_id="world",
                position=position,
                orientation=orientation if orientation is not None else self.arm.pose().orientation,
            )
            plan = self.arm.rpc.plan_to_poses(
                {self.arm.info.id: target}, speed_scale=0.15, auxiliary_groups=self.auxiliary_groups
            )
        if not plan.succeeded or plan.plan is None:
            raise MotionError("checked_radio_plan", plan)
        trajectory = plan.plan.trajectory
        effective = self.coordinator.prepare_development_trajectory(trajectory)
        checked = validate_development_trajectory(
            self.world, state, effective, self.selected_joints
        )
        self.evidence.append(
            {
                **checked,
                "plan_id": plan.plan.plan_id,
                "original_digest": trajectory_digest(trajectory),
                "contact_pairs_allowed": contact,
            }
        )
        # Validation can be expensive. Reject state/scene drift after the check.
        fresh = self.full_state()
        self._check_frozen(fresh)
        before = dict(zip(state.name, state.position, strict=True))
        if any(
            n not in before or abs(v - before[n]) > 0.002
            for n, v in zip(fresh.name, fresh.position, strict=True)
        ):
            raise RuntimeError("Measured trajectory start changed after validation")
        assert_scene_unchanged(self.reference, self.scene())
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("Radio path validation deadline elapsed")
        self.coordinator.authorize_development_trajectory(
            trajectory_digest(trajectory), checked["effective_digest"]
        )
        try:
            result = (
                executor(plan.plan.plan_id, remaining)
                if executor is not None
                else self.arm.rpc.execute(
                    blocking=True, timeout=remaining, plan_id=plan.plan.plan_id
                )
            )
        finally:
            try:
                self.evidence[-1]["coordinator_dispatch"] = (
                    self.coordinator.get_development_dispatch_evidence()
                )
            except Exception as error:
                self.evidence[-1]["dispatch_evidence_error"] = str(error)
        if result.status is not ExecutionStatus.COMPLETED:
            raise MotionError("checked_radio_execute", result)

    def _check_frozen(self, state: JointState) -> None:
        positions = dict(zip(state.name, state.position, strict=True))
        if any(n not in positions or abs(positions[n] - v) > 0.005 for n, v in self.frozen.items()):
            raise RuntimeError("Frozen base/opposite arm/gripper state changed")


def make_development_motion(
    app: Any,
    sim: Any,
    arm: Arm,
    target: Mapping[str, Any],
    report: dict[str, Any],
    auxiliary_groups: Sequence[str],
) -> DevelopmentRadioMotion:
    # Optional native backend is imported only for explicit development contact,
    # so discovery and lightweight tests do not require installed RoboPlan assets.
    import numpy as np
    from scipy.spatial.transform import Rotation

    from dimos.manipulation.planning.factory import create_planning_stack
    from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
    from dimos.manipulation.planning.spec.enums import ObstacleType
    from dimos.manipulation.planning.spec.models import Obstacle
    from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld
    from dimos.robot.galaxea.r1pro.config import R1PRO_PLANAR_BASE
    from dimos.robot.galaxea.r1pro.joints import coordinator_name
    from dimos.simulation.behavior.r1pro_model import simulation_model_config

    geometry = target.get("collision_scene")
    if not geometry or len(geometry["objects"]) != 2:
        raise ValueError("Development press requires explicit radio/table collision_scene")
    keys = [obj["key"] for obj in geometry["objects"]]
    episode = sim.get_status().episode.id

    def truth() -> Mapping[str, Any]:
        value: Mapping[str, Any] = sim.get_ground_truth()
        status = sim.get_status()
        age = time.monotonic() - value.get("observed_at_monotonic", -math.inf)
        if (
            status.episode.id != episode
            or value.get("episode") != episode
            or status.episode.step - value.get("step", -999) > 2
            or not 0 <= age <= 1.0
        ):
            raise RuntimeError("Development truth snapshot is stale or episode changed")
        return value

    model = simulation_model_config()
    model.planning_groups = [
        g for g in model.planning_groups if g.name in (arm.info.id, *auxiliary_groups)
    ]
    world, _, _ = create_planning_stack(model)
    if not isinstance(world, RoboPlanWorld):
        raise TypeError("Development radio guard requires RoboPlan")
    runtime = app.get_module("ManipulationModule")
    reference_truth = truth()
    current_reference = {key: reference_truth["objects"][key] for key in keys}
    registered = runtime.get_development_collision_scene()
    if registered is not None:
        if registered["source_definition"] != geometry or registered["scene_reference"] is None:
            raise ValueError("Conflicting development collision geometry or ownership")
        reference = registered["scene_reference"]
        assert_scene_unchanged(reference, current_reference)
        runtime_objects = registered["objects"]
    else:
        reference = copy.deepcopy(current_reference)
        calibration = np.array(geometry["physical_to_sdk_fk_translation"])
        if calibration.shape != (3,) or not np.isfinite(calibration).all():
            raise ValueError("Use an explicit finite geometry calibration")
        runtime_objects = []
        for obj in geometry["objects"]:
            actual = reference[obj["key"]]
            assert_scene_unchanged({obj["key"]: obj["reference"]}, reference)
            scale = actual.get("scale")
            if scale is None or any(not math.isfinite(v) or not 0 < v <= 1.001 for v in scale):
                raise RuntimeError("Nonconservative asset scale requires CPU geometry revalidation")
            rotation = Rotation.from_quat(actual["orientation"])
            center = (
                np.array(actual["position"]) + rotation.apply(obj["local_center"]) + calibration
            )
            runtime_objects.append(
                {
                    "name": obj["name"],
                    "position": center.tolist(),
                    "orientation": actual["orientation"],
                    "extent": obj["extent"],
                }
            )
    runtime.configure_development_collision_scene(runtime_objects, geometry, reference)
    for obj in runtime_objects:
        world.add_obstacle(
            Obstacle(
                name=obj["name"],
                obstacle_type=ObstacleType.BOX,
                pose=PoseStamped(
                    frame_id="world", position=obj["position"], orientation=obj["orientation"]
                ),
                dimensions=tuple(obj["extent"]),
            )
        )
    probe = app.get_module("BehaviorProbe")

    def full_state() -> JointState:
        value = truth()
        agent = value["objects"]["agent.n.01_1"]
        snapshot = probe.snapshot()
        age = time.time() - snapshot.get("joint_state_timestamp", -math.inf)
        if not 0 <= age <= 1.0:
            raise RuntimeError("Development joint feedback is stale")
        joints = snapshot["joints"]
        yaw = Rotation.from_quat(agent["orientation"]).as_euler("xyz")[2]
        positions = dict(
            zip(R1PRO_PLANAR_BASE.joint_names, [*agent["position"][:2], float(yaw)], strict=True)
        )
        positions.update({coordinator_name(n): v for n, v in joints.items()})
        return JointState(
            name=model.joint_names, position=[positions[n] for n in model.joint_names]
        )

    world.sync_from_joint_state(full_state())
    # The conservative boxes overlap at their real support surface. This only
    # permits the static support pair, never robot-vs-support contact.
    world._require_scene().setCollisions("table", "radio", False)
    side = arm.info.id.split("_", 1)[0]

    def contact_pairs(enabled: bool) -> None:
        runtime.set_development_contact_pairs(side, enabled)
        for index in (1, 2):
            world._require_scene().setCollisions(
                f"{side}_gripper_finger_link{index}", "radio", not enabled
            )

    def scene() -> Mapping[str, Any]:
        value = truth()
        return {key: value["objects"][key] for key in keys}

    selected = PlanningGroupRegistry(model.planning_groups).select([arm.info.id, *auxiliary_groups])
    report["development_collision_reference"] = reference
    return DevelopmentRadioMotion(
        arm,
        app.get_module("ControlCoordinator"),
        app.get_module("ManipulationModule"),
        world,
        full_state,
        scene,
        reference,
        selected.joint_names,
        contact_pairs,
        report.setdefault("validated_plans", []),
        auxiliary_groups,
    )
