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

"""Development-only policy facade; evaluator geometry stays with the supervisor.

This narrows the supported policy contract, not Python/process access. Broad
code-execution isolation is a separate concern. It makes no fairness claim.
"""

from collections.abc import Callable, Mapping, Sequence
import copy
from dataclasses import dataclass, replace
import math
from pathlib import Path
import threading
import time
from typing import Any, Literal, cast
from uuid import uuid4

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.manipulation.manipulation_spec import CommandResult, ExecutionResult, ExecutionStatus
from dimos.manipulation.sdk import Arm
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.simulation.behavior.radio_checkpoint import (
    RadioEpisodeTerminalError,
    make_radio_grasp_checkpoint,
)
from dimos.simulation.behavior.radio_evidence import (
    CAMERA_STREAMS,
    observation_fingerprint,
    persist_grounding,
    persist_observation,
)
from dimos.simulation.behavior.radio_policy_checkpoint import (
    SINGLE_HAND_PHASES,
    RadioCheckpointPolicyMotion,
)


@dataclass(frozen=True)
class PolicyAction:
    id: str
    state: Literal["running", "completed", "failed", "cancelled", "uncertain"] = "running"
    error: str | None = None
    cancel_requested: bool = False
    stop_confirmed: bool = False


def vector(values: Sequence[float], size: int) -> tuple[float, ...]:
    if len(values) != size or any(not math.isfinite(v) for v in values):
        raise ValueError(f"Expected {size} finite coordinates")
    return tuple(float(v) for v in values)


def camera_observation(raw: Mapping[str, Any], camera: str) -> dict[str, Any]:
    """Copy one actual RGB/depth bundle and only camera-to-base robot TF.

    World odometry, task-object state, goals, evaluator images, and arbitrary
    fields are excluded. Streams are asynchronous: reject stale or skewed data.
    """
    if camera not in CAMERA_STREAMS:
        raise ValueError("Use head or left_wrist")
    capture = {"capture_id": None, "episode": None, "step": None, "captured_at": None}
    if "capture_id" in raw or "sensors" in raw:
        if (
            not isinstance(raw.get("capture_id"), str)
            or not raw["capture_id"]
            or not isinstance(raw.get("episode"), str)
            or not raw["episode"]
            or type(raw.get("step")) is not int
            or raw["step"] < 0
            or not isinstance(raw.get("captured_at"), (int, float))
            or not math.isfinite(raw["captured_at"])
            or not isinstance(raw.get("sensors"), Mapping)
        ):
            raise ValueError("Invalid atomic sensor capture metadata")
        capture = {key: raw[key] for key in capture}
        raw = raw["sensors"]
    rgb_key, depth_key, calibration_key, frame = CAMERA_STREAMS[camera]
    rgb, depth, calibration = (raw[k] for k in (rgb_key, depth_key, calibration_key))
    if not isinstance(rgb, Image) or not isinstance(depth, Image):
        raise ValueError("Actual camera messages required")
    stamps = (rgb.ts, depth.ts, calibration.ts)
    if any(not math.isfinite(t) for t in stamps) or max(stamps) - min(stamps) > 0.1:
        raise ValueError("Camera streams are not sufficiently synchronized")
    if not 0 <= time.time() - min(stamps) <= 1:
        raise ValueError("Camera observations are stale")
    if rgb.frame_id != frame or depth.frame_id != frame or calibration.frame_id != frame:
        raise ValueError("Camera frame mismatch")
    if rgb.data.shape[:2] != depth.data.shape or depth.data.shape != (
        calibration.height,
        calibration.width,
    ):
        raise ValueError("Camera dimensions mismatch")
    if any(not math.isfinite(v) or v != 0 for v in calibration.D):
        raise ValueError("Policy grounding requires undistorted pinhole camera data")
    transforms = [
        t for t in raw["tf"].transforms if t.frame_id == "base_link" and t.child_frame_id == frame
    ]
    if len(transforms) != 1 or abs(transforms[0].ts - rgb.ts) > 0.1:
        raise ValueError("Camera-to-base transform unavailable or skewed")
    if capture["capture_id"] is not None and any(
        stamp != capture["captured_at"] for stamp in (*stamps, transforms[0].ts)
    ):
        raise ValueError("Sensor messages do not belong to the same native capture")
    observation = {
        "id": f"{capture['capture_id']}:{camera}" if capture["capture_id"] else uuid4().hex,
        "camera": camera,
        "capture": capture,
        "rgb": rgb.copy(),
        "depth": depth.copy(),
        "calibration": copy.deepcopy(calibration),
        "camera_to_base": copy.deepcopy(transforms[0]),
        "provenance": "RGB/depth/calibration and robot TF; no task-object/evaluator truth",
    }
    observation["fingerprint"] = observation_fingerprint(observation)
    return observation


def ground_pixel(observation: Mapping[str, Any], u: int, v: int) -> dict[str, Any]:
    """Unproject a policy-selected RGB pixel with optical-Z depth into base XYZ.

    This does not detect a radio, infer a switch normal, or choose a target.
    Invalid/missing depth and image borders fail rather than borrowing truth.
    """
    if observation.get("fingerprint") != observation_fingerprint(observation):
        raise ValueError("Observation content no longer matches its fingerprint")
    image = observation["depth"]
    if type(u) is not int or type(v) is not int:
        raise ValueError("Use integer pixel coordinates")
    height, width = image.data.shape
    if not 1 <= u < width - 1 or not 1 <= v < height - 1:
        raise ValueError("Pixel requires an in-image 3x3 depth neighborhood")
    window = image.data[v - 1 : v + 2, u - 1 : u + 2]
    valid = window[np.isfinite(window) & (window > 0)]
    if valid.size < 5:
        raise ValueError("Insufficient valid measured depth")
    z = float(np.median(valid))
    if float(np.max(valid) - np.min(valid)) > 0.02:
        raise ValueError("Depth discontinuity requires another observation")
    k = np.asarray(observation["calibration"].K, dtype=float).reshape(3, 3)
    if not np.isfinite(k).all() or abs(np.linalg.det(k)) < 1e-12:
        raise ValueError("Invalid camera intrinsics")
    ray = np.linalg.solve(k, [u, v, 1])
    if ray[2] <= 0:
        raise ValueError("Invalid optical depth ray")
    point = ray * (z / ray[2])
    transform = observation["camera_to_base"]
    xyz = Rotation.from_quat(transform.rotation.to_tuple()).apply(point)
    xyz += np.asarray(transform.translation.to_tuple())
    return {
        "position": xyz.tolist(),
        "frame": "base_link",
        "observation_id": observation["id"],
        "fingerprint": observation["fingerprint"],
        "capture": copy.deepcopy(observation["capture"]),
        "pixel": [u, v],
        "depth": z,
        "provenance": "Policy-selected RGB pixel + measured optical-Z depth + robot TF",
    }


class RadioPolicySupervisor:
    """One owned arm/action; truth-aware checked motion is never returned to policy.

    Admission and cancellation share a lock. Deadline cancellation is automatic,
    and uncertain stop latches the supervisor against subsequent commands.
    """

    def __init__(
        self,
        arm: Arm,
        motion: Any,
        observe: Callable[[], Mapping[str, Any]],
        base_to_world: Callable[[], Transform],
        evidence_directory: Path | None = None,
    ) -> None:
        self._evidence_directory = evidence_directory
        self._arm, self._motion = arm, motion
        self._observe, self._base_to_world = observe, base_to_world
        self._lock = threading.RLock()
        self._action: PolicyAction | None = None
        self._cancel = threading.Event()
        self._worker: threading.Thread | None = None
        self._latched = False
        self._closed = False
        self._last_observation: dict[str, Any] | None = None

    def observe(self, camera: str = "left_wrist") -> dict[str, Any]:
        observation = camera_observation(self._observe(), camera)
        with self._lock:
            previous = self._last_observation
            if (
                previous
                and previous["id"] == observation["id"]
                and previous["fingerprint"] != observation["fingerprint"]
            ):
                raise ValueError("Capture ID reused for conflicting sensor data")
            if self._evidence_directory is not None:
                persist_observation(self._evidence_directory / "observations", observation)
            self._last_observation = copy.deepcopy(observation)
        return observation

    def ground(self, observation_id: str, u: int, v: int) -> dict[str, Any]:
        """Ground a pixel against the exact retained sensor frame, never task truth."""
        with self._lock:
            observation = self._last_observation
            if observation is None or observation["id"] != observation_id:
                raise ValueError("Observation replaced or unavailable")
            if not 0 <= time.time() - observation["rgb"].ts <= 10:
                raise ValueError("Reobserve before grounding an expired sensor frame")
            if observation["capture"]["capture_id"] is not None:
                current = self._observe()
                if current.get("episode") != observation["capture"]["episode"]:
                    raise ValueError("Episode changed since the retained sensor capture")
                if (
                    type(current.get("step")) is not int
                    or current["step"] < observation["capture"]["step"]
                ):
                    raise ValueError("Sensor capture clock moved backwards")
            result = ground_pixel(observation, u, v)
            if self._evidence_directory is not None:
                persist_grounding(self._evidence_directory, observation, result)
            return result

    def pose(self) -> PoseStamped:
        """Encoder/FK end-effector pose in base coordinates, not object truth."""
        base = self._base_to_world()
        pose = self._arm.pose()
        rotation = Rotation.from_quat(base.rotation.to_tuple())
        xyz = rotation.inv().apply(
            np.asarray(pose.position.to_tuple()) - np.asarray(base.translation.to_tuple())
        )
        q = rotation.inv() * Rotation.from_quat(pose.orientation.to_tuple())
        return PoseStamped(frame_id="base_link", position=xyz, orientation=q.as_quat())

    def state(self) -> dict[str, Any]:
        """Selected-arm proprioception; base-frame pose and no task-object state."""
        state = self._arm.state()
        if state.joints is None or not 0 <= time.time() - state.joints.ts <= 1:
            raise RuntimeError("Arm feedback unavailable or stale")
        return {
            "pose": self.pose(),
            "joints": copy.deepcopy(state.joints),
            "gripper_position": state.gripper_position,
            "provenance": "Encoder-derived robot state; simulator localization/calibration assumptions remain",
        }

    def move_pose(
        self, position: Sequence[float], orientation: Sequence[float], timeout: float = 20
    ) -> PolicyAction:
        xyz, q = vector(position, 3), vector(orientation, 4)
        if math.hypot(*q) < 1e-12:
            raise ValueError("Nonzero quaternion required")
        return self._start(xyz, q, timeout, False)

    def set_gripper_position(self, position: float) -> CommandResult:
        """SDK command acceptance only; read state for actual travel/blocked closure.

        Gripper commands cannot race an owned motion or bypass an uncertain stop.
        This does not verify a grasp, release assistance, or declare task success.
        """
        if not math.isfinite(position) or not 0 <= position <= 1:
            raise ValueError("Use finite normalized gripper travel between zero and one")
        with self._lock:
            if self._closed or self._latched or (self._worker and self._worker.is_alive()):
                raise RuntimeError("Supervisor closed, busy, or stop unconfirmed")
            try:
                return self._arm.set_gripper_position(position)
            except Exception:
                # A lost command reply does not establish that actuation stopped.
                self._latched = True
                raise RuntimeError("gripper_command_unconfirmed") from None

    def press(self, delta: Sequence[float], timeout: float = 10) -> PolicyAction:
        """Bounded straight EE displacement; no symbolic toggle or success setter."""
        d = vector(delta, 3)
        if not 0 < math.hypot(*d) <= 0.02:
            raise ValueError("Press displacement must be within 20 mm")
        current = self.pose()
        target = tuple(a + b for a, b in zip(current.position.to_tuple(), d, strict=True))
        return self._start(target, current.orientation.to_tuple(), timeout, True)

    def move_checkpoint_pose(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
        phase: str,
        timeout: float = 20,
        contact_position: Sequence[float] | None = None,
        contact_normal: Sequence[float] | None = None,
    ) -> PolicyAction:
        """Caller-chosen base-frame intent; no task-derived target correction.

        Contact point/normal are declared sensor results, not attested perception.
        """
        if not isinstance(self._motion, RadioCheckpointPolicyMotion):
            raise RuntimeError("Checkpoint owner is not initialized")
        has_contact = contact_position is not None and contact_normal is not None
        if (
            phase not in SINGLE_HAND_PHASES
            or (phase == "table_press") != has_contact
            or ((contact_position is None) != (contact_normal is None))
        ):
            raise ValueError("Declare contact point/normal only for table_press")
        xyz, q = vector(position, 3), vector(orientation, 4)
        if math.hypot(*q) < 1e-12:
            raise ValueError("Nonzero quaternion required")
        intent: dict[str, Any] = {"phase": phase}
        if has_contact:
            assert contact_position is not None and contact_normal is not None
            intent["surface_base"] = vector(contact_position, 3)
            normal = vector(contact_normal, 3)
            if not math.isclose(math.hypot(*normal), 1, abs_tol=1e-6):
                raise ValueError("Contact normal must be a unit vector")
            intent["normal_base"] = normal
        return self._start(xyz, q, timeout, phase == "table_press", intent)

    def _start(
        self,
        xyz: Sequence[float],
        q: Sequence[float],
        timeout: float,
        contact: bool,
        intent: Mapping[str, Any] | None = None,
    ) -> PolicyAction:
        if not math.isfinite(timeout) or not 0 < timeout <= 30:
            raise ValueError("Use a timeout between zero and 30 seconds")
        if isinstance(self._motion, RadioCheckpointPolicyMotion) and intent is None:
            if contact:
                raise ValueError("Declare a sensor contact intent for checkpoint press")
            intent = {"phase": "table_approach"}
        with self._lock:
            if self._closed or self._latched or (self._worker and self._worker.is_alive()):
                raise RuntimeError("Supervisor closed, busy, or stop unconfirmed")
            base = self._base_to_world()
            rotation = Rotation.from_quat(base.rotation.to_tuple())
            world = rotation.apply(xyz) + np.asarray(base.translation.to_tuple())
            orientation = (rotation * Rotation.from_quat(q)).as_quat()
            if intent is not None:
                intent = dict(intent)
                if "surface_base" in intent:
                    intent["surface_world"] = (
                        rotation.apply(intent.pop("surface_base"))
                        + np.asarray(base.translation.to_tuple())
                    ).tolist()
                    intent["normal_world"] = rotation.apply(intent.pop("normal_base")).tolist()
            self._cancel.clear()
            self._action = PolicyAction(uuid4().hex)
            action = self._action
            self._worker = threading.Thread(
                target=self._run,
                args=(action.id, world.tolist(), orientation.tolist(), timeout, contact, intent),
                daemon=True,
                name="radio-policy-action",
            )
            self._worker.start()
            return action

    def _run(
        self,
        action_id: str,
        xyz: Sequence[float],
        q: Sequence[float],
        timeout: float,
        contact: bool,
        intent: Mapping[str, Any] | None = None,
    ) -> None:
        timer = threading.Timer(timeout, self.cancel, args=(action_id,))
        timer.daemon = True
        timer.start()
        try:
            if intent is None:
                self._motion.move(xyz, q, timeout, contact, executor=self._execute)
            else:
                self._motion.move_intent(
                    xyz, q, timeout, intent, dispatch=self._dispatch, cancelled=self._cancel.is_set
                )
            with self._lock:
                if self._action and not self._cancel.is_set():
                    self._action = replace(self._action, state="completed", stop_confirmed=True)
        except RadioEpisodeTerminalError as error:
            # Evaluator truth stays private; policy gets only confirmed stop feedback.
            with self._lock:
                confirmed = error.outcome.get("stop_confirmed") is True
                self._latched = not confirmed
                if self._action and not self._cancel.is_set():
                    self._action = replace(
                        self._action,
                        state="cancelled" if confirmed else "uncertain",
                        error="episode_ended" if confirmed else "stop_unconfirmed",
                        stop_confirmed=confirmed,
                    )
        except Exception:
            # Do not forward collision geometry/plan/debug exception text to policy.
            with self._lock:
                if self._action and not self._cancel.is_set():
                    self._action = replace(self._action, state="failed", error="motion_failed")
            self.cancel(action_id)
        finally:
            timer.cancel()

    def _execute(self, plan_id: str, timeout: float) -> ExecutionResult:
        result = self._dispatch(plan_id, timeout)
        if result.status is not ExecutionStatus.ACCEPTED:
            return result
        return self._arm.rpc.wait_for_execution(timeout=timeout)

    def _dispatch(self, plan_id: str, timeout: float) -> ExecutionResult:
        with self._lock:
            if self._cancel.is_set() or self._closed:
                raise RuntimeError("Cancelled before dispatch")
            return self._arm.rpc.execute(blocking=False, timeout=timeout, plan_id=plan_id)

    def status(self, action_id: str) -> PolicyAction:
        with self._lock:
            if self._action is None or self._action.id != action_id:
                raise ValueError("Unknown action ID")
            return self._action

    def cancel(self, action_id: str) -> PolicyAction:
        with self._lock:
            action = self.status(action_id)
            if action.state == "completed" or action.cancel_requested:
                return action
            self._cancel.set()
            self._action = replace(action, cancel_requested=True)
            try:
                result = self._arm.rpc.cancel()
                confirmed = result.status in {
                    ExecutionStatus.ABORTED,
                    ExecutionStatus.COMPLETED,
                    ExecutionStatus.NO_EXECUTION,
                }
            except Exception:
                confirmed = False
            self._latched = not confirmed
            self._action = replace(
                self._action,
                state=("failed" if action.state == "failed" else "cancelled")
                if confirmed
                else "uncertain",
                error=action.error if confirmed else "stop_unconfirmed",
                stop_confirmed=confirmed,
            )
            return self._action

    def close(self) -> PolicyAction | None:
        with self._lock:
            self._closed = True
            return self.cancel(self._action.id) if self._action else None


# The owner initializes this task-local module after the existing radio blueprint
# is running. Its Python facade below never exposes initialization or truth RPCs.
class RadioPolicyConfig(ModuleConfig):
    arm: Literal["left_arm", "right_arm"] = "left_arm"
    auxiliary_groups: tuple[Literal["torso"], ...] = ("torso",)
    motion_contract: Literal["legacy", "checkpoint"] = "legacy"


class RadioPolicyModule(Module):
    default_config = RadioPolicyConfig
    config: RadioPolicyConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._supervisor: RadioPolicySupervisor | None = None
        self._app: Any = None
        self._development_evidence: dict[str, Any] = {}

    @rpc
    def initialize_development_scene(
        self, collision_scene: dict[str, Any], evidence_directory: str | None = None
    ) -> None:
        """Supervisor-owner setup only; never a policy tool or model prompt input."""
        from dimos.porcelain.dimos import Dimos
        from dimos.simulation.behavior.radio_motion import make_development_motion

        if self._supervisor is not None:
            raise RuntimeError("Supervisor already initialized")
        if evidence_directory is None or not Path(evidence_directory).is_absolute():
            raise ValueError("Development policy owner must choose an absolute evidence directory")
        if len(set(self.config.auxiliary_groups)) != len(self.config.auxiliary_groups):
            raise ValueError("Auxiliary groups must be unique")
        if self.config.motion_contract == "checkpoint" and self.config.arm != "right_arm":
            raise ValueError("The verified checkpoint contract selects right_arm")
        app = Dimos.connect(timeout=5)
        try:
            sim: Any = app.get_module("BehaviorConnection")
            visuals = sim.describe().get("policy_visuals", {})
            if not visuals.get("toggle_markers_hidden") or visuals.get("hidden_count", 0) < 1:
                raise RuntimeError("Policy RGB requires hidden diagnostic toggle markers")
            arm = Arm.from_app(app, group=self.config.arm, instance_name="ManipulationModule")
            probe: Any = app.get_module("BehaviorProbe")
            motion: Any
            if self.config.motion_contract == "checkpoint":
                motion = RadioCheckpointPolicyMotion(
                    make_radio_grasp_checkpoint(
                        app, sim, collision_scene, self._development_evidence
                    ),
                    self.config.auxiliary_groups,
                )
            else:
                motion = make_development_motion(
                    app,
                    sim,
                    arm,
                    {"collision_scene": collision_scene},
                    self._development_evidence,
                    self.config.auxiliary_groups,
                )
            calibration = np.asarray(vector(collision_scene["physical_to_sdk_fk_translation"], 3))

            def base_to_world() -> Transform:
                transforms = [
                    t
                    for t in probe.observation()["tf"].transforms
                    if t.frame_id == "world" and t.child_frame_id == "base_link"
                ]
                if len(transforms) != 1 or not 0 <= time.time() - transforms[0].ts <= 1:
                    raise RuntimeError("Robot base feedback unavailable or stale")
                t = transforms[0]
                return Transform(
                    translation=Vector3(*(np.asarray(t.translation.to_tuple()) + calibration)),
                    rotation=t.rotation,
                    frame_id="world",
                    child_frame_id="base_link",
                    ts=t.ts,
                )

            self._supervisor = RadioPolicySupervisor(
                arm, motion, sim.get_sensor_snapshot, base_to_world, Path(evidence_directory)
            )
            self._app = app
        except Exception:
            app.stop()
            raise

    def _ready(self) -> RadioPolicySupervisor:
        if self._supervisor is None:
            raise RuntimeError("Supervisor owner has not initialized the development scene")
        return self._supervisor

    @rpc
    def observe(self, camera: str = "left_wrist") -> dict[str, Any]:
        return self._ready().observe(camera)

    @rpc
    def ground(self, observation_id: str, u: int, v: int) -> dict[str, Any]:
        return self._ready().ground(observation_id, u, v)

    @rpc
    def pose(self) -> PoseStamped:
        return self._ready().pose()

    @rpc
    def state(self) -> dict[str, Any]:
        return self._ready().state()

    @rpc
    def set_gripper_position(self, position: float) -> CommandResult:
        return self._ready().set_gripper_position(position)

    @rpc
    def move_checkpoint_pose(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
        phase: str,
        timeout: float = 20,
        contact_position: Sequence[float] | None = None,
        contact_normal: Sequence[float] | None = None,
    ) -> PolicyAction:
        return self._ready().move_checkpoint_pose(
            position, orientation, phase, timeout, contact_position, contact_normal
        )

    @rpc
    def move_pose(
        self, position: Sequence[float], orientation: Sequence[float], timeout: float = 20
    ) -> PolicyAction:
        return self._ready().move_pose(position, orientation, timeout)

    @rpc
    def press(self, delta: Sequence[float], timeout: float = 10) -> PolicyAction:
        return self._ready().press(delta, timeout)

    @rpc
    def status(self, action_id: str) -> PolicyAction:
        return self._ready().status(action_id)

    @rpc
    def cancel(self, action_id: str) -> PolicyAction:
        return self._ready().cancel(action_id)

    @rpc
    def stop(self) -> None:
        if self._supervisor is not None:
            self._supervisor.close()
        if self._app is not None:
            self._app.stop()  # Borrowed-client disconnect, not the simulator owner's stop.
        super().stop()


class RadioPolicy:
    """Small Python client facade using existing DimOS RPC, without MCP."""

    def __init__(self, rpc_proxy: Any) -> None:
        self._rpc = rpc_proxy

    @classmethod
    def from_app(cls, app: Any) -> "RadioPolicy":
        return cls(app.get_module("RadioPolicyModule"))

    def observe(self, camera: str = "left_wrist") -> dict[str, Any]:
        return cast("dict[str, Any]", self._rpc.observe(camera))

    def ground(self, observation_id: str, u: int, v: int) -> dict[str, Any]:
        return cast("dict[str, Any]", self._rpc.ground(observation_id, u, v))

    def pose(self) -> PoseStamped:
        return cast("PoseStamped", self._rpc.pose())

    def state(self) -> dict[str, Any]:
        return cast("dict[str, Any]", self._rpc.state())

    def set_gripper_position(self, position: float) -> CommandResult:
        return cast("CommandResult", self._rpc.set_gripper_position(position))

    def move_checkpoint_pose(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
        phase: str,
        timeout: float = 20,
        contact_position: Sequence[float] | None = None,
        contact_normal: Sequence[float] | None = None,
    ) -> PolicyAction:
        return cast(
            "PolicyAction",
            self._rpc.move_checkpoint_pose(
                position, orientation, phase, timeout, contact_position, contact_normal
            ),
        )

    def move_pose(
        self, position: Sequence[float], orientation: Sequence[float], timeout: float = 20
    ) -> PolicyAction:
        return cast("PolicyAction", self._rpc.move_pose(position, orientation, timeout))

    def press(self, delta: Sequence[float], timeout: float = 10) -> PolicyAction:
        return cast("PolicyAction", self._rpc.press(delta, timeout))

    def status(self, action_id: str) -> PolicyAction:
        return cast("PolicyAction", self._rpc.status(action_id))

    def cancel(self, action_id: str) -> PolicyAction:
        return cast("PolicyAction", self._rpc.cancel(action_id))
