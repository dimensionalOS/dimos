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

"""Calibrated WebXR arm poses and an application-independent calibration RPC contract."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any, Literal, Protocol

import numpy as np
from numpy.typing import NDArray
from pydantic import Field
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation

from dimos.constants import STATE_DIR
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.std_msgs.String import String
from dimos.robot.assets.model import JointDescription, RobotModel
from dimos.spec.utils import Spec
from dimos.teleop.webxr.arm_calibration import (
    INPUT_MAX_AGE_S,
    ArmCalibration,
    CalibrationPose,
    InputKind,
    RobotArmGeometry,
    Side,
    head_frame,
)
from dimos.teleop.webxr.controller_types import Hand, WebXRControllerState
from dimos.teleop.webxr.extensions import ArmTeleopModule
from dimos.teleop.webxr.module import WebXRTeleopConfig


class CalibrationHoldSpec(Spec, Protocol):
    def hold_for_arm_calibration(self, held: bool) -> None: ...


class CalibratedArmConfig(WebXRTeleopConfig):
    model: RobotModel
    body_frame: str
    # Published TF name when the deployment namespaces the model's links.
    body_tf_frame: str | None = None
    base_frame: str = "base_link"
    target_frames: dict[Side, str]
    shoulder_frames: dict[Side, str]
    wrist_frames: dict[Side, str]
    shoulder_offsets: dict[Side, tuple[float, float, float]] = Field(default_factory=dict)
    neutral_joints: dict[str, float]
    reach_joints: dict[str, float]
    calibration_directory: Path = STATE_DIR / "teleop" / "arm-calibration"


class CalibratedArmTeleopModule(ArmTeleopModule):
    """The same mapping for tracked wrists and controller grips, before arm IK.

    Profiles are explicit selections. Capturing, saving, loading and deleting
    never engages an arm. A coordinator holds motion during capture/review;
    activation clears that hold and requires the usual fresh device engagement.
    """

    config: CalibratedArmConfig
    _calibration_control: CalibrationHoldSpec
    webxr_head: In[PoseStamped]
    robot_feedback: In[JointState]
    arm_calibration_state: Out[String]
    arm_retarget_state: Out[String]
    webxr_pose_observed: Out[PoseStamped]
    webxr_head_observed: Out[PoseStamped]
    webxr_joy_observed: Out[Joy]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._calibration: ArmCalibration | None = None
        self._target_chains: dict[Side, tuple[JointDescription, ...]] = {}
        self._last_retarget_publish = -math.inf
        self._body_chain: tuple[JointDescription, ...] = ()
        self._body_positions: dict[str, tuple[float, float]] = {}
        self._head: NDArray[np.float64] | None = None
        self._head_receipt = -math.inf
        self._head_stamp = -math.inf
        self._head_frames: OrderedDict[float, NDArray[np.float64]] = OrderedDict()
        self._pending_hands: dict[Side, tuple[float, CalibrationPose, float]] = {}
        self._buttons_released = False
        self._last_calibration_publish = -math.inf

    @rpc
    def start(self) -> None:
        model = self.config.model.load()
        parents = {j.child_link: j for j in model.joints}

        def chain(link: str) -> tuple[JointDescription, ...]:
            joints = []
            while link != self.config.base_frame:
                joint = parents[link]
                joints.append(joint)
                link = joint.parent_link
            return tuple(reversed(joints))

        self._body_chain = chain(self.config.body_frame)
        neutral_body = self._fk(self._body_chain, self.config.neutral_joints)
        reach_body = self._fk(self._body_chain, self.config.reach_joints)
        if not np.allclose(neutral_body, reach_body):
            raise ValueError("Calibration reach must not move the robot body")
        self._target_chains = {s: chain(f) for s, f in self.config.target_frames.items()}
        geometry: dict[Side, RobotArmGeometry] = {}
        for side, target_chain in self._target_chains.items():
            shoulder_chain = chain(self.config.shoulder_frames[side])
            wrist_chain = chain(self.config.wrist_frames[side])
            shoulder = np.linalg.inv(neutral_body) @ self._fk(
                shoulder_chain, self.config.neutral_joints
            )
            extended_shoulder = np.linalg.inv(reach_body) @ self._fk(
                shoulder_chain, self.config.reach_joints
            )
            offset = np.asarray(self.config.shoulder_offsets.get(side, (0.0, 0.0, 0.0)))
            shoulder[:3, 3] += shoulder[:3, :3] @ offset
            extended_shoulder[:3, 3] += extended_shoulder[:3, :3] @ offset
            if not np.allclose(shoulder[:3, 3], extended_shoulder[:3, 3]):
                raise ValueError("The shoulder reference must stay fixed in the body")
            tool = np.linalg.inv(neutral_body) @ self._fk(target_chain, self.config.neutral_joints)
            wrist = np.linalg.inv(neutral_body) @ self._fk(wrist_chain, self.config.neutral_joints)
            extended_tool = np.linalg.inv(reach_body) @ self._fk(
                target_chain, self.config.reach_joints
            )
            extended_wrist = np.linalg.inv(reach_body) @ self._fk(
                wrist_chain, self.config.reach_joints
            )
            tool_to_wrist = np.linalg.inv(tool) @ wrist
            if not np.allclose(tool_to_wrist, np.linalg.inv(extended_tool) @ extended_wrist):
                raise ValueError("The wrist-to-tool transform must be rigid")
            geometry[side] = RobotArmGeometry(
                shoulder=tuple(shoulder[:3, 3]),
                reach_m=float(np.linalg.norm(extended_wrist[:3, 3] - shoulder[:3, 3])),
                neutral_orientation=tuple(Rotation.from_matrix(tool[:3, :3]).as_quat()),
                tool_to_wrist=tuple(tool_to_wrist[:3, 3]),
            )
        identity = json.dumps(
            {
                "joints": [asdict(j) for j in model.joints],
                "body": self.config.body_frame,
                "version": 2,
                "geometry": {s: g.model_dump() for s, g in geometry.items()},
            },
            sort_keys=True,
        )
        self._calibration = ArmCalibration(
            geometry,
            hashlib.sha256(identity.encode()).hexdigest()[:24],
            self.config.calibration_directory,
        )
        self.register_disposable(
            Disposable(self.robot_feedback.subscribe(self._on_calibration_feedback))
        )
        super().start()

    @staticmethod
    def _fk(
        chain: tuple[JointDescription, ...], positions: dict[str, float]
    ) -> NDArray[np.float64]:
        matrix = np.eye(4)
        for joint in chain:
            matrix = matrix @ joint.parent_from_child(positions.get(joint.name, 0.0))
        return matrix

    def _on_calibration_feedback(self, sample: JointState) -> None:
        if (
            len(sample.name) != len(sample.position)
            or not math.isfinite(sample.ts)
            or abs(time.time() - sample.ts) > INPUT_MAX_AGE_S
        ):
            return
        required = {
            j.name
            for chain in (self._body_chain, *self._target_chains.values())
            for j in chain
            if j.type != "fixed"
        }
        with self._lock:
            for name, value in zip(sample.name, sample.position, strict=True):
                if (
                    name in required
                    and math.isfinite(value)
                    and sample.ts > self._body_positions.get(name, (0, -math.inf))[1]
                ):
                    self._body_positions[name] = (value, sample.ts)

    async def handle_webxr_head(self, sample: PoseStamped) -> None:
        with self._lock:
            if sample.ts <= self._head_stamp or not math.isfinite(sample.ts):
                return
            p, q = sample.position, sample.orientation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if (
                not all(math.isfinite(x) for x in values)
                or abs(np.linalg.norm(values[3:]) - 1) > 0.01
            ):
                return
            turn = Rotation.from_quat(values[3:]).as_matrix()
            conversion = np.array(((0, 0, -1), (-1, 0, 0), (0, 1, 0)))
            forward = conversion @ turn @ np.array((0, 0, -1))
            if np.linalg.norm(forward[:2]) < 0.2:
                # Looking straight down must not expire otherwise valid tracking.
                # Heading is undefined there; retain the last observable yaw.
                if self._head is None:
                    return
                yaw = math.atan2(self._head[1, 0], self._head[0, 0])
            else:
                yaw = math.atan2(forward[1], forward[0])
            self.webxr_head_observed.publish(sample)
            self._head = head_frame((-p.z, -p.x, p.y), yaw)
            self._head_receipt, self._head_stamp = time.monotonic(), sample.ts
            self._head_frames[sample.ts] = self._head
            if len(self._head_frames) > 60:
                self._head_frames.popitem(last=False)
            for side in tuple(self._pending_hands):
                self._observe_paired_hand(side)

    def _on_pose(self, msg: PoseStamped) -> None:
        p, q = msg.position, msg.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if (
            not math.isfinite(msg.ts)
            or not all(math.isfinite(v) for v in values)
            or abs(np.linalg.norm(values[3:]) - 1) > 0.01
        ):
            return
        self.webxr_pose_observed.publish(msg)
        super()._on_pose(msg)
        with self._lock:
            c, now = self._calibration, time.monotonic()
            hand = self._resolve_hand(msg.frame_id)
            pose = self._current_poses[hand]
            if c is None or pose is None:
                return
            side: Side = "left" if hand == Hand.LEFT else "right"
            pending = self._pending_hands.get(side)
            if msg.ts <= c.last_stamps[side] or (pending is not None and msg.ts <= pending[0]):
                return
            self._pending_hands[side] = (msg.ts, self._pose(pose), now)
            self._observe_paired_hand(side)

    def _observe_paired_hand(self, side: Side) -> None:
        """Only combine poses captured in the same XR frame, regardless of delivery order.

        Separate latest-value channels can arrive out of order or drop frames.
        Pairing avoids a transient arm swing during a body turn; pending state
        holds only the newest hand pose and never extends input freshness.
        """
        stamp, pose, receipt = self._pending_hands[side]
        if time.monotonic() - receipt > INPUT_MAX_AGE_S:
            self._pending_hands.pop(side)
            return
        head = self._head_frames.get(stamp)
        if head is not None and self._calibration is not None:
            self._calibration.observe(side, pose, head, receipt, stamp)
            self._pending_hands.pop(side)

    def _on_joy(self, msg: Joy) -> bool:
        valid = super()._on_joy(msg)
        if valid:
            self.webxr_joy_observed.publish(msg)
        return valid

    def _actual(self) -> dict[str, Any]:
        positions = {name: value for name, (value, _) in self._body_positions.items()}
        body = self._fk(self._body_chain, positions)
        actual: dict[str, Any] = {}
        for side, chain in self._target_chains.items():
            if any(
                j.type != "fixed"
                and (
                    j.name not in self._body_positions
                    or time.time() - self._body_positions[j.name][1] > INPUT_MAX_AGE_S
                )
                for j in chain
            ):
                continue
            actual[side] = CalibrationPose.from_matrix(
                np.linalg.inv(body) @ self._fk(chain, positions)
            ).model_dump()
        return actual

    @staticmethod
    def _pose(pose: PoseStamped) -> CalibrationPose:
        p, q = pose.position, pose.orientation
        return CalibrationPose(position=(p.x, p.y, p.z), orientation=(q.x, q.y, q.z, q.w))

    def _get_output_pose(self, hand: Hand) -> PoseStamped | None:
        c, current = self._calibration, super()._get_output_pose(hand)
        if c is None or current is None or time.monotonic() - self._head_receipt > INPUT_MAX_AGE_S:
            return None
        side: Side = "left" if hand == Hand.LEFT else "right"
        if not c.history[side] or time.monotonic() - c.history[side][-1].time > INPUT_MAX_AGE_S:
            return None
        required = [j.name for j in self._body_chain if j.type != "fixed"]
        if any(
            name not in self._body_positions
            or time.time() - self._body_positions[name][1] > INPUT_MAX_AGE_S
            for name in required
        ):
            return None
        mapped = c.map(side)
        if mapped is None:
            return None
        body = self._fk(
            self._body_chain, {name: value for name, (value, _) in self._body_positions.items()}
        )
        position = body[:3, :3] @ np.asarray(mapped.position) + body[:3, 3]
        orientation = Rotation.from_matrix(body[:3, :3]) * Rotation.from_quat(mapped.orientation)
        return PoseStamped(
            ts=time.time(),
            frame_id=self.config.base_frame,
            position=position.tolist(),
            orientation=orientation.as_quat().tolist(),
        )

    def _publish_button_state(
        self, left: WebXRControllerState | None, right: WebXRControllerState | None
    ) -> None:
        c = self._calibration
        if c is None or not c.active or c.draft is not None:
            self._buttons_released = False
            super()._publish_button_state(None, None)
        else:
            if left is not None and right is not None and left.grip < 0.5 and right.grip < 0.5:
                self._buttons_released = True
            super()._publish_button_state(
                left if self._buttons_released else None, right if self._buttons_released else None
            )
        now = time.monotonic()
        if c is not None and now - self._last_calibration_publish > 1:
            self._last_calibration_publish = now
            self.arm_calibration_state.publish(
                String(
                    json.dumps(
                        {
                            "version": 2,
                            "operator_reference": "live_head_yaw",
                            "active": c.active,
                            "profile": c.profile.model_dump() if c.profile else None,
                            "robot": c.robot,
                            "geometry": {s: g.model_dump() for s, g in c.geometry.items()},
                        }
                    )
                )
            )

        if c is not None and now - self._last_retarget_publish >= 0.1:
            self._last_retarget_publish = now
            self.arm_retarget_state.publish(
                String(
                    json.dumps(
                        {
                            "version": 2,
                            "operator_reference": "live_head_yaw",
                            "ts": time.time(),
                            "active": c.active,
                            "body_frame": self.config.body_tf_frame or self.config.body_frame,
                            "actual": self._actual(),
                            **c.mapping_status(now),
                        }
                    )
                )
            )

    def _engine(self) -> ArmCalibration:
        if self._calibration is None:
            raise RuntimeError("Calibration module is not started")
        return self._calibration

    @rpc
    def get_arm_calibration(self, profile_id: str | None = None) -> dict[str, Any]:
        """Read profiles, capture quality, targets and review without changing control."""
        with self._lock:
            status = self._engine().status(time.monotonic(), profile_id)
            status["actual"] = self._actual()
            status["head_fresh"] = time.monotonic() - self._head_receipt <= INPUT_MAX_AGE_S
            status["body_frame"] = self.config.body_tf_frame or self.config.body_frame
            return status

    @rpc
    def begin_arm_calibration(
        self, name: str, input_kind: InputKind, device: str, session: str
    ) -> dict[str, Any]:
        """Hold the robot and start a draft for one operator and tracked input profile."""
        self._calibration_control.hold_for_arm_calibration(True)
        with self._lock:
            self._engine().begin(name, input_kind, device, session)
            self._head_stamp, self._head_receipt = -math.inf, -math.inf
            self._head_frames.clear()
            self._pending_hands.clear()
            self._buttons_released = False
            return self.get_arm_calibration()

    @rpc
    def capture_arm_calibration(
        self, stage: Literal["shoulders", "neutral", "left_reach", "right_reach"], session: str
    ) -> dict[str, Any]:
        """Capture the recent stable pose; stale or noisy input is rejected."""
        with self._lock:
            if time.monotonic() - self._head_receipt > INPUT_MAX_AGE_S:
                raise ValueError("Head tracking is stale")
            self._engine().capture(stage, time.monotonic(), session)
            return self.get_arm_calibration()

    @rpc
    def save_arm_calibration(self, session: str) -> dict[str, Any]:
        """Atomically save a complete draft; leave motion held until explicit activation."""
        with self._lock:
            self._engine().save(time.time(), session)
            return self.get_arm_calibration()

    @rpc
    def activate_arm_calibration(
        self, profile_id: str, input_kind: InputKind, device: str, session: str
    ) -> dict[str, Any]:
        """Use a compatible profile with fresh, live head-relative tracking.

        This does not engage arms; release and re-press the device's enable control.
        """
        with self._lock:
            self._engine().activate(profile_id, input_kind, device, session)
            self._head_stamp, self._head_receipt = -math.inf, -math.inf
            self._head_frames.clear()
            self._pending_hands.clear()
            self._buttons_released = False
        self._calibration_control.hold_for_arm_calibration(False)
        return self.get_arm_calibration()

    @rpc
    def cancel_arm_calibration(self, session: str) -> dict[str, Any]:
        """Discard the draft, keeping saved profiles; do not resume tracking."""
        with self._lock:
            self._engine().cancel(session)
        self._calibration_control.hold_for_arm_calibration(False)
        return self.get_arm_calibration()

    @rpc
    def delete_arm_calibration(self, profile_id: str) -> dict[str, Any]:
        """Remove a saved profile and stop calibrated input."""
        self._calibration_control.hold_for_arm_calibration(True)
        with self._lock:
            self._engine().delete(profile_id)
        self._calibration_control.hold_for_arm_calibration(False)
        return self.get_arm_calibration()
