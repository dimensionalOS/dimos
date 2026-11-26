# Copyright 2025-2026 Dimensional Inc.
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

"""CPU R1 Pro dual-wrist actor, called at 50 Hz by ControlCoordinator."""

from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.control.task import BaseControlTask, ControlMode, JointCommandOutput, ResourceClaim
from dimos.robot.galaxea.r1pro.connection import R1PRO_BASE_JOINTS, R1PRO_UPPER_BODY_JOINTS
from dimos.robot.galaxea.r1pro.mujoco_plant import DT
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
TASK_TYPE = "r1pro_policy"


def unpack_pose(pose):
    p = np.asarray([pose.position.x, pose.position.y, pose.position.z])
    q = np.asarray([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    if (
        not np.isfinite(p).all()
        or not np.isfinite(q).all()
        or not np.isclose(np.linalg.norm(q), 1, atol=1e-3)
    ):
        raise ValueError("Invalid wrist/root SE(3) pose")
    return p, Rotation.from_quat(q).as_matrix()


def pack_goal(wrists, root):
    goal = np.zeros((1, 60), dtype=np.float32)
    p, r = root
    for i, (position, rotation) in enumerate(wrists, 1):
        goal[0, 10 * i : 10 * i + 3] = r.T @ (position - p)
        goal[0, 10 * i + 3 : 10 * i + 9] = (r.T @ rotation)[:, :2].T.ravel()
        goal[0, 10 * i + 9] = 1.0
    return goal


class R1ProActor:
    def __init__(self, model_path: str) -> None:
        import onnxruntime as ort

        path = Path(model_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(
                f"Set R1PRO_POLICY_ONNX to a compatible R1 Pro ONNX checkpoint; missing {path}"
            )
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = opts.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            str(path), sess_options=opts, providers=["CPUExecutionProvider"]
        )
        if [(i.name, i.type, i.shape) for i in self.session.get_inputs()] != [
            ("state_history", "tensor(float)", ["N", 10, 60]),
            ("urr_goal", "tensor(float)", ["N", 60]),
        ] or [(o.name, o.type, o.shape) for o in self.session.get_outputs()] != [
            (name, "tensor(float)", ["N", 21]) for name in ("raw_action", "command")
        ]:
            raise ValueError("Expected r1pro-teleop-actor-input-v0.8 ONNX ABI")
        self.history = np.zeros((1, 10, 60), dtype=np.float32)
        self.raw = np.zeros(21, dtype=np.float32)
        self.initialized = False
        self.infer(np.zeros((1, 60), dtype=np.float32))  # warm up before accepting input

    def reset(self):
        self.initialized = False
        self.raw.fill(0)

    def observe(self, q, dq, omega):
        frame = np.concatenate((q, dq, omega, self.raw), dtype=np.float32)
        if frame.shape != (60,) or not np.isfinite(frame).all():
            raise ValueError("Invalid r1pro state_history frame")
        if self.initialized:
            self.history[0, :-1] = self.history[0, 1:]
            self.history[0, -1] = frame
        else:
            self.history[:] = frame
            self.initialized = True

    def infer(self, goal):
        if not np.isfinite(goal).all():
            raise ValueError("Invalid r1pro urr_goal")
        raw, command = self.session.run(
            ["raw_action", "command"],
            {
                "state_history": self.history,
                "urr_goal": goal,
            },
        )
        if not all(
            x.dtype == np.float32 and x.shape == (1, 21) and np.isfinite(x).all()
            for x in (raw, command)
        ):
            raise ValueError("Unexpected or non-finite r1pro actor output")
        return raw[0], command[0]


class R1ProPolicyTask(BaseControlTask):
    """Same-stamp physics feedback; world targets or grip-relative WebXR input.

    The coordinator serializes callbacks with compute. Missing or stale data
    withdraws both claims; the normal source holds positions and zeros base speed.
    """

    def __init__(self, name, model_path, priority=40, timeout_s=0.15):
        self._name = name
        self.priority = priority
        self.timeout_s = timeout_s
        self.actor = R1ProActor(model_path)
        self.estopped = False
        self.target = None
        self.anchor = None
        self.samples = {}
        self.last_sample = 0.0
        self.last_step = None
        self.xr_rearm = True

    def claim(self):
        return ResourceClaim(
            frozenset((*R1PRO_UPPER_BODY_JOINTS, *R1PRO_BASE_JOINTS)), self.priority
        )

    def is_active(self):
        return not self.estopped and self.target is not None

    def set_estop(self, estopped):
        self.estopped = estopped
        self._withdraw()

    def _withdraw(self):
        self.target = self.anchor = None
        self.xr_rearm = True
        self.last_step = None
        self.actor.reset()

    def on_preempted(self, by_task, joints):
        self._withdraw()

    def _sample(self, msg, now, key):
        # All three messages were acquired at the same physics step. Never mix
        # a newly received root with old q/dq or measured wrist anchors.
        if msg.ts <= self.last_sample or not 0 <= time.time() - msg.ts <= self.timeout_s:
            return
        self.samples.setdefault(msg.ts, {})[key] = msg
        for stamp in sorted(self.samples)[:-4]:
            del self.samples[stamp]

    def on_joints(self, msg, now):
        if list(msg.name) == R1PRO_UPPER_BODY_JOINTS:
            self._sample(msg, now, "joints")

    def on_odometry(self, msg, now):
        self._sample(msg, now, "odometry")

    def on_wrists(self, msg, now):
        self._sample(msg, now, "wrists")

    def on_world_target(self, msg, now):
        self._target(msg, now, relative=False)

    def on_controllers(self, msg, now):
        if self.estopped or not 0 <= time.time() - msg.ts <= self.timeout_s:
            return
        if msg.frame_id == "webxr_released" and not msg.poses:
            if self.target is None or self.target[0]:
                self._withdraw()
                self.xr_rearm = False
        elif msg.frame_id == "webxr_idle" and not msg.poses:
            if self.target is None or self.target[0]:
                self._withdraw()
        elif msg.frame_id == "webxr" and not self.xr_rearm:
            self._target(msg, now, relative=True)

    def _target(self, msg, now, relative):
        if self.estopped or not 0 <= time.time() - msg.ts <= self.timeout_s:
            return
        if relative and self.target is not None and not self.target[0]:
            return  # programmatic input owns the current session
        if self.target is not None and msg.ts <= self.target[3]:
            return
        if self.target is not None and (
            now - self.target[2] > self.timeout_s or relative != self.target[0]
        ):
            self._withdraw()
            if relative:
                return
        try:
            if len(msg.poses) != 2:
                raise ValueError("Expected [left wrist, right wrist]")
            transforms = tuple(map(unpack_pose, msg.poses))
        except ValueError:
            self._withdraw()
            return
        self.target = relative, transforms, now, msg.ts, msg.frame_id

    def compute(self, state):
        if self.target is None:
            return None
        relative, wrists, received, target_stamp, frame = self.target
        if state.t_now - received > self.timeout_s or time.time() - target_stamp > self.timeout_s:
            self._withdraw()
            return None
        available = [t for t, parts in self.samples.items() if len(parts) == 3]
        if not available or time.time() - max(available) > self.timeout_s:
            # At initial engagement let the first coherent feedback arrive.
            if self.last_step is not None:
                self._withdraw()
            return None
        stamp = max(available)
        joints, odom, measured = (
            self.samples[stamp][key] for key in ("joints", "odometry", "wrists")
        )
        if stamp <= self.last_sample:
            # No new physics; don't duplicate a history row or resend an action.
            return None
        try:
            if len(joints.position) != 18 or len(joints.velocity) != 18:
                raise ValueError("Expected 18 measured upper positions and velocities")
            if measured.frame_id != odom.frame_id or len(measured.poses) != 2:
                raise ValueError("Expected two measured wrists in the odometry frame")
            if not relative and frame != odom.frame_id:
                raise ValueError(f"World targets must be expressed in {odom.frame_id!r}")
            if relative:
                if self.anchor is None:
                    self.anchor = wrists, tuple(map(unpack_pose, measured.poses))
                inputs, anchors = self.anchor
                wrists = tuple(
                    (start_p + p - input_p, r @ input_r.T @ start_r)
                    for (p, r), (input_p, input_r), (start_p, start_r) in zip(
                        wrists, inputs, anchors, strict=True
                    )
                )
            w = odom.twist.angular
            self.actor.observe(joints.position, joints.velocity, [w.x, w.y, w.z])
            raw, command = self.actor.infer(pack_goal(wrists, unpack_pose(odom.pose)))
            if (
                time.perf_counter() - state.t_now > DT
                or time.time() - target_stamp > self.timeout_s
            ):
                self._withdraw()
                return None
        except Exception:
            logger.exception("R1 Pro policy withdrew after invalid feedback or inference")
            self._withdraw()
            return None
        self.actor.raw[:] = raw
        self.last_sample = stamp
        self.last_step = state.t_now
        return [
            JointCommandOutput(R1PRO_UPPER_BODY_JOINTS, positions=command[3:].tolist()),
            JointCommandOutput(
                R1PRO_BASE_JOINTS, velocities=command[:3].tolist(), mode=ControlMode.VELOCITY
            ),
        ]
