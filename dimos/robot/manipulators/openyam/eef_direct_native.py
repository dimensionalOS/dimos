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

"""Release-local arm write adaptation; stock OpenYAM IO and lifecycle."""
from dataclasses import replace
import json
import threading
import time

import numpy as np

from dimos.control.contract.limits import movable_joints
from dimos.core.stream import Out
from dimos.msgs.std_msgs.String import String
from dimos.robot.manipulators.damiao.arm import DamiaoArm
from dimos.robot.manipulators.openyam.connection import OpenYamConnection
from dimos.robot.manipulators.openyam.eef_direct import JOINTS, KP, KD

ARM = tuple(f'arm/{name}' for name in JOINTS)
JAW = 'arm/joint7'
ACTION = 'eef-direct-11p5k:'
JAW_ONLY = 'eef-direct-11p5k-hold'
PUBLICATION_TIMEOUT = 0.030  # existing native Quest source-tick-to-commit bound


def arm_limits(model):
    joints = {j.name: j for j in movable_joints(model)}
    lower = np.array([joints[n].lower for n in JOINTS])
    upper = np.array([joints[n].upper for n in JOINTS])
    speed = np.minimum([joints[n].velocity for n in JOINTS], [10, 10, 10, 30, 30, 30])
    return np.maximum(lower, -12.5), np.minimum(upper, 12.5), speed


def check_arm(q, dq, bounds):
    q, dq = np.asarray(q), np.asarray(dq)
    lower, upper, speed = bounds
    if (q.shape != (6,) or dq.shape != (6,) or not np.isfinite([q, dq]).all()
            or np.any(q < lower) or np.any(q > upper) or np.any(np.abs(dq) > speed)):
        raise ValueError('OpenYAM six-axis q/dq outside canonical joint/codec bounds')


class EefDirectArm(DamiaoArm):
    def command_policy(self, jaw_target, q=None, dq=None, source_time=None):
        native = self._native
        if native is None or not self.active:
            raise RuntimeError('Damiao robot is inactive')
        jaw, = self.robot.grippers
        closed, opened = self._travel[jaw.joint]
        opening = (jaw_target - closed) / (opened - closed)
        if not 0 <= opening <= 1:
            raise ValueError(f'{jaw.joint} target {jaw_target} is outside its URDF range')
        self._targets[jaw.joint] = jaw_target
        # Same calibrated jaw call and resend on every stock command callback.
        self._grippers[jaw.name].set_opening(opening)
        if q is not None:
            if not 0 <= time.time() - source_time <= PUBLICATION_TIMEOUT:
                raise TimeoutError('native Quest actor command expired before MIT submission')
            rows = np.column_stack((KP, KD, q, dq, np.zeros(6)))
            self._arms['arm'].mit_control(rows)
        native.tick(1_000)


class EefDirectConnection(OpenYamConnection):
    """The stock reader, startup, watchdog, holds and ordinary position writes."""
    _base = OpenYamConnection.robot
    robot = replace(_base, gravity=False, buses=[replace(_base.buses[0], kp=list(KP), kd=list(KD))])
    actor_receipt: Out[String]

    def connect(self):
        # Same topology check and startup read as DamiaoConnection.connect.
        model = self.config.model
        if model is None or {j.name for j in movable_joints(model)} != set(self.joint_names):
            raise ValueError('connection model joints must match the vendor topology')
        if tuple(self.robot.arm_joint_names) != JOINTS:
            raise ValueError('native arm joint order differs from the EEF policy')
        self._state_frame = self.frame(model.load().root_link)
        self._arm_limits = arm_limits(model)
        self._policy_write = threading.local()
        arm = EefDirectArm(self.robot, self.can_ports(), teach=self.config.teach)
        self._arm = arm
        arm.connect()
        self._read_state()

    def _on_position(self, msg):
        token = msg.frame_id
        if token != JAW_ONLY and not token.startswith(ACTION):
            return super()._on_position(msg)
        # Carry the absolute q_des and fixed zero dq_des through the existing validation/deadman path.
        # The context is local to this callback, not shared with read/halt.
        ctx = [msg, 'command refused by stock connection']
        self._policy_write.context = ctx
        try:
            super()._on_position(msg)
        finally:
            del self._policy_write.context
        if token != JAW_ONLY:
            self.actor_receipt.publish(String(json.dumps([token, ctx[1]])))

    def write(self, values):
        ctx = getattr(self._policy_write, 'context', None)
        if ctx is None:
            return super().write(values)
        try:
            if self._fault is not None:
                raise RuntimeError(self._fault)
            msg = ctx[0]
            arm = self._arm
            assert isinstance(arm, EefDirectArm)
            jaw = values[f'{JAW}/position']
            if msg.frame_id == JAW_ONLY:
                arm.command_policy(jaw)
            else:
                index = {name: i for i, name in enumerate(msg.name)}
                q = [values[f'{name}/position'] for name in ARM]
                dq = [msg.velocity[index[name]] for name in ARM]
                check_arm(q, dq, self._arm_limits)
                arm.command_policy(jaw, q, dq, msg.ts)
            ctx[1] = None
        except Exception as exc:
            ctx[1] = f'{type(exc).__name__}: {exc}'
            raise  # stock error reporting and halt remain responsible for IO faults
