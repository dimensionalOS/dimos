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

"""Stock physical OpenYAM Quest assembly with the direct TCP 11.5k policy."""
import json
import time
from types import SimpleNamespace
from uuid import uuid4

import numpy as np

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.openyam_policy_task.openyam_policy_task import (
    OpenYamPolicyTask, OpenYamPolicyTaskConfig, PolicyTaskFault,
)
from dimos.control.tasks.registry import control_task_registry
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.std_msgs.String import String
from dimos.robot.manipulators.damiao.damiao_blueprint import DamiaoTeleopCoordinator
from dimos.robot.manipulators.openyam.blueprints.quest_input import QuestArmTeleopModule, RIGHT_GRIP_OBSERVED
from dimos.robot.manipulators.openyam.blueprints.teleop import (
    OPENYAM_QUEST_KINEMATICS, OPENYAM_QUEST_MODEL, openyam_quest_tasks,
)
from dimos.robot.manipulators.openyam.eef_direct import (
    DT, POLICY_HASH, DirectActorProcess, DirectHistory, DirectTcpReference,
)
from dimos.robot.manipulators.openyam.eef_direct_native import (
    ACTION, ARM, JAW_ONLY, PUBLICATION_TIMEOUT, EefDirectConnection, arm_limits, check_arm,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
TASK_TYPE = 'superreach_eef_direct_11p5k_real'
TICK_RATE = 100.0  # stock OpenYAM teleop, two ticks per actor update


class EefDirectArmTask(OpenYamPolicyTask):
    def __init__(self, cfg, source):
        reference = DirectTcpReference()
        params = OpenYamPolicyTaskConfig(
            joint_names=ARM, model_dir='', manifest_path='', every_n_ticks=2,
            priority=cfg.priority, max_episode_ticks=None,
        )
        super().__init__(cfg.name, params,
                         runtime=SimpleNamespace(manifest=SimpleNamespace(timing=SimpleNamespace(policy_hz=1 / DT))),
                         reference=reference,
                         feedback_fresh=source.ready_for_control)
        self.bounds = arm_limits(EefDirectConnection.robot.model)
        self.training = reference.bounds
        self.worker = DirectActorProcess()
        self._runtime = DirectHistory(infer=self.worker.run)
        self._rearm_required = True
        self.staged = self.pending = self.receipt = None
        self.hold = None

    def on_teleop_buttons(self, msg, now):
        if msg.data & RIGHT_GRIP_OBSERVED:
            return super().on_teleop_buttons(msg, now)
        with self._lock:
            if not self._estop:
                self._invalidate_locked(PolicyTaskFault.INTENT)
        return True

    def on_session_ended(self):
        super().on_session_ended()
        self.staged = self.pending = self.receipt = None
        # Nothing may commit after revocation. The tick owner resets the adapter
        # on the next engagement, not concurrently with a running inference.

    def _read_feedback(self, state):
        result = super()._read_feedback(state)
        if result is not None:
            q, dq = result
            check_arm(q, dq, self.bounds)
            if np.any(q < self.training[0]) or np.any(q > self.training[1]):
                raise ValueError('stock q outside the trained reference envelope')
        return result

    def prepare(self, state):
        # A completed host submission advances history only on a due actor tick.
        # It is paired with the stock q/dq view available then, not a new frame claim.
        if self.pending is not None and self.receipt is None:
            if self._clock() - self.pending[2] > PUBLICATION_TIMEOUT:
                raise TimeoutError('native MIT submission result missing by the publication deadline')
            if not self._input_fresh_locked(state.t_now, self._config.input_timeout_s):
                self._invalidate_locked(PolicyTaskFault.INTENT)
                return None
            if not self._feedback_fresh() or self._read_feedback(state) is None:
                self._invalidate_locked(PolicyTaskFault.FEEDBACK)
                return None
            # Leave the stock gripper cadence untouched; the next actor slot is
            # still determined by the existing two-tick cadence counter.
            self._cadence_due(state.t_now)
            return self._held_command()
        if self.pending is not None and self.receipt is not None:
            _, action, _ = self.pending
            feedback = self._read_feedback(state)
            if feedback is not None and self._feedback_fresh() and self._input_fresh_locked(
                state.t_now, self._config.input_timeout_s
            ):
                self.hold = action
                # Defer append to act() so only actor ticks consume a stock view.
            else:
                self._invalidate_locked(PolicyTaskFault.FEEDBACK)
                return None
        return super().prepare(state)

    def reset_policy(self, tick):
        q, dq, _ = tick.payload
        self.hold = q
        self._runtime.reset(q, dq)

    def act(self, tick):
        q, dq, row = tick.payload
        if self.pending is not None and self.receipt is not None:
            self._runtime.commit(self.pending[1], q, dq)
            self.pending = self.receipt = None
        action = self._runtime.command(row)
        check_arm(action, np.zeros(6), self.bounds)
        # The arbitration position is a safe stock hold, not an unacknowledged action.
        self.new_action = action
        return self.hold

    def commit_guard(self, tick, command):
        now = self._clock()
        if now - tick.t_now > PUBLICATION_TIMEOUT or not self._input_fresh_locked(
            now, self._config.input_timeout_s
        ):
            self._invalidate_locked(PolicyTaskFault.INFERENCE)
            return None
        accepted = super().commit_guard(tick, command)
        if accepted is not None and not self._dry_run:
            self.staged = (self.new_action, tick.t_now)
        elif accepted is not None:
            # No write in dry-run; never turn a dry-run result into history.
            self._invalidate_locked(PolicyTaskFault.RELEASED)
            return None
        return accepted

    def compute(self, state):
        try:
            return super().compute(state)
        except Exception:
            logger.exception('SuperReach arm tick refused')
            with self._lock:
                self._invalidate_locked(PolicyTaskFault.FEEDBACK)
            return None

    def on_actor_receipt(self, msg, now):
        token, error = json.loads(msg.data)
        with self._lock:
            if self.pending is None or self.pending[0] != token:
                return
            if error is not None or now - self.pending[2] > PUBLICATION_TIMEOUT:
                logger.error('SuperReach MIT write failed', error=error or 'late host submission result')
                self._invalidate_locked(PolicyTaskFault.INFERENCE)
            else:
                self.receipt = token

    def decorate_command(self, msg):
        with self._lock:
            if not self.session_active or self._dry_run:
                return msg
            now = self._clock()
            if not self._input_fresh_locked(now, self._config.input_timeout_s):
                self._invalidate_locked(PolicyTaskFault.INTENT)
                return msg
            if self.staged is None:
                msg.frame_id = JAW_ONLY
                return msg
            action, source_time = self.staged
            self.staged = None
            if now - source_time > PUBLICATION_TIMEOUT:
                self._invalidate_locked(PolicyTaskFault.INFERENCE)
                return msg
            token = ACTION + uuid4().hex
            indices = {name: i for i, name in enumerate(msg.name)}
            msg.velocity = [0.0] * len(msg.name)
            for name, q in zip(ARM, action, strict=True):
                index = indices[name]
                msg.position[index] = float(q)
            msg.frame_id = token
            deadline = min(
                source_time + PUBLICATION_TIMEOUT,
                min(self._pose_at, self._buttons_at) + self._config.input_timeout_s,
            )
            # Encode the residual authorization in the existing host command
            # stamp only. This is not a feedback timestamp.
            msg.ts = time.time() - (now + PUBLICATION_TIMEOUT - deadline)
            self.pending = token, action, source_time
            return msg

    def state_snapshot(self):
        with self._lock:
            return {'session': 'active' if self.session_active else 'disengaged',
                    'fault': self.fault.value, 'actor_updates': 11500, 'onnx_sha256': POLICY_HASH,
                    'history_commits': self._runtime.tick if self.session_active else 0}

    def stop(self):
        super().stop()
        if not self.worker.closed:
            self.worker.close()


class EefDirectCoordinator(DamiaoTeleopCoordinator):
    actor_receipt: In[String]

    def _create_task_from_config(self, cfg):
        if cfg.type != TASK_TYPE:
            return super()._create_task_from_config(cfg)
        control_task_registry.register_bindings(
            TASK_TYPE, consumes={
                'teleop_buttons': ('on_teleop_buttons', 'broadcast'),
                'right_cartesian_command': ('on_right_cartesian_command', 'direct'),
                'actor_receipt': ('on_actor_receipt', 'direct'),
            }, exposes=['release', 'state_snapshot'],
        )
        self.arm_policy = EefDirectArmTask(cfg, self._connections['arm'])
        return self.arm_policy

    def _publish_command(self, robot, values):
        if self._estopped:
            return
        for out in self._connections[robot].command_messages(values, time.time()):
            self._command_ports[robot][out.port].publish(self.arm_policy.decorate_command(out.message))

    @rpc
    def stop(self):
        try:
            super().stop()
        finally:
            if hasattr(self, 'arm_policy'):
                self.arm_policy.stop()


def real_eef_direct(cfg):
    model = OPENYAM_QUEST_MODEL
    tasks = openyam_quest_tasks()
    tasks[0] = TaskConfig(name='openyam_policy', type=TASK_TYPE, joint_names=list(ARM), priority=40)
    return autoconnect(
        QuestArmTeleopModule.blueprint(zero_button='B', server_host=cfg.quest_bind,
                                       server_port=cfg.quest_port, input_timeout_s=0.25,
                                       ssl_certs_dir=cfg.certs),
        EefDirectCoordinator.blueprint(instance_name='ControlCoordinator', model=model.model,
                                         tasks=tasks, zero_stiffness_joints=[], tick_rate=TICK_RATE,
                                         reading_timeout_s=0.2),
        EefDirectConnection.blueprint(source='arm', can_port=cfg.can_port, teach=False),
        ManipulationModule.blueprint(model=model, gripper_tasks={'manipulator': 'arm_gripper'},
                                     kinematics=OPENYAM_QUEST_KINEMATICS,
                                     visualization={'backend': 'viser', 'host': cfg.viser_bind,
                                                    'port': cfg.viser_port}),
    ).remappings([
        (EefDirectConnection, 'position_command', 'arm_position_command'),
        (EefDirectCoordinator, 'arm_joint_state', 'joint_state'),
        (QuestArmTeleopModule, 'right_controller_output', 'right_cartesian_command'),
    ]).global_config(
        transport='zenoh', zenoh_domain=cfg.zenoh_domain, zenoh_scout_addr=cfg.zenoh_scout_addr,
        zenoh_scouting=False, serve_coordinator_rpc=False, viewer='none', build_native=False,
        simulation="",
    )
