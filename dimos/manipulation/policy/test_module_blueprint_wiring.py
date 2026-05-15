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

"""End-to-end wiring: `PolicyModule` (with `TestPolicy`) → coordinator-compatible
joint command path.

This test does NOT bring up a real `ControlCoordinator` — that pulls in
hardware drivers we don't want in unit tests. Instead it constructs a
`JointServoTask` directly (the same task the coordinator instantiates
when `task_type="servo"`) and feeds the `PolicyModule`'s published
`JointState` into it via the same `_on_joint_command`-style routing the
coordinator performs. The assertion is that the policy-driven `JointState`
actually arrives at the servo task as a target the coordinator could
write to hardware on the next tick.
"""

from __future__ import annotations

import time

import pytest

from dimos.control.tasks.servo_task import JointServoTask, JointServoTaskConfig
from dimos.manipulation.policy import (
    PolicyModule,
    TestPolicy,
    policy_servo_task_config,
    register_backend,
)
from dimos.msgs.sensor_msgs.JointState import JointState


@pytest.fixture
def policy_module():
    """Policy node wired with a deterministic TestPolicy backend."""
    register_backend(
        "__test_e2e__",
        lambda **_: TestPolicy(
            joint_names=["arm/j1", "arm/j2"],
            amplitude=0.0,
            frequency=1.0,
            center=[0.5, -0.5],  # constant outputs make the assertion robust.
        ),
    )
    node = PolicyModule(
        backend="__test_e2e__",
        policy_rate=200.0,
        joint_names=["arm/j1", "arm/j2"],
        camera_key="main",
    )
    # Inject a backend that we control (the registered factory above just
    # constructs a fresh TestPolicy; we do the same for the test path).
    node._backend = TestPolicy(
        joint_names=["arm/j1", "arm/j2"],
        amplitude=0.0,
        frequency=1.0,
        center=[0.5, -0.5],
    )
    node._backend.initialize()
    yield node
    try:
        node._close_module()
    except Exception:
        pass


def test_policy_publishes_joint_state_consumable_by_servo_task(policy_module):
    # Build the same servo task config the coordinator helper would
    # produce when no teleop tasks are wired alongside.
    servo_cfg = policy_servo_task_config(
        name="policy_servo",
        joint_names=["arm/j1", "arm/j2"],
    )
    assert servo_cfg.priority == 0  # no teleop overlap

    servo_task = JointServoTask(
        servo_cfg.name,
        JointServoTaskConfig(
            joint_names=list(servo_cfg.joint_names),
            priority=servo_cfg.priority,
        ),
    )
    servo_task.start()

    # Subscribe the servo task to the policy's joint_command output the
    # same way `ControlCoordinator._on_joint_command` would route by name.
    def _coord_route(msg: JointState) -> None:
        positions_by_name = dict(zip(msg.name, msg.position, strict=False))
        servo_task.set_target_by_name(positions_by_name, t_now=time.perf_counter())

    policy_module.joint_command.subscribe(_coord_route)

    # Enable rollout (off by default since policy-rollout-deployment) so
    # _tick_once is allowed to publish. Also satisfy the first-Buttons gate
    # since the default config requires a Buttons message before publishing.
    policy_module._rollout_enabled = True
    policy_module._first_buttons_received = True

    # Drive a tick: the published JointState must round-trip into the
    # servo task as a position target the coordinator would write to
    # hardware on the next tick.
    policy_module._on_joint_state(JointState(name=["arm/j1", "arm/j2"], position=[0.0, 0.0]))
    cmd = policy_module._tick_once()
    assert cmd is not None

    assert servo_task.is_streaming() is True
    # The servo task's internal target should reflect the policy output.
    assert servo_task._target == pytest.approx([0.5, -0.5], abs=1e-9)
