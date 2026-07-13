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

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

import numpy as np
import pytest

from dimos.control.components import make_humanoid_joints
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.g1_dual_arm_ik_task.g1_dual_arm_ik_task import (
    ARM_JOINT_SHORT_NAMES,
    G1DualArmIK,
    G1DualArmIKSolverConfig,
    G1DualArmIKTask,
    G1DualArmIKTaskConfig,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.teleop.quest.quest_types import Buttons

_URDF = Path(__file__).resolve().parents[3] / "robot" / "unitree" / "g1" / "g1.urdf"
_ARM_JOINTS = make_humanoid_joints("g1")[15:]


def _pose(x: float, y: float, z: float, frame_id: str, ts: float = 1.0) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, y, z),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ts=ts,
        frame_id=frame_id,
    )


def _state_at(t_now: float, positions: dict[str, float] | None = None) -> CoordinatorState:
    pos = {n: 0.0 for n in _ARM_JOINTS}
    if positions:
        pos.update(positions)
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions=pos,
            joint_velocities={n: 0.0 for n in _ARM_JOINTS},
            joint_efforts={n: 0.0 for n in _ARM_JOINTS},
            timestamp=t_now,
        ),
        t_now=t_now,
        dt=0.02,
    )


def _buttons(left_trigger: float, right_trigger: float) -> Buttons:
    buttons = Buttons()
    buttons.pack_analog_triggers(left=left_trigger, right=right_trigger)
    return buttons


@pytest.fixture(scope="module")
def solver() -> G1DualArmIK:
    return G1DualArmIK(_URDF)


@pytest.fixture
def task() -> Iterator[G1DualArmIKTask]:
    task = G1DualArmIKTask(
        "dual_arm_ik",
        G1DualArmIKTaskConfig(joint_names=list(_ARM_JOINTS), model_path=_URDF),
    )
    task.start()
    yield task
    task.stop()


def test_solver_reduces_to_14_dof(solver: G1DualArmIK) -> None:
    assert set(solver.joint_order) == set(ARM_JOINT_SHORT_NAMES)


def test_solver_recovers_reachable_wrist_targets() -> None:
    # Posture cost intentionally biases solutions toward neutral (a few cm
    # of steady-state offset); disable it to test pure target recovery.
    solver = G1DualArmIK(_URDF, G1DualArmIKSolverConfig(posture_cost=0.0))
    q_ref = np.zeros(14)
    q_ref[solver.joint_order.index("left_elbow")] = 0.6
    q_ref[solver.joint_order.index("right_elbow")] = 0.6
    left_target, right_target = solver.forward_wrists(q_ref)

    # The solver is streaming (few iterations per control tick, warm-started
    # from the previous tick) — emulate a handful of ticks.
    q_sol = np.zeros(14)
    for _ in range(5):
        q_sol = solver.solve(left_target, right_target, q_sol)
    left_sol, right_sol = solver.forward_wrists(q_sol)

    assert np.linalg.norm(left_sol.translation - left_target.translation) < 0.005
    assert np.linalg.norm(right_sol.translation - right_target.translation) < 0.005


def test_default_posture_cost_keeps_targets_within_workspace_slack(solver: G1DualArmIK) -> None:
    q_ref = np.zeros(14)
    q_ref[solver.joint_order.index("left_elbow")] = 0.6
    q_ref[solver.joint_order.index("right_elbow")] = 0.6
    left_target, right_target = solver.forward_wrists(q_ref)

    q_sol = np.zeros(14)
    for _ in range(5):
        q_sol = solver.solve(left_target, right_target, q_sol)
    left_sol, right_sol = solver.forward_wrists(q_sol)

    assert np.linalg.norm(left_sol.translation - left_target.translation) < 0.05
    assert np.linalg.norm(right_sol.translation - right_target.translation) < 0.05


def test_inactive_without_engage_or_history(task: G1DualArmIKTask) -> None:
    assert task.compute(_state_at(1.0)) is None
    assert not task.is_active()


def test_routes_hand_from_frame_id_suffix(task: G1DualArmIKTask) -> None:
    assert task.on_cartesian_command(_pose(0.3, 0.2, 0.2, "dual_arm_ik/left"), t_now=1.0)
    assert task.on_cartesian_command(_pose(0.3, -0.2, 0.2, "dual_arm_ik/right"), t_now=1.0)
    assert not task.on_cartesian_command(_pose(0.3, 0.0, 0.2, "dual_arm_ik/head"), t_now=1.0)


def test_engaged_tracking_outputs_all_arm_joints(task: G1DualArmIKTask) -> None:
    task.on_buttons(_buttons(1.0, 1.0))
    task.on_cartesian_command(_pose(0.3, 0.2, 0.2, "dual_arm_ik/left"), t_now=1.0)
    task.on_cartesian_command(_pose(0.3, -0.2, 0.2, "dual_arm_ik/right"), t_now=1.0)

    out = task.compute(_state_at(1.0))
    assert out is not None
    assert set(out.joint_names) == set(_ARM_JOINTS)
    assert len(out.positions) == 14
    assert all(np.isfinite(out.positions))


def test_disengage_holds_last_solution(task: G1DualArmIKTask) -> None:
    task.on_buttons(_buttons(1.0, 1.0))
    task.on_cartesian_command(_pose(0.3, 0.2, 0.2, "dual_arm_ik/left"), t_now=1.0)
    task.on_cartesian_command(_pose(0.3, -0.2, 0.2, "dual_arm_ik/right"), t_now=1.0)
    tracked = task.compute(_state_at(1.0))
    assert tracked is not None

    task.on_buttons(_buttons(0.0, 0.0))
    held = task.compute(_state_at(2.0))
    assert held is not None
    assert held.positions == tracked.positions
    assert task.is_active()


def test_stale_targets_freeze_instead_of_tracking(task: G1DualArmIKTask) -> None:
    task.on_buttons(_buttons(1.0, 1.0))
    task.on_cartesian_command(_pose(0.3, 0.2, 0.2, "dual_arm_ik/left"), t_now=1.0)
    task.on_cartesian_command(_pose(0.3, -0.2, 0.2, "dual_arm_ik/right"), t_now=1.0)
    first = task.compute(_state_at(1.0))
    assert first is not None

    # No fresh targets for > timeout: output must stay pinned to the last
    # solution even though we are still engaged.
    later = task.compute(_state_at(10.0))
    assert later is not None
    assert later.positions == first.positions


def test_output_is_rate_limited_toward_solution(task: G1DualArmIKTask) -> None:
    import math

    task._config.max_joint_speed_deg_s = 1.0
    task.on_buttons(_buttons(1.0, 1.0))
    task.on_cartesian_command(_pose(0.35, 0.25, 0.3, "dual_arm_ik/left"), t_now=1.0)
    task.on_cartesian_command(_pose(0.35, -0.25, 0.3, "dual_arm_ik/right"), t_now=1.0)

    # Measured pose is all zeros; the command may move at most
    # max_joint_speed * dt away from it on the first tick, and must keep
    # creeping (not jump) on the next.
    state = _state_at(1.0)
    first = task.compute(state)
    assert first is not None
    step = math.radians(1.0) * state.dt
    assert max(abs(p) for p in first.positions) <= step + 1e-12

    second = task.compute(_state_at(1.02))
    assert second is not None
    deltas = np.abs(np.array(second.positions) - np.array(first.positions))
    assert deltas.max() <= step + 1e-12
    assert deltas.max() > 0.0
