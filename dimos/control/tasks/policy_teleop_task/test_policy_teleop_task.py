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

import math

import pytest

from dimos.control.task import (
    ControlMode,
    ControlTask,
    CoordinatorState,
    JointCommandOutput,
    JointStateSnapshot,
    ResourceClaim,
)
from dimos.control.tasks.gripper_task.gripper_task import (
    GripperControlTask,
    GripperControlTaskConfig,
)
from dimos.control.tasks.policy_teleop_task.policy_teleop_task import (
    PolicyTeleopTask,
    PolicyTeleopTaskParams,
    TakeoverContext,
)

JOINTS = ["j1", "j2", "gripper"]


def _state(t_now=1.0, dt=0.5):
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"j1": 0.1, "j2": 0.2, "gripper": 0.3}),
        t_now=t_now,
        dt=dt,
    )


def _output(**positions):
    return JointCommandOutput(
        joint_names=list(positions),
        positions=list(positions.values()),
        mode=ControlMode.SERVO_POSITION,
    )


@pytest.fixture
def sources(mocker):
    tasks = {}
    for name in ("operator", "policy"):
        task = mocker.Mock(spec=ControlTask)
        task.name = name
        task.claim.return_value = ResourceClaim(frozenset(JOINTS))
        task.is_active.return_value = True
        task.compute.return_value = None
        tasks[name] = task
    return tasks


@pytest.fixture
def make_task(sources):
    def make(**kwargs):
        params = PolicyTeleopTaskParams(
            operator_tasks=["operator"], policy_tasks=["policy"], **kwargs
        )
        return PolicyTeleopTask("blend", JOINTS, params, sources, priority=20)

    return make


@pytest.mark.parametrize("alpha, expected", [(0, -2), (0.5, 1), (1, 4)])
def test_blends_opposing_joint_targets_with_policy_share(make_task, sources, alpha, expected):
    sources["operator"].compute.return_value = _output(j1=-2)
    sources["policy"].compute.return_value = _output(j1=4)
    task = make_task(alpha=alpha)

    assert task.compute(_state()) == _output(j1=expected)
    assert task.claim() == ResourceClaim(frozenset(JOINTS), 20, ControlMode.SERVO_POSITION)


@pytest.mark.parametrize("side", ["operator", "policy"])
def test_a_single_source_passes_through_unchanged(make_task, sources, side):
    sources[side].compute.return_value = _output(j1=4, gripper=0.8)
    task = make_task()

    assert task.compute(_state(dt=0.01)) == _output(j1=4, gripper=0.8)
    assert task.get_status()[f"{side}_present"] == ["gripper", "j1"]


def test_presence_and_timeout_are_per_joint_and_hold_inactive_policy_targets(make_task, sources):
    sources["operator"].compute.return_value = _output(j1=-2, gripper=0.8)
    sources["policy"].compute.return_value = _output(j1=4, j2=6)
    task = make_task()
    assert task.compute(_state()) == _output(j1=1, j2=6, gripper=0.8)

    sources["operator"].compute.return_value = None
    sources["policy"].is_active.return_value = False
    assert task.compute(_state(1.1)) == _output(j1=1, j2=6, gripper=0.8)
    sources["operator"].compute.return_value = _output(j1=-2)
    assert task.compute(_state(1.21)) == _output(j1=-2)
    assert task.get_status() == {
        "weights": {"j1": 0.0, "j2": 1.0, "gripper": 0.0},
        "operator_present": ["j1"],
        "policy_present": [],
    }
    sources["operator"].compute.return_value = None
    assert task.compute(_state(1.5)) is None


def test_operator_arrival_and_departure_ramp_without_stopping_policy(make_task, sources):
    task = make_task(alpha=0.0, weight_rate=2)
    sources["policy"].compute.return_value = _output(j1=4)
    assert task.compute(_state(0, 0.1)) == _output(j1=4)
    sources["operator"].compute.return_value = _output(j1=-2)

    assert task.compute(_state(0.1, 0.1)).positions == pytest.approx([2.8])
    assert task.compute(_state(0.2, 0.1)).positions == pytest.approx([1.6])
    sources["operator"].is_active.return_value = False
    task.compute(_state(0.3, 0.1))
    task.compute(_state(0.4, 0.1))
    # The last operator target now expires; it remains the ramp's U endpoint.
    assert task.compute(_state(0.5, 0.1)).positions == pytest.approx([0.4])
    task.compute(_state(0.6, 0.1))
    task.compute(_state(0.7, 0.1))
    assert task.compute(_state(0.8, 0.1)).positions == pytest.approx([4])
    assert sources["policy"].compute.call_count == 9
    sources["policy"].on_preempted.assert_not_called()


def test_fade_towards_operator_after_policy_timeout(make_task, sources):
    task = make_task(alpha=1.0)
    sources["operator"].compute.return_value = _output(j1=-2)
    sources["policy"].compute.return_value = _output(j1=4)
    task.compute(_state())
    sources["policy"].compute.return_value = None

    assert task.compute(_state(1.21, 0.1)).positions == pytest.approx([2.8])
    assert task.get_status()["policy_present"] == []


def test_takeover_receives_q_u_p_once_and_clips_per_joint_with_fallback(make_task, sources, mocker):
    takeover = mocker.Mock(return_value={"j1": -10, "j2": 10})
    task = make_task(alpha=0.25, takeover_fn=takeover)
    task.compute(_state())
    takeover.assert_not_called()
    sources["operator"].compute.return_value = _output(j1=-2, j2=-4, gripper=0)
    sources["policy"].compute.return_value = _output(j1=4, j2=8, gripper=1)

    assert task.compute(_state()) == _output(j1=-2, j2=8, gripper=0.25)
    takeover.assert_called_once()
    context = takeover.call_args.args[0]
    assert context == TakeoverContext(
        tuple(JOINTS),
        {"j1": 0.1, "j2": 0.2, "gripper": 0.3},
        {"j1": -2, "j2": -4, "gripper": 0},
        {"j1": 4, "j2": 8, "gripper": 1},
        1,
        0.5,
    )
    with pytest.raises(TypeError):
        context.operator["j1"] = 999


@pytest.mark.parametrize("result", [0.75, {"j1": 0.75}])
def test_takeover_accepts_scalar_or_mapping(make_task, sources, mocker, result):
    task = make_task(takeover_fn=mocker.Mock(return_value=result))
    sources["operator"].compute.return_value = _output(j1=-2)
    sources["policy"].compute.return_value = _output(j1=4)
    assert task.compute(_state()) == _output(j1=2.5)


@pytest.mark.parametrize("failure", [RuntimeError("failed"), math.nan, {"j1": math.inf}])
def test_takeover_failure_falls_back_for_whole_tick_and_logs_once(
    make_task, sources, mocker, failure
):
    takeover = mocker.Mock()
    if isinstance(failure, Exception):
        takeover.side_effect = failure
    else:
        takeover.return_value = failure
    log = mocker.patch("dimos.control.tasks.policy_teleop_task.policy_teleop_task.logger.exception")
    task = make_task(takeover_fn=takeover)
    sources["operator"].compute.return_value = _output(j1=-2, j2=0)
    sources["policy"].compute.return_value = _output(j1=4, j2=4)

    assert task.compute(_state()) == _output(j1=1, j2=2)
    assert task.compute(_state(2)) == _output(j1=1, j2=2)
    log.assert_called_once()


def context_takeover(context: TakeoverContext) -> float:
    return context.measured["j1"]


def test_takeover_resolves_import_path(make_task, sources):
    task = make_task(takeover_fn=f"{__name__}:context_takeover")
    sources["operator"].compute.return_value = _output(j1=0)
    sources["policy"].compute.return_value = _output(j1=10)
    assert task.compute(_state()) == _output(j1=1)


@pytest.mark.parametrize("alpha, expected", [(0.49, 0), (0.5, 1), (1, 1)])
def test_switch_joint_selects_a_target_without_interpolation(make_task, sources, alpha, expected):
    task = make_task(alpha=alpha, switch_joints=["gripper"])
    sources["operator"].compute.return_value = _output(gripper=0)
    sources["policy"].compute.return_value = _output(gripper=1)
    assert task.compute(_state()) == _output(gripper=expected)


def test_independent_arm_and_gripper_operator_sources_share_one_blend(sources):
    gripper = GripperControlTask(
        "operator_gripper",
        GripperControlTaskConfig(joint_names=["gripper"], hold_duration=0.1),
        limits=[(0, 1)],
    )
    sources[gripper.name] = gripper
    sources["operator"].claim.return_value = ResourceClaim(frozenset({"j1", "j2"}))
    sources["operator"].compute.return_value = _output(j1=-2)
    sources["policy"].compute.return_value = _output(j1=4, gripper=1)
    assert gripper.set_position([0], t_now=1)
    task = PolicyTeleopTask(
        "blend",
        JOINTS,
        PolicyTeleopTaskParams(
            operator_tasks=["operator", gripper.name],
            policy_tasks=["policy"],
            alpha=0.25,
            switch_joints=["gripper"],
        ),
        sources,
    )

    assert task.compute(_state()) == _output(j1=-0.5, gripper=0)
    # A gripper's hold expiry does not remove the independent arm source.
    task.compute(_state(1.11))
    assert task.compute(_state(1.21)) == _output(j1=-0.5, gripper=1)
    assert task.get_status()["operator_present"] == ["j1"]


def test_estop_and_runtime_reset_forget_targets_and_weights(make_task, sources):
    task = make_task(alpha=0)
    sources["operator"].compute.return_value = _output(j1=0)
    sources["policy"].compute.return_value = _output(j1=1)
    task.compute(_state())
    task.set_estop(True)
    task.set_estop(False)
    sources["operator"].is_active.return_value = False
    sources["policy"].is_active.return_value = False

    assert task.is_active()
    assert task.compute(_state(1.01)) is None
    assert task.get_status() == {
        "weights": dict.fromkeys(JOINTS, 1.0),
        "operator_present": [],
        "policy_present": [],
    }
    sources["policy"].is_active.return_value = True
    assert task.compute(_state(1.02)) == _output(j1=1)
    task.reset_runtime_state()
    sources["policy"].is_active.return_value = False
    assert task.compute(_state(1.03)) is None


def test_real_preemption_is_forwarded_to_every_source(make_task, sources):
    task = make_task()
    task.on_preempted("safety", frozenset({"j1"}))
    for source in sources.values():
        source.on_preempted.assert_called_once_with("safety", frozenset({"j1"}))


@pytest.mark.parametrize(
    "params",
    [
        {"operator_tasks": ["missing"]},
        {"operator_tasks": ["operator", "operator"]},
        {"operator_tasks": ["policy"]},
        {"policy_tasks": ["blend"]},
        {"switch_joints": ["unknown"]},
        {"takeover_fn": "bad_path"},
        {"takeover_fn": "math:pi"},
    ],
)
def test_bad_source_configuration_fails_at_creation(sources, params):
    settings = {"operator_tasks": ["operator"], "policy_tasks": ["policy"], **params}
    with pytest.raises(ValueError):
        PolicyTeleopTask("blend", JOINTS, PolicyTeleopTaskParams(**settings), sources)


@pytest.mark.parametrize(
    "params",
    [
        {"alpha": -0.1},
        {"alpha": 1.1},
        {"alpha": math.nan},
        {"weight_rate": 0},
        {"weight_rate": math.inf},
        {"source_timeout_s": -1},
        {"source_timeout_s": math.nan},
    ],
)
def test_invalid_numeric_configuration_is_rejected(make_task, params):
    with pytest.raises(ValueError):
        make_task(**params)


def test_all_source_joints_must_belong_to_blender(sources):
    with pytest.raises(ValueError, match="every source joint"):
        PolicyTeleopTask(
            "blend",
            ["j1"],
            PolicyTeleopTaskParams(operator_tasks=["operator"], policy_tasks=["policy"]),
            sources,
        )


def test_sources_cannot_be_shared_by_two_blenders(make_task, sources):
    sources["blend"] = make_task()
    with pytest.raises(ValueError, match="nested or shared"):
        PolicyTeleopTask(
            "second",
            JOINTS,
            PolicyTeleopTaskParams(operator_tasks=["operator"], policy_tasks=["policy"]),
            sources,
        )
