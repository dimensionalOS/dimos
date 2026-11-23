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

"""The two real reference consumers against a recording, co-located test plant."""

from collections.abc import Callable, Iterator
import os
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.test_coordinator_connections import LocalTransport
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import ExecutionStatus, OperationStatus
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.openyam.blueprints.teleop import openyam_trimode
from dimos.robot.manipulators.openyam.joints import OPENYAM_ARM_JOINTS, OPENYAM_GRIPPER_JOINT
from dimos.robot.manipulators.openyam.trimode_coordinator import (
    TrimodeCoordinator,
    trimode_connection,
)
from dimos.robot.manipulators.openyam.trimode_quest import TrimodeQuest
from dimos.robot.manipulators.openyam.trimode_tasks import (
    JOINT,
    JOINTS,
    POSE,
    QUEST_FRESH,
    QUEST_TIMEOUT,
    Feedback,
    PlannedReference,
    QuestReference,
)
from dimos.robot.manipulators.openyam.trimode_tracker import (
    Actor,
    JointGoal,
    Vector,
    load_actor,
    tcp_pose,
)
from dimos.teleop.webxr.controller_types import Buttons, Hand, WebXRControllerState

HOME = (0.0, 1.047, 1.047, 0.0, 0.0, 0.0)


class Plant:
    def __init__(self) -> None:
        self.q = HOME
        self.dq: Vector = (0.0,) * 6
        self.jaw = 0.04
        self.arm_writes: list[Vector] = []
        self.jaw_writes: list[float] = []
        self.include_velocity = True

    def read(self) -> JointState:
        return JointState(
            name=[*JOINTS, OPENYAM_GRIPPER_JOINT],
            position=[*self.q, self.jaw],
            velocity=[*self.dq, 0.0] if self.include_velocity else [],
        )

    def submit_arm(self, q_des: Vector) -> None:
        self.arm_writes.append(q_des)

    def submit_jaw(self, position: float) -> None:
        self.jaw_writes.append(position)
        self.jaw = position


def test_opt_in_blueprint_has_no_stock_connection() -> None:
    blueprint = openyam_trimode(model_path=Path("unused"), plant_factory=Plant).solidify()
    atoms = {atom.module: atom for atom in blueprint.active_blueprints}
    assert set(atoms) == {TrimodeQuest, TrimodeCoordinator, ManipulationModule}
    ctrl = atoms[TrimodeCoordinator]
    assert ctrl.kwargs["plant_factory"] is Plant
    assert set(ctrl.kwargs["connections"][0].joint_commands) == {"joint7"}
    assert (
        blueprint.remapping_map[(TrimodeQuest.name, "right_controller_output")]
        == "right_cartesian_command"
    )


def wait(predicate: Callable[[], Any], timeout: float = 2.0) -> None:
    end = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < end, "timed out waiting for test condition"
        time.sleep(0.003)


def model() -> RobotModel:
    names = [*OPENYAM_ARM_JOINTS, "joint7"]
    xml = '<robot name="test"><link name="base"/>'
    parent = "base"
    for i, name in enumerate(names):
        lo, hi, kind = (-3.0, 3.0, "revolute") if i < 6 else (0.01, 0.06, "prismatic")
        xml += (
            f'<link name="l{i}"/><joint name="{name}" type="{kind}">'
            f'<parent link="{parent}"/><child link="l{i}"/>'
            f'<limit lower="{lo}" upper="{hi}" velocity="2" effort="10"/></joint>'
        )
        parent = f"l{i}"
    return RobotModel.from_urdf_string(xml + "</robot>")


def plan(end: Vector, duration: float = 0.12) -> GeneratedPlan:
    # The external layout need not be the model order.
    names = list(reversed(JOINTS))
    trajectory = JointTrajectory(
        joint_names=names,
        points=[
            TrajectoryPoint(positions=list(reversed(HOME)), time_from_start=0.0),
            TrajectoryPoint(positions=list(reversed(end)), time_from_start=duration),
        ],
    )
    return GeneratedPlan(
        group_ids=("manipulator",), trajectory=trajectory, status=PlanningStatus.SUCCESS
    )


class Rig:
    def __init__(self, path: Path, mode: str) -> None:
        robot = model()
        self.plant = Plant()
        self.control = TrimodeCoordinator(
            instance_name="ControlCoordinator",
            model_path=path,
            quest_mode=mode,
            connections=[trimode_connection(robot)],
            publish_joint_state=False,
        )
        self.control.bind_local_plant(self.plant)
        self.quest = TrimodeQuest()
        for src, dst in (
            ("teleop_buttons", "teleop_buttons"),
            ("right_controller_output", "right_cartesian_command"),
            ("right_gripper_command", "right_gripper_command"),
        ):
            wire = LocalTransport()
            self.control.set_transport(dst, wire)  # type: ignore[arg-type]
            self.quest.set_transport(src, wire)  # type: ignore[arg-type]
        for dst in ("joint_command", "left_cartesian_command"):
            self.control.set_transport(dst, LocalTransport())  # type: ignore[arg-type]
        self.control.start()
        self.manipulation = ManipulationModule(
            model=RobotModelConfig(model=robot, joint_names=list(JOINTS), base_link="base")
        )
        self.manipulation._control_coordinator = self.control
        self.manipulation._initialize_execution()
        self.manipulation._started = True
        wait(lambda: self.control._read_group(JOINT))

    def quest_input(self, grip: float, *, fresh: bool = True, x: float = 0.0) -> None:
        now = time.monotonic() if fresh else None
        self.quest._last_pose_update[Hand.RIGHT] = self.quest._last_controller_update[
            Hand.RIGHT
        ] = now
        self.quest._publish_button_state(
            None, WebXRControllerState(is_left=False, grip=grip, trigger=0.75)
        )
        if fresh:
            self.quest._publish_msg(Hand.RIGHT, PoseStamped(position=(x, 0.0, 0.0)))

    def execute_viser(self, generated: GeneratedPlan) -> bool:
        # Viser's actual callback, with a materialized synthetic plan already staged.
        self.manipulation._pending_plan = generated
        self.manipulation._state = OperationStatus.HAS_PLAN
        return self.manipulation._execute_generated_plan(generated)

    def close(self) -> None:
        self.quest.stop()
        self.manipulation.stop()
        self.control.stop()


@pytest.fixture
def make_rig(monkeypatch: pytest.MonkeyPatch) -> Iterator[Callable[[Actor, str], Rig]]:
    rigs: list[Rig] = []

    def make(actor: Actor, mode: str = "pose") -> Rig:
        monkeypatch.setattr(
            "dimos.robot.manipulators.openyam.trimode_coordinator.load_actor", lambda _: actor
        )
        rig = Rig(Path("not-used-by-injected-loader"), mode)
        rigs.append(rig)
        return rig

    yield make
    for rig in rigs:
        rig.close()


@pytest.mark.parametrize("mode, code", [("pose", 0), ("position", 1)])
def test_real_cpu_quest_and_viser_share_single_arm_sink(
    make_rig: Callable[[Actor, str], Rig], mode: str, code: int
) -> None:
    artifact = os.getenv("DIMOS_TRIMODE_ONNX")
    if not artifact:
        pytest.skip("set DIMOS_TRIMODE_ONNX to the pinned local actor")
    infer = load_actor(Path(artifact))
    observations: list[np.ndarray] = []
    outputs: list[Vector] = []

    def actor(obs: np.ndarray) -> np.ndarray:
        observations.append(obs.copy()[0])
        action = infer(obs)
        outputs.append(tuple(action[0].tolist()))
        return action

    rig = make_rig(actor, mode)
    wait(lambda: rig.plant.jaw_writes)
    assert not rig.plant.arm_writes
    rig.quest_input(0.0)
    rig.quest_input(1.0)
    wait(lambda: rig.plant.arm_writes)
    rig.quest_input(1.0, x=0.004)
    wait(lambda: len(rig.plant.arm_writes) >= 3 and abs(rig.plant.jaw - 0.0475) < 1e-5)
    rig.quest_input(0.0)
    assert observations[0][317] == code
    np.testing.assert_allclose(observations[0][304:311], tcp_pose(HOME), atol=1e-7)
    np.testing.assert_array_equal(observations[0][311:317], 0.0)
    np.testing.assert_array_equal(observations[0][:304].reshape(16, 19)[:, -1], [0.0] * 15 + [1.0])
    np.testing.assert_array_equal(
        observations[1][297:303], np.asarray(outputs[0], dtype=np.float32)
    )
    end = (0.04, *HOME[1:])
    assert rig.execute_viser(plan(end))
    wait(lambda: any(obs[317] == 2 and np.isclose(obs[311], end[0]) for obs in observations))
    np.testing.assert_allclose(observations[-1][304:311], tcp_pose(end), atol=1e-7)
    rig.plant.q = end
    rig.plant.dq = (1.0,) * 6
    wait(lambda: len([o for o in observations if o[317] == 2 and np.all(o[291:297] == 1.0)]) >= 2)
    assert (
        rig.control.task_invoke("joint_trajectory", "get_status", {"t_now": None}).state
        is TrajectoryState.EXECUTING
    )
    rig.plant.dq = (0.0,) * 6
    assert rig.manipulation.wait_for_execution(timeout=1.0).status == ExecutionStatus.COMPLETED
    # Native endpoint reporting, not arm setpoint substitution; exactly the actor outputs went to the sink.
    pending = iter(outputs)
    assert all(command in pending for command in rig.plant.arm_writes)
    before = len(rig.plant.arm_writes)
    time.sleep(0.06)
    assert len(rig.plant.arm_writes) == before


@pytest.mark.parametrize("event", ["cancel", "takeover", "estop", "velocity_stale", "reset"])
def test_blocked_inference_cannot_write_after_authority_loss(
    make_rig: Callable[[Actor, str], Rig], event: str
) -> None:
    entered, finish = threading.Event(), threading.Event()

    def actor(obs: np.ndarray) -> np.ndarray:
        entered.set()
        assert finish.wait(1.0)
        return np.asarray([HOME], dtype=np.float32)

    rig = make_rig(actor, "pose")
    assert rig.execute_viser(plan((0.1, *HOME[1:]), 1.0))
    try:
        assert entered.wait(1.0)
        if event == "cancel":
            assert rig.manipulation._execution_manager.cancel().status == ExecutionStatus.ABORTED
        elif event == "takeover":
            rig.quest_input(0.0)
            rig.quest_input(1.0)
            rig.quest_input(0.0)
        elif event == "estop":
            rig.control.set_estop(True)
        elif event == "reset":
            assert all(rig.control.reset_runtime_state().values())
        else:
            rig.plant.include_velocity = False
            wait(lambda: rig.control._read_group(JOINT) is None)
            wait(
                lambda: rig.control.task_invoke("joint_trajectory", "get_state")
                is TrajectoryState.FAULT
            )
            rig.plant.include_velocity = True
            wait(lambda: rig.control._read_group(JOINT))
    finally:
        finish.set()
    time.sleep(0.065)
    assert not rig.plant.arm_writes


def test_disconnect_neutral_is_not_physical_release(make_rig: Callable[[Actor, str], Rig]) -> None:
    rig = make_rig(lambda _: np.asarray([HOME], dtype=np.float32), "pose")
    rig.quest_input(0.0)
    rig.quest_input(1.0)
    wait(lambda: rig.plant.arm_writes)
    rig.quest_input(0.0, fresh=False)
    count = len(rig.plant.arm_writes)
    for _ in range(5):
        rig.quest_input(1.0)
        time.sleep(0.02)
    assert len(rig.plant.arm_writes) == count
    rig.quest_input(0.0)
    rig.quest_input(1.0)
    wait(lambda: len(rig.plant.arm_writes) > count)


def test_first_held_packet_after_idle_gap_cannot_take_over() -> None:
    takeovers: list[bool] = []
    task = QuestReference(POSE, lambda _: None, lambda: takeovers.append(True))
    released = Buttons(data=QUEST_FRESH)
    held = Buttons(data=QUEST_FRESH)
    held.right_grip = True
    task.on_teleop_buttons(released, 1.0)
    task.on_teleop_buttons(held, 1.0 + QUEST_TIMEOUT)
    task.on_teleop_buttons(held, 1.3)
    assert not takeovers and not task.is_active()
    task.on_teleop_buttons(released, 1.4)
    task.on_teleop_buttons(held, 1.5)
    assert takeovers == [True]


@pytest.mark.parametrize("stale", ["pose", "joy"])
def test_raw_gap_clears_cached_pair_before_replacement(
    monkeypatch: pytest.MonkeyPatch, request: pytest.FixtureRequest, stale: str
) -> None:
    now = 100.0
    monkeypatch.setattr(time, "monotonic", lambda: now)
    quest = TrimodeQuest()
    request.addfinalizer(quest.stop)
    buttons: list[Buttons] = []
    poses: list[PoseStamped] = []
    monkeypatch.setattr(
        quest, "_publish_buttons", lambda msg: buttons.append(Buttons.lcm_decode(msg.lcm_encode()))
    )
    monkeypatch.setattr(quest, "_publish_gripper_commands", lambda *_: None)
    wire = LocalTransport()
    wire.subscribe(lambda msg: poses.append(PoseStamped.lcm_decode(msg.lcm_encode())))
    quest.set_transport("right_controller_output", wire)  # type: ignore[arg-type]

    def pose() -> None:
        quest._on_pose(PoseStamped(frame_id="right"))

    def joy(grip: float) -> None:
        assert quest._on_joy(Joy(frame_id="right", axes=[0, 0, 0, grip], buttons=[0] * 6))

    def publish() -> None:
        quest._publish_button_state(None, quest._controllers[Hand.RIGHT])
        if (msg := quest._get_output_pose(Hand.RIGHT)) is not None:
            quest._publish_msg(Hand.RIGHT, msg)

    pose()
    joy(0.0)
    publish()
    now = 100.125
    if stale == "pose":
        joy(0.0)
    else:
        pose()
    publish()
    assert len(buttons) == 1 and buttons[0].data & QUEST_FRESH and not buttons[0].right_grip
    assert [msg.ts for msg in poses] == [100.25, 100.25]  # cached publishes do not extend it

    now = 100.25  # no control-loop expiry iteration before the next ingress
    if stale == "pose":
        pose()
    else:
        joy(1.0)
    publish()
    assert len(poses) == 2
    assert all(not msg.data & QUEST_FRESH for msg in buttons[1:])
    if stale == "pose":
        joy(1.0)
    else:
        pose()
    publish()
    assert buttons[-1].right_grip and not buttons[-1].data & QUEST_FRESH
    assert len(poses) == 2
    takeovers: list[bool] = []
    task = QuestReference(POSE, lambda _: None, lambda: takeovers.append(True))
    for i, msg in enumerate(buttons):
        task.on_teleop_buttons(msg, 100.0 + 0.02 * i)
    assert not takeovers
    now += 0.01
    pose()
    joy(0.0)
    publish()
    assert buttons[-1].data & QUEST_FRESH and not buttons[-1].right_grip
    now += 0.01
    pose()
    joy(1.0)
    publish()
    assert buttons[-1].data & QUEST_FRESH and buttons[-1].right_grip


def test_cancel_revokes_arm_during_blocked_inference_while_jaw_keeps_writing(
    make_rig: Callable[[Actor, str], Rig],
) -> None:
    entered, finish, returned = threading.Event(), threading.Event(), threading.Event()
    calls = 0

    def actor(_: np.ndarray) -> np.ndarray:
        nonlocal calls
        calls += 1
        if calls == 2:
            entered.set()
            assert finish.wait(1.0)
            returned.set()
        return np.asarray([HOME], dtype=np.float32)

    rig = make_rig(actor, "pose")
    assert rig.execute_viser(plan((0.1, *HOME[1:]), 1.0))
    try:
        assert entered.wait(1.0)
        assert len(rig.plant.arm_writes) == 1
        assert rig.manipulation._execution_manager.cancel().status is ExecutionStatus.ABORTED
        jaw_count = len(rig.plant.jaw_writes)
        wait(lambda: len(rig.plant.jaw_writes) > jaw_count)
    finally:
        finish.set()
    assert returned.wait(1.0)
    jaw_count = len(rig.plant.jaw_writes)
    wait(lambda: len(rig.plant.jaw_writes) > jaw_count + 3)
    assert len(rig.plant.arm_writes) == 1


@pytest.mark.parametrize("bad", ["position", "velocity", "expired", None])
def test_planned_settle_validates_measurement_and_reports_progress(
    monkeypatch: pytest.MonkeyPatch, bad: str | None
) -> None:
    now = 100.0
    monkeypatch.setattr(time, "monotonic", lambda: now)
    q, dq, expires = (2.99, *HOME[1:]), (0.0,) * 6, 103.0
    task = PlannedReference(
        ((-3.0, 3.0),) * 6,
        lambda _: Feedback(1, now, expires, q, dq),
        lambda: False,
    )
    trajectory = JointTrajectory(
        joint_names=list(JOINTS),
        points=[
            TrajectoryPoint(positions=list(q), time_from_start=0.0),
            TrajectoryPoint(positions=[3.0, *q[1:]], time_from_start=1.0),
        ],
    )
    task.execute(trajectory, {})
    state = CoordinatorState(joints=JointStateSnapshot(), t_now=0.0)
    assert task.compute(state) is not None
    assert task.get_progress(0.5) == task.get_status(0.5).progress == 0.5
    state.t_now = 1.0
    end = task.compute(state)
    assert end is not None and isinstance(end.payload, JointGoal) and end.payload.terminal
    assert end.payload.session.submit(end.payload, lambda _: None, q)
    now += 0.02
    q = ((3.01 if bad == "position" else 3.0), *q[1:])
    if bad == "velocity":
        dq = (float("nan"), *dq[1:])
    if bad == "expired":
        expires = now
    assert task.compute(state) is None
    status = task.get_status(state.t_now)
    assert status.state is (TrajectoryState.FAULT if bad else TrajectoryState.COMPLETED)
    assert task.get_progress(state.t_now) == status.progress


@pytest.mark.parametrize("bad", ["shape", "nan", "limits"])
def test_actor_failure_aborts_the_plan_without_fallback(
    make_rig: Callable[[Actor, str], Rig], bad: str
) -> None:
    def actor(obs: np.ndarray) -> np.ndarray:
        if bad == "shape":
            return np.zeros((6,), dtype=np.float32)
        return np.full((1, 6), np.nan if bad == "nan" else 10.0, dtype=np.float32)

    rig = make_rig(actor, "pose")
    assert rig.execute_viser(plan((0.1, *HOME[1:])))
    result = rig.manipulation.wait_for_execution(timeout=1.0)
    assert result.status is ExecutionStatus.FAULT
    assert not rig.plant.arm_writes
    assert rig.plant.jaw_writes


def test_pinned_model_rejects_other_bytes(tmp_path: Path) -> None:
    pytest.importorskip("onnxruntime")
    candidate = tmp_path / "model.onnx"
    candidate.write_bytes(b"not the pinned model")
    with pytest.raises(ValueError, match="digest mismatch"):
        load_actor(candidate)


def test_tcp_matches_native_browser_scene() -> None:
    scene = os.getenv("DIMOS_TRIMODE_SCENE")
    if not scene:
        pytest.skip("set DIMOS_TRIMODE_SCENE to the local browser MJCF")
    mujoco = pytest.importorskip("mujoco")
    plant = mujoco.MjModel.from_xml_path(scene)
    data = mujoco.MjData(plant)
    tcp = plant.body("arm_tcp").id
    for q in (HOME, (0.4, 0.4, 1.7, -0.5, 0.6, -0.4), (-1.1, 1.4, 0.5, 0.1, -0.7, 0.3)):
        data.qpos[:6] = q
        mujoco.mj_kinematics(plant, data)
        expected = np.r_[data.xpos[tcp], data.xquat[tcp] * np.sign(data.xquat[tcp, 0])]
        np.testing.assert_allclose(tcp_pose(q), expected, rtol=0.0, atol=1e-9)
