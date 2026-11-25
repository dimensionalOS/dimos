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

"""The two real reference consumers against a recording, no-hardware connection."""

from collections.abc import Callable, Iterator
import os
from pathlib import Path
import threading
import time
from typing import Any
from unittest.mock import Mock

import numpy as np
import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.test_coordinator_connections import LocalTransport
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import ExecutionStatus, OperationStatus
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.damiao.arm import ArmReading
from dimos.robot.manipulators.openyam.blueprints.teleop import openyam_trimode
from dimos.robot.manipulators.openyam.joints import OPENYAM_ARM_JOINTS
from dimos.robot.manipulators.openyam.trimode_connection import (
    TrimodeArm,
    TrimodeConnection,
    declaration,
)
from dimos.robot.manipulators.openyam.trimode_coordinator import TrimodeCoordinator
from dimos.robot.manipulators.openyam.trimode_quest import TrimodeQuest
from dimos.robot.manipulators.openyam.trimode_tasks import (
    JOINTS,
    POSE,
    QUEST_FRESH,
    QUEST_TIMEOUT,
    PlannedReference,
    QuestReference,
)
from dimos.robot.manipulators.openyam.trimode_tracker import (
    REFERENCE_TIMEOUT,
    Actor,
    ArmSample,
    JointGoal,
    Vector,
    load_actor,
    tcp_pose,
)
from dimos.teleop.webxr.controller_types import Buttons, Hand, WebXRControllerState

HOME = (0.0, 1.047, 1.047, 0.0, 0.0, 0.0)


class Plant(TrimodeArm):
    def __init__(self) -> None:
        self.q = HOME
        self.dq: Vector = (0.0,) * 6
        self.jaw = 0.04
        self.arm_writes: list[Vector] = []
        self.jaw_writes: list[float] = []
        self.include_velocity = True
        self.holds = 0
        self._active = False

    def connect(self) -> None:
        pass

    def enable(self) -> None:
        self._active = True

    def disable(self) -> None:
        self._active = False

    def read(self) -> ArmReading:
        return ArmReading(
            position=[*self.q, self.jaw],
            velocity=[*self.dq, 0.0] if self.include_velocity else [],
            effort=[0.0] * 7,
        )

    def submit_arm(self, q_des: Vector) -> None:
        assert self.active
        self.arm_writes.append(q_des)

    def hold_arm(self, q: Vector) -> None:
        self.holds += 1

    def command(self, values: dict[str, float]) -> None:
        self.jaw = values["joint7"]
        self.jaw_writes.append(self.jaw)

    def hold(self) -> None:
        pass


class TestConnection(TrimodeConnection):
    __test__ = False

    def _make_arm(self) -> Plant:
        return Plant()

    @rpc
    def evidence(self) -> tuple[int, list[Vector], list[float], int]:
        with self._hardware_lock:
            plant = self._arm
            assert isinstance(plant, Plant)
            return os.getpid(), list(plant.arm_writes), list(plant.jaw_writes), plant.holds

    @rpc
    def move_measurement(self, q: Vector, dq: Vector) -> None:
        with self._hardware_lock:
            plant = self._arm
            assert isinstance(plant, Plant)
            plant.q, plant.dq = q, dq


def test_physical_blueprint_is_static() -> None:
    blueprint = openyam_trimode.solidify()
    atoms = {atom.module: atom for atom in blueprint.active_blueprints}
    assert set(atoms) == {TrimodeQuest, TrimodeCoordinator, TrimodeConnection, ManipulationModule}
    assert set(atoms[TrimodeCoordinator].kwargs["connections"][0].joint_commands) == {"joint7"}
    assert TrimodeConnection.dedicated_worker
    assert (
        blueprint.remapping_map[(TrimodeConnection.name, "joint_reference")]
        == "arm_joint_reference"
    )
    parsed = BlueprintConfigParser(blueprint).parse(
        [
            "--can-port",
            "follower_r",
            "--model-path",
            "/models/actor.onnx",
            "--quest-mode",
            "position",
        ],
        environ={},
    )
    config = parsed.module_kwargs("trimodeconnection")
    assert config["can_port"] == "follower_r" and config["source"] == "arm"
    assert config["model_path"] == Path("/models/actor.onnx")
    assert config["group_commands"].keys() == {"tcp_pose", "tcp_position", "joint_reference"}
    assert parsed.module_kwargs("ControlCoordinator")["quest_mode"] == "position"


def test_stock_sdk_calls_split_jaw_resend_from_native_mit() -> None:
    # Actual adapter with recording SDK boundary; never connect/calibrate.
    arm = TrimodeArm(TrimodeConnection.robot, ["fake"])
    native, motors, jaw = Mock(), Mock(), Mock()
    arm._native = native
    arm._arms["arm"] = motors
    arm._grippers["gripper"] = jaw
    arm._travel["joint7"] = (0.06, 0.01)
    arm._active = True
    arm.command({"joint7": 0.035})
    arm.command({"joint7": 0.035})
    assert jaw.set_opening.call_count == native.tick.call_count == 2
    assert jaw.set_opening.call_args.args[0] == pytest.approx(0.5)
    motors.mit_control.assert_not_called()
    arm.submit_arm(HOME)
    motors.mit_control.assert_called_once()
    rows = motors.mit_control.call_args.args[0]
    np.testing.assert_allclose(rows[:, 0], [50, 60, 50, 25, 15, 10])
    np.testing.assert_allclose(rows[:, 1], [3, 4, 3, 2, 1, 0.8])
    np.testing.assert_allclose(rows[:, 2], HOME)
    np.testing.assert_array_equal(rows[:, 3:], 0)
    assert jaw.set_opening.call_count == 2
    arm._physical = Mock()
    arm._physical.gravity.return_value = np.asarray([0.1] * 6)
    arm.hold_arm(HOME)
    hold = motors.mit_control.call_args.args[0]
    (bus,) = TrimodeConnection.robot.buses
    np.testing.assert_allclose(hold[:, 0], bus.kp)
    np.testing.assert_allclose(hold[:, 1], bus.kd)
    np.testing.assert_allclose(hold[:, 4], 0.1)
    assert jaw.set_opening.call_count == 2


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
        contract = declaration(robot)
        self.connection = TestConnection(can_port="fake", model_path=path, **contract.model_dump())
        self.control = TrimodeCoordinator(
            instance_name="ControlCoordinator",
            quest_mode=mode,
            connections=[contract],
            publish_joint_state=False,
        )
        self.control._lifecycle = self.connection
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
        for ctrl, plant in (
            ("arm_position_command", "position_command"),
            ("arm_joint_state", "joint_state"),
            ("arm_sample", "arm_sample"),
            ("arm_joint_reference", "joint_reference"),
            ("arm_tcp_pose", "tcp_pose"),
            ("arm_tcp_position", "tcp_position"),
        ):
            wire = LocalTransport()
            self.control.set_transport(ctrl, wire)  # type: ignore[arg-type]
            self.connection.set_transport(plant, wire)  # type: ignore[arg-type]
        self.control.start()
        self.connection.start()
        assert isinstance(self.connection._arm, Plant)
        self.plant = self.connection._arm
        self.manipulation = ManipulationModule(
            model=RobotModelConfig(model=robot, joint_names=list(JOINTS), base_link="base")
        )
        self.manipulation._control_coordinator = self.control
        self.manipulation._initialize_execution()
        self.manipulation._started = True
        wait(lambda: self.control._read_sample())

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
        self.connection.stop()


@pytest.fixture
def make_rig(monkeypatch: pytest.MonkeyPatch) -> Iterator[Callable[[Actor, str], Rig]]:
    rigs: list[Rig] = []

    def make(actor: Actor, mode: str = "pose") -> Rig:
        monkeypatch.setattr(
            "dimos.robot.manipulators.openyam.trimode_connection.load_actor", lambda _: actor
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


@pytest.mark.parametrize("event", ["cancel", "takeover", "estop", "expired", "reset"])
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
            with rig.connection._hardware_lock:
                rig.connection._deadline = time.monotonic() - 1
            wait(
                lambda: rig.control.task_invoke("joint_trajectory", "get_state")
                is TrajectoryState.FAULT
            )
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
    task = QuestReference(POSE, lambda: None, lambda: takeovers.append(True), Mock())
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
    task = QuestReference(POSE, lambda: None, lambda: takeovers.append(True), Mock())
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


def test_delayed_and_expired_references_cannot_restart_arm(
    make_rig: Callable[[Actor, str], Rig],
) -> None:
    rig = make_rig(lambda _: np.asarray([HOME], dtype=np.float32), "pose")
    worker = rig.connection
    port = "joint_reference"
    session = worker.open_reference(port, time.monotonic() + REFERENCE_TIMEOUT)
    assert session is not None
    before = time.monotonic() - 0.20
    worker._offer(port, JointGoal(session, before, before + REFERENCE_TIMEOUT, HOME))
    assert not rig.plant.arm_writes
    now = time.monotonic()
    worker._offer(port, JointGoal(session, now, now + REFERENCE_TIMEOUT, HOME))
    wait(lambda: len(rig.plant.arm_writes) == 1)
    jaws = len(rig.plant.jaw_writes)
    wait(lambda: rig.plant.holds == 1)
    now = time.monotonic()
    worker._offer(port, JointGoal(session, now, now + REFERENCE_TIMEOUT, HOME))
    wait(lambda: len(rig.plant.jaw_writes) > jaws + 3)
    assert len(rig.plant.arm_writes) == 1


@pytest.mark.parametrize("bad", ["position", "velocity", "expired", None])
def test_planned_settle_validates_measurement_and_reports_progress(
    monkeypatch: pytest.MonkeyPatch, bad: str | None
) -> None:
    now = 100.0
    monkeypatch.setattr(time, "monotonic", lambda: now)
    q, dq, at, terminal = (2.99, *HOME[1:]), (0.0,) * 6, now, None
    lifecycle = Mock()
    lifecycle.open_reference.return_value = "token"
    task = PlannedReference(
        ((-3.0, 3.0),) * 6,
        lambda: ArmSample(at, q, dq, "token", terminal),
        lambda: False,
        lifecycle,
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
    terminal = now
    now += 0.02
    at = now
    q = ((3.01 if bad == "position" else 3.0), *q[1:])
    if bad == "velocity":
        dq = (float("nan"), *dq[1:])
    if bad == "expired":
        at = now - 1
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


class TestQuest(TrimodeQuest):
    __test__ = False

    @rpc
    def start(self) -> None:
        Module.start(self)  # no HTTP/WebXR service in this acceptance test

    @rpc
    def right(self, grip: float, x: float = 0.0) -> None:
        # Real producer, from decoded native right-hand messages to typed output.
        self._on_pose(PoseStamped(frame_id="right", position=(x, 0.0, 0.0)))
        self._on_joy(Joy(frame_id="right", axes=[0, 0, 0.75, grip], buttons=[0] * 6))
        self._publish_button_state(None, self._controllers[Hand.RIGHT])
        self._publish_msg(Hand.RIGHT, PoseStamped(position=(x, 0.0, 0.0)))


class TestCoordinator(TrimodeCoordinator):
    __test__ = False

    @rpc
    def pid(self) -> int:
        return os.getpid()


@pytest.mark.parametrize("mode", ["pose", "position"])
def test_cross_worker_pinned_policy_and_actual_viser_handler(mode: str) -> None:
    artifact = os.getenv("DIMOS_TRIMODE_ONNX")
    if not artifact:
        pytest.skip("set DIMOS_TRIMODE_ONNX to the pinned local actor")
    robot = model()
    contract = declaration(robot)
    blueprint = (
        autoconnect(
            TestQuest.blueprint(),
            TestCoordinator.blueprint(
                instance_name="ControlCoordinator",
                connections=[contract],
                quest_mode=mode,
            ),
            TestConnection.blueprint(can_port="fake", model_path=Path(artifact), **dict(contract)),
        )
        .remappings(
            [
                (TestQuest, "right_controller_output", "right_cartesian_command"),
                (TestCoordinator, "arm_joint_state", "joint_state"),
                (TestConnection, "position_command", "arm_position_command"),
                (TestConnection, "joint_reference", "arm_joint_reference"),
                (TestConnection, "tcp_pose", "arm_tcp_pose"),
                (TestConnection, "tcp_position", "arm_tcp_position"),
            ]
        )
        .solidify()
    )
    config = BlueprintConfigParser(blueprint).parse(
        environ={},
        global_overrides={
            "viewer": "none",
            "build_native": False,
            "zenoh_scout_addr": os.environ["ZENOH_SCOUT_ADDR"],
            "serve_coordinator_rpc": False,
        },
    )
    system = ModuleCoordinator.build(blueprint, config)
    manipulation = ManipulationModule(
        model=RobotModelConfig(model=robot, joint_names=list(JOINTS), base_link="base")
    )
    try:
        control, connection, quest = (
            system.get_instance(TestCoordinator),
            system.get_instance(TestConnection),
            system.get_instance(TestQuest),
        )
        manipulation._control_coordinator = control  # type: ignore[assignment]
        manipulation._initialize_execution()
        manipulation._started = True
        wait(lambda: connection.evidence()[2])
        plant_pid, actions, _, _ = connection.evidence()
        assert control.pid() != plant_pid and not actions
        quest.right(0.0)
        time.sleep(0.025)
        quest.right(1.0, 0.004)
        wait(lambda: connection.evidence()[1])
        quest.right(0.0)
        wait(lambda: connection.evidence()[3] == 1)
        count = len(connection.evidence()[1])
        end = (0.04, *HOME[1:])
        generated = plan(end, 0.20)
        manipulation._pending_plan = generated
        manipulation._state = OperationStatus.HAS_PLAN
        assert manipulation._execute_generated_plan(generated)
        wait(lambda: len(connection.evidence()[1]) > count + 4)
        connection.move_measurement(end, (1.0,) * 6)
        time.sleep(0.2)
        assert control.task_invoke("joint_trajectory", "get_state") is TrajectoryState.EXECUTING
        connection.move_measurement(end, (0.0,) * 6)
        assert manipulation.wait_for_execution(timeout=1.0).status == ExecutionStatus.COMPLETED
        _, actions, jaws, holds = connection.evidence()
        assert holds == 2 and jaws
        time.sleep(REFERENCE_TIMEOUT * 1.2)
        assert connection.evidence()[1] == actions
    finally:
        try:
            manipulation.stop()
        finally:
            system.stop()
