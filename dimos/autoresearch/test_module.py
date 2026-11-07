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

"""The lab module over a fake bench: the sim, the arm, the recorder and the
scene are fakes, the candidate programs run in a real subprocess on the fake
backend, and a feeder thread plays the bench's pose and joint streams."""

from collections.abc import Iterator, Mapping
from dataclasses import dataclass, field
import json
from pathlib import Path
import threading
import time
from unittest.mock import call

from dimos_lcm.geometry_msgs import Pose
from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
import numpy as np
import pytest
import pytest_mock

from dimos.autoresearch.module import AutoresearchModule
from dimos.autoresearch.session import Budget, Session
from dimos.autoresearch.status import AutoresearchStatus
from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.collection.episode_monitor import EpisodeCommand
from dimos.manipulation.manipulation_spec import (
    CommandResult,
    CommandStatus,
    ExecutionResult,
    ExecutionStatus,
    ManipulationSnapshot,
    OperationStatus,
    PlanningGroupInfo,
    PlanningGroupState,
    PlanResult,
    PlanStatus,
)
from dimos.manipulation.planning.spec.models import GeneratedPlan, PlanningGroupID
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.robot.manipulators.damiao._damiao_test_helpers import LocalTransport
from dimos.simulation.engines.sim_scene_spec import BodyPlacement
from dimos.utils.testing.waiting import wait_until

FAKE = "dimos.autoresearch._test_helpers:fake_backend"
JOINTS = [f"arm/joint{i}" for i in range(1, 8)]
HOME = [0.0, -1.87, 2.15, -1.85, 0.0, 0.0]
PROGRAM = "def run(robot, task):\n    robot.arm.home()\n"
ENDLESS = "def run(robot, task):\n    while True:\n        robot.arm.joints()\n"


class FakeBench:
    """The bench's state: where the cube is, the arm's joints, and what the lab
    asked of the sim and the arm. The cube rises 10 cm at the after-snapshot
    (the second cancel) of every reset whose number is in ``lift_on``."""

    def __init__(self) -> None:
        self.cube = (0.26, 0.11, 0.02)
        self.joints = [*HOME, 0.0]
        self.resets: list[Mapping[str, BodyPlacement]] = []
        self.homes = 0
        self.grippers: list[float] = []
        self.cancels = 0
        self.cancels_since_reset = 0
        self.lift_on: set[int] = set()

    def reset(self, placements: Mapping[str, BodyPlacement] | None = None) -> bool:
        assert placements is not None
        self.resets.append(dict(placements))
        x, y, z, _ = placements["cube"]
        self.cube = (x, y, z)
        self.cancels_since_reset = 0
        return True

    def lift_if_due(self) -> None:
        self.cancels += 1
        self.cancels_since_reset += 1
        if self.cancels_since_reset == 2 and len(self.resets) in self.lift_on:
            x, y, z = self.cube
            self.cube = (x, y, z + 0.1)


class FakeArm:
    """The planner as the lab's Arm SDK sees it: every plan succeeds and lands at once."""

    def __init__(self, bench: FakeBench) -> None:
        self.bench = bench
        self.target: JointState | None = None

    def list_planning_groups(self) -> tuple[PlanningGroupInfo, ...]:
        return (PlanningGroupInfo("arm", tuple(JOINTS[:6]), "world", "gripper_base", True),)

    def get_state(self) -> ManipulationSnapshot:
        return ManipulationSnapshot(
            timestamp=time.time(),
            operation_status=OperationStatus.IDLE,
            error=None,
            has_pending_plan=False,
            execution_status=ExecutionStatus.IDLE,
            groups={
                "arm": PlanningGroupState(
                    joints=JointState(name=JOINTS[:6], position=self.bench.joints[:6]),
                    end_effector_pose=PoseStamped(frame_id="world"),
                    gripper_position=0.0,
                    joint_presets={"home": JointState(name=JOINTS[:6], position=HOME)},
                )
            },
        )

    def plan_to_joints(
        self, targets: Mapping[PlanningGroupID, JointState], speed_scale: float | None = None
    ) -> PlanResult:
        self.target = targets["arm"]
        return PlanResult(PlanStatus.SUCCEEDED, plan=GeneratedPlan(("arm",), JointTrajectory()))

    def execute(
        self, blocking: bool = True, timeout: float | None = None, *, plan_id: str | None = None
    ) -> ExecutionResult:
        assert self.target is not None
        self.bench.joints[:6] = list(self.target.position)
        self.bench.homes += 1
        return ExecutionResult(ExecutionStatus.COMPLETED)

    def set_gripper_position(
        self, position: float, planning_group: PlanningGroupID | None = None
    ) -> CommandResult:
        self.bench.grippers.append(position)
        return CommandResult(CommandStatus.SUCCEEDED)

    def cancel(self) -> ExecutionResult:
        self.bench.lift_if_due()
        return ExecutionResult(ExecutionStatus.NO_EXECUTION)


class FakeMonitor:
    def __init__(self) -> None:
        self.events: list[EpisodeCommand] = []
        self.saved = 0
        self.ts = 1234.5

    def get_status(self) -> EpisodeStatus:
        return EpisodeStatus(
            ts=self.ts, state="idle", episodes_saved=self.saved, episodes_discarded=0
        )

    def command(self, event: EpisodeCommand) -> EpisodeStatus:
        self.events.append(event)
        self.saved += event == "save"
        return self.get_status()


class FakeScene:
    def scan_scene(self, text: list[str] | None = None) -> Detection3DArray:
        detection = Detection3D()
        detection.header = Header(1.0, "world")
        detection.id = "cube"
        detection.results = [ObjectHypothesisWithPose(hypothesis=ObjectHypothesis("cube", 1.0))]
        detection.results_length = 1
        detection.bbox = BoundingBox3D(
            center=Pose(position=Vector3(0.2, 0.1, 0.02), orientation=Quaternion()),
            size=Vector3(0.04, 0.04, 0.04),
        )
        return Detection3DArray(
            detections_length=1, header=Header(1.0, "world"), detections=[detection]
        )


@dataclass
class Lab:
    module: AutoresearchModule
    bench: FakeBench
    monitor: FakeMonitor
    dir: Path
    statuses: list[AutoresearchStatus] = field(default_factory=list)

    @property
    def session(self) -> Session:
        return Session(self.dir)

    def done(self) -> bool:
        return not self.module.get_status().busy


@pytest.fixture
def lab(tmp_path: Path, mocker: pytest_mock.MockerFixture) -> Iterator[Lab]:
    bench, monitor = FakeBench(), FakeMonitor()
    module = AutoresearchModule(
        session_dir=tmp_path / "lift",
        task="lift the cube",
        budget=Budget(
            iterations=1, candidates_per_round=2, dev_trials=2, heldout_trials=1, threshold=0.8
        ),
        program_timeout_s=60.0,
        backend_factory=FAKE,
    )
    module._sim = bench
    module._arm = FakeArm(bench)
    module._episodes = monitor
    module._scene = FakeScene()
    mocker.patch.object(module, "start_tool")
    mocker.patch.object(module, "stop_tool")
    poses: LocalTransport[TFMessage] = LocalTransport()
    joints: LocalTransport[JointState] = LocalTransport()
    frames: LocalTransport[Image] = LocalTransport()
    module.body_poses.transport = poses
    module.coordinator_joint_state.transport = joints
    module.color_image.transport = frames
    lab = Lab(module, bench, monitor, tmp_path / "lift")
    module.autoresearch_status.subscribe(
        lambda msg: lab.statuses.append(AutoresearchStatus.model_validate_json(msg.data))
    )
    stop = threading.Event()

    def feed() -> None:
        while not stop.is_set():
            x, y, z = bench.cube
            poses.publish(
                TFMessage(
                    Transform(Vector3(x, y, z), Quaternion(), "world", "cube"),
                    Transform(Vector3(0.36, 0.0, 0.0), Quaternion(), "world", "bowl"),
                )
            )
            joints.publish(JointState(name=JOINTS, position=list(bench.joints)))
            frames.publish(Image(data=np.zeros((4, 4, 3), np.uint8), format=ImageFormat.RGB))
            time.sleep(0.02)

    feeder = threading.Thread(target=feed, daemon=True)
    feeder.start()
    module.start()
    yield lab
    module.stop()
    stop.set()
    feeder.join()


def test_start_homes_the_arm_and_opens_the_session(lab: Lab) -> None:
    assert lab.session.info().status == "idle" and lab.session.info().task == "lift the cube"
    assert (lab.bench.homes, lab.bench.grippers) == (1, [1.0])
    assert [status.state for status in lab.statuses] == ["idle"]
    task = json.loads(lab.module.task())
    assert task["budget"]["candidates_left"] == 2 and "robot.arm.move_pose" in task["api"]
    scene = json.loads(lab.module.scene())
    assert scene["objects"] == [{"name": "cube", "position": [0.2, 0.1, 0.02], "size": [0.04] * 3}]
    assert Path(scene["keyframe"]).is_file()


def test_run_returns_at_once_then_records_every_dev_trial_and_releases_the_tool(lab: Lab) -> None:
    lab.bench.lift_on = {2}
    candidate = lab.module.submit(PROGRAM, "home is enough")
    started = time.monotonic()
    assert lab.module.run(candidate).startswith("started candidate 1 on 2 dev trials")
    assert time.monotonic() - started < 1.0
    assert lab.module.start_tool.call_args_list == [call("run")]  # type: ignore[attr-defined]
    assert (
        lab.module.run(candidate)
        == "a run is in progress; wait for its results or call stop_run first"
    )
    assert lab.session.info().status == "running"
    wait_until(lab.done, timeout=60.0)

    records = lab.session.results(candidate)
    assert [(r.split, r.index, r.success, r.stage) for r in records] == [
        ("dev", 0, False, "reach"),
        ("dev", 1, True, "done"),
    ]
    assert (records[1].episode, records[1].start_ts) == (0, lab.monitor.ts)
    assert lab.monitor.events == ["start", "discard", "start", "save"]
    assert lab.module.stop_tool.call_args_list == [call("run")]  # type: ignore[attr-defined]
    assert lab.session.info().status == "idle"
    assert [(s.state, s.busy, s.trial) for s in lab.statuses[1:]] == [
        ("running", True, 0),
        ("running", True, 1),
        ("idle", False, None),
    ]
    assert (lab.statuses[-1].dev_passed, lab.statuses[-1].dev_trials) == (1, 2)
    # Each dev trial starts from its own seeded placement, after homing the arm.
    assert [placement["cube"][:2] for placement in lab.bench.resets] == [
        lab.module.config.placement_bounds.sample(0)[:2],
        lab.module.config.placement_bounds.sample(1)[:2],
    ]
    assert lab.bench.homes == 3
    results = json.loads(lab.module.results(candidate))
    assert results["dev"] == {"dev_passed": 1, "dev_trials": 2} and not results["running"]
    assert Path(results["trials"][1]["trace"]).is_file()
    assert json.loads(lab.module.leaderboard())[0]["dev_rate"] == 0.5
    assert lab.module.run(candidate) == "candidate 1 has already run every dev trial"
    with pytest.raises(RuntimeError, match="freeze"):
        lab.module.run(candidate, "heldout")


@pytest.mark.parametrize("stopper", ["stop_run", "stop"])
def test_stopping_kills_the_program_cancels_the_arm_and_discards_the_trial(
    lab: Lab, stopper: str
) -> None:
    candidate = lab.module.submit(ENDLESS, "never ends")
    lab.module.run(candidate)
    trace = lab.session.candidate_dir(candidate) / "trials" / "dev-00" / "trace.jsonl"
    wait_until(lambda: trace.is_file() and trace.read_text() != "", timeout=60.0)
    cancels = lab.bench.cancels

    getattr(lab.module, stopper)()

    assert lab.module._thread is None
    assert lab.bench.cancels > cancels
    assert lab.session.results(candidate) == []
    assert lab.monitor.events == ["start", "discard"]
    assert lab.session.info().status == "idle"
    assert lab.module.stop_tool.call_args_list[0] == call("run")  # type: ignore[attr-defined]
    assert not lab.module.get_status().busy and lab.module.get_status().error is None


def test_freeze_runs_the_heldout_trials_and_scores_the_session(lab: Lab) -> None:
    lab.bench.lift_on = {1}
    candidate = lab.module.submit(PROGRAM, "home is enough")
    assert lab.module.freeze(candidate).startswith("frozen candidate 1; running 1 held-out trials")
    wait_until(lambda: lab.session.info().status == "scored", timeout=60.0)

    info = lab.session.info()
    assert (info.status, info.heldout_rate, lab.session.frozen()) == ("scored", 1.0, 1)
    [record] = lab.session.results(candidate)
    assert (record.split, record.index, record.success) == ("heldout", 0, True)
    wait_until(lab.done, timeout=5.0)
    assert (lab.statuses[-1].state, lab.statuses[-1].heldout_rate) == ("scored", 1.0)
    assert lab.module.stop_tool.call_args_list == [call("run")]  # type: ignore[attr-defined]
    with pytest.raises(RuntimeError, match="scored"):
        lab.module.submit(PROGRAM, "too late")
