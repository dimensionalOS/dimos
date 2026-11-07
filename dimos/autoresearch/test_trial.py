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

from pathlib import Path

import pytest

from dimos.autoresearch.episodes import NoEpisodes
from dimos.autoresearch.judge import BodyPose, LiftJudge, Snapshot
from dimos.autoresearch.session import Budget, Session
from dimos.autoresearch.trial import HELDOUT_SEED_BASE, FakeWorld, run_trial
from dimos.imitation.collection.episode import EpisodeEvent, EpisodeStatus
from dimos.imitation.collection.episode_monitor import EpisodeCommand

FAKE = "dimos.autoresearch.program:fake_backend"

PROGRAM = """
def run(robot, task):
    cube = robot.scene.objects(["cube"])[0]
    robot.arm.move_pose([cube.position[0], cube.position[1], 0.1])
    robot.gripper.close()
    robot.arm.move_linear(dz=0.1)
"""


def snapshot(cube_z: float) -> Snapshot:
    cube = BodyPose(position=(0.4, 0.0, cube_z), orientation=(0.0, 0.0, 0.0, 1.0))
    return Snapshot(bodies={"cube": cube}, gripper=0.0)


class FakeMonitor:
    """Counts episodes the way EpisodeMonitorModule does, without a bus."""

    def __init__(self) -> None:
        self.events: list[EpisodeCommand] = []
        self._recording = False
        self._saved = 0
        self._discarded = 0

    def get_status(self) -> EpisodeStatus:
        return self._status("init")

    def command(self, event: EpisodeCommand) -> EpisodeStatus:
        self.events.append(event)
        if event == "start":
            self._recording = True
        elif event == "save":
            self._saved += self._recording
            self._recording = False
        elif event == "discard":
            self._discarded += self._recording
            self._recording = False
        return self._status(event)

    def _status(self, event: EpisodeEvent) -> EpisodeStatus:
        return EpisodeStatus(
            ts=0.0,
            state="recording" if self._recording else "idle",
            episodes_saved=self._saved,
            episodes_discarded=self._discarded,
            last_event=event,
        )


@pytest.fixture
def session(tmp_path: Path) -> Session:
    budget = Budget(
        iterations=1, candidates_per_round=1, dev_trials=2, heldout_trials=1, threshold=0.8
    )
    return Session.create(tmp_path / "lift", task="lift the cube", budget=budget, model="test")


def test_success_saves_the_episode_and_records_the_trial(session: Session) -> None:
    candidate = session.submit(PROGRAM, "straight down, close, straight up")
    world = FakeWorld([snapshot(0.02), snapshot(0.12)])
    monitor = FakeMonitor()
    monitor.command("start")
    monitor.command("save")

    record = run_trial(session, candidate, "dev", 3, world, LiftJudge("cube"), monitor, FAKE, 60.0)

    assert (record.success, record.stage, record.status) == (True, "done", "completed")
    assert record.note == "cube rose 0.100 m"
    assert record.episode == 1
    assert monitor.events[-2:] == ["start", "save"]
    assert world.seeds == [3]
    assert record.trace == session.candidate_dir(candidate) / "trials/dev-03/trace.jsonl"
    assert record.trace.is_file()
    assert Session(session.dir).results(candidate) == [record]

    unrecorded = FakeWorld([snapshot(0.02), snapshot(0.12)])
    record = run_trial(
        session, candidate, "dev", 4, unrecorded, LiftJudge("cube"), NoEpisodes(), FAKE, 60.0
    )
    assert record.success and record.episode is None


def test_failure_discards_the_episode_and_keeps_the_program_error(session: Session) -> None:
    candidate = session.submit(
        "def run(robot, task):\n    raise ValueError('no cube')\n", "gives up"
    )
    world = FakeWorld([snapshot(0.02), snapshot(0.02)])
    monitor = FakeMonitor()

    record = run_trial(
        session, candidate, "heldout", 0, world, LiftJudge("cube"), monitor, FAKE, 60.0
    )

    assert (record.success, record.stage, record.status) == (False, "reach", "raised")
    assert record.note == "cube was not touched; program raised: ValueError: no cube"
    assert record.episode is None
    assert monitor.events == ["start", "discard"]
    assert world.seeds == [HELDOUT_SEED_BASE]
    assert record.trace.read_text() == ""
