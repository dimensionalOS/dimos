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

from collections.abc import Sequence
from pathlib import Path

import pytest

from dimos.autoresearch.mujoco.judge import BodyPose, LiftJudge, Snapshot
from dimos.autoresearch.mujoco.trial import HELDOUT_SEED_BASE, ArmEvaluator, NoEpisodes
from dimos.autoresearch.session import Trial
from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.collection.episode_monitor import EpisodeCommand
from dimos.manipulation.code_policy._test_helpers import FAKE_BACKEND

PROGRAM = """
def run(robot, task):
    cube = robot.scene.objects(["cube"])[0]
    robot.arm.move_pose([cube.position[0], cube.position[1], 0.1])
    robot.gripper.close()
    robot.arm.move_linear(dz=0.1)
"""


def snapshot(cube_z: float) -> Snapshot:
    cube = BodyPose(position=(0.4, 0.0, cube_z), orientation=(0.0, 0.0, 0.0, 1.0))
    return Snapshot(bodies={"cube": cube})


class FakeWorld:
    """``reset`` records the seed, ``snapshot`` plays back the given snapshots in order."""

    def __init__(self, snapshots: Sequence[Snapshot]) -> None:
        self._snapshots = iter(snapshots)
        self.seeds: list[int] = []

    def reset(self, seed: int) -> None:
        self.seeds.append(seed)

    def snapshot(self) -> Snapshot:
        return next(self._snapshots)


class FakeMonitor:
    """The recorder as the trial sees it: the commands it got and a saved count."""

    def __init__(self) -> None:
        self.events: list[EpisodeCommand] = []
        self.saved = 0

    def get_status(self) -> EpisodeStatus:
        return self._status()

    def command(self, event: EpisodeCommand) -> EpisodeStatus:
        self.events.append(event)
        self.saved += event == "save"
        return self._status()

    def _status(self) -> EpisodeStatus:
        return EpisodeStatus(ts=0.0, state="idle", episodes_saved=self.saved, episodes_discarded=0)


@pytest.fixture
def trial(tmp_path: Path) -> Trial:
    program = tmp_path / "program.py"
    program.write_text(PROGRAM)
    return Trial(
        artifact=program, task="lift the cube", directory=tmp_path / "dev-03", split="dev", index=3
    )


def test_success_saves_the_episode_and_reports_the_trace(trial: Trial) -> None:
    world = FakeWorld([snapshot(0.02), snapshot(0.12)])
    monitor = FakeMonitor()
    monitor.saved = 1

    result = ArmEvaluator(world, LiftJudge("cube"), monitor, FAKE_BACKEND, 60.0)(trial)

    assert result.score == 1
    assert result.note == "cube rose 0.100 m"
    assert result.evidence["episode"] == 1
    assert result.evidence["trace"] == str(trial.directory / "trace.jsonl")
    assert (trial.directory / "trace.jsonl").is_file()
    assert monitor.events == ["start", "save"]
    assert world.seeds == [3]


def test_without_a_recorder_no_episode_is_saved(trial: Trial) -> None:
    world = FakeWorld([snapshot(0.02), snapshot(0.12)])
    result = ArmEvaluator(world, LiftJudge("cube"), NoEpisodes(), FAKE_BACKEND, 60.0)(trial)
    assert result.score == 1 and result.evidence["episode"] is None


def test_failure_discards_the_episode_and_keeps_the_program_error(trial: Trial) -> None:
    trial.artifact.write_text("def run(robot, task):\n    raise ValueError('no cube')\n")
    world = FakeWorld([snapshot(0.02), snapshot(0.02)])
    monitor = FakeMonitor()
    result = ArmEvaluator(world, LiftJudge("cube"), monitor, FAKE_BACKEND, 60.0)(
        trial.model_copy(update={"split": "heldout", "index": 0})
    )
    assert (result.score, result.evidence["stage"], result.evidence["status"]) == (
        0,
        "reach",
        "raised",
    )
    assert result.note == "cube was not touched; program raised: ValueError: no cube"
    assert result.evidence["episode"] is None
    assert monitor.events == ["start", "discard"]
    assert world.seeds == [HELDOUT_SEED_BASE]
