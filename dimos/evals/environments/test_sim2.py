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

from contextlib import ExitStack
import json
from pathlib import Path
import time

import pytest

from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.evals.environments.sim2 import Intervention, Sim2Environment, scene_evidence
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.sim2.world.scene_types import (
    EntityState,
    SceneControlSpec,
    SceneDescription,
    SceneState,
    SceneUpdate,
)


def sample(t=0.0, *, generation=0, episode=0):
    return SceneState(
        world_id="world",
        scene_id="room",
        episode=episode,
        generation=generation,
        tick=round(t * 200),
        sim_time=t,
        ts=1000 + t,
        entities={
            "box": EntityState(
                pose=Pose(1, 2, 3),
                velocity=(1, 0, 0),
                angular_velocity=(0, 0, 1),
                bounds_min=(0, 1, 2),
                bounds_max=(2, 3, 4),
            )
        },
        robots={"g1": Pose(0, 0, 0.8)},
        joints={"door": 0.4},
        regions={},
        contacts=(("g1/foot", "floor"),),
    )


@pytest.fixture
def recorder(tmp_path, mocker):
    env = Sim2Environment(blueprint=["g1"], scene=Path("room.xml"))
    scene = mocker.create_autospec(SceneControlSpec, instance=True)
    mocker.patch.object(env, "scene", scene)
    with SqliteStore(path=str(tmp_path / "truth.db")) as store:
        mocker.patch.object(env, "_truth", store)
        yield env, scene, store
        env.stop()


def test_full_state_and_exact_intervention_roundtrip(recorder, tmp_path):
    env, scene, store = recorder
    before, after, final = sample(), sample(generation=1), sample(0.1, generation=1)
    scene.scene_state.side_effect = [before, final]
    scene.set_scene_state.return_value = after
    update = SceneUpdate(poses={"box": Pose(2, 3, 4)})
    assert env.intervene(update, reason="move key") == after
    env._stop_recording()
    with SqliteStore(path=str(tmp_path / "truth.db"), must_exist=True) as reopened:
        states, edits = scene_evidence(reopened)
        assert states == [before, after, final]
        assert edits == [
            Intervention(reason="move key", update=update, before=before, after=after, ts=after.ts)
        ]
        assert states[-1].joints == {"door": 0.4}
        assert states[-1].contacts == (("g1/foot", "floor"),)
        assert states[-1].entities["box"].velocity == (1, 0, 0)
    scene.set_scene_state.assert_called_once_with(update)


@pytest.mark.parametrize("change", [{"episode": 1}, {"generation": 1}, {"tick": -1}])
def test_unrecorded_changes_fail_without_completion_marker(recorder, change):
    env, scene, store = recorder
    scene.scene_state.side_effect = [sample(), sample().model_copy(update=change)]
    env._sample()
    with pytest.raises(RuntimeError, match="unrecorded"):
        env._stop_recording()
    assert "completed" not in store.streams


@pytest.mark.parametrize(
    "states,complete,error",
    [
        ([sample(), sample(0.1)], False, "incomplete"),
        ([sample(), sample(0.1, episode=1)], True, "mixed"),
        ([sample(), sample(0.1, generation=1)], True, "unrecorded"),
        ([sample(), sample(0.6)], True, "time gap"),
    ],
)
def test_grader_rejects_incomplete_or_invalid_evidence(tmp_path, states, complete, error):
    with SqliteStore(path=str(tmp_path / "truth.db")) as store:
        for state in states:
            store.stream("scene_state", SceneState).append(state)
        if complete:
            store.stream("completed", SceneState).append(states[-1])
        with pytest.raises(ValueError, match=error):
            scene_evidence(store)


def test_launch_is_fresh_and_truth_is_private():
    env = Sim2Environment(blueprint=["g1"], scene=Path("room.xml"))
    proc = DimosCliCall()
    env.configure_launch(proc)
    assert proc.simulator == "mujoco"
    assert "--scene-package" in proc.global_args
    assert env.config.agent_artifacts == ("recording",)
    assert "sim_truth" not in env.config.recorded_topics
    for kwargs in (
        {"attach": True},
        {"agent_artifacts": ("truth",)},
        {"agent_artifacts": ("setup",)},
        {"recorded_topics": ("sim_truth",)},
    ):
        with pytest.raises(ValueError):
            Sim2Environment(blueprint=["g1"], scene=Path("room.xml"), **kwargs)


def test_recorder_failure_propagates_and_resources_close(recorder, mocker, tmp_path):
    env, scene, _ = recorder
    scene.scene_state.side_effect = RuntimeError("physics died")
    mocker.patch.object(env, "_done", mocker.Mock())
    env._done.wait.return_value = False
    env._record_loop()
    with pytest.raises(RuntimeError, match="recording failed"):
        env._stop_recording()


def test_setup_and_first_state_are_retained_outside_agent_artifacts(tmp_path, mocker):
    scene_path = tmp_path / "scene.xml"
    scene_path.write_text("<mujoco/>")
    env = Sim2Environment(blueprint=["g1"], scene=scene_path, ready_streams=())
    app = mocker.patch("dimos.evals.environments.sim2.Dimos.connect").return_value
    scene = mocker.create_autospec(SceneControlSpec, instance=True)
    scene.describe_scene.return_value = SceneDescription(id="room")
    scene.scene_state.return_value = sample()
    app.find_module_by_spec.return_value = scene
    app.get_module.return_value.describe_run.return_value = {"world_id": "world"}
    with ExitStack() as resources:
        resources.callback(env.stop)
        recording = resources.enter_context(SqliteStore(path=str(tmp_path / "memory.db")))
        env.setup_scene()
        artifacts = env.prepare_recording(recording, tmp_path / "memory.db", time.monotonic() + 1)
        setup = json.loads(artifacts["setup"].read_text())
        assert setup["first_state"] == sample().model_dump(mode="json")
        assert setup["runtime"] == {"world_id": "world"}
        assert len(setup["scene"]["sha256"]) == 64
        assert set(artifacts).isdisjoint(env.config.agent_artifacts)
