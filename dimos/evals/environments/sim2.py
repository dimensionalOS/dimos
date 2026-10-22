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

"""Fresh sim2 applications with private scene evidence for the existing eval runner."""

from __future__ import annotations

from itertools import pairwise
import json
from pathlib import Path
from threading import Event, RLock, Thread
import time
from typing import Any, cast

from pydantic import Field, model_validator
from typing_extensions import Self

from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.evals.environments.sim import Sim, SimConfig
from dimos.memory.store.base import Store
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.porcelain.dimos import Dimos
from dimos.sim2.module import SimulationModule
from dimos.sim2.world.evidence import file_identity
from dimos.sim2.world.scene_types import SceneControlSpec, SceneRecord, SceneState, SceneUpdate


class Intervention(SceneRecord):
    reason: str
    update: SceneUpdate
    before: SceneState
    after: SceneState
    ts: float


class Sim2EnvironmentConfig(SimConfig):
    scene: Path
    initial: SceneUpdate = Field(default_factory=SceneUpdate)
    truth_hz: float = Field(default=10.0, gt=0, le=100)
    ready_streams: tuple[str, ...] = ("color_image", "odom", "global_costmap")
    recorded_topics: tuple[str, ...] = (
        "color_image",
        "depth_image",
        "camera_info",
        "tf",
        "odom",
        "lidar",
        "pointcloud",
        "cmd_vel",
        "path",
        "global_costmap",
    )
    module_env: dict[str, str] = Field(default_factory=dict)

    @model_validator(mode="after")
    def require_fresh_application(self) -> Self:
        if self.attach:
            raise ValueError("sim2 evals launch a fresh stack; physics reset does not clear maps")
        if {"truth", "setup"} & set(self.agent_artifacts) or "sim_truth" in self.recorded_topics:
            raise ValueError("scene truth must not be included in agent artifacts")
        return self


class Sim2Environment(Sim):
    """One case owns one application. Stop/start resets physics, policies, maps and memory.

    Private artifacts are not a security boundary: a shell with filesystem or
    simulator RPC access still requires an external sandbox.
    """

    config: Sim2EnvironmentConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.scene: SceneControlSpec | None = None
        self._truth: Store | None = None
        self._last: SceneState | None = None
        self._truth_error: Exception | None = None
        self._lock = RLock()
        self._done = Event()
        self._thread: Thread | None = None
        self._setup: dict[str, Any] = {}

    def configure_launch(self, proc: DimosCliCall) -> None:
        proc.simulator = "mujoco"
        proc.global_args = [
            "--transport",
            "zenoh",
            "--viewer",
            "none",
            "--scene-package",
            str(self.config.scene.resolve()),
            "--record-topics",
            ",".join(self.config.recorded_topics),
        ]
        proc.extra_env.update(self.config.module_env)
        proc.extra_env.setdefault("SIMULATIONMODULE__VIEWER", "false")

    def setup_scene(self) -> None:
        app = Dimos.connect()
        self._resources.callback(app.stop)
        self.scene = app.find_module_by_spec(SceneControlSpec)
        self._setup = {
            "config": self.config.model_dump(mode="json"),
            "scene": file_identity(self.config.scene),
            "description": self.scene.describe_scene().model_dump(mode="json"),
            "runtime": cast("SimulationModule", app.get_module("SimulationModule")).describe_run(),
        }
        if self.config.initial.poses or self.config.initial.joints:
            self.scene.set_scene_state(self.config.initial)

    def prepare_recording(self, recording: Store, path: Path, deadline: float) -> dict[str, Path]:
        truth_path = path.with_name("truth.db")
        self._truth = SqliteStore(path=str(truth_path))
        self._resources.callback(self._truth.stop)
        self._last = None
        self._truth_error = None
        self._done.clear()
        first = self._sample()
        setup_path = path.with_name("setup.json")
        setup_path.write_text(
            json.dumps(
                {**self._setup, "first_state": first.model_dump(mode="json")},
                indent=2,
                allow_nan=False,
            )
        )
        self._thread = Thread(target=self._record_loop, name="sim2-eval-truth", daemon=True)
        self._thread.start()
        self._resources.callback(self._stop_recording)
        while time.monotonic() < deadline:
            self._check_recorder()
            try:
                ages = [
                    time.time() - recording.streams[n].last().data.ts
                    for n in self.config.ready_streams
                ]
                if all(0 <= age < 3 for age in ages):
                    return {"truth": truth_path, "setup": setup_path}
            except (KeyError, LookupError):
                pass
            self._done.wait(0.05)
        raise TimeoutError(f"sim2 did not publish fresh {self.config.ready_streams}")

    def on_state(self, state: SceneState) -> None:
        """Case-specific intervention trigger, called by the private recorder."""

    def intervene(self, update: SceneUpdate, *, reason: str) -> SceneState:
        """Record an edit and its actual applied tick, without starting another episode."""
        with self._lock:
            assert self.scene is not None and self._truth is not None
            before = self._sample()
            after = self.scene.set_scene_state(update)
            if after.episode != before.episode or after.generation != before.generation + 1:
                raise RuntimeError("intervention raced with another scene mutation")
            self._truth.stream("interventions", Intervention).append(
                Intervention(reason=reason, update=update, before=before, after=after, ts=after.ts)
            )
            self._truth.stream("scene_state", SceneState).append(after)
            self._last = after
            return after

    def _sample(self) -> SceneState:
        with self._lock:
            assert self.scene is not None and self._truth is not None
            state = self.scene.scene_state()
            if self._last is not None and (
                state.episode != self._last.episode
                or state.generation != self._last.generation
                or state.tick < self._last.tick
            ):
                raise RuntimeError("unrecorded reset, edit or resume during eval")
            self._truth.stream("scene_state", SceneState).append(state)
            self._last = state
            return state

    def _record_loop(self) -> None:
        try:
            while not self._done.wait(1 / self.config.truth_hz):
                self.on_state(self._sample())
        except Exception as error:
            self._truth_error = error
            self._done.set()

    def _check_recorder(self) -> None:
        if self._truth_error is not None:
            raise RuntimeError("sim2 scene evidence recording failed") from self._truth_error

    def _stop_recording(self) -> None:
        self._done.set()
        if self._thread is not None:
            self._thread.join(timeout=10)
            if self._thread.is_alive():
                raise RuntimeError("sim2 scene recorder did not stop")
        self._check_recorder()
        final = self._sample()
        assert self._truth is not None
        self._truth.stream("completed", SceneState).append(final)

    def latest_pose(self, recording: Store) -> PoseStamped:
        return cast("PoseStamped", recording.streams.odom.last().data)


def scene_evidence(
    store: Store, *, max_gap_s: float = 0.5
) -> tuple[list[SceneState], list[Intervention]]:
    """Reject incomplete, mixed-episode or undersampled evidence before scoring."""
    try:
        states = [item.data for item in store.streams.scene_state]
        final = store.streams.completed.last().data
    except (AttributeError, LookupError) as error:
        raise ValueError("incomplete scene recording") from error
    if len(states) < 2 or final != states[-1]:
        raise ValueError("incomplete scene recording")
    edits = (
        [item.data for item in store.streams.interventions]
        if "interventions" in store.streams
        else []
    )
    generations = {edit.after.generation: edit for edit in edits}
    if len(generations) != len(edits):
        raise ValueError("duplicate intervention generations")
    for previous, state in pairwise(states):
        if (state.world_id, state.scene_id, state.episode) != (
            previous.world_id,
            previous.scene_id,
            previous.episode,
        ):
            raise ValueError("mixed scene episodes")
        if not 0 <= state.sim_time - previous.sim_time <= max_gap_s:
            raise ValueError("scene recording has a time gap or reversal")
        if state.generation != previous.generation:
            edit = generations.get(state.generation)
            if edit is None or edit.before != previous or edit.after != state:
                raise ValueError("unrecorded scene intervention")
    return states, edits
