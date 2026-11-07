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

"""The autoresearch lab as a module: one session on disk, candidate programs
run trial by trial on the simulated bench, judged from the simulator's true
object poses, with the successful episodes saved through the collection
recorder. The actor drives it through the skills below, over MCP."""

from __future__ import annotations

import json
import os
from pathlib import Path
import threading
import time
from typing import Any, Literal

from pydantic import Field, field_validator
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.mcp.spec import McpServerSpec
from dimos.autoresearch.actor import provider_key, run_actor
from dimos.autoresearch.judge import Judge, LiftJudge, PlaceAtJudge
from dimos.autoresearch.program import RunStoppedError
from dimos.autoresearch.session import Budget, Session, SessionStatus, Split
from dimos.autoresearch.status import ActorState, AutoresearchStatus
from dimos.autoresearch.trial import run_trial
from dimos.autoresearch.world import MujocoWorld, PlacementBounds
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.imitation.collection.episode_monitor import EpisodeControlSpec
from dimos.manipulation.manipulation_spec import ManipulationSpec
from dimos.manipulation.sdk import Arm
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.perception.experimental.object_scene_registration_spec import (
    ObjectSceneRegistrationSpec,
)
from dimos.simulation.engines.sim_scene_spec import SimSceneSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The one tool stream the trial loop holds, whichever skill started it.
_TOOL = "run"
# How long start() waits for the planner, which starts in parallel, to list the arm.
PLANNER_TIMEOUT_S = 120.0
# How long stop_actor waits for the Pi process to die and the actor thread to end.
ACTOR_STOP_TIMEOUT_S = 10.0
# GlobalConfig fields a client process needs to join this stack's bus.
_ZENOH_FIELDS = (
    "zenoh_mode",
    "zenoh_connect",
    "zenoh_scouting",
    "zenoh_interface",
    "zenoh_multicast",
    "zenoh_scout_addr",
    "zenoh_gossip",
    "zenoh_connect_timeout",
    "zenoh_domain",
)

API_REFERENCE = """\
def run(robot, task): ...     # task.text is the task in words; import nothing but math
robot.arm.joints() -> list[float] (radians); robot.arm.pose() -> Pose(position xyz m, orientation xyzw)
robot.arm.move_pose(xyz, orientation=None)   # orientation: roll, pitch, yaw or xyzw; None keeps the current one
robot.arm.move_linear(dx=0, dy=0, dz=0)      # straight line, metres
robot.arm.move_joints(positions); robot.arm.home()
robot.gripper.open(); robot.gripper.close(); robot.gripper.set(value); robot.gripper.range() -> (closed, open)
robot.scene.objects(["cube", "bowl"]) -> [SceneObject(id, name, position, size)]   # world metres, box centre
robot.observe() -> keyframe path; robot.log(text)
Motion calls block and raise RobotCallError (status PLANNING_FAILED, EXECUTION_FAILED or TIMEOUT).
Every call goes to the trial's trace.jsonl, with a keyframe before and after each motion call.
"""


def bus_env(g: GlobalConfig) -> dict[str, str]:
    """This process's environment plus the bus settings of ``g`` under their
    environment names, so a client process reaches the same stack."""
    env = dict(os.environ)
    env["DIMOS_TRANSPORT"] = g.transport
    for name in _ZENOH_FIELDS:
        value = getattr(g, name)
        if value is None:
            env.pop(name.upper(), None)
        else:
            env[name.upper()] = value if isinstance(value, str) else json.dumps(value)
    return env


def _wait_for_arm(rpc: ManipulationSpec, timeout_s: float) -> Arm:
    """The one pose-capable planning group, once the planner lists it and knows its joints."""
    deadline = time.monotonic() + timeout_s
    while True:
        groups = [group for group in rpc.list_planning_groups() if group.tip_frame is not None]
        if groups:
            state = rpc.get_state().groups.get(groups[0].id)
            if state is not None and state.joints is not None:
                return Arm(rpc, groups[0])
        if time.monotonic() > deadline:
            raise RuntimeError(f"the planner listed no arm with a joint state within {timeout_s} s")
        time.sleep(0.5)


class AutoresearchModuleConfig(ModuleConfig):
    """One session's settings. ``session_dir`` must not hold a session yet.
    ``judge`` decides a trial from ``judge_body``: lifted 5 cm, or placed within
    5 cm of ``place_body``. ``program_timeout_s`` kills a candidate run.
    ``backend_factory`` is what the program drives, ``"package.module:function"``.
    ``model`` at ``actor_provider`` is what ``start_actor`` runs unless told
    otherwise, for at most ``actor_max_steps`` model calls in ``actor_timeout_s``."""

    session_dir: Path
    task: str
    budget: Budget = Field(default_factory=Budget)
    model: str = "claude-sonnet-4-5"
    actor_provider: str = "anthropic"
    actor_max_steps: int = Field(default=600, gt=0)
    actor_timeout_s: float = Field(default=2 * 3600.0, gt=0)
    program_timeout_s: float = 120.0
    judge: Literal["lift", "place"] = "lift"
    judge_body: str = "cube"
    place_body: str = "bowl"
    placement_bounds: PlacementBounds = Field(default_factory=PlacementBounds)
    backend_factory: str = "dimos.autoresearch.live:backend"

    @field_validator("actor_provider")
    @classmethod
    def _known_provider(cls, provider: str) -> str:
        provider_key(provider)
        return provider


class AutoresearchModule(Module):
    """The lab. ``start`` opens the session and homes the arm; the skills are
    the actor's tools. ``run`` and ``freeze`` move the arm on a background
    thread and return at once; ``stop_run`` ends that thread. ``start_actor``
    runs the coding agent itself, on a thread, against this stack's MCP
    server. The status stream and ``get_status`` carry an ``AutoresearchStatus``."""

    config: AutoresearchModuleConfig

    body_poses: In[TFMessage]
    coordinator_joint_state: In[JointState]
    color_image: In[Image]
    # Not "status": the collection recorder records the episode monitor's stream of that name.
    autoresearch_status: Out[String]

    _sim: SimSceneSpec
    _arm: ManipulationSpec
    _episodes: EpisodeControlSpec
    _scene: ObjectSceneRegistrationSpec
    _mcp_server: McpServerSpec

    _session: Session
    _world: MujocoWorld
    _env: dict[str, str]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._run_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._actor_lock = threading.Lock()
        self._actor_stop = threading.Event()
        self._actor_thread: threading.Thread | None = None
        self._frame: Image | None = None
        self._frames = 0
        self._status = AutoresearchStatus(
            ts=time.time(), state="idle", session_dir=str(self.config.session_dir)
        )

    @rpc
    def start(self) -> None:
        super().start()
        self._session = Session.create(
            self.config.session_dir, self.config.task, self.config.budget, self.config.model
        )
        self._env = bus_env(self.config.g)
        self._world = MujocoWorld(
            self._sim,
            _wait_for_arm(self._arm, PLANNER_TIMEOUT_S),
            self.config.judge_body,
            self.config.placement_bounds,
            self._stop,
        )
        self.register_disposable(Disposable(self.body_poses.subscribe(self._world.on_body_poses)))
        self.register_disposable(
            Disposable(self.coordinator_joint_state.subscribe(self._world.on_joint_state))
        )
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_frame)))
        # Home and one gripper target before the first episode: the dataset
        # export seeds only joints that were commanded.
        self._world.home()
        self._emit(self.status())

    @rpc
    def stop(self) -> None:
        self._stop_actor()
        self._stop.set()
        try:
            self._halt()
        finally:
            super().stop()

    def status(self) -> AutoresearchStatus:
        """The lab's current status document."""
        with self._status_lock:
            return self._status

    @rpc
    def get_status(self) -> str:
        """The lab's current status as JSON text, the document the status stream carries."""
        return self.status().to_json()

    @rpc
    def start_actor(self, model: str | None = None, provider: str | None = None) -> str:
        """Start the coding agent that drives this lab over MCP, on a thread,
        and return the status at once; ``model`` and ``provider`` override the
        config. Refused while an actor runs or once the session is scored."""
        model = model or self.config.model
        provider = provider or self.config.actor_provider
        provider_key(provider)
        with self._actor_lock:
            if self._actor_thread is not None:
                raise RuntimeError("an actor is running; call stop_actor first")
            if self._session.info().status == "scored":
                raise RuntimeError("the session is scored; there is nothing left for an actor")
            self._actor_stop.clear()
            self._actor_thread = threading.Thread(
                target=self._actor, args=(model, provider), name="autoresearch-actor"
            )
            self._publish(
                actor="running", actor_model=model, actor_provider=provider, actor_error=None
            )
            self._actor_thread.start()
        return self.get_status()

    @rpc
    def stop_actor(self) -> str:
        """Stop the actor: its Pi process is killed and the status goes idle.
        The session stays as it is; a trial loop the actor started runs on."""
        self._stop_actor()
        return self.get_status()

    @skill
    def task(self) -> str:
        """The task in words, the robot API a program may use, the budget left
        and the library notes from earlier work. Call this first."""
        info = self._session.info()
        submitted = len(self._session.candidates())
        return json.dumps(
            {
                "task": info.task,
                "api": API_REFERENCE,
                "budget": {
                    **info.budget.model_dump(),
                    "candidates_submitted": submitted,
                    "candidates_left": _candidate_budget(info.budget) - submitted,
                },
                "status": info.status,
                "frozen": self._session.frozen(),
                "library": [path.read_text() for path in self._session.library()],
            },
            indent=1,
        )

    @skill
    def scene(self) -> str:
        """The objects on the bench now, with their box centres and sizes in
        world metres, and the path of a camera frame saved just now."""
        detections = self._scene.scan_scene(None)
        objects = [
            {
                "name": detection.results[0].hypothesis.class_id,
                "position": [
                    detection.bbox.center.position.x,
                    detection.bbox.center.position.y,
                    detection.bbox.center.position.z,
                ],
                "size": [detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z],
            }
            for detection in detections.detections
        ]
        return json.dumps(
            {
                "frame": detections.header.frame_id,
                "objects": objects,
                "keyframe": str(self._keyframe()),
            },
            indent=1,
        )

    @skill
    def submit(self, program: str, hypothesis: str) -> int:
        """Store a candidate program and return its number. ``program`` is Python
        source defining run(robot, task) and importing nothing but math;
        ``hypothesis`` says what it should fix and where it may still fail.
        Candidates never change: submit a new one to try a repair.

        Args:
            program: The program's source text.
            hypothesis: What this candidate is expected to do better, and its likely failure.
        """
        budget = self._session.info().budget
        if len(self._session.candidates()) >= _candidate_budget(budget):
            raise RuntimeError(f"the budget of {_candidate_budget(budget)} candidates is spent")
        return self._session.submit(program, hypothesis)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def run(self, candidate: int, split: Split = "dev") -> str:
        """Run a candidate on its pending trials in the background and return at
        once; poll results(candidate) until every trial is in. One run at a
        time: stop_run() ends it early.

        Args:
            candidate: The number submit returned.
            split: "dev" to iterate on, or "heldout" to resume a frozen candidate's held-out trials.
        """
        with self._run_lock:
            self.start_tool(_TOOL)
            if self._thread is not None:
                return "a run is in progress; wait for its results or call stop_run first"
            try:
                indexes = self._pending(candidate, split)
                if not indexes:
                    self.stop_tool(_TOOL)
                    return f"candidate {candidate} has already run every {split} trial"
                self._launch(candidate, split, indexes)
            except Exception:
                self.stop_tool(_TOOL)
                raise
        return f"started candidate {candidate} on {len(indexes)} {split} trials; poll results({candidate})"

    @skill
    def stop_run(self) -> str:
        """Stop the trial loop: the running program is killed, the arm stops and
        the interrupted trial is discarded, so it can run again."""
        return "stopped" if self._halt() else "no run in progress"

    @skill
    def results(self, candidate: int) -> str:
        """A candidate's recorded trials: split, index, success, the stage reached,
        the judge's note, the program's status, its duration and the trace and
        keyframe paths. Also whether its run is still going and what ended the
        last run early, if anything did.

        Args:
            candidate: The number submit returned.
        """
        records = self._session.results(candidate)
        status = self.status()
        return json.dumps(
            {
                "candidate": candidate,
                "hypothesis": self._session.hypothesis(candidate),
                "dev": _tally(self._session, candidate),
                "running": status.busy and status.candidate == candidate,
                "error": status.error,
                "trials": [
                    {
                        **record.model_dump(
                            mode="json", exclude={"backend", "start_ts", "episode"}
                        ),
                        "keyframes": str(record.trace.parent / "keyframes"),
                    }
                    for record in records
                ],
            },
            indent=1,
        )

    @skill
    def leaderboard(self) -> str:
        """Every candidate's dev tallies, best pass rate first."""
        return json.dumps(
            [
                {
                    "candidate": score.candidate,
                    "dev_passed": score.dev_passed,
                    "dev_trials": score.dev_trials,
                    "dev_rate": round(score.dev_rate, 3),
                }
                for score in self._session.leaderboard()
            ],
            indent=1,
        )

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def freeze(self, candidate: int) -> str:
        """Name the final candidate and start its held-out trials in the
        background. Nothing can be submitted afterwards; session.json says
        "scored", with the held-out pass rate, when they are done.

        Args:
            candidate: The number submit returned.
        """
        with self._run_lock:
            self.start_tool(_TOOL)
            if self._thread is not None:
                return "a run is in progress; wait for its results or call stop_run first"
            try:
                self._session.freeze(candidate)
                indexes = self._pending(candidate, "heldout")
                self._launch(candidate, "heldout", indexes)
            except Exception:
                self.stop_tool(_TOOL)
                raise
        return f"frozen candidate {candidate}; running {len(indexes)} held-out trials; poll results({candidate})"

    @skill
    def note(self, title: str, text: str) -> str:
        """Keep a library note for later sessions: a failure signature, when it
        applies, the repair and a code sketch. Returns the note's path.

        Args:
            title: A short name for the note.
            text: The note itself, Markdown.
        """
        return str(self._session.note(title, text))

    def _pending(self, candidate: int, split: Split) -> list[int]:
        """The trial indexes of ``split`` that ``candidate`` has no record for."""
        self._session.candidate_dir(candidate)
        budget = self._session.info().budget
        frozen = self._session.frozen()
        if split == "dev" and frozen is not None:
            raise RuntimeError(f"the session is frozen on candidate {frozen}; dev runs are over")
        if split == "heldout" and frozen != candidate:
            raise RuntimeError("held-out trials run once, on the frozen candidate: call freeze")
        count = budget.dev_trials if split == "dev" else budget.heldout_trials
        done = {
            record.index for record in self._session.results(candidate) if record.split == split
        }
        return [index for index in range(count) if index not in done]

    def _launch(self, candidate: int, split: Split, indexes: list[int]) -> None:
        if split == "dev":
            self._session.set_status("running")
        self._publish(
            state=self._session.info().status,
            busy=True,
            candidate=candidate,
            split=split,
            trial=indexes[0],
            error=None,
            **_tally(self._session, candidate),
        )
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._trials, args=(candidate, split, indexes), name="autoresearch-trials"
        )
        self._thread.start()

    def _trials(self, candidate: int, split: Split, indexes: list[int]) -> None:
        error: str | None = None
        try:
            for index in indexes:
                if self._stop.is_set():
                    break
                self._publish(
                    state=self._session.info().status,
                    busy=True,
                    candidate=candidate,
                    split=split,
                    trial=index,
                    error=None,
                    **_tally(self._session, candidate),
                )
                run_trial(
                    self._session,
                    candidate,
                    split,
                    index,
                    self._world,
                    self._judge(),
                    self._episodes,
                    self.config.backend_factory,
                    self.config.program_timeout_s,
                    self._env,
                    self._stop,
                )
        except RunStoppedError:
            logger.info("autoresearch: run stopped", candidate=candidate, split=split)
        except Exception as exc:
            if self._stop.is_set():
                logger.info("autoresearch: run stopped", candidate=candidate, split=split)
            else:
                logger.exception("autoresearch: trial failed", candidate=candidate, split=split)
                error = f"{type(exc).__name__}: {exc}"
        finally:
            with self._run_lock:
                self._thread = None
                self.stop_tool(_TOOL)
            state = self._finish(candidate, split)
            self._publish(
                state=state,
                busy=False,
                trial=None,
                error=error,
                heldout_rate=self._session.info().heldout_rate,
                **_tally(self._session, candidate),
            )

    def _actor(self, model: str, provider: str) -> None:
        actor: ActorState = "done"
        error: str | None = None
        try:
            trajectory = run_actor(
                self._mcp_server.mcp_url(),
                self._session,
                model,
                provider,
                self.config.actor_max_steps,
                timeout_s=self.config.actor_timeout_s,
                run_dir=self.config.session_dir / "actor",
                stop=self._actor_stop,
            )
            logger.info("autoresearch: actor ended", ended_by=trajectory.extra.ended_by)
            if trajectory.extra.ended_by == "error":
                actor, error = "failed", trajectory.extra.error
        except Exception as exc:
            logger.exception("autoresearch: actor failed", model=model, provider=provider)
            actor, error = "failed", f"{type(exc).__name__}: {exc}"
        finally:
            if self._actor_stop.is_set():
                actor, error = "idle", None
            with self._actor_lock:
                self._actor_thread = None
            self._publish(actor=actor, actor_error=error)

    def _stop_actor(self) -> None:
        with self._actor_lock:
            thread = self._actor_thread
        if thread is None:
            return
        self._actor_stop.set()
        thread.join(ACTOR_STOP_TIMEOUT_S)

    def _finish(self, candidate: int, split: Split) -> SessionStatus:
        """Settle session.json when a loop ends: scored once every held-out trial is in."""
        info = self._session.info()
        if split == "heldout":
            records = [r for r in self._session.results(candidate) if r.split == "heldout"]
            if len(records) < info.budget.heldout_trials:
                return info.status
            rate = sum(record.success for record in records) / len(records)
            self._session.set_status("scored", heldout_rate=rate)
            return "scored"
        if info.status == "running":
            self._session.set_status("idle")
            return "idle"
        return info.status

    def _halt(self) -> bool:
        """Stop a running loop and its program, then release the tool. False when none ran."""
        with self._run_lock:
            thread = self._thread
        if thread is None:
            return False
        self._stop.set()
        self._arm.cancel()
        thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
        self.stop_tool(_TOOL)
        return True

    def _judge(self) -> Judge:
        if self.config.judge == "lift":
            return LiftJudge(self.config.judge_body)
        return PlaceAtJudge(self.config.judge_body, self.config.place_body)

    def _on_frame(self, image: Image) -> None:
        with self._status_lock:
            self._frame = image

    def _keyframe(self) -> Path:
        with self._status_lock:
            frame = self._frame
            self._frames += 1
            number = self._frames
        if frame is None:
            raise RuntimeError("no camera frame received yet")
        path = self.config.session_dir / "scene" / f"{number:03d}.png"
        path.parent.mkdir(exist_ok=True)
        import cv2

        if not cv2.imwrite(str(path), frame.to_opencv()):
            raise RuntimeError(f"could not write {path}")
        return path

    def _publish(self, **changes: Any) -> None:
        """Apply ``changes`` to the status and publish it when anything but the time changed."""
        with self._status_lock:
            status = self._status.model_copy(update={"ts": time.time(), **changes})
            changed = status.model_dump(exclude={"ts"}) != self._status.model_dump(exclude={"ts"})
            self._status = status
        if changed:
            self._emit(status)

    def _emit(self, status: AutoresearchStatus) -> None:
        self.autoresearch_status.publish(String(status.to_json()))


def _candidate_budget(budget: Budget) -> int:
    return budget.iterations * budget.candidates_per_round


def _tally(session: Session, candidate: int) -> dict[str, int]:
    dev = [record for record in session.results(candidate) if record.split == "dev"]
    return {"dev_passed": sum(record.success for record in dev), "dev_trials": len(dev)}


autoresearch_module = AutoresearchModule.blueprint
