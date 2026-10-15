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

"""What a sandbox case is made of, what its human and evaluate functions are handed.

A case is plain data (pydantic, JSON round-trips), so suites can live in JSON files and a UI
can list and edit them. The two behaviors a case needs, the pretend human and the grader,
are named by :class:`FunctionRef` ("module:function" plus JSON keyword arguments) rather
than held as Python objects.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator, Sequence
import importlib
import json
import os
from pathlib import Path
import subprocess
from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field

from dimos.evals.sandbox.recorder import Recording
from dimos.evals.types import Trajectory

# The image the robot, recorder and model trace proxy run from (docker/evals_sandbox).
ROBOT_IMAGE = "dimensional/evals-robot"
AGENT_IMAGE = "dimensional/evals-agent"
# Where the agent finds its robot, whichever robot it is.
ROBOT_ENDPOINT = "tcp/robot:7447"
# Kills whatever the agent left running in its container, e.g. a script still driving the
# robot. Not the container's init and its sleep, which keep the container up.
KILL_AGENT_PROCESSES = (
    'for pid in $(pgrep -v -f "^(/sbin/docker-init|sleep infinity)"); do '
    '[ "$pid" != "$$" ] && kill -9 "$pid"; done; true'
)
# Where the robot container sees this case's output dir, e.g. to log ground truth for grading.
CASE_MOUNT = "/case"
# Where a robot running on the host (--robot-on-host) finds its case directory instead.
CASE_DIR_ENV = "DIMOS_SANDBOX_CASE_DIR"


def robot_case_dir() -> str:
    """The case directory as the robot sees it: mounted in its container, or on the host."""
    return os.environ.get(CASE_DIR_ENV, CASE_MOUNT)


class FunctionRef(BaseModel):
    """A function by name, ``"dimos.evals.sandbox.example:evaluate"``, and the keyword
    arguments it is called with after the runner's own."""

    model_config = ConfigDict(frozen=True)

    function: str
    params: dict[str, Any] = {}

    def __call__(self, *args: Any) -> Any:
        module, _, name = self.function.partition(":")
        return getattr(importlib.import_module(module), name)(*args, **self.params)


class DimosRobot(BaseModel):
    """Any blueprint, run in the robot container behind the router.

    ``args`` are what follows ``dimos`` on the command line, e.g.
    ``("--simulation", "mujoco", "run", "xarm-perception-sim")``. dimOS runs on zenoh
    with every session dialing the router, so the blueprint's own topics
    (``dimos/<stream>/<type>``) are what the agent sees.
    """

    model_config = ConfigDict(frozen=True)

    kind: Literal["dimos"] = "dimos"
    args: tuple[str, ...]
    # Zenoh endpoints inside the container the router also dials, for peers that only
    # listen, e.g. raw-robot-bridge's tcp/127.0.0.1:7448.
    forward: tuple[str, ...] = ()
    env: dict[str, str] = {}
    image: str = ROBOT_IMAGE
    # dimOS data directories (get_data names) the robot needs, mounted read-only from this
    # machine rather than downloaded in every container, e.g. ("scene_packages",).
    data: tuple[str, ...] = ()
    # False keeps dimOS's own topics off the router: the agent sees only what the
    # blueprint serves itself (raw-robot-bridge), not simulator ground truth.
    expose_dimos: bool = True


class ExternalRobot(BaseModel):
    """A robot already running elsewhere: real hardware, or a blueprint on another machine.

    On that machine, ``dimos evals robot -- run <blueprint>`` serves the same router the
    robot container does, so the agent sees the same keys. The sandbox's robot
    container is then only a router that dials ``connect``.
    """

    model_config = ConfigDict(frozen=True)

    kind: Literal["external"] = "external"
    connect: tuple[str, ...]
    image: str = ROBOT_IMAGE


Robot = Annotated[DimosRobot | ExternalRobot, Field(discriminator="kind")]


class Conversation:
    """What the pretend human sees each time the agent stops. The robot is still running.

    A human is a generator function, ``human(conversation, **params)``: after every agent
    turn the runner asks it for the next message. Each string it yields goes to the agent as
    the user's next message, in the same session; returning ends the conversation, and
    evaluate runs. So the human, not the done tool, decides when the agent is finished.
    """

    def __init__(
        self, *, exec: Callable[..., subprocess.CompletedProcess[str]], case_dir: Path
    ) -> None:
        self.exec = exec  # run a shell command in the agent container, e.g. the agent's system
        self.case_dir = case_dir  # this case's output dir
        self.turns = 0  # agent turns so far
        self.done: dict[str, Any] | None = None  # the done arguments of the latest turn

    @property
    def called_done(self) -> bool:
        """Whether the agent's latest turn ended with an accepted done call."""
        return self.done is not None


Human = Callable[..., Iterator[str]]


class SandboxOutcome:
    """What evaluate gets once the conversation is over. The robot and the recorder are still running."""

    def __init__(
        self,
        *,
        done: dict[str, Any],
        trajectory: Trajectory,
        exec: Callable[..., subprocess.CompletedProcess[str]],
        case_dir: Path,
        stop_recorder: Callable[[], Recording] | None,
    ) -> None:
        self.done = done  # the arguments of the agent's last accepted done call
        self.trajectory = trajectory
        self.exec = exec  # run a shell command in the agent container
        self.case_dir = case_dir  # this case's output dir, e.g. for ground truth or a report
        self._stop_recorder = stop_recorder
        self._recording: Recording | None = None

    @property
    def recording(self) -> Recording | None:
        """Every zenoh sample of the run so far; None without a robot.

        The first read stops the recorder, so read it after anything evaluate itself
        wants recorded (e.g. running the agent's system).
        """
        if self._recording is None and self._stop_recorder is not None:
            self._recording = self._stop_recorder()
        return self._recording


class SandboxCase(BaseModel):
    model_config = ConfigDict(frozen=True)

    id: str
    prompt: str  # the user's first message; tell the agent about the robot at ROBOT_ENDPOINT
    done_schema: dict[str, Any]  # JSON schema the done tool's arguments must satisfy
    # evaluate(outcome, **params) -> 0..1, after the human ends the conversation; only
    # called if the agent made an accepted done call.
    evaluate: FunctionRef
    # human(conversation, **params) yields the user's next message after each agent turn.
    human: FunctionRef = FunctionRef(function="dimos.evals.sandbox.human:until_done")
    robot: Robot | None = None
    # A zenoh key expression that must have published before the agent starts.
    ready_key: str | None = None
    ready_timeout_s: float = 300.0
    timeout_s: float = 600.0  # the whole conversation, every agent turn
    threshold: float = 1.0
    tags: tuple[str, ...] = ()
    # What the recorder keeps: zenoh key expression -> most samples a second (None keeps
    # every sample); keys matching none are dropped. None records everything.
    record: dict[str, float | None] | None = None
    done_tool: str = "done"
    done_description: str = (
        "Call this once the task is complete, with the requested result. It ends your turn."
    )


SandboxSuite = Sequence[SandboxCase]


def load_suite(source: str) -> list[SandboxCase]:
    """A suite from a JSON file (a list of cases) or a dotted module exporting SUITE."""
    if source.endswith(".json"):
        return [SandboxCase.model_validate(c) for c in json.loads(Path(source).read_text())]
    return list(importlib.import_module(source).SUITE)


def exec_in(project: str, compose_file: Path) -> Callable[..., subprocess.CompletedProcess[str]]:
    """``exec("ls /root/work")``: a shell command in the agent container, as the agent runs them."""

    def run(command: str, *, timeout: float = 60.0) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["docker", "compose", "-p", project, "-f", str(compose_file), "exec", "-T", "agent",
             "bash", "-lc", command],
            capture_output=True, text=True, timeout=timeout,
        )  # fmt: skip

    return run
