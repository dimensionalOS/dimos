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

"""What a sandbox case is made of, and what its evaluate function is handed."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
import subprocess
from typing import Any

from dimos.evals.sandbox.recorder import Recording
from dimos.evals.types import Trajectory

# The image the robot, recorder and model trace proxy run from (docker/evals_sandbox).
ROBOT_IMAGE = "dimensional/evals-robot"
AGENT_IMAGE = "dimensional/evals-agent"
# Where the agent finds its robot, whichever robot it is.
ROBOT_ENDPOINT = "tcp/robot:7447"
# Where the robot container sees this case's output dir, e.g. to log ground truth for grading.
CASE_MOUNT = "/case"


@dataclass(frozen=True, kw_only=True)
class DimosRobot:
    """Any blueprint, run in the robot container behind the router.

    ``args`` are what follows ``dimos`` on the command line, e.g.
    ``("--simulation", "mujoco", "run", "xarm-perception-sim")``. dimOS runs on zenoh
    with every session dialing the router, so the blueprint's own topics
    (``dimos/<stream>/<type>``) are what the agent sees.
    """

    args: tuple[str, ...]
    # Zenoh endpoints inside the container the router also dials, for peers that only
    # listen, e.g. raw-robot-bridge's tcp/127.0.0.1:7448.
    forward: tuple[str, ...] = ()
    env: Mapping[str, str] = field(default_factory=dict)
    image: str = ROBOT_IMAGE
    # dimOS data directories (get_data names) the robot needs, mounted read-only from this
    # machine rather than downloaded in every container, e.g. ("scene_packages",).
    data: tuple[str, ...] = ()
    # False keeps dimOS's own topics off the router: the agent sees only what the
    # blueprint serves itself (raw-robot-bridge), not simulator ground truth.
    expose_dimos: bool = True


@dataclass(frozen=True, kw_only=True)
class ExternalRobot:
    """A robot already running elsewhere: real hardware, or a blueprint on another machine.

    On that machine, ``dimos evals robot -- run <blueprint>`` serves the same router the
    robot container does, so the agent sees the same keys. The sandbox's robot
    container is then only a router that dials ``connect``.
    """

    connect: tuple[str, ...]
    image: str = ROBOT_IMAGE


Robot = DimosRobot | ExternalRobot


class SandboxOutcome:
    """What evaluate gets once the agent has stopped. The robot and the recorder are still running."""

    def __init__(
        self,
        *,
        done: dict[str, Any],
        trajectory: Trajectory,
        exec: Callable[..., subprocess.CompletedProcess[str]],
        case_dir: Path,
        stop_recorder: Callable[[], Recording] | None,
    ) -> None:
        self.done = done  # the arguments of the agent's accepted done call
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


@dataclass(frozen=True, kw_only=True)
class SandboxCase:
    id: str
    prompt: str  # the user message; tell the agent about the robot at ROBOT_ENDPOINT here
    done_schema: Mapping[str, Any]  # JSON schema the done tool's arguments must satisfy
    evaluate: Callable[[SandboxOutcome], float]  # 0..1; only called after an accepted done
    robot: Robot | None = None
    # A zenoh key expression that must have published before the agent starts.
    ready_key: str | None = None
    ready_timeout_s: float = 300.0
    timeout_s: float = 600.0
    threshold: float = 1.0
    tags: frozenset[str] = frozenset()
    # What the recorder keeps: zenoh key expression -> most samples a second (None keeps
    # every sample); keys matching none are dropped. None records everything.
    record: Mapping[str, float | None] | None = None
    # What the agent is told when it stops without an accepted done call (e.g. to ask the
    # user something), at most `reminders` times; None is a plain "call done" reminder.
    reminder: str | None = None
    reminders: int = 3
    done_tool: str = "done"
    done_description: str = (
        "Call this once the task is complete, with the requested result. It ends your run."
    )


SandboxSuite = Sequence[SandboxCase]


def exec_in(project: str, compose_file: Path) -> Callable[..., subprocess.CompletedProcess[str]]:
    """``exec("ls /root/work")``: a shell command in the agent container, as the agent runs them."""

    def run(command: str, *, timeout: float = 60.0) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["docker", "compose", "-p", project, "-f", str(compose_file), "exec", "-T", "agent",
             "bash", "-lc", command],
            capture_output=True, text=True, timeout=timeout,
        )  # fmt: skip

    return run
