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

"""Run sandbox cases: Pi in its own container, the robot in another, on one network.

Per case, one docker compose project:

- ``agent``: Ubuntu and Pi, nothing else. The runner starts Pi in it with the case
  prompt and reads its JSON events, then once per later turn with the human's message.
- ``robot``: the zenoh router at ``tcp/robot:7447``, in front of a blueprint
  (:class:`DimosRobot`) or forwarding to one running elsewhere (:class:`ExternalRobot`). A
  blueprint runs under ``dimos --record``, so every one of its streams lands in the case's
  ``recordings/<run id>/memory.db``.
- ``llm-proxy``: forwards Pi's model calls and saves each request and response.

Each agent turn ends when Pi stops, e.g. on an accepted call to the done tool. The case's
human then answers, and Pi carries on in the same session, until the human has nothing more
to say, the timeout, or the step budget. Evaluate then runs while the robot and agent
containers are still up.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from contextlib import closing
from dataclasses import asdict, dataclass, replace
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time
from typing import Any

from dimos.constants import CACHE_DIR, STATE_DIR
from dimos.evals.agents.lib.pi_config import Provider, RuntimeConfig, Thinking
from dimos.evals.agents.lib.pi_to_atif import PiToAtif
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.agents.pi import read_pi_events
from dimos.evals.constants import MAX_TOOL_SECONDS, PROVIDERS
from dimos.evals.runner import _git_record, _write_trajectory, summarize
from dimos.evals.sandbox.case import (
    AGENT_IMAGE,
    CASE_MOUNT,
    KILL_AGENT_PROCESSES,
    ROBOT_ENDPOINT,
    ROBOT_IMAGE,
    Conversation,
    DimosRobot,
    ExternalRobot,
    Robot,
    SandboxCase,
    SandboxOutcome,
    exec_in,
)
from dimos.evals.types import EvalResult, Step, Trajectory
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

PROXY_PORT = 8080
# Seconds each readiness probe of the robot waits for the case's ready_key, then tries again.
READY_PROBE_S = 10.0
# Run in the robot container: exits 0 once argv[1] (a key expression, or "" for none) publishes on
# the router at argv[2], within argv[3] seconds.
AWAIT_KEY = (
    "import sys; from dimos.evals.sandbox.robot import await_key; "
    "sys.exit(not await_key(sys.argv[1] or None, sys.argv[2], float(sys.argv[3])))"
)
# The robot container's cache, kept on this machine between cases.
ROBOT_CACHE = CACHE_DIR / "evals" / "sandbox_robot"
# Inside the agent container: the harness files, and where Pi works.
HARNESS = "/opt/harness"
# What the done tool answers when its arguments validate; the runner looks for it.
DONE_ACCEPTED = "Done: accepted."
# Extra seconds the runner waits past the in-container timeout before giving up on Pi.
TIMEOUT_GRACE_S = 15.0


@dataclass(frozen=True, kw_only=True)
class SandboxAgent:
    """The Pi setup in the agent container."""

    provider: Provider = "openai"
    model: str = "gpt-5.6-luna"
    thinking: Thinking = "medium"
    max_steps: int | None = 200  # model requests; None is unlimited
    max_output_tokens: int | None = None
    tools: tuple[str, ...] = ("read", "bash", "edit", "write", "grep", "find", "ls")
    max_tool_seconds: float = MAX_TOOL_SECONDS
    image: str = AGENT_IMAGE

    @property
    def key_env(self) -> str:
        return PROVIDERS[self.provider][0]


class SandboxRunner:
    def __init__(
        self,
        agent: SandboxAgent,
        *,
        robot: ExternalRobot | None = None,
        resume_from: Path | None = None,
        resume_message: str = "",
        out_dir: Path = STATE_DIR / "evals",
    ) -> None:
        """*robot* replaces every case's robot, e.g. a real robot in place of a blueprint.

        *resume_from*, an earlier run's directory, continues each case's Pi session there with
        *resume_message* as the next user turn, on a fresh robot, in a copy of its /root/work.
        """
        if resume_from is not None and not resume_message:
            raise ValueError("resuming needs a message for the agent")
        self.agent = agent
        self.robot_override = robot
        self.resume_from = resume_from
        self.resume_message = resume_message
        self.out_dir = out_dir
        self.run_dir = out_dir

    def run(self, cases: Sequence[SandboxCase], *, source: str = "") -> list[EvalResult]:
        ids = [case.id for case in cases]
        for case_id in ids:
            if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", case_id):
                raise ValueError(f"unsafe sandbox case ID: {case_id!r}")
        if len(ids) != len(set(ids)):
            raise ValueError("duplicate sandbox case IDs")
        if not os.environ.get(self.agent.key_env):
            raise RuntimeError(f"the agent needs {self.agent.key_env} in the environment")
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.run_dir = Path(
            tempfile.mkdtemp(prefix=time.strftime("sandbox-%Y%m%d-%H%M%S-"), dir=self.out_dir)
        )
        manifest = {
            "schema_version": 1,
            "kind": "sandbox",
            "source": source,
            "case_ids": ids,
            "agent": asdict(self.agent),
            "robot_override": self.robot_override.model_dump() if self.robot_override else None,
            "resumed_from": str(self.resume_from) if self.resume_from else None,
            "resume_message": self.resume_message or None,
            "code": _git_record(),
        }
        (self.run_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))
        results = []
        for case in cases:
            result = self.run_case(case)
            logger.info("sandbox case done", case=case.id, score=result.score, error=result.error)
            results.append(result)
            # Written as each case ends, so a crash or ctrl-c keeps the cases already run.
            with (self.run_dir / "results.jsonl").open("a") as out:
                out.write(json.dumps(asdict(result)) + "\n")
        summary = {**asdict(summarize(results)), "manifest": "manifest.json"}
        (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2))
        return results

    def run_case(self, case: SandboxCase) -> EvalResult:
        t0 = time.monotonic()
        case_dir = self.run_dir / case.id
        (case_dir / "raw").mkdir(parents=True)
        robot = (
            self.robot_override if case.robot is not None and self.robot_override else case.robot
        )
        project = re.sub(r"[^a-z0-9_-]", "-", f"{self.run_dir.name}-{case.id}".lower())
        compose_file = case_dir / "compose.json"
        self._write_harness(case, case_dir / "harness")
        previous = self.resume_from / case.id if self.resume_from else None
        if previous is not None:
            shutil.copytree(previous / "harness" / "session", case_dir / "harness" / "session")
        (case_dir / "case.json").write_text(case.model_dump_json(indent=2))
        compose_file.write_text(json.dumps(self._compose(robot, case_dir), indent=2))
        compose = ["docker", "compose", "-p", project, "-f", str(compose_file)]
        trajectory: Trajectory | None = None
        agent_s = 0.0
        try:
            subprocess.run([*compose, "up", "-d", "--quiet-pull"], check=True, capture_output=True)
            if robot is not None:
                self._wait_ready(compose, case)
            if previous is not None:
                _restore_work(compose, previous / "work", case_dir / "restored-work")
            agent_t0 = time.monotonic()
            trajectory = self._converse(compose, case, case_dir, exec_in(project, compose_file))
            agent_s = time.monotonic() - agent_t0
            _write_trajectory(case_dir, trajectory, [*self.agent.tools, case.done_tool])
            done = accepted_done(trajectory.steps, case.done_tool)
            if done is None:
                error = trajectory.extra.error or "the agent never made an accepted done call"
                return self._result(case, t0, trajectory, agent_s, error=error)

            outcome = SandboxOutcome(
                done=done,
                trajectory=trajectory,
                recording=next((case_dir / "recordings").glob("*/memory.db"), None),
                exec=exec_in(project, compose_file),
                case_dir=case_dir,
            )
            return self._result(case, t0, trajectory, agent_s, score=case.evaluate(outcome))
        except Exception as e:
            return self._result(case, t0, trajectory, agent_s, error=repr(e))
        finally:
            with (case_dir / "containers.log").open("w") as log:
                subprocess.run([*compose, "logs", "--no-color", "-t"], stdout=log, stderr=log)
                # What the agent wrote, before its container is removed.
                subprocess.run(
                    [*compose, "cp", "agent:/root/work", str(case_dir / "work")],
                    stdout=log,
                    stderr=log,
                )
            subprocess.run(
                [*compose, "down", "-v", "--remove-orphans", "-t", "5"], capture_output=True
            )

    def _write_harness(self, case: SandboxCase, harness: Path) -> None:
        """Pi's extensions, mounted into the agent container at HARNESS."""
        harness.mkdir()
        lib = Path(__file__).parent
        shutil.copyfile(lib.parent / "agents/lib/runtime.js", harness / "runtime.js")
        shutil.copyfile(lib / "done.js", harness / "done.js")
        runtime = RuntimeConfig(
            provider=self.agent.provider,
            base_url=f"http://llm-proxy:{PROXY_PORT}",
            key_env=self.agent.key_env,
            max_output_tokens=self.agent.max_output_tokens,
            max_tool_seconds=self.agent.max_tool_seconds,
        )
        (harness / "runtime.json").write_text(runtime.model_dump_json())
        done = {
            "name": case.done_tool,
            "description": case.done_description,
            "schema": case.done_schema,
            "accepted": DONE_ACCEPTED,
        }
        (harness / "done.json").write_text(json.dumps(done))

    def _compose(self, robot: Robot | None, case_dir: Path) -> dict[str, Any]:
        upstream = os.environ.get(
            f"{self.agent.provider.upper()}_BASE_URL", PROVIDERS[self.agent.provider][1]
        )

        # Root in the containers; keep what lands in the case dir deletable by the host user.
        def owned(*argv: str) -> list[str]:
            return ["sh", "-c", 'umask 0000; exec "$@"', "sh", *argv]

        services: dict[str, Any] = {
            "agent": {
                "image": self.agent.image,
                "init": True,
                "environment": [self.agent.key_env],  # the value comes from this shell
                "volumes": [f"{case_dir / 'harness'}:{HARNESS}"],
            },
            "llm-proxy": {
                "image": robot.image if robot else ROBOT_IMAGE,
                "init": True,
                "command": owned(
                    "python",
                    "-m",
                    "dimos.evals.agents.lib.model_trace_proxy",
                    "/case/raw",
                    upstream.rstrip("/"),
                    json.dumps(self.agent.max_steps),
                    "/case/model-request-limit-reached",
                    "0.0.0.0",
                    str(PROXY_PORT),
                ),
                "volumes": [f"{case_dir}:/case"],
            },
        }
        if robot is not None:
            service = _robot_service(robot, case_dir)
            service["command"] = owned(*service["command"])  # its recording, too
            services["robot"] = {"image": robot.image, "init": True, **service}
        return {"services": services}

    def _wait_ready(self, compose: list[str], case: SandboxCase) -> None:
        """Until the robot's router answers and, if the case names one, ready_key has published."""
        deadline = time.monotonic() + case.ready_timeout_s
        while time.monotonic() < deadline:
            probe = subprocess.run(
                [*compose, "exec", "-T", "robot", "python", "-c", AWAIT_KEY,
                 case.ready_key or "", ROBOT_ENDPOINT, str(READY_PROBE_S)],
                capture_output=True,
            )  # fmt: skip
            if probe.returncode == 0:
                return
            state = subprocess.run(
                [*compose, "ps", "-a", "--format", "{{.State}}", "robot"],
                capture_output=True, text=True,
            ).stdout.strip()  # fmt: skip
            if state == "exited":
                raise RuntimeError("the robot container exited before it was ready")
            time.sleep(1.0)
        raise TimeoutError(f"robot not ready after {case.ready_timeout_s:.0f}s ({case.ready_key})")

    def _converse(
        self,
        compose: list[str],
        case: SandboxCase,
        case_dir: Path,
        exec: Callable[..., subprocess.CompletedProcess[str]],
    ) -> Trajectory:
        """Agent turns, each answered by the case's human, until it has nothing more to say."""
        prompt = self.resume_message or case.prompt
        events = PiToAtif(
            case_dir / "raw", TrajectoryBuilder(prompt, name="SandboxPi", model=self.agent.model)
        )
        conversation = Conversation(exec=exec, case_dir=case_dir)
        human = case.human(conversation)
        deadline = time.monotonic() + case.timeout_s
        while True:
            steps = len(events.trajectory.steps)
            ended_by = self._turn(compose, case, case_dir, events, prompt, deadline)
            conversation.turns += 1
            conversation.done = accepted_done(events.trajectory.steps[steps:], case.done_tool)
            if ended_by != "answer":
                break
            try:
                prompt = next(human)
            except StopIteration:
                break
            if time.monotonic() >= deadline:
                ended_by = "timeout"
                break
            events.trajectory.user(prompt)
        return events.trajectory.build(ended_by, error=events.error)  # type: ignore[arg-type]

    def _turn(
        self,
        compose: list[str],
        case: SandboxCase,
        case_dir: Path,
        events: PiToAtif,
        prompt: str,
        deadline: float,
    ) -> str:
        """One Pi run on *prompt*, continuing the case's session if it has one; how it ended."""
        agent = self.agent
        sessions = sorted((case_dir / "harness" / "session").glob("*.jsonl"))
        session = ["--session", f"{HARNESS}/session/{sessions[-1].name}"] if sessions else []
        left_s = max(1, int(deadline - time.monotonic()))
        command = [
            *compose, "exec", "-T", "agent",
            "timeout", "-k", "5", str(left_s),
            "pi", "--mode", "json", "--model", f"{agent.provider}/{agent.model}",
            "--thinking", agent.thinking, "--session-dir", f"{HARNESS}/session", *session,
            "--tools", ",".join((*agent.tools, case.done_tool)),  # --tools gates extension tools too
            "--extension", f"{HARNESS}/runtime.js", "--extension", f"{HARNESS}/done.js",
            "--no-extensions", "--no-skills", "--no-prompt-templates", "--no-themes",
            "--no-context-files", "--no-approve", prompt,
        ]  # fmt: skip
        ended_by = "answer"
        events.error = ""
        with (
            (case_dir / "pi-stderr.txt").open("a") as stderr,
            # Pi reads an open stdin as more prompt, and would wait on it forever.
            subprocess.Popen(
                command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=stderr
            ) as proc,
        ):
            assert proc.stdout is not None
            try:
                with closing(read_pi_events(proc.stdout, deadline + TIMEOUT_GRACE_S)) as stream:
                    for event in stream:
                        events.append_event(event)
                proc.wait(timeout=max(0.0, deadline + TIMEOUT_GRACE_S - time.monotonic()))
            except (TimeoutError, subprocess.TimeoutExpired):
                ended_by = "timeout"
            except Exception as e:
                ended_by, events.error = "error", repr(e)
            finally:
                proc.kill()
                proc.wait()
        # Whatever the agent left running, e.g. a script still driving the robot.
        subprocess.run(
            [*compose, "exec", "-T", "agent", "bash", "-c", KILL_AGENT_PROCESSES],
            capture_output=True,
        )
        if proc.returncode == 124:  # timeout(1): the case's time ran out in the container
            ended_by = "timeout"
        if (case_dir / "model-request-limit-reached").exists():
            ended_by = "max_steps"
        elif ended_by == "answer" and proc.returncode not in (0, None) and not events.error:
            events.error = f"pi exit status {proc.returncode}"
        if events.error and ended_by == "answer":
            ended_by = "error"
        return ended_by

    def _result(
        self,
        case: SandboxCase,
        t0: float,
        trajectory: Trajectory | None,
        agent_s: float,
        *,
        score: float = 0.0,
        error: str = "",
    ) -> EvalResult:
        result = EvalResult(
            case_id=case.id,
            score=score,
            passed=score >= case.threshold and not error,
            duration_s=time.monotonic() - t0,
            agent_duration_s=agent_s,
            error=error,
        )
        if trajectory is None:
            return result
        totals = trajectory.final_metrics
        return replace(
            result,
            final_answer=json.dumps(accepted_done(trajectory.steps, case.done_tool)),
            steps=totals.total_steps,
            model_turns=sum(s.source == "agent" for s in trajectory.steps),
            tool_calls=sum(len(s.tool_calls or ()) for s in trajectory.steps),
            request_attempts=len(list((self.run_dir / case.id / "raw").glob("*-request.json"))),
            prompt_tokens=totals.total_prompt_tokens,
            completion_tokens=totals.total_completion_tokens,
            cached_tokens=totals.total_cached_tokens,
            reasoning_tokens=sum(s.extra.reasoning_tokens for s in trajectory.steps if s.extra),
            cost_usd=totals.total_cost_usd,
            ended_by=trajectory.extra.ended_by,
            trajectory=str(self.run_dir / case.id / "trajectory.json"),
        )


def _restore_work(compose: list[str], work: Path, staged: Path) -> None:
    """Put an earlier run's /root/work back in the agent container, minus any sign of grading."""
    shutil.copytree(work, staged, symlinks=True)
    try:
        log = staged / "grading-run.log"
        if log.exists():
            log.rename(staged / "run-output.log")
        subprocess.run(
            [*compose, "cp", f"{staged}/.", "agent:/root/work"], check=True, capture_output=True
        )
    finally:
        shutil.rmtree(staged)


def _robot_service(robot: Robot, case_dir: Path) -> dict[str, Any]:
    if isinstance(robot, DimosRobot):
        from dimos.utils.data import get_data

        forward = [a for endpoint in robot.forward for a in ("--forward", endpoint)]
        hide = [] if robot.expose_dimos else ["--hide-dimos"]
        ROBOT_CACHE.mkdir(parents=True, exist_ok=True)
        (case_dir / "recordings").mkdir(exist_ok=True)  # this user's, not the container's root
        record = ["--record", "sqlite"]
        return {
            "command": ["dimos", "evals", "robot", *forward, *hide, "--", *record, *robot.args],
            "environment": dict(robot.env),
            "volumes": [
                # Ground truth the blueprint logs for grading; the agent cannot reach it.
                f"{case_dir}:{CASE_MOUNT}",
                # The robot image is a checkout (/app/.git), so dimos records under /app/recordings.
                f"{case_dir / 'recordings'}:/app/recordings",
                # Compiled models and the like survive from one case to the next.
                f"{ROBOT_CACHE}:/cache",
                *(f"{get_data(name)}:/app/data/{name}:ro" for name in robot.data),
            ],
        }
    forward = [a for endpoint in robot.connect for a in ("--forward", endpoint)]
    return {
        "command": ["dimos", "evals", "robot", *forward],
        # A robot on this machine's own ports, e.g. tcp/host.docker.internal:7447.
        "extra_hosts": ["host.docker.internal:host-gateway"],
    }


def accepted_done(steps: Sequence[Step], tool: str) -> dict[str, Any] | None:
    """The arguments of the last done call Pi validated and ran, None if there was none."""
    for step in reversed(steps):
        accepted = {
            r.source_call_id
            for r in (step.observation.results if step.observation else ())
            if r.content == DONE_ACCEPTED
        }
        for call in step.tool_calls or ():
            if call.function_name == tool and call.tool_call_id in accepted:
                return dict(call.arguments)
    return None
