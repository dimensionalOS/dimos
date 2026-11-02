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

"""Run the handoff validation trials in hybrid-teleop-sim and write the report.

    MUJOCO_GL=egl nix develop -c uv run --no-sync python \\
        -m dimos.manipulation.evals.tool_handoff_validation --decision-maker llm

Starts the stack in this process (with the LLM agent on top for ``llm``),
then for each trial: puts the scene back, asks the decision-maker to pick the
apple, breaks the pick as the trial plan says, and watches whether the
coordinator's operator hold comes on. Writes handoff_validation_<date>.md and
.csv to ``--out``. ``llm`` needs OPENAI_API_KEY (from the environment or
.env); without it, or when the model does not answer, the run uses ``scripted``.
"""

from __future__ import annotations

import argparse
from collections import deque
from collections.abc import Callable, Sequence
import contextlib
from datetime import datetime
import math
import os
from pathlib import Path
import subprocess
import threading
import time
from typing import Any

from dotenv import load_dotenv
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skill_result import SkillResult
from dimos.cli.commands.lifecycle import DEFAULT_CONFIG_PATH
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OPERATOR_HOLD_TASK_NAME,
    OperatorHoldStatus,
)
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.manipulation.evals.handoff_validation import (
    PICK_COMPLETE,
    TRIAL_PLAN,
    AgentDecisionMaker,
    Attempt,
    Case,
    DecisionMaker,
    ScriptedDecisionMaker,
    Trial,
    write_report,
)
from dimos.robot.manipulators.common.agent_prompts import MANIPULATION_AGENT_SYSTEM_PROMPT
from dimos.robot.manipulators.xarm.blueprints.hybrid_teleop_sim import hybrid_teleop_sim
from dimos.utils.logging_config import set_run_log_dir, setup_logger

logger = setup_logger()

TARGET = "apple"  # the sim body to pick
PROMPT = "red ball"  # what to ask for: the detector finds the sim's apple as a red ball
OBJECTS = ("apple", "orange", "cup")
GRIPPER_BODY = "xarm_gripper_base_link"
TCP_OFFSET = np.array([0.0, 0.0, 0.172])  # link_tcp in the gripper base frame, metres
TABLE_X = (0.32, 0.58)  # table top, world frame, metres, less a 2 cm border
TABLE_Y = (-0.18, 0.18)
REMOVE_DISTANCE = 2.0  # "removed" puts the object this far (metres, +x) off the table
MOVE_TO = (0.36, -0.13)  # "moved": a free spot on the table, world x, y, metres
MISS_OFFSETS = (0.05, 0.06, 0.04, 0.05)  # "missed grasp" sideways shifts, metres, in turn

AGENT_PROMPT_EXTRA = """
# Asking a person for help
- **request_operator <reason>**: Stop the robot and ask a human operator for help.

If pick_object does not report "Pick complete", or you cannot finish the task, call
request_operator once with a short plain reason that says what failed (quote what
the pick reported), then stop. Do not retry the pick and do not move the arm
yourself. After "Pick complete", do not call request_operator; reply "done".
"""


class Bodies:
    """Latest world poses of the objects and the gripper, from the sim's body_poses stream."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._poses: dict[str, tuple[NDArray[np.float64], Rotation]] = {}
        self._tcp: deque[tuple[float, NDArray[np.float64]]] = deque(maxlen=30)

    def on_message(self, msg: Any) -> None:
        now = time.time()
        with self._lock:
            for tf in msg.transforms:
                position = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
                q = tf.rotation
                rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
                self._poses[tf.child_frame_id] = (position, rotation)
                if tf.child_frame_id == GRIPPER_BODY:
                    self._tcp.append((now, position + rotation.apply(TCP_OFFSET)))

    def position(self, name: str) -> NDArray[np.float64] | None:
        with self._lock:
            pose = self._poses.get(name)
        return None if pose is None else pose[0].copy()

    def gripper_axes(self) -> NDArray[np.float64] | None:
        """The gripper's x (across the jaws' sides), y (jaw travel) and z (approach) axes as columns."""
        with self._lock:
            pose = self._poses.get(GRIPPER_BODY)
        return None if pose is None else pose[1].as_matrix()

    def tcp(self) -> NDArray[np.float64] | None:
        with self._lock:
            return self._tcp[-1][1].copy() if self._tcp else None

    def tcp_speed(self, window_s: float = 0.3) -> float | None:
        """Mean tool-tip speed over the last ``window_s`` seconds, m/s."""
        with self._lock:
            samples = list(self._tcp)
        recent = [s for s in samples if s[0] >= samples[-1][0] - window_s] if samples else []
        if len(recent) < 2 or recent[-1][0] - recent[0][0] < 0.1:
            return None
        return float(np.linalg.norm(recent[-1][1] - recent[0][1]) / (recent[-1][0] - recent[0][0]))


class Stack:
    """hybrid-teleop-sim built in this process, plus the agent twin for ``llm``."""

    def __init__(self, with_agent: bool, log_dir: Path) -> None:
        self.with_agent = with_agent
        self.log_dir = log_dir
        self.bodies = Bodies()
        self.agent: AgentDecisionMaker | None = None
        self._coordinator: Any = None
        self._proxies: dict[str, Any] = {}
        self._unsubscribe: list[Callable[[], None]] = []

    def start(self) -> None:
        set_run_log_dir(self.log_dir)
        bodies = [*OBJECTS, GRIPPER_BODY]
        os.environ["MUJOCOSIMMODULE__TRACKED_BODIES"] = repr(bodies).replace("'", '"')
        blueprint = hybrid_teleop_sim
        if self.with_agent:
            # The detector and segmenter need room once the agent joins (see
            # xarm_grasp_agent in xarm/blueprints/agentic.py).
            blueprint = autoconnect(
                hybrid_teleop_sim,
                McpServer.blueprint(),
                McpClient.blueprint(
                    system_prompt=MANIPULATION_AGENT_SYSTEM_PROMPT + AGENT_PROMPT_EXTRA
                ),
            ).global_config(n_workers=6)
        parsed = BlueprintConfigParser(blueprint).parse(
            ["--headless", "true"],
            config_path=DEFAULT_CONFIG_PATH,
            environ=os.environ,
            global_overrides={},
        )
        self._coordinator = ModuleCoordinator.build(blueprint, parsed)
        self._proxies = {
            atom.module.__name__: self._coordinator.get_instance(atom.module)
            for atom in blueprint.active_blueprints
        }
        self._proxies["ControlCoordinator"] = self._proxies["HybridTeleopCoordinator"]
        self._subscribe("body_poses", self.bodies.on_message)
        if self.with_agent:
            self.agent = AgentDecisionMaker(lambda text: self.call("McpServer", "agent_send", text))
            self._subscribe("agent", self.agent.on_message)
            self._subscribe("agent_idle", self.agent.on_idle)
        deadline = time.monotonic() + 60.0
        while self.bodies.position(TARGET) is None or self.bodies.tcp() is None:
            if time.monotonic() > deadline:
                raise TimeoutError("no body poses from the sim after 60 s")
            time.sleep(0.2)

    def _subscribe(self, stream: str, callback: Callable[[Any], None]) -> None:
        transports = [t for (name, _), t in self._coordinator.transports.items() if name == stream]
        if not transports:
            raise RuntimeError(f"the stack has no {stream!r} stream")
        self._unsubscribe.append(transports[0].subscribe(callback))

    def stop(self) -> None:
        for unsubscribe in self._unsubscribe:
            with contextlib.suppress(Exception):
                unsubscribe()
        self._unsubscribe = []
        if self._coordinator is not None:
            self._coordinator.stop()
            self._coordinator = None

    def call(self, module: str, method: str, *args: Any, timeout: float = 60.0) -> Any:
        """Call an @rpc on a module by class name and wait up to ``timeout`` seconds."""
        proxy = self._proxies[module]
        name = f"{proxy.remote_name}/{method}"
        return proxy.rpc.call_sync(name, (list(args), {}), rpc_timeout=timeout)[0]

    def hold_status(self) -> OperatorHoldStatus | None:
        status = self.call(
            "ControlCoordinator", "task_invoke", OPERATOR_HOLD_TASK_NAME, "get_status"
        )
        return status if isinstance(status, OperatorHoldStatus) else None

    def restart_agent(self) -> None:
        """A fresh agent, so one trial's chat does not steer the next."""
        self._proxies["McpClient"] = self._coordinator.restart_module(
            McpClient, reload_source=False
        )

    # The skills the scripted decision-maker uses.

    def scan_objects(self, prompts: list[str]) -> SkillResult:
        return self.call("PickAndPlaceModule", "scan_objects", prompts, timeout=120.0)

    def pick_object(self, object_id: str) -> SkillResult:
        return self.call("PickAndPlaceModule", "pick_object", object_id, timeout=600.0)

    def request_operator(self, reason: str) -> SkillResult:
        return self.call("TeleopSkills", "request_operator", reason)


class _Watcher(threading.Thread):
    """A background thread that watches the stack during one trial until ``stop``."""

    def __init__(self, stack: Stack) -> None:
        super().__init__(daemon=True)
        self._stack = stack
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()
        self.join(timeout=2.0)


class HoldWatcher(_Watcher):
    """Polls the coordinator's hold status at 5 Hz and keeps the first time it is on."""

    t_on: float | None = None
    status: OperatorHoldStatus | None = None

    def run(self) -> None:
        while self.t_on is None and not self._stop_event.wait(0.2):
            with contextlib.suppress(Exception):
                status = self._stack.hold_status()
                if status is not None and status.on:
                    self.t_on, self.status = time.time(), status


class Injector(_Watcher):
    """Breaks the pick at the trial's moment by moving the target with the sim's move_object.

    Moved and removed: when the tool tip first comes within 12 cm of the
    object, which is when the arm reaches the pregrasp pose and turns down.
    Missed grasp: in the last 3 cm of that descent, the object shifts
    ``miss_offset`` metres sideways (across the jaws' travel) so the jaws
    close on air.
    """

    def __init__(self, stack: Stack, case: Case, miss_offset: float) -> None:
        super().__init__(stack)
        self._case = case
        self._miss_offset = miss_offset
        self.t_inject: float | None = None
        self.detail = ""
        self.lowest_tcp_z = math.inf  # world z, metres

    def run(self) -> None:
        bodies = self._stack.bodies
        approached = False
        while not self._stop_event.wait(0.05):
            tcp, target, speed = bodies.tcp(), bodies.position(TARGET), bodies.tcp_speed()
            if tcp is None or target is None:
                continue
            self.lowest_tcp_z = min(self.lowest_tcp_z, float(tcp[2]))
            if self._case is Case.CLEAN or self.t_inject is not None:
                continue
            distance = float(np.linalg.norm(tcp - target))
            # The pregrasp pose is 10 cm back from the grasp, so the tip passes
            # 12 cm as it arrives there and turns down toward the object.
            approached = approached or distance < 0.12
            if not approached:
                continue
            if self._case is Case.MOVED:
                x, y = MOVE_TO
                where = f"({target[0]:.2f}, {target[1]:.2f}) to ({x:.2f}, {y:.2f})"
                self._move(x, y, f"moved from {where} at the pregrasp pose")
            elif self._case is Case.REMOVED:
                x, y = target[0] + REMOVE_DISTANCE, target[1]
                self._move(x, y, f"removed to ({x:.2f}, {y:.2f}) at the pregrasp pose")
            # Missed grasp: the descent ends 1-2 cm from the object's centre and
            # the jaws close right after, so shift it in the last few centimetres,
            # or as soon as the tip stops short of that.
            elif distance < 0.03 or (distance < 0.05 and speed is not None and speed < 0.02):
                x, y = self._sideways(target)
                shift = f"shifted {self._miss_offset * 100:.0f} cm sideways to ({x:.2f}, {y:.2f})"
                self._move(x, y, f"{shift} with the tool tip {distance * 100:.1f} cm from it")

    def _sideways(self, target: NDArray[np.float64]) -> tuple[float, float]:
        """A spot ``miss_offset`` away along the gripper's x axis, on the side with more room."""
        bodies = self._stack.bodies
        axes = bodies.gripper_axes()
        assert axes is not None
        across = axes[:2, 0] / max(float(np.linalg.norm(axes[:2, 0])), 1e-9)
        others = [p[:2] for n in OBJECTS if n != TARGET and (p := bodies.position(n)) is not None]

        def room(spot: NDArray[np.float64]) -> float:
            # Distance to the nearest other object; a spot off the table loses by 10 m.
            on_table = TABLE_X[0] <= spot[0] <= TABLE_X[1] and TABLE_Y[0] <= spot[1] <= TABLE_Y[1]
            nearest = min((float(np.linalg.norm(spot - o)) for o in others), default=1.0)
            return nearest - (0 if on_table else 10)

        best = max((target[:2] + sign * self._miss_offset * across for sign in (1, -1)), key=room)
        return float(best[0]), float(best[1])

    def _move(self, x: float, y: float, detail: str) -> None:
        moved = self._stack.call("MujocoSimModule", "move_object", TARGET, x, y)
        self.t_inject = time.time()
        self.detail = detail if moved else f"move_object refused: {detail}"


class Runner:
    """Runs the trials on one stack, restarting it when a trial or the scene reset breaks."""

    def __init__(self, decision_maker: str, out_dir: Path) -> None:
        self.dm_name = decision_maker
        self.out_dir = out_dir
        self.stack = Stack(with_agent=decision_maker == "llm", log_dir=out_dir / "stack")
        self.spawn: dict[str, NDArray[np.float64]] = {}
        self.trials: list[Trial] = []
        sha = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True
        )
        self.run_facts = f"Commit {sha.stdout.strip()}, target {TARGET} asked for as {PROMPT!r}."
        self.stem = out_dir / f"handoff_validation_{datetime.now():%Y-%m-%d}"

    def start_stack(self) -> None:
        self.stack.start()
        if not self.spawn:
            bodies = self.stack.bodies
            self.spawn = {n: p for n in OBJECTS if (p := bodies.position(n)) is not None}
        # The first scans after start-up can miss the camera's pose and find nothing.
        deadline = time.monotonic() + 60.0
        while not self.stack.scan_objects([PROMPT]).metadata.get("objects"):
            if time.monotonic() > deadline:
                break
            time.sleep(2.0)
        if self.stack.agent is not None:
            self._check_agent(self.stack.agent)

    def restart_stack(self, why: str) -> None:
        logger.warning(f"restarting the stack: {why}")
        self.stack.stop()
        time.sleep(5.0)
        self.stack = Stack(with_agent=self.dm_name == "llm", log_dir=self.out_dir / "stack")
        self.start_stack()

    def _check_agent(self, agent: AgentDecisionMaker) -> None:
        """Fall back to scripted when the agent cannot see the tools or the model does not answer."""
        adapter = McpAdapter(self.stack.call("McpServer", "mcp_url"))
        missing = {"scan_objects", "pick_object", "request_operator"}
        missing -= {tool["name"] for tool in adapter.list_tools()}
        reply = (
            "" if missing else agent.ask("Reply with the single word ready. Do not call any tools.")
        )
        if missing or not reply or "failed with" in reply.lower():
            why = (
                f"tools missing: {sorted(missing)}" if missing else f"model reply: {reply[:300]!r}"
            )
            logger.warning(f"LLM run could not start ({why}); using scripted")
            self.run_facts += f" LLM run could not start ({why}); used scripted."
            self.dm_name = "scripted"

    def decision_maker(self) -> DecisionMaker:
        if self.dm_name == "llm" and self.stack.agent is not None:
            try:
                self.stack.restart_agent()
            except Exception as exc:
                logger.warning(f"agent restart failed, keeping the old one: {exc}")
            return self.stack.agent
        return ScriptedDecisionMaker(self.stack)

    def put_back(self, trial: Trial | None = None, lowest_tcp_z: float = math.inf) -> None:
        """Release the hold, put a held object down, send the arm home, and put the objects back.

        The sim's own reset is not used: its "home" keyframe puts the scene's
        loose objects at the world origin, inside the robot's pedestal, and
        they fly off. Sending the arm home through the planner also keeps the
        coordinator's last command where the arm is.

        Args:
            trial: The trial just run; a complete pick's object is put down with place_at.
            lowest_tcp_z: Lowest tool-tip height in that trial, world z, metres (the grasp height).
        """
        call = self.stack.call
        status = self.stack.hold_status()
        if status is not None and status.on:
            call("ControlCoordinator", "task_invoke", OPERATOR_HOLD_TASK_NAME, "acknowledge")
        if trial is not None and trial.attempt.pick_message.startswith(PICK_COMPLETE):
            x, y = self.spawn[TARGET][:2]
            z = (lowest_tcp_z if lowest_tcp_z < math.inf else 0.17) + 0.01
            placed = call("PickAndPlaceModule", "place_at", float(x), float(y), z, timeout=300.0)
            if not str(placed).startswith("Place complete"):
                return self.restart_stack(f"place_at did not finish: {placed}")
        home = call("ManipulationSkills", "go_init", timeout=120.0)
        if str(home).startswith(("No plan", "Execution ended")):
            return self.restart_stack(f"go_init: {home}")
        call("ManipulationModule", "set_gripper_position", 1.0, None)
        for name, spawn in self.spawn.items():
            now = self.stack.bodies.position(name)
            if now is None or np.linalg.norm(now - spawn) > 0.005:
                call("MujocoSimModule", "move_object", name, *map(float, spawn))
        time.sleep(2.0)

    def run_trial(self, number: int, case: Case) -> tuple[Trial, float]:
        """One trial, and the lowest tool-tip height in it (world z, metres)."""
        miss_offset = MISS_OFFSETS[sum(t.case is case for t in self.trials) % len(MISS_OFFSETS)]
        dm = self.decision_maker()
        watcher = HoldWatcher(self.stack)
        injector = Injector(self.stack, case, miss_offset)
        watcher.start()
        injector.start()
        notes = []
        try:
            attempt = dm.attempt(PROMPT)
        except Exception as exc:
            attempt = Attempt()
            notes.append(f"decision-maker raised {type(exc).__name__}: {exc}")
        if attempt.requested:  # give the coordinator 10 s to show the hold on
            deadline = time.time() + 10.0
            while watcher.t_on is None and time.time() < deadline:
                time.sleep(0.1)
        injector.stop()
        watcher.stop()
        target, tcp = self.stack.bodies.position(TARGET), self.stack.bodies.tcp()
        # Held: the object is 3 cm above where it started and within 8 cm of the tool tip.
        held = bool(
            target is not None
            and tcp is not None
            and target[2] > self.spawn[TARGET][2] + 0.03
            and np.linalg.norm(target - tcp) < 0.08
        )
        if case is not Case.CLEAN and injector.t_inject is None:
            notes.append("the injection never fired")
        if attempt.requested and watcher.t_on is None:
            notes.append("asked for the hold but the coordinator never showed it on")
        status = watcher.status
        trial = Trial(
            number=number,
            case=case,
            decision_maker=dm.name,
            held=held,
            attempt=attempt,
            t_inject=injector.t_inject,
            t_hold=watcher.t_on,
            route=status.route if status else "",
            hold_reason=status.reason if status else "",
            injection=injector.detail,
            note="; ".join(notes),
        )
        return trial, injector.lowest_tcp_z

    def run(self, plan: Sequence[Case]) -> None:
        self.start_stack()
        self.put_back()
        for number, case in enumerate(plan, start=1):
            logger.info(f"trial {number}/{len(plan)}: {case.value}")
            try:
                trial, lowest_tcp_z = self.run_trial(number, case)
            except Exception as exc:
                self.restart_stack(f"trial {number} crashed: {exc}")
                continue
            self.trials.append(trial)
            logger.info(write_report(self.trials, self.stem, self.run_facts))
            if number < len(plan):
                try:
                    self.put_back(trial, lowest_tcp_z)
                except Exception as exc:
                    self.restart_stack(f"putting the scene back failed: {exc}")
        write_report(self.trials, self.stem, self.run_facts)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--decision-maker", choices=("scripted", "llm"), default="scripted")
    parser.add_argument("--cases", default="", help="comma-separated cases instead of the plan")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    load_dotenv()
    if args.decision_maker == "llm" and not os.environ.get("OPENAI_API_KEY"):
        logger.warning("OPENAI_API_KEY is not set; using the scripted decision-maker")
        args.decision_maker = "scripted"
    out_dir = args.out or Path.home() / "handoff-validation" / f"{datetime.now():%Y%m%d-%H%M%S}"
    out_dir.mkdir(parents=True, exist_ok=True)
    logger.info(f"output: {out_dir}")
    runner = Runner(args.decision_maker, out_dir)
    try:
        runner.run([Case(c) for c in args.cases.split(",")] if args.cases else TRIAL_PLAN)
    finally:
        runner.stack.stop()


if __name__ == "__main__":
    main()
