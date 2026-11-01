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
.env); without it the run uses ``scripted``.
"""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime
import os
from pathlib import Path
import subprocess
import threading
import time
from typing import Any

from dotenv import load_dotenv
import numpy as np
from numpy.typing import NDArray
import requests
from scipy.spatial.transform import Rotation

from dimos.agents.skill_result import SkillResult
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OPERATOR_HOLD_TASK_NAME,
    OperatorHoldStatus,
)
from dimos.manipulation.evals.handoff_validation import (
    TRIAL_PLAN,
    AgentDecisionMaker,
    Attempt,
    Case,
    DecisionMaker,
    ScriptedDecisionMaker,
    Trial,
    render_csv,
    render_markdown,
    summarize,
)

BLUEPRINT = "hybrid-teleop-sim"
OBJECTS = ("apple", "orange", "cup")
GRIPPER_BODY = "xarm_gripper_base_link"
TCP_OFFSET = np.array([0.0, 0.0, 0.172])  # link_tcp in the gripper base frame, metres
PREGRASP_OFFSET = 0.10  # PickAndPlaceModule's default pregrasp_offset, metres
TABLE_X = (0.32, 0.58)  # table top, world frame, metres, less a 2 cm border
TABLE_Y = (-0.18, 0.18)
REMOVE_DISTANCE = 2.0  # "removed" puts the object this far (metres, +x) off the table
MOVE_TO = {"apple": (0.36, -0.13)}  # "moved": a free spot on the table, world x, y
MISS_OFFSETS = (0.05, 0.06, 0.04, 0.05)  # "missed grasp" sideways shifts, metres, in turn

AGENT_PROMPT_EXTRA = """
# Asking a person for help
- **request_operator <reason>**: Stop the robot and ask a human operator for help.

If pick_object does not report "Pick complete", or you cannot finish the task, call
request_operator once with a short plain reason that says what failed (quote what
the pick reported), then stop. Do not retry the pick and do not move the arm
yourself. After "Pick complete", do not call request_operator; reply "done".
"""


def _status(value: Any) -> OperatorHoldStatus | None:
    return value if isinstance(value, OperatorHoldStatus) else None


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
                rotation = Rotation.from_quat(
                    [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]
                )
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
        if len(samples) < 2:
            return None
        recent = [s for s in samples if s[0] >= samples[-1][0] - window_s]
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
        self._unsubscribe: list[Any] = []

    def start(self) -> None:
        from dimos.cli.commands.lifecycle import DEFAULT_CONFIG_PATH
        from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
        from dimos.core.coordination.blueprints import autoconnect
        from dimos.core.coordination.module_coordinator import ModuleCoordinator
        from dimos.robot.manipulators.xarm.blueprints.hybrid_teleop_sim import hybrid_teleop_sim
        from dimos.utils.logging_config import set_run_log_dir

        set_run_log_dir(self.log_dir)
        os.environ["MUJOCOSIMMODULE__TRACKED_BODIES"] = repr([*OBJECTS, GRIPPER_BODY]).replace(
            "'", '"'
        )
        blueprint = hybrid_teleop_sim
        if self.with_agent:
            from dimos.agents.mcp.mcp_client import McpClient
            from dimos.agents.mcp.mcp_server import McpServer
            from dimos.robot.manipulators.common.agent_prompts import (
                MANIPULATION_AGENT_SYSTEM_PROMPT,
            )

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
            self.agent = AgentDecisionMaker(
                lambda text: self.call("McpServer", "agent_send", text), timeout_s=300.0
            )
            self._subscribe("agent", self.agent.on_message)
            self._subscribe("agent_idle", self.agent.on_idle)
        deadline = time.monotonic() + 60.0
        while self.bodies.position("apple") is None or self.bodies.tcp() is None:
            if time.monotonic() > deadline:
                raise TimeoutError("no body poses from the sim after 60 s")
            time.sleep(0.2)

    def _subscribe(self, stream: str, callback: Any) -> None:
        transports = [t for (name, _), t in self._coordinator.transports.items() if name == stream]
        if not transports:
            raise RuntimeError(f"the stack has no {stream!r} stream")
        self._unsubscribe.append(transports[0].subscribe(callback))

    def stop(self) -> None:
        for unsubscribe in self._unsubscribe:
            try:
                unsubscribe()
            except Exception:
                pass
        self._unsubscribe = []
        if self._coordinator is not None:
            self._coordinator.stop()
            self._coordinator = None

    def call(
        self, module: str, method: str, *args: Any, timeout: float = 60.0, **kwargs: Any
    ) -> Any:
        """Call an @rpc on a module by class name and wait up to ``timeout`` seconds."""
        proxy = self._proxies[module]
        result, _ = proxy.rpc.call_sync(
            f"{proxy.remote_name}/{method}", (list(args), kwargs), rpc_timeout=timeout
        )
        return result

    def hold_status(self) -> OperatorHoldStatus | None:
        return _status(
            self.call("ControlCoordinator", "task_invoke", OPERATOR_HOLD_TASK_NAME, "get_status")
        )

    def restart_agent(self) -> None:
        """A fresh agent, so one trial's chat does not steer the next."""
        from dimos.agents.mcp.mcp_client import McpClient

        self._proxies["McpClient"] = self._coordinator.restart_module(
            McpClient, reload_source=False
        )

    def mcp_tools(self) -> list[str]:
        url = self.call("McpServer", "mcp_url")
        body = {"jsonrpc": "2.0", "id": 1, "method": "tools/list"}
        tools = requests.post(url, json=body, timeout=30).json()["result"]["tools"]
        return sorted(tool["name"] for tool in tools)


class RpcPickSkills:
    def __init__(self, stack: Stack) -> None:
        self._stack = stack

    def scan_objects(self, prompts: list[str]) -> SkillResult:
        return self._stack.call("PickAndPlaceModule", "scan_objects", prompts, timeout=120.0)

    def pick_object(self, object_id: str) -> SkillResult:
        return self._stack.call("PickAndPlaceModule", "pick_object", object_id, timeout=600.0)

    def request_operator(self, reason: str) -> SkillResult:
        return self._stack.call("TeleopSkills", "request_operator", reason)


class HoldWatcher(threading.Thread):
    """Polls the coordinator's hold status at 5 Hz and keeps the first time it is on."""

    def __init__(self, stack: Stack) -> None:
        super().__init__(daemon=True)
        self._stack = stack
        self._stop_event = threading.Event()
        self.t_on: float | None = None
        self.status: OperatorHoldStatus | None = None

    def run(self) -> None:
        while not self._stop_event.is_set() and self.t_on is None:
            try:
                status = self._stack.hold_status()
            except Exception:
                status = None
            if status is not None and status.on:
                self.t_on, self.status = time.time(), status
            self._stop_event.wait(0.2)

    def stop(self) -> None:
        self._stop_event.set()
        self.join(timeout=2.0)


class Injector(threading.Thread):
    """Breaks the pick at the trial's moment by moving the target in the sim.

    Moved and removed: when the tool tip first comes within 12 cm of the
    object, which is when the arm reaches the pregrasp pose and turns down.
    Missed grasp: in the last 3 cm of that descent, the object shifts sideways
    (across the jaws' travel) so the jaws close on air.
    """

    def __init__(self, stack: Stack, case: Case, target: str, miss_offset: float) -> None:
        super().__init__(daemon=True)
        self._stack = stack
        self._case = case
        self._target = target
        self._miss_offset = miss_offset
        self._stop_event = threading.Event()
        self.t_inject: float | None = None
        self.detail = ""
        self.lowest_tcp_z: float | None = None

    def run(self) -> None:
        bodies = self._stack.bodies
        approached = False
        while not self._stop_event.wait(0.05):
            tcp, target, speed = bodies.tcp(), bodies.position(self._target), bodies.tcp_speed()
            if tcp is None or target is None:
                continue
            self.lowest_tcp_z = (
                tcp[2] if self.lowest_tcp_z is None else min(self.lowest_tcp_z, tcp[2])
            )
            if self._case is Case.CLEAN or self.t_inject is not None:
                continue
            distance = float(np.linalg.norm(tcp - target))
            # The pregrasp pose is 10 cm back from the grasp, so the tip passes
            # 12 cm as it arrives there and turns down toward the object.
            approached = approached or distance < 0.12
            if not approached:
                continue
            if self._case is Case.MOVED:
                x, y = MOVE_TO[self._target]
                where = f"({target[0]:.2f}, {target[1]:.2f}) to ({x:.2f}, {y:.2f})"
                self._move(x, y, f"moved from {where} as the arm reached the pregrasp pose")
            elif self._case is Case.REMOVED:
                x, y = target[0] + REMOVE_DISTANCE, target[1]
                self._move(
                    x, y, f"removed to ({x:.2f}, {y:.2f}) as the arm reached the pregrasp pose"
                )
            # Missed grasp: the descent ends 1-2 cm from the object's centre and
            # the jaws close right after, so shift it in the last few centimetres,
            # or as soon as the tip stops short of that.
            elif distance < 0.03 or (distance < 0.05 and speed is not None and speed < 0.02):
                x, y = self._sideways(target)
                self._move(
                    x,
                    y,
                    f"shifted {self._miss_offset * 100:.0f} cm sideways to ({x:.2f}, {y:.2f}) "
                    f"with the tool tip {distance * 100:.1f} cm from it, jaws still open",
                )

    def _sideways(self, target: NDArray[np.float64]) -> tuple[float, float]:
        """A spot ``miss_offset`` away along the gripper's x axis, on the side with more room."""
        axes = self._stack.bodies.gripper_axes()
        assert axes is not None
        across = np.array([axes[0, 0], axes[1, 0]])
        across /= max(float(np.linalg.norm(across)), 1e-9)
        others = [
            p[:2]
            for name in OBJECTS
            if name != self._target and (p := self._stack.bodies.position(name)) is not None
        ]

        def room(spot: NDArray[np.float64]) -> float:
            on_table = TABLE_X[0] <= spot[0] <= TABLE_X[1] and TABLE_Y[0] <= spot[1] <= TABLE_Y[1]
            return (min(float(np.linalg.norm(spot - o)) for o in others) if others else 1.0) - (
                0 if on_table else 10
            )

        spots = [target[:2] + sign * self._miss_offset * across for sign in (1.0, -1.0)]
        best = max(spots, key=room)
        return float(best[0]), float(best[1])

    def _move(self, x: float, y: float, detail: str) -> None:
        moved = self._stack.call("MujocoSimModule", "move_object", self._target, x, y)
        self.t_inject = time.time()
        self.detail = detail if moved else f"move_object refused: {detail}"

    def stop(self) -> None:
        self._stop_event.set()
        self.join(timeout=2.0)


class Runner:
    def __init__(self, args: argparse.Namespace, out_dir: Path) -> None:
        self.args = args
        self.out_dir = out_dir
        self.dm_name = args.decision_maker
        self.fallback_note = ""
        self.stack = Stack(with_agent=self.dm_name == "llm", log_dir=out_dir / "stack")
        self.spawn: dict[str, NDArray[np.float64]] = {}
        self.trials: list[Trial] = []
        self.date = datetime.now().strftime("%Y-%m-%d")

    # Stack and scene

    def start_stack(self) -> None:
        print("starting the stack", flush=True)
        self.stack.start()
        if not self.spawn:
            self.spawn = {
                name: p for name in OBJECTS if (p := self.stack.bodies.position(name)) is not None
            }
        # The first scans after start-up can miss the camera's pose and find nothing.
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            scan = self.stack.call("PickAndPlaceModule", "scan_objects", [self.args.prompt])
            if scan.metadata.get("objects"):
                break
            time.sleep(2.0)
        if self.stack.with_agent:
            self._check_agent()

    def restart_stack(self, why: str) -> None:
        print(f"restarting the stack: {why}", flush=True)
        self.stack.stop()
        time.sleep(5.0)
        self.stack = Stack(with_agent=self.dm_name == "llm", log_dir=self.out_dir / "stack")
        self.start_stack()

    def _check_agent(self) -> None:
        """Fail over to scripted when the agent cannot see the tools or the model does not answer."""
        tools = self.stack.mcp_tools()
        missing = {"scan_objects", "pick_object", "request_operator"} - set(tools)
        error = f"the MCP server lacks {sorted(missing)}" if missing else ""
        if not error:
            assert self.stack.agent is not None
            reply = self.stack.agent.ask("Reply with the single word ready. Do not call any tools.")
            if not reply or "failed with" in reply.lower():
                error = f"the model did not answer: {reply[:300] or 'no reply in 120 s'}"
        print(f"agent check: tools {tools}; {error or 'model answered'}", flush=True)
        if error:
            self.fallback_note = f"LLM run could not start ({error}); used scripted."
            self.dm_name = "scripted"

    def decision_maker(self) -> DecisionMaker:
        if self.dm_name == "llm" and self.stack.agent is not None:
            try:
                self.stack.restart_agent()
            except Exception as exc:
                print(f"agent restart failed, keeping the old one: {exc}", flush=True)
            return self.stack.agent
        skills = RpcPickSkills(self.stack)
        return ScriptedDecisionMaker(skills, skills, pick_timeout_s=self.args.pick_timeout)

    def put_back(self, trial: Trial | None, lowest_tcp_z: float | None) -> None:
        """Release the hold, put a held object down, send the arm home, and put the objects back.

        The sim's own reset is not used: its "home" keyframe puts the scene's
        loose objects at the world origin, inside the robot's pedestal, and
        they fly off. Sending the arm home through the planner also keeps the
        coordinator's last command where the arm is.
        """
        call = self.stack.call
        status = self.stack.hold_status()
        if status is not None and status.on:
            call("ControlCoordinator", "task_invoke", OPERATOR_HOLD_TASK_NAME, "acknowledge")
        if trial is not None and trial.attempt.pick_message.startswith("Pick complete"):
            x, y = self.spawn[self.args.target][:2]
            z = (lowest_tcp_z or 0.17) + 0.01
            placed = call(
                "PickAndPlaceModule", "place_at", float(x), float(y), float(z), timeout=300.0
            )
            if not str(placed).startswith("Place complete"):
                self.restart_stack(f"place_at did not finish: {placed}")
                return
        home = call("ManipulationSkills", "go_init", timeout=120.0)
        if str(home).startswith(("No plan", "Execution ended")):
            self.restart_stack(f"go_init: {home}")
            return
        call("ManipulationModule", "set_gripper_position", 1.0, None)
        for name, spawn in self.spawn.items():
            now = self.stack.bodies.position(name)
            if now is None or np.linalg.norm(now - spawn) > 0.005:
                call("MujocoSimModule", "move_object", name, *map(float, spawn))
        time.sleep(2.0)

    # Trials

    def run_trial(self, number: int, case: Case) -> Trial:
        target = self.args.target
        injected_failures = sum(t.case is case for t in self.trials)
        miss_offset = MISS_OFFSETS[injected_failures % len(MISS_OFFSETS)]
        dm = self.decision_maker()
        watcher = HoldWatcher(self.stack)
        injector = Injector(self.stack, case, target, miss_offset)
        t_start = time.time()
        watcher.start()
        injector.start()
        note = ""
        try:
            attempt = dm.attempt(self.args.prompt)
        except Exception as exc:
            attempt = Attempt(log=[f"decision-maker raised {type(exc).__name__}: {exc}"])
            note = f"decision-maker raised {type(exc).__name__}: {exc}"
        if attempt.requested:
            deadline = time.time() + 10.0
            while watcher.t_on is None and time.time() < deadline:
                time.sleep(0.1)
        injector.stop()
        watcher.stop()
        if time.time() - t_start > self.args.trial_limit:
            note = (
                note + " " if note else ""
            ) + f"trial ran {time.time() - t_start:.0f} s, over the limit"
        target_now = self.stack.bodies.position(target)
        tcp = self.stack.bodies.tcp()
        held = bool(
            target_now is not None
            and tcp is not None
            and target_now[2] > self.spawn[target][2] + 0.03
            and np.linalg.norm(target_now - tcp) < 0.08
        )
        if case is not Case.CLEAN and injector.t_inject is None:
            note = (note + " " if note else "") + "the injection never fired"
        if attempt.requested and watcher.t_on is None:
            note = (
                note + " " if note else ""
            ) + "asked for the hold but the coordinator never showed it on"
        status = watcher.status
        trial = Trial(
            number=number,
            case=case,
            decision_maker=dm.name,
            held=held,
            attempt=attempt,
            t_start=t_start,
            t_inject=injector.t_inject,
            t_hold=watcher.t_on,
            route=status.route if status else "",
            hold_reason=status.reason if status else "",
            injection=injector.detail,
            note=note,
        )
        self._lowest_tcp_z = injector.lowest_tcp_z
        return trial

    def run(self) -> None:
        plan = list(TRIAL_PLAN)[: self.args.trials]
        if self.args.cases:
            plan = [Case(c) for c in self.args.cases.split(",")]
        self.start_stack()
        self.put_back(None, None)
        for number, case in enumerate(plan, start=1):
            print(f"trial {number}/{len(plan)}: {case.value}", flush=True)
            try:
                trial = self.run_trial(number, case)
            except Exception as exc:
                print(f"trial {number} crashed: {exc}", flush=True)
                self.restart_stack(f"trial {number} crashed: {exc}")
                continue
            self.trials.append(trial)
            print(
                f"  held={trial.held} hold={'on' if trial.t_hold else 'off'} "
                f"route={trial.route} pick={trial.attempt.pick_message[:120]!r} "
                f"stuck->hold={trial.stuck_to_hold_s} inject->hold={trial.inject_to_hold_s}",
                flush=True,
            )
            self.write_report()
            if number < len(plan):
                try:
                    self.put_back(trial, self._lowest_tcp_z)
                except Exception as exc:
                    self.restart_stack(f"putting the scene back failed: {exc}")
        self.write_report()
        self.stack.stop()

    def write_report(self) -> None:
        sha = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, check=False
        ).stdout.strip()
        used = sorted({t.decision_maker for t in self.trials}) or [self.dm_name]
        header = {
            "Date": self.date,
            "Commit": sha,
            "Blueprint": BLUEPRINT
            + (" + McpServer + McpClient (default model)" if "llm" in used else ""),
            "Decision-maker": ", ".join(used),
            "Target": f"{self.args.target} (asked for as {self.args.prompt!r})",
            "Trials": str(len(self.trials)),
            "Stack log": str(self.out_dir / "stack"),
        }
        if self.fallback_note:
            header["Note"] = self.fallback_note
        stem = self.out_dir / f"handoff_validation_{self.date}"
        stem.with_suffix(".md").write_text(render_markdown(self.trials, header))
        stem.with_suffix(".csv").write_text(render_csv(self.trials))
        for check in summarize(self.trials):
            print(
                f"  {check.name}: {check.value} ({'pass' if check.passed else 'fail'})", flush=True
            )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--decision-maker", choices=("scripted", "llm"), default="scripted")
    parser.add_argument("--trials", type=int, default=len(TRIAL_PLAN))
    parser.add_argument("--cases", default="", help="comma-separated cases instead of the plan")
    parser.add_argument("--target", default="apple", help="sim body to pick")
    parser.add_argument(
        "--prompt",
        default="red ball",
        help="what to ask for; the detector finds the sim's apple as a red ball, not an apple",
    )
    parser.add_argument("--pick-timeout", type=float, default=180.0, help="seconds")
    parser.add_argument("--trial-limit", type=float, default=300.0, help="seconds")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    load_dotenv()
    if args.decision_maker == "llm" and not os.environ.get("OPENAI_API_KEY"):
        print("OPENAI_API_KEY is not set; using the scripted decision-maker", flush=True)
        args.decision_maker = "scripted"
    out_dir = args.out or Path.home() / "handoff-validation" / datetime.now().strftime(
        "%Y%m%d-%H%M%S"
    )
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"output: {out_dir}", flush=True)
    runner = Runner(args, out_dir)
    try:
        runner.run()
    finally:
        runner.stack.stop()


if __name__ == "__main__":
    main()
