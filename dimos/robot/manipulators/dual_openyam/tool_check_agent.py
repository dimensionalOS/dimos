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

"""Check bimanual agent completion through MCP; live mode calls the configured model."""

import argparse
from contextlib import ExitStack
from dataclasses import replace
import json
import os
from pathlib import Path
from queue import Empty, Queue
import threading
import time
from typing import Any, cast

from langchain_core.messages import BaseMessage, HumanMessage, ToolMessage
import numpy as np

from dimos.agents.mcp.mcp_client import McpClient
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.dual_openyam.blueprints.agentic import dual_openyam_sim_agent
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM_HOME_PER_ARM
from dimos.robot.manipulators.dual_openyam.sim import DualOpenYamSimModule
from dimos.robot.manipulators.dual_openyam.sim_demo import SimDemoSkills
from dimos.robot.manipulators.dual_openyam.tool_generate_demos import require
from dimos.visualization.rerun.bridge import RerunBridgeModule

TRIAL_PROMPT = (
    "Reset the simulated scene, then put bottle_1 and bottle_4 in the bin using the "
    "appropriate arm for each. Return both arms home and verify both are in the bin. "
    "Do not retry a failed pick/place during this acceptance trial; report the failure."
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--mcp-port", type=int, default=global_config.mcp_port)
    parser.add_argument("--zenoh-scout-addr", default="")
    parser.add_argument("--fixture", type=Path, help="Local model playback, never a live LLM check")
    parser.add_argument("--no-cameras", action="store_true", help="CPU-only MCP diagnostic")
    args = parser.parse_args()
    messages_path = args.report.with_suffix(".messages.jsonl")
    if args.episodes < 1 or args.timeout <= 0 or not 1 <= args.mcp_port <= 65535:
        parser.error("episodes, timeout, and mcp-port must be positive and valid")
    if args.report.exists() or messages_path.exists():
        parser.error("report or message log already exists")
    if args.fixture and (not args.fixture.is_file() or os.getenv("RECORD")):
        parser.error("fixture must exist, and RECORD must be unset for local playback")
    args.report.parent.mkdir(parents=True, exist_ok=True)
    base = dual_openyam_sim_agent.disabled_modules(RerunBridgeModule)
    atoms = []
    for atom in base.blueprints:
        overrides: dict[str, Any] = {}
        if atom.module is McpClient:
            overrides = {
                "mcp_server_url": f"http://{global_config.listen_host}:{args.mcp_port}/mcp",
                "trace_dir": args.report.with_suffix(".traces"),
                "model_fixture": str(args.fixture.resolve()) if args.fixture else None,
            }
        elif atom.module is DualOpenYamSimModule and args.no_cameras:
            overrides = {"extra_cameras": []}
        atoms.append(replace(atom, kwargs={**atom.kwargs, **overrides}))
    blueprint = replace(base, blueprints=tuple(atoms)).global_config(
        viewer="none", n_workers=4, mcp_port=args.mcp_port, zenoh_scout_addr=args.zenoh_scout_addr
    )
    coordinator = None
    tool_failures: Queue[dict[str, Any]] = Queue()
    done = threading.Event()
    busy = threading.Event()
    successes = 0

    def idle_changed(idle: bool) -> None:
        if not idle:
            busy.set()
            done.clear()
        elif busy.is_set():
            done.set()

    with ExitStack() as stack:
        report = stack.enter_context(args.report.open("x"))
        messages = stack.enter_context(messages_path.open("x"))

        def record_message(message: BaseMessage) -> None:
            messages.write(message.model_dump_json() + "\n")
            messages.flush()
            if isinstance(message, ToolMessage) and isinstance(message.content, str):
                try:
                    result = json.loads(message.content)
                except json.JSONDecodeError:
                    return
                if isinstance(result, dict) and result.get("success") is False:
                    tool_failures.put({"tool": message.name, "result": result})

        coordinator = ModuleCoordinator.build(blueprint)
        stack.callback(coordinator.stop)
        stack.callback(coordinator.transports[("agent_idle", bool)].subscribe(idle_changed))
        stack.callback(coordinator.transports[("agent", BaseMessage)].subscribe(record_message))
        agent = cast("McpClient", coordinator.get_instance(McpClient))
        demo = cast("SimDemoSkills", coordinator.get_instance(SimDemoSkills))
        manipulation = cast("ManipulationModule", coordinator.get_instance(ManipulationModule))
        for trial in range(args.episodes):
            # Every trial begins with empty bin state, even if the model omits reset.
            require(demo.reset_scene())
            busy.clear()
            done.clear()
            agent.add_message(HumanMessage(content=TRIAL_PROMPT))
            if not done.wait(args.timeout):
                report.write(
                    json.dumps({"trial": trial, "success": False, "error": "timeout"}) + "\n"
                )
                return 1
            time.sleep(1.0)
            scene = require(demo.inspect_sim_scene()).metadata
            groups = manipulation.get_state().groups
            home = {}
            for side in ("left", "right"):
                joints = groups[f"{side}_manipulator"].joints
                home[side] = joints is not None and bool(
                    np.allclose(joints.position, DUAL_OPENYAM_HOME_PER_ARM, atol=0.05, rtol=0)
                )
            failures = []
            while True:
                try:
                    failures.append(tool_failures.get_nowait())
                except Empty:
                    break
            success = bool(
                not failures
                and scene["inside_bin"]["bottle_1"]
                and scene["inside_bin"]["bottle_4"]
                and all(home.values())
            )
            row = {
                "trial": trial,
                "success": success,
                "fixture": args.fixture is not None,
                "cameras": not args.no_cameras,
                "home": home,
                "tool_failures": failures,
                "scene": scene,
            }
            successes += int(success)
            report.write(json.dumps(row) + "\n")
            report.flush()
            print(json.dumps(row), flush=True)
    return 0 if successes == args.episodes else 1


if __name__ == "__main__":
    raise SystemExit(main())
