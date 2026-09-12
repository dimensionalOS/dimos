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

"""Exercise the actual interactive MCP skills on a native ACT simulation."""

import argparse
from collections.abc import Callable
from dataclasses import replace
import json
import os
from pathlib import Path
from queue import Empty, Queue
import time
from typing import Any

from langchain_core.messages import AIMessage, BaseMessage
import requests

from dimos.agents.mcp.mcp_client import McpClient
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.transport_factory import make_transport
from dimos.imitation.policy.lerobot.module import R1ProObjectPackingPolicy
from dimos.robot.galaxea.r1pro.object_agent_blueprint import (
    r1pro_objects_sim,
    r1pro_objects_sim_agent,
)
from dimos.robot.galaxea.r1pro.object_packing_sim import R1ProObjectPackingSim
from dimos.robot.galaxea.r1pro.object_skills import R1ProObjectSkills
from dimos.robot.galaxea.r1pro.sim_session import reserve_demo_session


def agent_turn(
    message: str,
    scene: Callable[[], dict[str, Any]],
) -> dict[str, Any]:
    """Exercise the same human-input and response streams used by humancli."""
    messages: Queue[BaseMessage] = Queue()
    responses = make_transport("/agent")
    user_input = make_transport("/human_input")
    unsubscribe = responses.subscribe(messages.put)
    history = []
    try:
        user_input.start()
        user_input.publish(message)
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            try:
                response = messages.get(timeout=1)
            except Empty:
                continue
            history.append(response.model_dump(mode="json"))
            if isinstance(response, AIMessage) and not response.tool_calls:
                return dict(message=message, history=history, scene=scene())
        raise RuntimeError("Agent turn did not finish; inspect its trace and action files")
    finally:
        unsubscribe()
        responses.stop()
        user_input.stop()


def run(args: argparse.Namespace) -> None:
    args.output.mkdir(parents=True, exist_ok=True)
    with reserve_demo_session(args.zenoh_scout_addr, args.output):
        atoms = []
        source = r1pro_objects_sim_agent if args.agent_message else r1pro_objects_sim
        for atom in source.blueprints:
            changes = {}
            if atom.module is R1ProObjectPackingSim:
                changes = dict(
                    output=args.output / "session",
                    seed=args.seed,
                    headless=not args.viewer,
                    scene_package=args.scene_package,
                )
            elif atom.module is R1ProObjectPackingPolicy and args.artifact is not None:
                changes = dict(artifact=str(args.artifact))
            elif atom.module is R1ProObjectSkills:
                changes = dict(pick_timeout=args.seconds)
            elif atom.module is McpClient:
                changes = dict(
                    mcp_server_url=f"http://127.0.0.1:{args.mcp_port}/mcp",
                    trace_dir=args.output / "agent-trace",
                    model_fixture=str(args.model_fixture) if args.model_fixture else None,
                )
            atoms.append(replace(atom, kwargs={**atom.kwargs, **changes}))
        blueprint = replace(source, blueprints=tuple(atoms)).global_config(
            zenoh_scout_addr=args.zenoh_scout_addr, mcp_port=args.mcp_port
        )
        report: dict[str, Any] = dict(seed=args.seed, actions=[])
        coordinator = ModuleCoordinator.build(blueprint)
        try:
            with requests.Session() as client:
                url = f"http://127.0.0.1:{args.mcp_port}/mcp"

                def call(tool: str, arguments: dict[str, Any] | None = None) -> dict[str, Any]:
                    response = client.post(
                        url,
                        json={
                            "jsonrpc": "2.0",
                            "id": 1,
                            "method": "tools/call",
                            "params": {"name": tool, "arguments": arguments or {}},
                        },
                        timeout=40,
                    )
                    response.raise_for_status()
                    data = response.json()
                    if "error" in data:
                        raise RuntimeError(data["error"])
                    result = data["result"]
                    if result.get("isError"):
                        raise RuntimeError(result)
                    value: dict[str, Any] = json.loads(result["content"][0]["text"])
                    return value

                def wait() -> dict[str, Any]:
                    deadline = time.monotonic() + 160
                    while time.monotonic() < deadline:
                        outcome = call("wait_for_action", {"seconds": 20})
                        if outcome["state"] != "running":
                            return outcome
                    raise RuntimeError("Interactive action did not finish")

                initial = call("get_scene")
                report["initial"] = initial
                report["unsupported_arm"] = call(
                    "pick_object", {"object": "nearest", "arm": "left"}
                )
                assert report["unsupported_arm"]["accepted"] is False
                if args.agent_message:
                    report["agent"] = agent_turn(args.agent_message, lambda: call("get_scene"))
                for selector in [] if args.agent_message else args.selectors:
                    before = call("get_scene")
                    accepted = call("pick_object", {"object": selector, "arm": "right"})
                    if args.cancel_after is not None and accepted["accepted"]:
                        time.sleep(args.cancel_after)
                        report["cancel"] = call("stop_action")
                    outcome = wait() if accepted["accepted"] else accepted
                    row = dict(
                        selector=selector,
                        before=before,
                        accepted=accepted,
                        outcome=outcome,
                        after=call("get_scene"),
                    )
                    report["actions"].append(row)
                    (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
                    print(json.dumps(dict(selector=selector, outcome=outcome)), flush=True)
                    if outcome.get("recovery_required"):
                        break
                if args.reset_check:
                    accepted = call("reset_scene")
                    assert accepted["accepted"]
                    report["reset"] = wait()
                    report["after_reset"] = call("get_scene")
                    assert report["reset"]["success"]
                    assert all(not r["inside"] for r in report["after_reset"]["objects"])
                report["finished"] = True
        finally:
            (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
            coordinator.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-package", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=210000)
    parser.add_argument("--seconds", type=float, default=50)
    parser.add_argument("--selectors", nargs="+", default=["rightmost", "nearest"])
    parser.add_argument("--zenoh-scout-addr", required=True)
    parser.add_argument("--mcp-port", type=int, required=True)
    parser.add_argument("--reset-check", action="store_true")
    parser.add_argument("--cancel-after", type=float)
    parser.add_argument("--agent-message")
    parser.add_argument("--model-fixture", type=Path, help="Local recorded responses; no model API")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--artifact", type=Path)
    args = parser.parse_args()
    if args.model_fixture and (not args.agent_message or os.getenv("RECORD")):
        parser.error("Fixture playback needs --agent-message and RECORD must be unset")
    if args.seconds <= 0 or (args.cancel_after is not None and args.cancel_after < 0):
        parser.error("Use a positive timeout and a nonnegative cancellation delay")
    run(args)


if __name__ == "__main__":
    main()
