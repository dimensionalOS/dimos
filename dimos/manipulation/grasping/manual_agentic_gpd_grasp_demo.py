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

"""Manual RPC driver for the agent-facing GPD MuJoCo grasp demo."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
import time
from typing import Protocol, cast

from dimos.agents.skill_result import SkillResult
from dimos.core.rpc_client import RPCClient
from dimos.manipulation.agentic_manipulation_module import AgenticGraspManipulationModule
from dimos.manipulation.skill_errors import ManipulationSkillError


class AgenticGraspToolClient(Protocol):
    def scan_objects(self, target_name: str = "object") -> SkillResult[ManipulationSkillError]: ...
    def generate_grasps(
        self,
        target_name: str = "object",
        object_id: str | None = None,
        filter_collisions: bool = False,
    ) -> SkillResult[ManipulationSkillError]: ...
    def execute_grasp(
        self,
        candidate_index: int = 0,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]: ...
    def stop_rpc_client(self) -> None: ...


class ManualAgenticGpdGraspDemoError(RuntimeError):
    """Raised when a manual GPD grasp demo step fails."""


def run_manual_agentic_gpd_grasp_sequence(
    target_name: str = "sphere",
    candidate_index: int = 0,
    robot_name: str | None = None,
    scan_attempts: int = 90,
    retry_interval_s: float = 0.5,
    client: AgenticGraspToolClient | None = None,
) -> Sequence[SkillResult[ManipulationSkillError]]:
    """Run scan -> generate -> execute against a running manual agentic GPD demo.

    Start `manual-agentic-gpd-mujoco-grasp-demo` in another process before calling this helper.
    The helper uses the agent-facing tool surface directly over RPC and does not require
    `McpClient`, an LLM, or an API key.
    """
    owns_client = client is None
    tool_client = client or cast(
        "AgenticGraspToolClient",
        RPCClient.remote(AgenticGraspManipulationModule),
    )
    try:
        scan = _retry_skill(
            lambda: tool_client.scan_objects(target_name),
            attempts=scan_attempts,
            retry_interval_s=retry_interval_s,
        )
        _raise_if_failed("scan_objects", scan)

        generate = tool_client.generate_grasps(target_name)
        _raise_if_failed("generate_grasps", generate)
        if int(generate.metadata.get("candidate_count", 0)) < 1:
            raise ManualAgenticGpdGraspDemoError(
                "generate_grasps succeeded without reporting cached candidates."
            )

        execute = tool_client.execute_grasp(candidate_index, robot_name)
        _raise_if_failed("execute_grasp", execute)
        return [scan, generate, execute]
    finally:
        if owns_client:
            tool_client.stop_rpc_client()


def _retry_skill(
    call: Callable[[], SkillResult[ManipulationSkillError]],
    attempts: int,
    retry_interval_s: float,
) -> SkillResult[ManipulationSkillError]:
    last_result: SkillResult[ManipulationSkillError] | None = None
    for _ in range(max(1, attempts)):
        result = call()
        if result.is_success():
            return result
        last_result = result
        time.sleep(max(0.0, retry_interval_s))
    if last_result is None:
        raise ManualAgenticGpdGraspDemoError("No scan attempt was made.")
    return last_result


def _raise_if_failed(step_name: str, result: SkillResult[ManipulationSkillError]) -> None:
    if result.is_success():
        return
    raise ManualAgenticGpdGraspDemoError(
        f"{step_name} failed: {result.error_code or 'UNKNOWN'}: {result.message}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run scan_objects -> generate_grasps -> execute_grasp over RPC."
    )
    parser.add_argument("--target-name", default="sphere")
    parser.add_argument("--candidate-index", type=int, default=0)
    parser.add_argument("--robot-name", default=None)
    parser.add_argument("--scan-attempts", type=int, default=90)
    parser.add_argument("--retry-interval-s", type=float, default=0.5)
    args = parser.parse_args()

    results = run_manual_agentic_gpd_grasp_sequence(
        target_name=args.target_name,
        candidate_index=args.candidate_index,
        robot_name=args.robot_name,
        scan_attempts=args.scan_attempts,
        retry_interval_s=args.retry_interval_s,
    )
    for result in results:
        print(result.message)


if __name__ == "__main__":
    main()
