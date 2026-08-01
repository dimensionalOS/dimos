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

"""Command-line assembly for one attached DimSim/Pi smoke attempt."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import re
import sys

from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.benchmark.agent_eval.config import load_smoke_config, select_destination
from dimos.benchmark.agent_eval.dimos_control import AttachedDimosControl
from dimos.benchmark.agent_eval.dimsim_backend import DimSimEvaluationBackend
from dimos.benchmark.agent_eval.pi_process import NodePiSessionFactory
from dimos.benchmark.agent_eval.runner import LocalAgentEvalRunner

_ENDPOINT = re.compile(r"^https?://(?P<host>[A-Za-z0-9.-]+):(?P<port>[0-9]{1,5})/?$")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m dimos.benchmark.agent_eval")
    subcommands = parser.add_subparsers(dest="command", required=True)
    run = subcommands.add_parser("run", help="run one attached local smoke episode")
    run.add_argument("--config", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.command != "run":
        parser.error("unsupported command")
    try:
        return _run(args.config)
    except Exception as exc:
        print(f"agent-eval preflight failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


def _run(config_path: Path) -> int:
    loaded = load_smoke_config(config_path)
    selected = select_destination(
        Path(loaded.resolved.release_root),
        loaded.resolved.task_id,
    )
    if selected.contract.source.scene_id != loaded.resolved.dimsim.expected_scene_id:
        raise ValueError("configured DimSim scene does not match selected task")
    host, port = _parse_endpoint(loaded.resolved.dimsim.endpoint)
    adapter_entrypoint = (
        Path(__file__).resolve().parents[3]
        / "packages"
        / "pi-spatial-adapter"
        / "dist"
        / "code-policy-main.js"
    )
    if not adapter_entrypoint.is_file():
        raise FileNotFoundError(
            "Pi adapter is not built; run npm run build in packages/pi-spatial-adapter"
        )
    mcp = McpAdapter(
        loaded.resolved.mcp_endpoint,
        timeout=math.ceil(loaded.resolved.timeouts.mcp_call_s),
    )
    dimos_control = AttachedDimosControl.connect(loaded.resolved.timeouts.readiness_s)
    backend = DimSimEvaluationBackend(
        host=host,
        port=port,
        selected_contract=selected.contract,
        oracle_timeout_s=max(
            loaded.resolved.timeouts.readiness_s,
            loaded.resolved.timeouts.reset_s,
        ),
    )
    pi_factory = NodePiSessionFactory(
        command=("node", str(adapter_entrypoint)),
        credential=loaded.credential,
        model=loaded.resolved.pi_model,
        thinking_level=loaded.resolved.pi_thinking_level,
        startup_timeout_s=loaded.resolved.timeouts.readiness_s,
    )
    result = LocalAgentEvalRunner(
        config=loaded.resolved,
        selected=selected,
        backend=backend,
        mcp=mcp,
        code_policy=dimos_control,
        pi_factory=pi_factory,
    ).run()
    print(
        f"{result.outcome.task_result}: {result.outcome.reason} (artifacts: {result.attempt_path})"
    )
    return result.exit_code


def _parse_endpoint(value: str) -> tuple[str, int]:
    match = _ENDPOINT.fullmatch(value)
    if match is None:
        raise ValueError("DimSim endpoint must be http(s)://host:port")
    port = int(match.group("port"))
    if not 1 <= port <= 65535:
        raise ValueError("DimSim endpoint port is outside 1..65535")
    return match.group("host"), port
