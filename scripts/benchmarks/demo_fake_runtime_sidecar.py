#!/usr/bin/env python3
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

"""Run the fake simulator runtime module smoke demo."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
FAKE_SIDECAR_SRC = REPO_ROOT / "packages" / "dimos-fake-runtime-sidecar" / "src"

for package_src in (PROTOCOL_SRC, FAKE_SIDECAR_SRC):
    sys.path.insert(0, str(package_src))

from dimos_fake_runtime_sidecar.module import FakeRuntimeModule
from dimos_runtime_protocol import EpisodeResetRequest, MotorActionFrame, StepRequest

from dimos.benchmark.runtime.artifacts import write_json
from dimos.benchmark.runtime.config import (
    BenchmarkEpisodeConfig,
    resolve_runtime_plan,
)


def _load_config(path: Path) -> BenchmarkEpisodeConfig:
    return BenchmarkEpisodeConfig.model_validate_json(path.read_text())


def _trace_summary(trace: list[dict[str, object]]) -> dict[str, object]:
    final = trace[-1] if trace else {}
    return {
        "ticks": len(trace),
        "first_command_sequence": trace[0].get("command_sequence") if trace else None,
        "final_command_sequence": final.get("command_sequence"),
        "final_state_sequence": final.get("state_sequence"),
        "final_command_q": final.get("command_q"),
        "final_state_q": final.get("state_q"),
    }


def run_demo(config_path: Path) -> Path:
    config = _load_config(config_path)
    module = FakeRuntimeModule(
        robot_id=config.robot_id,
        dof=config.dof,
        step_hz=config.control_step_hz,
    )
    cleanup_status: dict[str, object] = {
        "runtime_module_stopped": False,
    }
    try:
        description = module.describe()
        plan = resolve_runtime_plan(config, description)
        reset = module.reset(
            EpisodeResetRequest(
                episode_id=plan.episode_id,
                task_id=plan.task_id,
                seed=config.seed,
            )
        )

        trace: list[dict[str, object]] = []
        target = [plan.target_position] * len(plan.motor_names)
        for tick in range(plan.ticks):
            action = MotorActionFrame(
                robot_id=plan.robot_id,
                names=plan.motor_names,
                q=target,
                sequence=tick,
            )
            response = module.step(
                StepRequest(episode_id=plan.episode_id, tick_id=tick, action=action)
            )
            trace.append(
                {
                    "tick": tick,
                    "command_sequence": action.sequence,
                    "state_sequence": response.motor_state.sequence,
                    "command_q": action.q,
                    "state_q": response.motor_state.q,
                }
            )

        score = module.score()
        if not score.success:
            raise RuntimeError(f"fake runtime smoke demo failed score: {score.reason}")

        artifact_dir = (REPO_ROOT / plan.artifact_dir).resolve()
        write_json(artifact_dir / "episode_config.json", config)
        write_json(artifact_dir / "runtime_description.json", description)
        write_json(artifact_dir / "resolved_runtime_plan.json", plan)
        write_json(artifact_dir / "reset_response.json", reset)
        write_json(artifact_dir / "motor_trace.json", trace)
        write_json(artifact_dir / "protocol_trace_summary.json", _trace_summary(trace))
        write_json(artifact_dir / "score.json", score)
        write_json(
            artifact_dir / "module_status.json", {"ok": True, "runtime_id": description.runtime_id}
        )
        return artifact_dir
    finally:
        try:
            module.stop()
            cleanup_status["runtime_module_stopped"] = True
        except Exception as exc:
            cleanup_status["runtime_module_error"] = str(exc)
        if config.artifact_dir:
            artifact_dir = (REPO_ROOT / config.artifact_dir).resolve()
            artifact_dir.mkdir(parents=True, exist_ok=True)
            write_json(artifact_dir / "cleanup_status.json", cleanup_status)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPO_ROOT
        / "dimos"
        / "benchmark"
        / "runtime"
        / "configs"
        / "fake_runtime_smoke.json",
    )
    args = parser.parse_args()
    artifact_dir = run_demo(args.config)
    print(json.dumps({"ok": True, "artifact_dir": str(artifact_dir)}, indent=2))


if __name__ == "__main__":
    main()
