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

"""Run the fake runtime sidecar smoke demo with a real ControlCoordinator."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[2]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
FAKE_SIDECAR_SRC = REPO_ROOT / "packages" / "dimos-fake-runtime-sidecar" / "src"

for package_src in (PROTOCOL_SRC, FAKE_SIDECAR_SRC):
    sys.path.insert(0, str(package_src))

from dimos_runtime_protocol import EpisodeResetRequest, MotorActionFrame, StepRequest

from dimos.benchmark.runtime.artifacts import write_json
from dimos.benchmark.runtime.config import (
    BenchmarkEpisodeConfig,
    resolve_runtime_plan,
)
from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.hardware.whole_body.spec import MotorState
from dimos.simulation.runtime_client.http_client import RuntimeSidecarClient
from dimos.simulation.runtime_client.shm_motor import MotorShmOwner


def _load_config(path: Path) -> BenchmarkEpisodeConfig:
    return BenchmarkEpisodeConfig.model_validate_json(path.read_text())


def _sidecar_env() -> dict[str, str]:
    env = dict(os.environ)
    existing = env.get("PYTHONPATH", "")
    paths = [str(PROTOCOL_SRC), str(FAKE_SIDECAR_SRC)]
    if existing:
        paths.append(existing)
    env["PYTHONPATH"] = os.pathsep.join(paths)
    return env


def _start_fake_sidecar(config: BenchmarkEpisodeConfig) -> subprocess.Popen[str]:
    return subprocess.Popen(
        [
            sys.executable,
            "-m",
            "dimos_fake_runtime_sidecar.server",
            "--host",
            config.runtime_host,
            "--port",
            str(config.runtime_port),
            "--robot-id",
            config.robot_id,
            "--dof",
            str(config.dof),
        ],
        cwd=REPO_ROOT,
        env=_sidecar_env(),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def _command_frame(owner: MotorShmOwner, robot_id: str) -> tuple[int, MotorActionFrame]:
    sequence, commands = owner.read_commands()
    return sequence, MotorActionFrame(
        robot_id=robot_id,
        names=owner.motor_names,
        q=[command.q for command in commands],
        dq=[command.dq for command in commands],
        kp=[command.kp for command in commands],
        kd=[command.kd for command in commands],
        tau=[command.tau for command in commands],
        sequence=sequence,
    )


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
    sidecar = _start_fake_sidecar(config)
    client = RuntimeSidecarClient(f"http://{config.runtime_host}:{config.runtime_port}")
    owner: MotorShmOwner | None = None
    coordinator: ControlCoordinator | None = None
    sidecar_output = ""
    cleanup_status: dict[str, object] = {
        "coordinator_stopped": False,
        "shm_unlinked": False,
        "sidecar_stopped": False,
    }
    try:
        health = client.wait_until_healthy(timeout_s=5.0)
        description = client.describe()
        plan = resolve_runtime_plan(config, description)
        reset = client.reset(
            EpisodeResetRequest(
                episode_id=plan.episode_id,
                task_id=plan.task_id,
                seed=config.seed,
            )
        )

        owner = MotorShmOwner(plan.shm_key, plan.motor_names)
        owner.write_state(
            [MotorState(q=0.0) for _ in plan.motor_names],
            sequence=0,
        )

        hardware = HardwareComponent(
            hardware_id=plan.robot_id,
            hardware_type=HardwareType.WHOLE_BODY,
            joints=plan.motor_names,
            adapter_type="benchmark_runtime",
            address=plan.shm_key,
            adapter_kwargs={"motor_names": plan.motor_names, "connect_timeout_s": 5.0},
        )
        task_name = f"servo_{plan.robot_id}"
        coordinator = ControlCoordinator(
            tick_rate=float(plan.control_step_hz),
            publish_joint_state=False,
            hardware=[hardware],
            tasks=[
                TaskConfig(
                    name=task_name,
                    type="servo",
                    joint_names=plan.motor_names,
                    auto_start=True,
                    params={"timeout": 0.0, "default_positions": [0.0] * len(plan.motor_names)},
                )
            ],
        )
        coordinator.start()

        trace: list[dict[str, object]] = []
        target = [plan.target_position] * len(plan.motor_names)
        for tick in range(plan.ticks):
            accepted = coordinator.task_invoke(
                task_name, "set_target", {"positions": target, "t_now": None}
            )
            if accepted is not True:
                raise RuntimeError(f"servo task rejected target at tick {tick}")
            time.sleep(1.0 / plan.control_step_hz)
            command_sequence, action = _command_frame(owner, plan.robot_id)
            response = client.step(
                StepRequest(episode_id=plan.episode_id, tick_id=tick, action=action)
            )
            owner.write_state(
                [
                    MotorState(
                        q=response.motor_state.q[i],
                        dq=response.motor_state.dq[i],
                        tau=response.motor_state.tau[i],
                    )
                    for i in range(len(plan.motor_names))
                ],
                sequence=response.motor_state.sequence,
            )
            trace.append(
                {
                    "tick": tick,
                    "command_sequence": command_sequence,
                    "state_sequence": response.motor_state.sequence,
                    "command_q": action.q,
                    "state_q": response.motor_state.q,
                }
            )

        score = client.score()
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
        write_json(artifact_dir / "health.json", health)
        return artifact_dir
    finally:
        if coordinator is not None:
            try:
                coordinator.stop()
                cleanup_status["coordinator_stopped"] = True
            except Exception as exc:
                cleanup_status["coordinator_error"] = str(exc)
        if owner is not None:
            try:
                owner.close()
                owner.unlink()
                cleanup_status["shm_unlinked"] = True
            except Exception as exc:
                cleanup_status["shm_error"] = str(exc)
        sidecar.terminate()
        try:
            sidecar_output, _ = sidecar.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            sidecar.kill()
            sidecar_output, _ = sidecar.communicate(timeout=2.0)
        cleanup_status["sidecar_returncode"] = sidecar.returncode
        cleanup_status["sidecar_stopped"] = sidecar.returncode is not None
        if config.artifact_dir:
            sidecar_log = (REPO_ROOT / config.artifact_dir / "fake_sidecar.log").resolve()
            sidecar_log.parent.mkdir(parents=True, exist_ok=True)
            sidecar_log.write_text(sidecar_output)
            write_json(sidecar_log.parent / "cleanup_status.json", cleanup_status)


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
