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

from __future__ import annotations

from argparse import Namespace
import json
from pathlib import Path

from dimos_runtime_protocol import (
    EpisodeResetRequest,
    EpisodeResetResponse,
    MotorStateFrame,
    RuntimeActionFrame,
    RuntimeDescription,
    StepRequest,
    StepResponse,
)
import numpy as np

from dimos.control.coordinator import ControlCoordinator
from dimos.robot_learning.policy_rollout.evaluation import (
    BenchmarkEpisodeSpec,
    PolicyEvalRuntimeSession,
    RuntimeStreamSnapshot,
)
from dimos.robot_learning.policy_rollout.models import BackendBatch, RobotPolicyActionChunk
from dimos.robot_learning.policy_rollout.robot_policy_module import RobotPolicyModule
from dimos.robot_learning.policy_rollout.vla_jepa_libero_contract import (
    VLA_JEPA_LIBERO_ACTION_SPACE_ID,
)
from dimos.robot_learning.policy_rollout.xvla_libero_contract import (
    XVLA_LIBERO_ABSOLUTE_ACTION_SPACE_ID,
)
from scripts.benchmarks.demo_lerobot_libero_policy_rollout import (
    FixedActionBackend,
    _LivePolicyCoordinator,
    _run_live_episode_loop,
    run_policy_gate,
)


def _live_args() -> Namespace:
    return Namespace(
        fake_backend=True,
        fixed_action="0.1,-0.2,0.3,-0.4,0.5,-0.6,0.7",
        policy_family="vla_jepa",
        checkpoint=None,
        contract_type=None,
        gate_stage=None,
        device="cpu",
        control_step_hz=100.0,
        max_steps=1,
        live_chunk_timeout_s=2.0,
        live_max_stale_waits=3,
        save_videos=False,
        camera_names=(),
    )


class _FakeLiveRuntimeSession(PolicyEvalRuntimeSession):
    def __init__(self) -> None:
        self.last_action: RuntimeActionFrame | None = None
        self._snapshot = RuntimeStreamSnapshot(
            observations=(),
            values={
                "agentview": np.zeros((128, 128, 3), dtype=np.uint8),
                "robot0_eye_in_hand": np.zeros((128, 128, 3), dtype=np.uint8),
                "robot_state": np.asarray(
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float32
                ),
            },
        )
        self.closed = False

    def reset(self, request: EpisodeResetRequest) -> EpisodeResetResponse:
        return EpisodeResetResponse(
            episode_id=request.episode_id,
            runtime_description=RuntimeDescription(
                runtime_id="fake-libero",
                backend="fake",
                robot_surfaces=[],
                control_step_hz=20,
                observation_streams=["agentview", "robot0_eye_in_hand", "robot_state"],
                metadata={"language": "pick up the object"},
            ),
        )

    def step(self, request: StepRequest) -> StepResponse:
        if not isinstance(request.action, RuntimeActionFrame):
            raise TypeError(f"expected RuntimeActionFrame, got {type(request.action).__name__}")
        self.last_action = request.action
        return StepResponse(
            episode_id=request.episode_id,
            tick_id=request.tick_id,
            motor_state=MotorStateFrame(robot_id="panda", names=[], q=[], dq=[], tau=[]),
            reward=1.0,
            done=True,
            success=True,
        )

    def latest_observation_snapshot(self) -> RuntimeStreamSnapshot:
        return self._snapshot

    def close(self) -> None:
        self.closed = True


def test_fixed_action_backend_can_emit_single_action_chunk() -> None:
    backend = FixedActionBackend((0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7), use_action_chunk=True)
    backend.initialize()

    output = backend.infer_batch(BackendBatch(payload={}, metadata={}))

    assert output.output == ((0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7),)
    assert output.metadata["use_action_chunk"] is True


def test_live_policy_coordinator_uses_module_stream_and_ref_wiring() -> None:
    live = _LivePolicyCoordinator(args=_live_args())
    try:
        live.start()
        assert live._module_coordinator is not None
        assert (
            "robot_policy_action_chunk",
            RobotPolicyActionChunk,
        ) in live._module_coordinator._transport_registry
        assert (
            ControlCoordinator,
            "_robot_policy",
        ) in live._module_coordinator._resolved_module_refs
        assert (
            live._module_coordinator._resolved_module_refs[(ControlCoordinator, "_robot_policy")]
            is RobotPolicyModule
        )
    finally:
        live.close()


def test_xvla_configuration_can_run_synchronous_benchmark_without_control_coordinator(
    tmp_path: Path, mocker
) -> None:
    args = _xvla_args(tmp_path, live_policy_stream=False)
    runtime = _FakeLiveRuntimeSession()
    mocker.patch(
        "scripts.benchmarks.demo_lerobot_libero_policy_rollout._start_runtime",
        return_value=runtime,
    )

    summary = run_policy_gate(args)

    assert summary.passed is True
    assert runtime.last_action is not None
    assert runtime.last_action.space_id == XVLA_LIBERO_ABSOLUTE_ACTION_SPACE_ID
    assert runtime.closed is True
    run_config = json.loads((tmp_path / "run_config.json").read_text())
    assert run_config["policy_family"] == "xvla"
    assert run_config["checkpoint"] == "lerobot/xvla-libero"
    assert run_config["contract_type"] == "xvla_libero"
    assert run_config["action_space_id"] == XVLA_LIBERO_ABSOLUTE_ACTION_SPACE_ID
    assert run_config["runtime_action_mode"] == "native_absolute"
    assert run_config["gate_stage"] == "benchmark"
    assert run_config["expected_gate_episodes"] == 10


def test_live_episode_loop_steps_runtime_with_control_coordinator_action(tmp_path: Path) -> None:
    runtime = _FakeLiveRuntimeSession()
    live = _LivePolicyCoordinator(args=_live_args())
    try:
        live.start()
        result, diagnostics = _run_live_episode_loop(
            _live_args(),
            BenchmarkEpisodeSpec(
                episode_id="episode-1",
                task_id="libero_object",
                task_index=0,
                init_state_index=0,
            ),
            runtime,
            live,
            tmp_path,
        )

        assert result.success is True
        assert result.action_shape == (7,)
        assert runtime.last_action is not None
        assert runtime.last_action.space_id == VLA_JEPA_LIBERO_ACTION_SPACE_ID
        np.testing.assert_allclose(
            runtime.last_action.values,
            [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7],
        )
        assert diagnostics["consumed_actions"] == 1
        assert diagnostics["refill_triggers"] == 1
        inference_status_counts = diagnostics["inference_status_counts"]
        assert isinstance(inference_status_counts, dict)
        assert inference_status_counts["started"] == 1
    finally:
        live.close()


def test_xvla_live_configuration_deploys_same_module_native_path() -> None:
    live = _LivePolicyCoordinator(args=_xvla_args(Path("unused"), live_policy_stream=True))
    try:
        live.start()
        assert live._module_coordinator is not None
        assert (
            "robot_policy_action_chunk",
            RobotPolicyActionChunk,
        ) in live._module_coordinator._transport_registry
        assert (
            ControlCoordinator,
            "_robot_policy",
        ) in live._module_coordinator._resolved_module_refs
        assert live.describe_backend().backend_type == "fixed_action"
    finally:
        live.close()


def _xvla_args(tmp_path: Path, *, live_policy_stream: bool) -> Namespace:
    return Namespace(
        fake_backend=True,
        fixed_action="0.1,-0.2,0.3,-0.4,0.5,-0.6,0.7",
        policy_family="xvla",
        checkpoint=None,
        contract_type=None,
        gate_stage=None,
        device="cpu",
        control_step_hz=100.0,
        max_steps=1,
        live_policy_stream=live_policy_stream,
        live_chunk_timeout_s=2.0,
        live_max_stale_waits=3,
        save_videos=False,
        camera_names=(),
        artifact_dir=tmp_path,
        success_threshold=0.5,
        enforce_gate=True,
        episodes_limit=1,
        bddl_root=tmp_path,
        init_states_root=tmp_path,
        allow_asset_bootstrap=False,
        visualize=False,
        camera_height=128,
        camera_width=128,
    )
