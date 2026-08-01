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

from datetime import UTC, datetime

from pydantic import ValidationError
import pytest

from dimos.benchmark.agent_eval.models import (
    DimSimBackendOptions,
    InfrastructureTimeouts,
    NormalizedOutcome,
    PiSettings,
    SmokeConfig,
)

_TASK_ID = "task_" + "a" * 64


def _config_payload() -> dict[str, object]:
    return {
        "release_root": "/releases/smoke",
        "task_id": _TASK_ID,
        "output_root": "/attempts",
        "mcp_endpoint": "http://127.0.0.1:9990/mcp",
        "pi": PiSettings(
            model="anthropic/claude-sonnet",
            thinking_level="medium",
            auth_mode="environment",
            credential_env="ANTHROPIC_API_KEY",
        ),
        "timeouts": InfrastructureTimeouts(
            readiness_s=10.0,
            mcp_call_s=10.0,
            reset_s=20.0,
            evaluation_start_s=10.0,
            cancellation_s=5.0,
        ),
        "episode_timeout_s": 180.0,
        "dimsim": DimSimBackendOptions(
            endpoint="http://127.0.0.1:8090",
            expected_scene_id="dimsim-apartment",
        ),
    }


def test_smoke_config_is_strict_and_has_no_predicate_overrides() -> None:
    config = SmokeConfig(**_config_payload())

    assert config.episode_timeout_s == 180.0
    with pytest.raises(ValidationError, match="threshold_m"):
        SmokeConfig(**_config_payload(), threshold_m=2.0)


def test_pi_auth_configuration_never_contains_credential_value() -> None:
    with pytest.raises(ValidationError, match="credential"):
        PiSettings(
            model="model",
            thinking_level="medium",
            auth_mode="environment",
            credential_env=None,
        )


def test_outcome_rejects_mixed_infrastructure_and_task_states() -> None:
    with pytest.raises(ValidationError, match="failed infrastructure"):
        NormalizedOutcome(
            attempt_id="attempt_" + "1" * 32,
            attempt_status="failed",
            task_result="failed",
            terminal_stage="reset",
            reason="reset failed",
            required_artifacts_complete=False,
            finished_at=datetime.now(UTC),
            duration_s=1.0,
        )
