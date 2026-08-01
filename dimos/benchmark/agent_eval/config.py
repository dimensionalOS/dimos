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

"""Strict local smoke configuration and generated destination selection."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import os
from pathlib import Path

from pydantic import JsonValue

from dimos.benchmark.agent_eval.models import ResolvedSmokeConfig, SmokeConfig
from dimos.benchmark.agent_eval.pi_adapter import credential_binding_sha256
from dimos.benchmark.dimsim.apartment_profile import (
    APARTMENT_CANONICAL_SPAWN,
    APARTMENT_PROFILE_REVISION,
    APARTMENT_SCENE_ID,
)
from dimos.benchmark.dimsim.bundle import load_full_release
from dimos.benchmark.dimsim.models import (
    ExpectedOutcome,
    Manifest,
    NavigateContract,
    Pose2,
    PublicTask,
    SceneOracleView,
    TaskContract,
    TerminalOutcome,
)
from dimos.benchmark.dimsim.utilities import model_bytes, oracle_view_digest


@dataclass(frozen=True)
class RuntimeCredential:
    auth_mode: str
    binding_name: str
    value: str | None


@dataclass(frozen=True)
class LoadedSmokeConfig:
    configured: SmokeConfig
    resolved: ResolvedSmokeConfig
    credential: RuntimeCredential


@dataclass(frozen=True)
class SelectedDestination:
    manifest: Manifest
    public: PublicTask
    contract: TaskContract
    outcome: ExpectedOutcome
    start_pose: Pose2
    contract_sha256: str
    outcome_sha256: str


def load_smoke_config(path: Path) -> LoadedSmokeConfig:
    """Load one JSON config, resolve paths/auth, and retain no credential secret."""
    configured = SmokeConfig.model_validate_json(path.read_bytes())
    if configured.pi.model != "gpt-5.6-luna" or configured.pi.thinking_level != "medium":
        raise ValueError("local smoke currently supports only gpt-5.6-luna/medium")
    base = path.resolve().parent
    release_root = _resolved_path(base, configured.release_root)
    output_root = _resolved_path(base, configured.output_root)
    if configured.pi.auth_mode == "environment":
        assert configured.pi.credential_env is not None
        binding_name = configured.pi.credential_env
        value = os.environ.get(binding_name)
        if not value:
            raise ValueError(f"credential environment variable {binding_name!r} is unset")
    else:
        assert configured.pi.credential_path is not None
        credential_path = _resolved_path(base, configured.pi.credential_path)
        if not credential_path.is_file():
            raise ValueError("subscription credential file does not exist")
        binding_name = str(credential_path)
        value = None
        binding_material: str | bytes | None = credential_path.read_bytes()
    if configured.pi.auth_mode == "environment":
        binding_material = value
    binding_digest = credential_binding_sha256(
        configured.pi.auth_mode,
        binding_name,
        binding_material,
    )
    resolved = ResolvedSmokeConfig(
        release_root=str(release_root),
        task_id=configured.task_id,
        output_root=str(output_root),
        mcp_endpoint=configured.mcp_endpoint,
        pi_model=configured.pi.model,
        pi_thinking_level=configured.pi.thinking_level,
        auth_mode=configured.pi.auth_mode,
        credential_binding_sha256=binding_digest,
        timeouts=configured.timeouts,
        episode_timeout_s=configured.episode_timeout_s,
        dimsim=configured.dimsim,
    )
    return LoadedSmokeConfig(
        configured=configured,
        resolved=resolved,
        credential=RuntimeCredential(
            auth_mode=configured.pi.auth_mode,
            binding_name=binding_name,
            value=value,
        ),
    )


def select_destination(release_root: Path, task_id: str) -> SelectedDestination:
    manifest, public, contracts, outcomes = load_full_release(release_root)
    selected_public = [item for item in public if item.task_id == task_id]
    selected_contract = [item for item in contracts if item.task_id == task_id]
    selected_outcome = [item for item in outcomes if item.task_id == task_id]
    if not (len(selected_public) == len(selected_contract) == len(selected_outcome) == 1):
        raise ValueError("selected task does not join exactly once in the release")
    public_task = selected_public[0]
    contract = selected_contract[0]
    outcome = selected_outcome[0]
    if (
        public_task.category != "destination"
        or public_task.response_type != "terminal"
        or not isinstance(contract.contract, NavigateContract)
        or not isinstance(outcome.expected, TerminalOutcome)
    ):
        raise ValueError("local smoke supports only a generated destination triple")
    source = contract.source
    if (
        source.profile_revision != APARTMENT_PROFILE_REVISION
        or source.scene_id != APARTMENT_SCENE_ID
    ):
        raise ValueError("selected task does not use the compatible apartment profile")
    return SelectedDestination(
        manifest=manifest,
        public=public_task,
        contract=contract,
        outcome=outcome,
        start_pose=Pose2(
            x_m=APARTMENT_CANONICAL_SPAWN[0],
            z_m=APARTMENT_CANONICAL_SPAWN[1],
            yaw_rad=0.0,
        ),
        contract_sha256=hashlib.sha256(model_bytes(contract)).hexdigest(),
        outcome_sha256=hashlib.sha256(model_bytes(outcome)).hexdigest(),
    )


def verify_fresh_oracle(
    selected: SelectedDestination,
    view: SceneOracleView,
) -> dict[str, JsonValue]:
    """Fail closed unless the post-reset private view is the selected source."""
    source = selected.contract.source
    expected = {
        "scene_id": source.scene_id,
        "scene_revision": source.scene_revision,
        "reset_revision": source.reset_revision,
        "upstream_revision": source.upstream_revision,
        "profile_revision": source.profile_revision,
    }
    actual = {key: getattr(view, key) for key in expected}
    if actual != expected:
        raise ValueError("fresh post-reset oracle source revisions do not match task")
    digest = oracle_view_digest(view)
    if digest != source.oracle_view_digest:
        raise ValueError("fresh post-reset oracle content digest does not match task")
    if view.embodiment.canonical_spawn != selected.start_pose:
        raise ValueError("fresh post-reset oracle spawn does not match profile")
    return {**actual, "oracle_view_digest": digest}


def _resolved_path(base: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return (base / path).resolve() if not path.is_absolute() else path.resolve()
