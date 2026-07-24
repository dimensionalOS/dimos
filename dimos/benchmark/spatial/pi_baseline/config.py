# Copyright 2026 Dimensional Inc.
"""Fail-closed configuration for the opt-in Pi spatial baseline runner."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import json
import os
from pathlib import Path
import re
import stat
from typing import Literal

from dotenv import dotenv_values
from pydantic import Field, field_validator, model_validator

from dimos.benchmark.spatial.models import SpatialModel

PromptMode = Literal["visualization-forbidden", "visualization-encouraged"]
ThinkingLevel = Literal["medium"]
AuthMode = Literal["codex-oauth", "openai-api-key"]
_ID_PATTERN = r"^[A-Za-z0-9][A-Za-z0-9_-]{0,127}$"
_DIGEST_PATTERN = r"^.+@sha256:[0-9a-f]{64}$"


def validate_node_adapter_command(command: Sequence[str]) -> list[str]:
    """Accept only a direct Node executable and one absolute JS entrypoint.

    This is intentionally a structural admission check, rather than a shell
    command parser: there is no supported wrapper, package manager, runtime
    flag, or additional argv in the Pi protocol.
    """
    if len(command) != 2 or any(
        not isinstance(argument, str) or not argument for argument in command
    ):
        raise ValueError(
            "Pi adapter command must be [absolute node executable, absolute adapter file]"
        )
    executable, adapter = (Path(argument) for argument in command)
    if not executable.is_absolute() or executable.name not in {"node", "nodejs"}:
        raise ValueError("Pi adapter executable must be an approved absolute Node executable")
    if not executable.is_file() or not executable.stat().st_mode & 0o111:
        raise ValueError("Pi adapter executable must be an executable file")
    if not adapter.is_absolute() or adapter.suffix != ".js":
        raise ValueError("Pi adapter command must name one absolute .js file")
    return list(command)


class ModelConfig(SpatialModel):
    provider: Literal["openai-codex", "openai"]
    model_id: Literal["gpt-5.6-luna"]
    thinking_level: ThinkingLevel


class ResourceLimits(SpatialModel):
    cpu_cores: float = Field(gt=0)
    memory_mb: int = Field(gt=0)
    agent_environment_mb: int = Field(default=512, gt=0)
    pids: int = Field(gt=0)
    timeout_seconds: int = Field(gt=0)

    @model_validator(mode="after")
    def validate_agent_environment(self) -> ResourceLimits:
        if self.agent_environment_mb > self.memory_mb:
            raise ValueError("agent_environment_mb must not exceed memory_mb")
        return self


class AuditNetworkPolicy(SpatialModel):
    network_access: Literal["general-outbound"]
    audit: Literal["heuristic"]
    audit_limitations: Literal["cannot-prove-no-online-use"]


class PublicSelection(SpatialModel):
    scene_id: str = Field(pattern=_ID_PATTERN)
    trajectory_id: str = Field(pattern=_ID_PATTERN)
    question_id: str = Field(pattern=_ID_PATTERN)
    variant: Literal["clean", "noisy-01", "noisy-02"]
    instance_id: str = Field(pattern=_ID_PATTERN)


class Budgets(SpatialModel):
    max_turns: int = Field(ge=1, le=100)
    max_tool_calls: int = Field(ge=1, le=100)
    timeout_ms: int = Field(ge=1_000, le=900_000)


class FixedSmokeIdentity(PublicSelection):
    """The immutable case identity used by both paired prompt modes."""


class ImplementationDigests(SpatialModel):
    adapter: str = Field(pattern=_DIGEST_PATTERN)
    scorer: str = Field(pattern=_DIGEST_PATTERN)
    protocol: str = Field(pattern=_DIGEST_PATTERN)


class PiBaselineConfig(SpatialModel):
    """All host-side inputs needed before an external runner may start."""

    model: ModelConfig
    node_adapter_command: list[str] = Field(min_length=2, max_length=2)
    auth_mode: AuthMode = "codex-oauth"
    codex_oauth_auth_path: str | None = Field(default=None, min_length=1)
    openai_api_key_env_path: str | None = Field(default=None, min_length=1)
    runner_image: str = Field(pattern=_DIGEST_PATTERN)
    rootless_podman_required: Literal[True]
    resource_limits: ResourceLimits
    output_root: str = Field(min_length=1)
    audit_network_policy: AuditNetworkPolicy
    prompt_modes: list[PromptMode] = Field(min_length=1)
    corpus_root: str = Field(min_length=1)
    oracle_root: str = Field(min_length=1)
    private_root: str = Field(min_length=1)
    ledger_path: str = Field(min_length=1)
    selection: PublicSelection
    budgets: Budgets
    scorer_revision: str = Field(min_length=1)
    fixed_smoke_identity: FixedSmokeIdentity
    implementation_digests: ImplementationDigests
    case_id: str | None = Field(default=None, pattern=_ID_PATTERN)
    run_id: str | None = Field(default=None, pattern=_ID_PATTERN)

    @field_validator("node_adapter_command")
    @classmethod
    def validate_node_command(cls, value: list[str]) -> list[str]:
        return validate_node_adapter_command(value)

    @field_validator("codex_oauth_auth_path", "openai_api_key_env_path")
    @classmethod
    def validate_auth_path(cls, value: str | None) -> str | None:
        if value is None:
            return None
        path = Path(value).expanduser()
        if not path.is_file():
            raise ValueError("authentication path must point to an existing file")
        return str(path)

    @field_validator("output_root")
    @classmethod
    def validate_output_root(cls, value: str) -> str:
        path = Path(value).expanduser()
        if path.exists() and not path.is_dir():
            raise ValueError("output root must be a directory")
        return str(path)

    @field_validator("corpus_root", "oracle_root", "private_root", "ledger_path")
    @classmethod
    def validate_roots(cls, value: str) -> str:
        path = Path(value).expanduser()
        if path.exists() and not (path.is_dir() or path.parent.is_dir()):
            raise ValueError("configured root/path is not usable")
        return str(path)

    @field_validator("prompt_modes")
    @classmethod
    def validate_prompt_modes(cls, value: list[PromptMode]) -> list[PromptMode]:
        if set(value) != {"visualization-forbidden", "visualization-encouraged"}:
            raise ValueError("prompt_modes must contain both supported prompt modes")
        return value

    @field_validator("fixed_smoke_identity")
    @classmethod
    def validate_fixed_identity(cls, value: FixedSmokeIdentity) -> FixedSmokeIdentity:
        return value

    @model_validator(mode="after")
    def validate_selection_identity(self) -> PiBaselineConfig:
        if self.selection.model_dump() != self.fixed_smoke_identity.model_dump():
            raise ValueError("selection and fixed_smoke_identity must identify the same case")
        if self.auth_mode == "codex-oauth":
            if self.model.provider != "openai-codex":
                raise ValueError("Codex OAuth requires the openai-codex provider")
            if self.codex_oauth_auth_path is None or self.openai_api_key_env_path is not None:
                raise ValueError("Codex OAuth requires only codex_oauth_auth_path")
        else:
            if self.model.provider != "openai":
                raise ValueError("OpenAI API-key auth requires the openai provider")
            if self.openai_api_key_env_path is None or self.codex_oauth_auth_path is not None:
                raise ValueError("OpenAI API-key auth requires only openai_api_key_env_path")
            load_openai_api_key(Path(self.openai_api_key_env_path))
        return self

    @property
    def auth_path(self) -> Path:
        value = (
            self.codex_oauth_auth_path
            if self.auth_mode == "codex-oauth"
            else self.openai_api_key_env_path
        )
        if value is None:
            raise ValueError("authentication path is unavailable")
        return Path(value)


BaselineConfig = PiBaselineConfig


def load_openai_api_key(path: Path) -> str:
    """Read one API key from a private dotenv file without mutating the environment."""
    info = path.lstat()
    if (
        not stat.S_ISREG(info.st_mode)
        or info.st_uid != os.getuid()
        or stat.S_IMODE(info.st_mode) & 0o077
    ):
        raise ValueError("API-key dotenv file must be owner-only regular file")
    value = dotenv_values(path).get("OPENAI_API_KEY")
    if not isinstance(value, str) or not value.strip() or "\x00" in value:
        raise ValueError("OPENAI_API_KEY is missing from the dotenv file")
    return value


def validate_identifier(value: str) -> str:
    """Validate a case, run, or instance identifier without executing a run."""
    if not re.fullmatch(_ID_PATTERN, value):
        raise ValueError("identifier must contain only letters, digits, '_' or '-'")
    return value


def load_config(path: str | Path) -> PiBaselineConfig:
    """Load and validate a JSON configuration; no OAuth or model call is made."""
    config_path = Path(path)
    with config_path.open(encoding="utf-8") as handle:
        payload: Mapping[str, object] = json.load(handle)
    return PiBaselineConfig.model_validate(payload)
