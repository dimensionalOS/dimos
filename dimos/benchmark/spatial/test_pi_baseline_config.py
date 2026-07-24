import json
from pathlib import Path

from pydantic import ValidationError
import pytest

from dimos.benchmark.spatial.pi_baseline.config import (
    PiBaselineConfig,
    load_config,
    validate_node_adapter_command,
)


def valid_payload(auth_path: Path) -> dict[str, object]:
    return {
        "model": {
            "provider": "openai-codex",
            "model_id": "gpt-5.6-luna",
            "thinking_level": "medium",
        },
        "node_adapter_command": ["/usr/bin/node", str(auth_path.parent / "adapter.js")],
        "codex_oauth_auth_path": str(auth_path),
        "runner_image": "registry.example/pi@sha256:" + "a" * 64,
        "rootless_podman_required": True,
        "resource_limits": {
            "cpu_cores": 2.0,
            "memory_mb": 1024,
            "pids": 128,
            "timeout_seconds": 60,
        },
        "output_root": "/tmp/pi-output",
        "audit_network_policy": {
            "network_access": "general-outbound",
            "audit": "heuristic",
            "audit_limitations": "cannot-prove-no-online-use",
        },
        "prompt_modes": ["visualization-forbidden", "visualization-encouraged"],
        "corpus_root": "/tmp/pi-corpus",
        "oracle_root": "/tmp/pi-oracle",
        "private_root": "/tmp/pi-private",
        "ledger_path": "/tmp/pi-private/ledger.jsonl",
        "selection": {
            "scene_id": "scene-1",
            "trajectory_id": "trajectory-1",
            "question_id": "question-1",
            "variant": "clean",
            "instance_id": "instance-1",
        },
        "budgets": {"max_turns": 4, "max_tool_calls": 8, "timeout_ms": 10000},
        "scorer_revision": "scorer-v1",
        "fixed_smoke_identity": {
            "scene_id": "scene-1",
            "trajectory_id": "trajectory-1",
            "question_id": "question-1",
            "variant": "clean",
            "instance_id": "instance-1",
        },
        "implementation_digests": {
            "adapter": "adapter@sha256:" + "b" * 64,
            "scorer": "scorer@sha256:" + "c" * 64,
            "protocol": "protocol@sha256:" + "d" * 64,
        },
        "case_id": "case-1",
        "run_id": "run-1",
    }


def test_config_loads_without_external_calls(tmp_path: Path) -> None:
    auth = tmp_path / "oauth.json"
    auth.write_text("{}", encoding="utf-8")
    (tmp_path / "adapter.js").write_text("", encoding="utf-8")
    config_path = tmp_path / "config.json"
    config_path.write_text(json.dumps(valid_payload(auth)), encoding="utf-8")
    assert load_config(config_path).model.model_id == "gpt-5.6-luna"
    assert load_config(config_path).model.thinking_level == "medium"


def test_api_key_mode_loads_only_private_dotenv_source(tmp_path: Path) -> None:
    auth = tmp_path / ".env"
    auth.write_text("OPENAI_API_KEY=test-key\nUNRELATED=value\n", encoding="utf-8")
    auth.chmod(0o600)
    (tmp_path / "adapter.js").write_text("", encoding="utf-8")
    payload = valid_payload(auth)
    payload.pop("codex_oauth_auth_path")
    payload.update(
        {
            "auth_mode": "openai-api-key",
            "openai_api_key_env_path": str(auth),
            "model": {
                "provider": "openai",
                "model_id": "gpt-5.6-luna",
                "thinking_level": "medium",
            },
        }
    )

    config = PiBaselineConfig.model_validate(payload)

    assert config.auth_mode == "openai-api-key"
    assert config.auth_path == auth
    assert config.codex_oauth_auth_path is None


@pytest.mark.parametrize("contents", ["UNRELATED=value\n", "OPENAI_API_KEY=\n"])
def test_api_key_mode_rejects_missing_key_or_public_file(tmp_path: Path, contents: str) -> None:
    auth = tmp_path / ".env"
    auth.write_text(contents, encoding="utf-8")
    auth.chmod(0o600)
    (tmp_path / "adapter.js").write_text("", encoding="utf-8")
    payload = valid_payload(auth)
    payload.pop("codex_oauth_auth_path")
    payload.update(
        {
            "auth_mode": "openai-api-key",
            "openai_api_key_env_path": str(auth),
            "model": {
                "provider": "openai",
                "model_id": "gpt-5.6-luna",
                "thinking_level": "medium",
            },
        }
    )
    with pytest.raises(ValidationError):
        PiBaselineConfig.model_validate(payload)

    auth.write_text("OPENAI_API_KEY=test-key\n", encoding="utf-8")
    auth.chmod(0o644)
    with pytest.raises(ValidationError):
        PiBaselineConfig.model_validate(payload)


@pytest.mark.parametrize(
    ("auth_mode", "provider"),
    [("codex-oauth", "openai"), ("openai-api-key", "openai-codex")],
)
def test_auth_mode_rejects_mismatched_model_provider(
    tmp_path: Path, auth_mode: str, provider: str
) -> None:
    auth = tmp_path / (".env" if auth_mode == "openai-api-key" else "oauth.json")
    auth.write_text(
        "OPENAI_API_KEY=test-key\n" if auth_mode == "openai-api-key" else "{}",
        encoding="utf-8",
    )
    auth.chmod(0o600)
    (tmp_path / "adapter.js").write_text("", encoding="utf-8")
    payload = valid_payload(auth)
    payload["auth_mode"] = auth_mode
    payload["model"] = {
        "provider": provider,
        "model_id": "gpt-5.6-luna",
        "thinking_level": "medium",
    }
    if auth_mode == "openai-api-key":
        payload.pop("codex_oauth_auth_path")
        payload["openai_api_key_env_path"] = str(auth)

    with pytest.raises(ValidationError):
        PiBaselineConfig.model_validate(payload)


@pytest.mark.parametrize(
    "command",
    [
        ["node", "adapter.js"],
        ["/usr/bin/npm", "run", "adapter"],
        ["/usr/bin/node", "adapter.js", "--extra"],
    ],
)
def test_adapter_command_rejects_wrappers_relative_files_and_extra_argv(
    command: list[str], tmp_path: Path
) -> None:
    adapter = tmp_path / "adapter.js"
    adapter.write_text("", encoding="utf-8")
    if command == ["/usr/bin/node", "adapter.js", "--extra"]:
        command[1] = str(adapter)
    with pytest.raises(ValueError):
        validate_node_adapter_command(command)


def test_adapter_command_accepts_only_exact_direct_invocation(tmp_path: Path) -> None:
    adapter = tmp_path / "adapter.js"
    adapter.write_text("", encoding="utf-8")
    assert validate_node_adapter_command(["/usr/bin/node", str(adapter)]) == [
        "/usr/bin/node",
        str(adapter),
    ]


@pytest.mark.parametrize(
    "change",
    [
        {
            "model": {
                "provider": "openai-codex",
                "model_id": "gpt-5.6-luna-medium",
                "thinking_level": "medium",
            }
        },
        {
            "model": {
                "provider": "openai-codex",
                "model_id": "gpt-5.6-luna",
                "thinking_level": "high",
            }
        },
        {"codex_oauth_auth_path": "/does/not/exist"},
        {"runner_image": "registry.example/pi:latest"},
        {"prompt_modes": ["unsupported"]},
        {"case_id": "bad id"},
        {"resource_limits": {"cpu_cores": 1.0, "memory_mb": 512, "pids": 10}},
    ],
)
def test_config_rejects_unsafe_or_incomplete_values(
    tmp_path: Path, change: dict[str, object]
) -> None:
    auth = tmp_path / "oauth.json"
    auth.write_text("{}", encoding="utf-8")
    payload = valid_payload(auth)
    payload.update(change)
    with pytest.raises(ValidationError):
        PiBaselineConfig.model_validate(payload)
