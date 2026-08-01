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

import json
from pathlib import Path

import pytest

from dimos.benchmark.agent_eval.config import (
    load_smoke_config,
    select_destination,
    verify_fresh_oracle,
)
from dimos.benchmark.dimsim.bundle import generate_smoke_release
from dimos.benchmark.dimsim.fixture import apartment_oracle_fixture


def _release(tmp_path: Path):
    root = tmp_path / "release"
    view = apartment_oracle_fixture()
    generate_smoke_release(view, root)
    destination = next(
        line
        for line in (root / "public" / "tasks.jsonl").read_text().splitlines()
        if json.loads(line)["category"] == "destination"
    )
    return root, view, json.loads(destination)["task_id"]


def _config(root: Path, task_id: str) -> dict[str, object]:
    return {
        "release_root": str(root),
        "task_id": task_id,
        "output_root": "attempts",
        "mcp_endpoint": "http://127.0.0.1:9990/mcp",
        "pi": {
            "model": "gpt-5.6-luna",
            "thinking_level": "medium",
            "auth_mode": "environment",
            "credential_env": "OPENAI_API_KEY",
        },
        "timeouts": {
            "readiness_s": 10.0,
            "mcp_call_s": 10.0,
            "reset_s": 20.0,
            "evaluation_start_s": 10.0,
            "cancellation_s": 5.0,
        },
        "episode_timeout_s": 180.0,
        "dimsim": {
            "endpoint": "http://127.0.0.1:8090",
            "expected_scene_id": "dimsim-apartment",
        },
    }


def test_load_config_resolves_paths_and_never_retains_secret(tmp_path, monkeypatch) -> None:
    root, _, task_id = _release(tmp_path)
    payload = _config(root, task_id)
    config_path = tmp_path / "smoke.json"
    config_path.write_text(json.dumps(payload))
    monkeypatch.setenv("OPENAI_API_KEY", "secret-value")

    loaded = load_smoke_config(config_path)

    assert loaded.resolved.output_root == str((tmp_path / "attempts").resolve())
    assert loaded.credential.value == "secret-value"
    assert "secret-value" not in loaded.resolved.model_dump_json()


def test_config_rejects_predicate_override(tmp_path, monkeypatch) -> None:
    root, _, task_id = _release(tmp_path)
    payload = _config(root, task_id)
    payload["threshold_m"] = 2.0
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(payload))
    monkeypatch.setenv("OPENAI_API_KEY", "secret")

    with pytest.raises(ValueError, match="threshold_m"):
        load_smoke_config(path)


def test_selects_only_destination_and_derives_profile_spawn(tmp_path) -> None:
    root, view, task_id = _release(tmp_path)

    selected = select_destination(root, task_id)

    assert selected.public.category == "destination"
    assert (selected.start_pose.x_m, selected.start_pose.z_m) == (2.0, 3.0)
    with pytest.raises(ValueError, match="spawn"):
        verify_fresh_oracle(selected, view)


def test_fresh_oracle_rejects_revision_or_content_change(tmp_path) -> None:
    root, view, task_id = _release(tmp_path)
    selected = select_destination(root, task_id)

    with pytest.raises(ValueError, match="source revisions"):
        verify_fresh_oracle(
            selected,
            view.model_copy(update={"reset_revision": "different-reset"}),
        )
    changed = view.model_copy(
        update={
            "entities": (
                view.entities[0].model_copy(update={"position": (99.0, 99.0)}),
                *view.entities[1:],
            )
        }
    )
    with pytest.raises(ValueError, match="content digest"):
        verify_fresh_oracle(selected, changed)
