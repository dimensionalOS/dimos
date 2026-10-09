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

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import threading
from typing import Any

import pytest
from pytest_mock import MockerFixture
import requests_mock
import yaml

from experimental.gateway.utils import config, runs


def test_concurrent_launches_spawn_only_one_process(
    server_home: Path, checkout: Path, mocker: MockerFixture
) -> None:
    entered = threading.Event()
    release = threading.Event()
    second = threading.Event()
    child = mocker.Mock(pid=123456, wait=mocker.Mock())

    def spawn(*args: Any, **kwargs: Any) -> Any:
        entered.set()
        assert release.wait(5)
        return child

    popen = mocker.patch.object(runs.subprocess, "Popen", side_effect=spawn)
    mocker.patch.object(
        runs,
        "current_launch",
        side_effect=lambda: {"phase": "starting", "blueprint": "fixture", "pid": child.pid}
        if runs.launch_file().exists()
        else None,
    )

    def launch(name: str) -> dict[str, Any]:
        second.set()
        return runs.start(checkout, name, runs.LaunchConfig())

    with ThreadPoolExecutor(max_workers=2) as pool:
        first = pool.submit(runs.start, checkout, "fixture-a", runs.LaunchConfig())
        try:
            assert entered.wait(5)
            other = pool.submit(launch, "fixture-b")
            assert second.wait(5)
        finally:
            release.set()
        assert first.result()["phase"] == "starting"
        with pytest.raises(runs.StillRunningError):
            other.result()
    assert popen.call_count == 1
    assert json.loads(runs.launch_file().read_text())["blueprint"] == "fixture-a"


def test_offline_config_transactions_keep_both_edits(server_home: Path) -> None:
    with ThreadPoolExecutor(max_workers=2) as pool:
        one = pool.submit(config.set_global_config_overrides, {"robot_ip": "fixture"})
        two = pool.submit(config.set_module_config, "fixture", {"camera": {"fps": 12}})
        one.result()
        two.result()
    assert config.global_config_overrides() == {"robot_ip": "fixture"}
    assert config.module_config("fixture") == {"camera": {"fps": 12}}


def test_running_desktop_is_the_only_config_writer(
    server_home: Path, monkeypatch: pytest.MonkeyPatch, requests_mock: requests_mock.Mocker
) -> None:
    path = config.config_file()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(
            {
                "desktop": {"port": 4567},
                "dimos": {
                    "global_config": {"robot_ip": "old", "replay": True},
                    "module_config": {"fixture": {"camera": {"fps": 24}, "audio": {"gain": 2}}},
                },
            }
        )
    )
    before = path.read_bytes()
    monkeypatch.setenv("DESKTOP_URL", "http://desktop.invalid")
    request = requests_mock.put("http://desktop.invalid/api/config", json={})
    config.set_global_config_overrides({"robot_ip": "new"})
    assert request.last_request.json() == {
        "dimos": {"global_config": {"robot_ip": "new", "replay": None}}
    }
    config.set_module_config("fixture", {"camera": {"fps": 12}})
    assert request.last_request.json() == {
        "dimos": {"module_config": {"fixture": {"camera": {"fps": 12}, "audio": {"gain": None}}}}
    }
    config.set_module_config("fixture", {})
    assert request.last_request.json() == {
        "dimos": {"module_config": {"fixture": {"camera": {"fps": None}, "audio": {"gain": None}}}}
    }
    assert path.read_bytes() == before


def test_cleared_desktop_overrides_can_be_readded(server_home: Path) -> None:
    path = config.config_file()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "dimos:\n  global_config:\n    replay: null\n  module_config:\n    fixture:\n      camera:\n        fps: null\n"
    )
    assert config.global_config_overrides() == {}
    assert config.module_config("fixture") == {}
    config.set_module_config("fixture", {"camera": {"fps": 30}})
    config.set_global_config_overrides({"replay": True})
    assert config.module_config("fixture") == {"camera": {"fps": 30}}
    assert config.global_config_overrides() == {"replay": True}


def test_atomic_writes_use_independent_temporary_files(tmp_path: Path) -> None:
    path = tmp_path / "state.json"
    values = [json.dumps({"value": n}) for n in range(20)]
    with ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(lambda value: config.write_atomic(path, value), values))
    assert path.read_text() in values
    assert list(tmp_path.iterdir()) == [path]


def test_desktop_replaces_nested_arguments_without_losing_nulls(
    server_home: Path, monkeypatch: pytest.MonkeyPatch, requests_mock: Any
) -> None:
    path = config.config_file()
    path.parent.mkdir(parents=True, exist_ok=True)
    saved = {
        "dimos": {
            "global_config": {"options": {"stale": 1}},
            "module_config": {"fixture": {"Echo": {"settings": {"keep": 1, "remove": 2}}}},
        }
    }
    path.write_text(yaml.safe_dump(saved))
    monkeypatch.setenv("DESKTOP_URL", "http://desktop.invalid")

    def merge(target: dict[str, Any], patch: dict[str, Any]) -> None:
        for key, value in patch.items():
            if isinstance(value, dict) and isinstance(target.get(key), dict):
                merge(target[key], value)
            else:
                target[key] = value

    def desktop_put(request: Any, context: Any) -> dict[str, Any]:
        merge(saved, request.json())
        path.write_text(yaml.safe_dump(saved))
        return {}

    requests_mock.put("http://desktop.invalid/api/config", json=desktop_put)
    config.set_module_config("fixture", {"Echo": {"settings": {"keep": 3, "nullable": None}}})
    assert config.module_config("fixture") == {"Echo": {"settings": {"keep": 3, "nullable": None}}}
    config.set_global_config_overrides({"options": {"nullable": None}})
    assert config.global_config_overrides() == {"options": {"nullable": None}}


def test_failed_desktop_replacement_restores_reset_fields(
    server_home: Path, monkeypatch: pytest.MonkeyPatch, requests_mock: Any
) -> None:
    path = config.config_file()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("dimos:\n  global_config:\n    options:\n      old: 1\n")
    monkeypatch.setenv("DESKTOP_URL", "http://desktop.invalid")
    request = requests_mock.put(
        "http://desktop.invalid/api/config",
        [
            {"json": {}},
            {"status_code": 500},
            {"json": {}},
        ],
    )
    import requests

    with pytest.raises(requests.HTTPError):
        config.set_global_config_overrides({"options": {"new": 2}})
    assert request.call_count == 3
    assert request.last_request.json() == {"dimos": {"global_config": {"options": {"old": 1}}}}


def test_desktop_replacement_blocks_gateway_readers(
    server_home: Path, monkeypatch: pytest.MonkeyPatch, requests_mock: Any
) -> None:
    path = config.config_file()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("dimos:\n  global_config:\n    options:\n      old: 1\n")
    monkeypatch.setenv("DESKTOP_URL", "http://desktop.invalid")
    cleared = threading.Event()
    release = threading.Event()
    reader_started = threading.Event()
    calls = 0

    def put(request: Any, context: Any) -> dict[str, Any]:
        nonlocal calls
        calls += 1
        if calls == 1:
            path.write_text("dimos:\n  global_config:\n    options: null\n")
            cleared.set()
            assert release.wait(5)
        else:
            path.write_text(yaml.safe_dump(request.json()))
        return {}

    def read() -> dict[str, Any]:
        reader_started.set()
        return config.global_config_overrides()

    requests_mock.put("http://desktop.invalid/api/config", json=put)
    with ThreadPoolExecutor(max_workers=2) as pool:
        writer = pool.submit(config.set_global_config_overrides, {"options": {"new": 2}})
        assert cleared.wait(5)
        reader = pool.submit(read)
        assert reader_started.wait(5)
        assert not reader.done()
        release.set()
        writer.result()
        assert reader.result() == {"options": {"new": 2}}
