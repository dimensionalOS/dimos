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

import json
from pathlib import Path
from typing import Any, Callable

import httpx

from dimos.agents.llm_trace import request_path, response_path, tracing_http_client


def _client(trace_dir: Path, handler: Callable[[httpx.Request], httpx.Response]) -> httpx.Client:
    return tracing_http_client(trace_dir, transport=httpx.MockTransport(handler))


def _read(path: Path) -> dict[str, Any]:
    record: dict[str, Any] = json.loads(path.read_text())
    return record


def test_records_request_response_pair(tmp_path: Path) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, json={"pong": 1}, headers={"x-model": "gpt"})

    _client(tmp_path, handler).post(
        "https://api.test/v1/chat/completions",
        json={"ping": 1},
        headers={"authorization": "Bearer secret", "x-run": "7"},
    )

    request = _read(request_path(tmp_path, 0))
    assert request["method"] == "POST"
    assert request["url"] == "https://api.test/v1/chat/completions"
    assert request["body"] == {"ping": 1}
    assert "authorization" not in request["headers"]
    assert request["headers"]["x-run"] == "7"

    response = _read(response_path(tmp_path, 0))
    assert response["status"] == 200
    assert response["body"] == {"pong": 1}
    assert response["headers"]["x-model"] == "gpt"
    assert response["latency_s"] >= 0


def test_non_json_bodies_fall_back_to_text(tmp_path: Path) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(502, text="bad gateway")

    _client(tmp_path, handler).post("https://api.test/v1/chat/completions", content=b"raw bytes")

    assert _read(request_path(tmp_path, 0))["body"] == "raw bytes"
    assert _read(response_path(tmp_path, 0))["body"] == "bad gateway"


def test_successive_calls_get_increasing_seq(tmp_path: Path) -> None:
    client = _client(tmp_path, lambda request: httpx.Response(200, json={}))
    client.post("https://api.test/a", json={})
    client.post("https://api.test/b", json={})

    assert _read(request_path(tmp_path, 1))["url"] == "https://api.test/b"
    assert _read(response_path(tmp_path, 1))["status"] == 200
