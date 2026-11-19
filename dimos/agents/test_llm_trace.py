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

import base64
from collections.abc import Callable
import json
from pathlib import Path
from typing import Any

import httpx
from pytest_mock import MockerFixture

from dimos.agents.llm_trace import (
    last_request_unanswered,
    latest_pair,
    list_llm_trace_pairs,
    request_path,
    response_path,
    save_request_images,
    seen_dir,
    tracing_http_client,
    write_normalized,
)


def _client(trace_dir: Path, handler: Callable[[httpx.Request], httpx.Response]) -> httpx.Client:
    return tracing_http_client(trace_dir, transport=httpx.MockTransport(handler))


def _read(path: Path) -> dict[str, Any]:
    record: dict[str, Any] = json.loads(path.read_text())
    return record


def test_records_request_response_pair(tmp_path: Path, mocker: MockerFixture) -> None:
    clock = mocker.patch("dimos.agents.llm_trace.time")
    clock.time.return_value = 1_700_000_000.0
    clock.monotonic.side_effect = [0.0, 0.25]

    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, json={"pong": 1}, headers={"x-model": "gpt"})

    with _client(tmp_path, handler) as client:
        client.post(
            "https://api.test/v1/chat/completions",
            json={"ping": 1},
            headers={"authorization": "Bearer secret", "x-goog-api-key": "secret", "x-run": "7"},
        )

    request = _read(request_path(tmp_path, 0))
    assert request["started_at"] == 1_700_000_000.0
    assert request["method"] == "POST"
    assert request["url"] == "https://api.test/v1/chat/completions"
    assert request["body"] == {"ping": 1}
    assert "authorization" not in request["headers"]
    assert "x-goog-api-key" not in request["headers"]
    assert request["headers"]["x-run"] == "7"

    response = _read(response_path(tmp_path, 0))
    assert response["status"] == 200
    assert response["body"] == {"pong": 1}
    assert response["headers"]["x-model"] == "gpt"
    assert response["latency_s"] == 0.25


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


def test_trace_discovery_orders_complete_calls_and_respects_sequence_boundary(
    tmp_path: Path,
) -> None:
    for seq in (1000, 999):
        request_path(tmp_path, seq).write_text("{}")
        response_path(tmp_path, seq).write_text("{}")
    request_path(tmp_path, 1001).write_text("{}")  # still in flight
    (tmp_path / "unrelated-request.json").write_text("{}")
    (tmp_path / "unrelated-response.json").write_text("{}")

    expected = [
        (seq, request_path(tmp_path, seq), response_path(tmp_path, seq)) for seq in (999, 1000)
    ]
    assert list_llm_trace_pairs(tmp_path) == expected
    assert latest_pair(tmp_path, after=1000) == expected[-1]
    assert latest_pair(tmp_path, after=1001) is None
    assert last_request_unanswered(tmp_path)
    response_path(tmp_path, 1001).write_text("{}")
    assert not last_request_unanswered(tmp_path)


def _b64(data: bytes) -> str:
    return base64.b64encode(data).decode()


def _data_url(data: str) -> dict[str, str]:
    return {"type": "input_image", "image_url": f"data:image/jpeg;base64,{data}"}


def test_images_are_saved_once_as_their_request_goes_out(tmp_path: Path) -> None:
    raw = tmp_path / "raw"
    seen = seen_dir(raw)
    prompt, frame, crop = _b64(b"prompt"), _b64(b"frame"), _b64(b"crop")
    bodies = [
        {"input": [_data_url(prompt)]},
        # The whole history again, plus the frame a tool fetched (Anthropic's form).
        {
            "input": [_data_url(prompt)],
            "messages": [
                {
                    "type": "image",
                    "source": {"type": "base64", "media_type": "image/png", "data": frame},
                }
            ],
        },
        # Gemini's inline data.
        {"contents": [{"parts": [{"inline_data": {"mime_type": "image/webp", "data": crop}}]}]},
    ]
    saved_after = []

    def handler(request: httpx.Request) -> httpx.Response:
        saved_after.append(sorted(path.name for path in seen.glob("*")) if seen.exists() else [])
        return httpx.Response(200, json={})

    with _client(raw, handler) as client:
        for body in bodies:
            client.post("https://api.test/v1/responses", json=body)

    # Each image is on disk before the model answers the request that carries it.
    assert saved_after == [
        ["r000-0.jpg"],
        ["r000-0.jpg", "r001-0.png"],
        ["r000-0.jpg", "r001-0.png", "r002-0.webp"],
    ]
    assert (seen / "r000-0.jpg").read_bytes() == b"prompt"
    assert (seen / "r001-0.png").read_bytes() == b"frame"
    assert (seen / "r002-0.webp").read_bytes() == b"crop"


def test_normalized_calls_save_their_images_too(tmp_path: Path, mocker: MockerFixture) -> None:
    raw = tmp_path / "raw"
    message = mocker.Mock()
    message.model_dump.return_value = {
        "content": [
            {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{_b64(b'shot')}"}}
        ]
    }
    result = mocker.Mock()
    result.model_dump.return_value = {}
    write_normalized(raw, [message], result)
    write_normalized(raw, [message], result)
    assert [path.name for path in seen_dir(raw).iterdir()] == ["r000-0.png"]


def test_requests_without_images_leave_no_seen_folder(tmp_path: Path) -> None:
    raw = tmp_path / "raw"
    with _client(raw, lambda request: httpx.Response(200, json={})) as client:
        client.post("https://api.test/v1/responses", json={"input": "hi"})
    assert not seen_dir(raw).exists()


def _message(mocker: MockerFixture, *urls: str) -> Any:
    message = mocker.Mock()
    message.model_dump.return_value = {
        "content": [{"type": "image_url", "image_url": {"url": url}} for url in urls]
    }
    return message


def test_text_that_only_looks_like_an_image_is_skipped(
    tmp_path: Path, mocker: MockerFixture
) -> None:
    raw = tmp_path / "raw"
    result = mocker.Mock()
    result.model_dump.return_value = {"ok": 1}
    quoted = "data:image/png;base64,hello"  # a prompt quoting an example, not base64
    seq, _, response = write_normalized(
        raw, [_message(mocker, quoted, f"data:image/png;base64,{_b64(b'real')}")], result
    )
    assert _read(response)["result"] == {"ok": 1}
    assert [path.name for path in seen_dir(raw).iterdir()] == ["r000-0.png"]


def test_images_are_saved_before_a_normalized_call_goes_out(
    tmp_path: Path, mocker: MockerFixture
) -> None:
    raw = tmp_path / "raw"
    messages = [_message(mocker, f"data:image/jpeg;base64,{_b64(b'frame')}")]
    save_request_images(raw, messages)
    assert [path.name for path in seen_dir(raw).iterdir()] == ["r000-0.jpg"]
    result = mocker.Mock()
    result.model_dump.return_value = {}
    write_normalized(raw, messages, result)  # recorded after the model answers: no second copy
    assert [path.name for path in seen_dir(raw).iterdir()] == ["r000-0.jpg"]
