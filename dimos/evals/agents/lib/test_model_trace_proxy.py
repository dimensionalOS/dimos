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

from contextlib import closing
from http.server import ThreadingHTTPServer
import json
from queue import Queue
from threading import Thread

import httpx
import pytest
import requests

from dimos.agents.llm_trace import tracing_http_client
from dimos.evals.agents.lib import model_trace_proxy as proxy


@pytest.fixture
def proxy_server(tmp_path, mocker, request):
    upstream = mocker.Mock(
        side_effect=[
            httpx.Response(503, text="retry"),
            *[httpx.Response(201, json={"answer": 42}) for _ in range(2)],
        ]
    )
    servers = Queue()

    def make_server(*args):
        server = ThreadingHTTPServer(*args)
        servers.put(server)
        return server

    mocker.patch.object(proxy, "ThreadingHTTPServer", side_effect=make_server)
    with closing(
        tracing_http_client(tmp_path / "raw", transport=httpx.MockTransport(upstream))
    ) as client:
        mocker.patch.object(proxy, "tracing_http_client", return_value=client)
        thread = Thread(
            target=proxy._serve,
            args=(tmp_path / "raw", "https://provider.test/v1", request.param, tmp_path / "limit"),
        )
        thread.start()
        server = servers.get(timeout=5)
        try:
            yield f"http://127.0.0.1:{server.server_port}", upstream
        finally:
            server.shutdown()
            thread.join(timeout=5)
        assert not thread.is_alive()


@pytest.mark.parametrize(
    ("proxy_server", "last_status", "forwarded", "limited"),
    [(None, 201, 3, False), (2, 400, 2, True)],
    indirect=["proxy_server"],
)
def test_forwarding_records_retries_and_counts_failed_attempts(
    proxy_server, tmp_path, last_status, forwarded, limited
):
    url, upstream = proxy_server
    body = b'{"model": "pi", "stream": false}'
    with requests.Session() as session:
        session.trust_env = False
        responses = [
            session.post(
                url + "/chat/completions?version=1",
                data=body,
                timeout=5,
                headers={"Authorization": "Bearer secret", "Host": "client.test", "X-Run": "7"},
            )
            for _ in range(3)
        ]
    assert [response.status_code for response in responses] == [503, 201, last_status]
    assert responses[0].content == b"retry"
    assert responses[0].headers["Content-Type"] == "text/plain; charset=utf-8"
    assert responses[1].json() == {"answer": 42}
    assert (tmp_path / "limit").exists() == limited
    assert upstream.call_count == forwarded
    sent = upstream.call_args.args[0]
    assert str(sent.url) == "https://provider.test/v1/chat/completions?version=1"
    assert sent.content == body
    assert sent.headers["host"] == "provider.test"
    assert sent.headers["authorization"] == "Bearer secret"
    recorded = {
        path.name: json.loads(path.read_text()) for path in (tmp_path / "raw").glob("*.json")
    }
    assert len(recorded) == forwarded * 2
    request = recorded["000-request.json"]
    assert request["body"] == {"model": "pi", "stream": False}
    assert request["headers"]["x-run"] == "7"
    assert "authorization" not in request["headers"]
    failed, success = recorded["000-response.json"], recorded[f"{forwarded - 1:03d}-response.json"]
    assert (failed["status"], failed["body"]) == (503, "retry")
    assert (success["status"], success["body"]) == (201, {"answer": 42})


def test_proxy_stops_process_when_caller_fails(tmp_path, mocker):
    popen = mocker.spy(proxy.subprocess, "Popen")
    with requests.Session() as session:
        session.trust_env = False
        with (
            pytest.raises(ValueError, match="caller failed"),
            proxy.model_trace_proxy(
                tmp_path / "raw",
                "https://provider.test/",
                max_requests=0,
                limit_reached=tmp_path / "limit",
            ) as url,
        ):
            assert session.post(url, json={}, timeout=5).status_code == 400
            raise ValueError("caller failed")
    assert popen.spy_return.poll() is not None
    assert (tmp_path / "limit").exists()
