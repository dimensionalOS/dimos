# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Iterator
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture
import requests

from dimos.agents.mcp.mcp_client import McpClient


@pytest.fixture
def session(mocker: MockerFixture) -> MagicMock:
    session: MagicMock = mocker.create_autospec(requests.Session, instance=True, spec_set=True)
    mocker.patch("dimos.agents.mcp.mcp_client.requests.Session", return_value=session)
    return session


@pytest.fixture
def client(session: MagicMock) -> Iterator[McpClient]:
    client = McpClient("http://localhost:9990/mcp")
    yield client
    client.close()


def test_request_raises_on_jsonrpc_error(client: McpClient, session: MagicMock) -> None:
    session.post.return_value.json.return_value = {
        "jsonrpc": "2.0",
        "id": 1,
        "error": {"code": -32601, "message": "Unknown: bad/method"},
    }

    with pytest.raises(RuntimeError, match="Unknown: bad/method"):
        client.request("bad/method")


def test_call_tool_sends_progress_token(client: McpClient, session: MagicMock) -> None:
    session.post.return_value.json.return_value = {"jsonrpc": "2.0", "id": 1, "result": {}}

    client.call_tool("add", {"x": 1, "y": 2})

    body = session.post.call_args.kwargs["json"]
    assert body["method"] == "tools/call"
    assert body["params"]["name"] == "add"
    assert body["params"]["arguments"] == {"x": 1, "y": 2}
    assert body["params"]["_meta"]["progressToken"]


def test_list_tools_raises_when_no_server(client: McpClient, session: MagicMock) -> None:
    session.post.side_effect = requests.ConnectionError

    with pytest.raises(RuntimeError, match="Failed to fetch tools"):
        client.list_tools(timeout=0)
