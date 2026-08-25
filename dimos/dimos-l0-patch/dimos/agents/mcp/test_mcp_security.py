from __future__ import annotations

import asyncio
from collections.abc import Iterator
from contextlib import contextmanager
import json
from unittest.mock import MagicMock

import httpx
import pytest

from dimos.agents.mcp.mcp_server import (
    MAX_MCP_BODY_BYTES,
    _check_bind_allowed,
    app,
    handle_request,
)
from dimos.core.global_config import global_config
from dimos.core.module import SkillInfo

_TOKEN = "test-secret-token"
_INIT_RPC = {"jsonrpc": "2.0", "id": 1, "method": "initialize"}
_REMOTE_HOST = "10.0.0.5"
_LOOPBACK_HOST = "127.0.0.1"


@contextmanager
def _auth_token(token: str | None) -> Iterator[None]:
    """Set mcp_auth_token for the duration of a test, then restore."""
    old_token = global_config.mcp_auth_token
    global_config.update(mcp_auth_token=token)
    try:
        yield
    finally:
        global_config.update(mcp_auth_token=old_token)


def _post(host: str, body: bytes, headers: dict[str, str] | None = None) -> httpx.Response:
    """POST to /mcp through ASGI with the given TCP peer host."""

    async def _go() -> httpx.Response:
        transport = httpx.ASGITransport(app=app, client=(host, 12345))
        async with httpx.AsyncClient(transport=transport, base_url="http://test") as client:
            return await client.post(
                "/mcp",
                content=body,
                headers={"content-type": "application/json", **(headers or {})},
            )

    return asyncio.run(_go())


def _get(host: str, headers: dict[str, str] | None = None) -> httpx.Response:
    async def _go() -> httpx.Response:
        transport = httpx.ASGITransport(app=app, client=(host, 12345))
        async with httpx.AsyncClient(transport=transport, base_url="http://test") as client:
            return await client.get("/mcp", headers=headers or {})

    return asyncio.run(_go())


def _options(host: str, headers: dict[str, str]) -> httpx.Response:
    async def _go() -> httpx.Response:
        transport = httpx.ASGITransport(app=app, client=(host, 12345))
        async with httpx.AsyncClient(transport=transport, base_url="http://test") as client:
            return await client.options("/mcp", headers=headers)

    return asyncio.run(_go())


def _rpc(body: dict[str, object]) -> bytes:
    return json.dumps(body).encode()


def test_loopback_request_needs_no_token() -> None:
    """I3: loopback behavior is unchanged from before the hardening."""
    with _auth_token(None):
        response = _post(_LOOPBACK_HOST, _rpc(_INIT_RPC))
    assert response.status_code == 200
    assert response.json()["result"]["serverInfo"]["name"] == "dimensional"


def test_non_loopback_without_token_is_401() -> None:
    with _auth_token(_TOKEN):
        response = _post(_REMOTE_HOST, _rpc(_INIT_RPC))
    assert response.status_code == 401
    assert response.json() == {"error": "unauthorized"}


def test_non_loopback_is_401_when_no_token_configured() -> None:
    """Default config is fail-closed: no token = loopback-only access."""
    with _auth_token(None):
        response = _post(_REMOTE_HOST, _rpc(_INIT_RPC))
    assert response.status_code == 401
    assert response.json() == {"error": "unauthorized"}


def test_wrong_token_response_is_indistinguishable_from_missing_token() -> None:
    """I1: the 401 body must not leak whether a token was supplied."""
    with _auth_token(_TOKEN):
        missing = _post(_REMOTE_HOST, _rpc(_INIT_RPC))
        wrong = _post(
            _REMOTE_HOST, _rpc(_INIT_RPC), headers={"authorization": "Bearer wrong-token"}
        )
    assert missing.status_code == wrong.status_code == 401
    assert missing.content == wrong.content


def test_correct_token_is_accepted() -> None:
    with _auth_token(_TOKEN):
        response = _post(
            _REMOTE_HOST, _rpc(_INIT_RPC), headers={"authorization": f"Bearer {_TOKEN}"}
        )
    assert response.status_code == 200
    assert response.json()["result"]["serverInfo"]["name"] == "dimensional"


def test_bind_guard_refuses_non_loopback_without_token() -> None:
    """I2: fail-closed at startup, no warn-and-continue middle state."""
    with pytest.raises(RuntimeError, match="mcp_auth_token"):
        _check_bind_allowed("0.0.0.0", None)
    # Token present or loopback bind: both fine.
    _check_bind_allowed("0.0.0.0", _TOKEN)
    _check_bind_allowed("127.0.0.1", None)


def test_sse_channel_follows_the_same_auth_rule() -> None:
    """GET /mcp (SSE) is protected by the same middleware as POST."""
    with _auth_token(_TOKEN):
        response = _get(_REMOTE_HOST)
    assert response.status_code == 401
    assert response.json() == {"error": "unauthorized"}


def _agent_send_call(client_host: str | None) -> tuple[dict[str, object] | None, MagicMock]:
    schema = json.dumps({"type": "object", "properties": {"message": {"type": "string"}}})
    skill = SkillInfo(class_name="McpServer", func_name="agent_send", args_schema=schema)
    rpc_call = MagicMock(return_value="sent")
    response = asyncio.run(
        handle_request(
            {
                "jsonrpc": "2.0",
                "id": 1,
                "method": "tools/call",
                "params": {"name": "agent_send", "arguments": {"message": "hi"}},
            },
            [skill],
            {"agent_send": rpc_call},
            client_host=client_host,
        )
    )
    return response, rpc_call


def test_agent_send_refused_for_non_loopback_caller() -> None:
    # TODO(L2): remove once RiskLevel.EXTERNAL lands
    response, rpc_call = _agent_send_call(_REMOTE_HOST)
    assert response is not None
    assert response["error"]["code"] == -32600  # type: ignore[index]
    rpc_call.assert_not_called()


def test_agent_send_allowed_for_loopback_caller() -> None:
    response, rpc_call = _agent_send_call(_LOOPBACK_HOST)
    assert response is not None
    assert "error" not in response
    rpc_call.assert_called_once_with(message="hi")


def test_payload_over_limit_is_413() -> None:
    big_body = b" " * (MAX_MCP_BODY_BYTES + 1)
    response = _post(_LOOPBACK_HOST, big_body)
    assert response.status_code == 413


def test_cors_denies_cross_origin_by_default() -> None:
    """I4: empty allowlist = browsers get no allow-origin (preflight 400)."""
    response = _options(
        _LOOPBACK_HOST,
        headers={
            "origin": "http://evil.example",
            "access-control-request-method": "POST",
        },
    )
    assert response.status_code == 400
    assert "access-control-allow-origin" not in {
        k.lower() for k in response.headers.keys()
    }
