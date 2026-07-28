from dimos.agents.mcp.mcp_adapter import McpAdapter


def test_loopback_mcp_bypasses_environment_proxies() -> None:
    assert McpAdapter("http://127.0.0.1:9990/mcp")._session.trust_env is False
    assert McpAdapter("http://localhost:9990/mcp")._session.trust_env is False


def test_remote_mcp_keeps_environment_proxy_support() -> None:
    assert McpAdapter("https://robot.example/mcp")._session.trust_env is True
