# L0 MCP 端点加固 实现规格

对应 DASL proposal §5.0。P0 交付物，半天到一天的量，MCP server 裸监听、CORS 全开、无认证，同网段任何人 curl 一下就能让机器人动。

改动面：`mcp_server.py` 加认证中间件、收紧 listen_host 和 CORS；`GlobalConfig` 加两个字段；`agent_send` 标注高风险。不新增模块，不改 RPC 路径。

## 1. 现状（代码里能直接看到的）
- `CORSMiddleware(allow_origins=["*"], allow_headers=["*"])` —— 任何网页都能跨域调；
- `_handle_tools_call` 没有任何调用方身份概念；
- `listen_host` 从 `GlobalConfig` 来，默认不是 `127.0.0.1`；
- `agent_send` 是个普通 skill，谁都能调，效果是往 `/human_input` 塞任意文本——等于任何人都能伪装成用户给 Agent 下指令。

## 2. 配置项

`GlobalConfig` 新增两个字段（沿用现有 env 前缀惯例）：

| 字段 | env | 默认 | 说明 |
|------|-----|------|------|
| `mcp_auth_token` | `DIMOS_MCP_AUTH_TOKEN` | `None` | 不设 = 仅 localhost 可访问 |
| `mcp_allowed_origins` | `DIMOS_MCP_ALLOWED_ORIGINS` | `[]` | CORS 白名单，逗号分隔 |

语义就一条规则，别搞复杂：

- `listen_host` 是 `127.0.0.1`（回环）：不要 token，本机进程默认可信；
- `listen_host` 绑到非回环地址：必须设 `mcp_auth_token`，否则 `McpServer.start` 直接拒绝启动。

第二条是 fail-closed 的关键——"绑到 0.0.0.0 又忘了设 token"这个组合必须在启动时就炸，而不是运行时被扫到。

## 3. 认证

HTTP Bearer token，加在 FastAPI 中间件里，不用引新依赖：

```python
@app.middleware("http")
async def auth(request: Request, call_next):
    if request.url.path != "/mcp":
        return await call_next(request)
    if _is_loopback(request.client.host):
        return await call_next(request)
    expected = global_config.mcp_auth_token
    got = request.headers.get("authorization", "").removeprefix("Bearer ")
    if not expected or not hmac.compare_digest(got, expected):
        return JSONResponse({"error": "unauthorized"}, status_code=401)
    return await call_next(request)
```

要点：

- 比较用 `hmac.compare_digest`，不用 `==`。时序攻击对机器人场景威胁不大，但一行就能堵的事没理由留着；
- `McpClient` 侧对应改动：非回环地址时自动带上 `Authorization: Bearer <token>`，token 同样从 `GlobalConfig` 读——同一份配置，client/server 不用各配一遍；
- SSE 端点（`GET /mcp`）走同一个中间件，一起保护，别只守 POST。

**明确不做**：mTLS、JWT、多用户权限分级。机器人上就是一个操作员对一个实例，Bearer 够了。真要多用户，那是以后 L1 信任级体系成熟之后的事。

## 4. CORS 收紧

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=global_config.mcp_allowed_origins,  # 默认空列表 = 全拒
    allow_methods=["POST", "GET"],
    allow_headers=["authorization", "content-type"],
)
```

默认空白名单意味着浏览器跨域全拒，本机 curl/CLI 不受影响（CORS 只管浏览器）。要用 web UI 再显式配 origin。

## 5. `agent_send` 降级处理

`agent_send` 在 L2 落地后会被标成 `RiskLevel.EXTERNAL`（触发确认流）。但 L0 等不到 L2，现在就要处理：

- 在 `_handle_tools_call` 里加一个临时硬编码检查：`agent_send` 只允许回环调用；
- 代码里注释标 `TODO(L2): remove once RiskLevel.EXTERNAL lands`，L2 合入时删掉。

临时独立成立，而且删的时候就是删一个 if。

## 6. 请求体大小限制

`/mcp` 端点现在 `await request.body()` 无限制读。加 1MB 上限，超了直接 413。JSON-RPC 的参数里不可能有接近 1MB 的正常请求，真有就是异常。

## 7. 不变量

1. **I1**：非回环、无 token、token 错误三种情况，一律 401，且响应体不区分是哪种（不泄露"token 存在但你错了"）。
2. **I2**：`listen_host` 非回环 + 未配 token → 启动失败，没有警告降级这种中间态。
3. **I3**：本机回环调用的行为与加固前完全一致（现有 CLI、`dimos mcp call` 不受影响）。
4. **I4**：CORS 默认全拒，配了白名单也只放名单内的 origin。

## 8. 测试（`dimos/agents/mcp/test_mcp_security.py`）

| 用例 | 做法 | 期望 |
|------|------|------|
| 回环免 token | 127.0.0.1 请求 /mcp，无 header | 正常响应 |
| 非回环无 token | mock client.host 为非回环 | 401 |
| 错误 token | 带错误 Bearer | 401，响应体与无 token 相同（I1） |
| 正确 token | 带正确 Bearer | 正常响应 |
| 启动保护 | listen_host=0.0.0.0 + 无 token | `start` 抛错 |
| SSE 同规则 | GET /mcp 无 token（非回环） | 401 |
| agent_send 限制 | 非回环调 agent_send | 拒绝；回环调正常 |
| 请求体超限 | POST 2MB body | 413 |
| 现有回归 | `test_mcp_server.py` 全量 | 全绿（I3） |

## 9. 验收

1. 上面用例全过，进 fast tests。
2. `mypy --strict` 过。
3. 不加任何新第三方依赖。
4. 文档 `docs/usage/cli.md` 补一段：远程访问 MCP 需要配什么。
