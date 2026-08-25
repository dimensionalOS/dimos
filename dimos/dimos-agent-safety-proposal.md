# L2 PolicyGate 实现规格

对应 DASL proposal §5.2。P1 阶段做，依赖 P0 的 `@skill` 元数据和审计骨架。

改动面：`dimos/agents/safety/` 是新目录；`annotation.py` 加装饰器参数；`mcp_server.py` 加一段前置检查；`module.py` 的 `SkillInfo` 加一个字段。就这些。

## 1. 要解决的问题

现在任何一个 `tools/call` 进来，server 查一下 capability 没被占就直接 RPC 出去了。我们要在这之前加一道确定性检查：风险分级、参数边界、限流、确认。要求是纯同步、亚毫秒——这条路在每个工具调用的热路径上，慢一点都不行。

为什么不放进 CapabilityRegistry？那层管的是"资源互斥"，语义上跟"这次调用该不该发生"是两回事，混在一起以后没法分别演进。

范围内：风险元数据、信任级检查、参数边界、限流、确认、shadow 模式、策略文件加载、审计事件。
不在范围内：L1 具体怎么给消息打标（这里只消费结果）、L3 监视器、L4 执行验证、确认按钮长什么样。

## 2. 模块结构

```
dimos/agents/safety/
├── __init__.py
├── policy.py            # PolicyGate, PolicyDecision, TrustLevel, MIN_TRUST
├── policy_loader.py     # YAML 加载与校验 → PolicyConfig
├── rate_limiter.py      # SlidingWindowLimiter
├── confirmation.py      # ConfirmationRegistry（挂起/批准/拒绝/超时）
└── policies/
    └── default.yaml     # 内置默认策略
```

## 3. 核心类型

### 3.1 枚举与元数据（`annotation.py` 扩展，P0 已交付）

```python
class RiskLevel(Enum):
    READONLY = "readonly"
    MOTION = "motion"
    DYNAMIC = "dynamic"
    MANIPULATION = "manipulation"
    EXTERNAL = "external"

class ConfirmMode(Enum):
    NONE = "none"
    OPERATOR = "operator"

@dataclass(frozen=True)
class SkillSafety:
    risk: RiskLevel = RiskLevel.READONLY
    bounds: dict[str, tuple[float, float]] = field(default_factory=dict)
    rate_limit: str | None = None          # "<n>/<s|min|h>"，如 "6/min"
    confirm: ConfirmMode = ConfirmMode.NONE
    verify: str | None = None
```

### 3.2 信任级（`policy.py`）

```python
class TrustLevel(IntEnum):
    UNTRUSTED = 0    # 网络文本、未认证 agent_send
    TOOL_RESULT = 1  # 工具/传感器返回值
    USER_VOICE = 2   # 现场语音（STT）
    OPERATOR = 3     # 本地认证操作员 / teleop UI

# 触发链信任级下限（硬规则，见 §5.1）
MIN_TRUST: dict[RiskLevel, TrustLevel] = {
    RiskLevel.READONLY:     TrustLevel.UNTRUSTED,
    RiskLevel.MOTION:       TrustLevel.TOOL_RESULT,
    RiskLevel.DYNAMIC:      TrustLevel.USER_VOICE,
    RiskLevel.MANIPULATION: TrustLevel.USER_VOICE,
    RiskLevel.EXTERNAL:     TrustLevel.OPERATOR,
}
```

### 3.3 裁决结果

```python
@dataclass(frozen=True)
class PolicyDecision:
    allowed: bool
    reason: str = ""                    # 人类可读；shadow 命中时带 "[shadow-would-deny] " 前缀
    needs_confirmation: bool = False
    confirmation_id: str | None = None  # needs_confirmation=True 时非空
```

### 3.4 策略配置（`policy_loader.py` 输出）

```python
@dataclass(frozen=True)
class PolicyConfig:
    mode: Literal["enforce", "shadow", "off"] = "shadow"
    defaults: dict[RiskLevel, SkillSafety]        # 按风险级的默认策略
    overrides: dict[str, SkillSafety]             # 按 skill 名的覆盖（优先级最高）
    confirmation_timeout_s: float = 30.0
```

**合并规则**：装饰器声明 < `defaults[risk]` < `overrides[skill_name]`，逐字段合并（不是整体替换，override 只写要改的那个字段就行）。

## 4. PolicyGate API

```python
class PolicyGate:
    def __init__(self, config: PolicyConfig,
                 confirmations: ConfirmationRegistry,
                 audit: AuditTrail | None) -> None: ...

    def check(self,
              skill_name: str,
              safety: SkillSafety,       # 来自 SkillInfo（装饰器元数据）
              args: dict[str, Any],
              provenance: TrustLevel) -> PolicyDecision: ...
```

几条硬约定：

- 纯同步、无 I/O（审计写盘除外），单次 < 1ms。它在每个工具调用的热路径上。
- 内部任何异常都不许抛出去，统一变成 `PolicyDecision(False, "policy gate error: ...")`。安全层自己出 bug 的时候宁可误拒，不能误放。
- 不碰机器人状态。允许的副作用只有限流窗口计数和审计事件。
- `mode="off"` 恒放行，但审计照记——不然排查问题时会怀疑"是不是安全层拦的"，答案必须可查。

## 5. 裁决顺序

```
1. mode == "off"            → allow
2. risk == READONLY         → allow（仍写审计）
3. provenance < MIN_TRUST[risk]
                            → deny "trust {X} insufficient for risk '{Y}'"
4. 参数边界（合并后 bounds）:
   对每个 (param, (lo, hi))，若 args[param] 是数值且越界
                            → deny "'{param}={v}' outside safe range [{lo}, {hi}]"
   （拒绝并告知合法区间，禁止静默裁剪）
5. 速率限制命中              → deny "rate limit {spec} exceeded for '{skill}'"
6. confirm == OPERATOR:
   - 无有效 confirmation    → ConfirmationRegistry.register() 生成 id，
                              返回 deny + needs_confirmation=True + confirmation_id
   - 调用方在 meta 携带已批准 id → 消费该 id，放行
7. 其余                     → allow
```

顺序是固定的，命中即返回，别加"智能"重排。

**shadow 模式**：3–6 步产生 deny 时改写成 allow，reason 加 `"[shadow-would-deny] "` 前缀。注意 shadow 下第 6 步**不**真的创建 ConfirmationRequest，只记审计——不然 shadow 期间会攒一堆没人处理的确认单。

## 6. 子组件规格

### 6.1 SlidingWindowLimiter（`rate_limiter.py`）

```python
class SlidingWindowLimiter:
    def hit(self, key: str, spec: str) -> bool:
        """记录一次调用；返回 True 表示已超限（应拒绝）。"""
```

- `spec` 语法：`"<n>/<s|min|h>"`，如 `"6/min"`。解析在加载期做，非法格式直接 `ValueError`，运行期不存在坏 spec。
- 每个 skill 一个 `deque[float]`（`time.monotonic()`），过期元素用的时候再弹，不维护后台清理。
- 线程安全：server 在 executor 线程里调 `check()`，窗口操作要拿锁。临界区只有 deque 的弹出和追加，微秒级，不会成为瓶颈。
- 被拒的调用不计入窗口。不然限流一旦触发就永远解锁不了——拒绝风暴自我锁死。

### 6.2 ConfirmationRegistry（`confirmation.py`）

```python
@dataclass(frozen=True)
class ConfirmationRequest:
    id: str                    # uuid4 hex
    skill_name: str
    args: dict[str, Any]
    created_ts: float
    expires_ts: float

class ConfirmationRegistry:
    def register(self, skill_name: str, args: dict) -> ConfirmationRequest: ...
    def approve(self, confirmation_id: str) -> bool: ...   # 不存在/过期 → False
    def consume(self, confirmation_id: str) -> bool: ...   # 一次性：取出并删除
    def pending(self) -> list[ConfirmationRequest]: ...
```

- 存活期 `confirmation_timeout_s`（默认 30s），过期在下次访问时顺手清掉。
- `consume` 是一次性的：同一个 id 放行一次就没了，不能拿一个 id 反复刷。
- 批准走 CLI：`dimos mcp confirm <id>`，待批列表 `dimos mcp confirmations`。复用现有 MCP 管理面，不加新端口。

### 6.3 策略 YAML（`policies/default.yaml`）

```yaml
mode: shadow
confirmation_timeout_s: 30
defaults:
  motion:        {rate_limit: "30/min"}
  dynamic:       {confirm: operator, rate_limit: "2/min"}
  manipulation:  {confirm: operator}
  external:      {confirm: operator}
overrides:
  move:
    bounds: {x: [-0.5, 0.5], duration: [0.0, 5.0]}
  execute_sport_command:
    rate_limit: "2/min"
```

- 加载期就把错都报出来：不认识的 risk 名、非法 rate_limit、bounds 的 `lo > hi`，一律 `ValueError`，宁可启动失败也不带错跑。
- 路径走 `GlobalConfig.safety_policy`，没配就用 `policies/default.yaml`。
- blueprint 可以传自己的 YAML，和 default 按上面的规则逐字段合并。

## 7. 接入 MCP server

改 `_handle_tools_call`，在现有 `CapabilityRegistry.acquire` 前面加一段，大概 15 行，原来的逻辑一行不动：

```python
safety: SkillSafety = skill_info.safety if skill_info else SkillSafety()
provenance = TrustLevel[meta.get("dimos/trust", "OPERATOR")]  # L1 注入；未接入 L1 前默认 OPERATOR

decision = app.state.policy_gate.check(name, safety, args, provenance)
app.state.audit.record({
    "skill": name, "args": args, "risk": safety.risk.value,
    "trust": provenance.name,
    "decision": "allow" if decision.allowed else "deny",
    "reason": decision.reason,
    "confirmation_id": decision.confirmation_id,
})
if not decision.allowed:
    if decision.needs_confirmation:
        return _jsonrpc_result_text(
            req_id,
            f"Action requires operator confirmation. id={decision.confirmation_id}. "
            f"Ask the operator to run: dimos mcp confirm {decision.confirmation_id}")
    return _jsonrpc_result_text(req_id, f"Safety policy: {decision.reason}")
```

其他要动的地方：

| 位置 | 改动 | 备注 |
|------|------|------|
| `dimos/core/module.py` | `SkillInfo` 加 `safety: SkillSafety` 字段 | `get_skills()` 顺手带上 `__safety__`；老代码没标的默认 READONLY |
| `McpServer.start` | 建 `PolicyGate` 挂到 `app.state.policy_gate` | 策略路径从 `GlobalConfig.safety_policy` 拿 |
| `McpServer.stop` | 不动 | 没有后台资源要收 |
| 启动失败 | YAML 非法 → `start` 直接抛错，模块不起来 | 没有策略文件就不开 MCP 面，这点不让步 |

另外 `_handle_tools_list` 的 `_meta` 里加 `dimos/risk`，客户端能提前知道风险级。纯信息展示，不参与裁决。

## 8. 几条不能破的原则

review 的时候照着这六条查：

1. **I1**：任何 `tools/call` 都必须过 `PolicyGate.check`，不存在第二条到 RPC 的路。
2. **I2**：`check()` 永远不抛异常给调用方，出错就是 deny。
3. **I3**：越界参数到不了 skill 实现——我们是拒绝，不是偷偷改成边界值。裁剪会让 Agent 学到错误的因果，下次还敢。
4. **I4**：一个 confirmation_id 只放行一次。
5. **I5**：`mode` 只决定拦不拦，不决定记不记。
6. **I6**：被拒的调用不消耗限流配额。

## 9. 测试（`dimos/agents/safety/test_policy.py`）

| 用例 | 输入 | 期望 |
|------|------|------|
| readonly 直通 | risk=READONLY, trust=UNTRUSTED | allow |
| 信任级不足 | risk=DYNAMIC, trust=TOOL_RESULT | deny, reason 含 "insufficient" |
| 信任级恰好达标 | risk=DYNAMIC, trust=USER_VOICE | 进入后续检查 |
| 参数越界 | bounds x∈[-0.5,0.5], args x=10.0 | deny, reason 含合法区间 |
| 边界值 | args x=0.5 / -0.5 | allow |
| 非数值参数 | args x="fast" | 跳过 bounds 检查（schema 层职责） |
| YAML override 优先 | 装饰器 bounds 宽、override 窄，取窄值越界 | deny |
| 速率限制 | "2/min" 连调 3 次 | 第 3 次 deny；被拒后窗口内第 4 次仍 deny（不消耗配额） |
| 窗口滑动 | 超限后 mock 时间前进 61s | allow |
| 确认流 | confirm=OPERATOR 首调 | deny + needs_confirmation + id |
| 确认消费 | approve(id) 后携带 id 重调 | allow；再次用同一 id → deny（I4） |
| 确认过期 | 注册后 mock 时间 +31s | approve → False |
| shadow 模式 | mode=shadow + 越界调用 | allow, reason 带 "[shadow-would-deny]"，审计记录原 deny |
| off 模式 | mode=off + 越界调用 | allow，审计仍记录（I5） |
| 内部异常 | bounds 传入畸形结构 | deny "policy gate error"（I2），不抛 |
| 并发 | 10 线程并发 hit 同一窗口 | 恰好 n 次 allow |
| 集成 | 起 McpServer，`tools/call` 越界 move | 返回 "Safety policy: ..."，capability 未被 acquire |

## 10. 验收

1. 上面的用例全过，`uv run pytest dimos/agents/safety/` 进默认 fast tests。
2. `check()` p99 < 1ms。写个 benchmark 用例卡住数量级就行，不搞严格性能门。
3. `mypy --strict` 过。
4. 存量蓝图在默认 `mode: shadow` 下行为不变——`test_tool_stream.py` 必须全绿，这是回归底线。
5. 每次 `tools/call` 在审计日志里有且只有一条记录。
