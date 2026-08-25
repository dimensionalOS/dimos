# L5 AuditTrail 实现规格

对应 DASL proposal §5.5。P0 交付物，安全层里最先做的一块——不依赖 L1/L2/L3 的任何东西，反过来 L2 的审计事件要等它。纯 Python，零 dimos 内部依赖（只读 run registry 的路径约定），所以可以单独提 PR、单独合。

改动面：`dimos/agents/safety/audit.py` 新文件，`dimos log` 加 `--audit` 选项。没了。

## 1. 要解决的问题

现在工具调用只有 logger 散落的文本日志，跟普通运行日志混在一起。真出了事故（机器人撞了东西、伤了人），要回答的问题是"哪条指令、什么来源、谁批准的、当时安全层是什么状态"——现有日志根本拼不出这条链。

所以审计日志跟普通日志是两种东西，要求不一样：

- **追加式**，只写不改不删；
- **可验证完整性**——事后有没有人动过手脚，能查出来；
- **结构化**，每条记录字段固定，能被程序回放，不是给人肉眼 grep 的。

普通 logger 继续记它该记的，两者不合并。

## 2. 文件位置与格式

落在现有 run registry 目录里，跟其他运行产物放一起：

```
~/.local/state/dimos/logs/<run-id>/audit.jsonl
```

每行一条 JSON 记录（JSONL），字段：

```json
{
  "ts": 1722470400.123,
  "prev": "a94f3c...",
  "event": "tool_call",
  "skill": "move",
  "args": {"x": 0.5, "duration": 2.0},
  "risk": "motion",
  "trust": "OPERATOR",
  "decision": "allow",
  "reason": "",
  "confirmation_id": null,
  "monitor_state": "clear",
  "hash": "7be21d..."
}
```

必须有的字段：`ts, prev, event, hash`。其余按事件类型来（`tool_call` / `confirmation` / `monitor_transition` / `verify_result`），缺省 `null`，不允许缺 key——回放程序按固定 schema 读，"有时候有这个 key"是最烦人的格式。

## 3. 哈希链

每条记录带两个字段：

- `prev`：上一条记录的 `hash`；
- `hash`：`sha256(prev + canonical_json(本条除 hash 外的全部内容))`。

`canonical_json` = `json.dumps(obj, sort_keys=True, separators=(",", ":"), default=str)`。不定这个细节，不同进程算出来的 hash 会对不上，链就没意义了。

链头是字符串 `"genesis"`。重启后续链：读文件最后一行的 `hash` 作为新记录的 `prev`，链不断。

这个设计防的是"事后悄悄改一条记录"——改任何一条，它后面所有的 `prev` 都对不上。它不防"整个文件换掉"，那是备份和权限的事，不是这一层的事。

## 4. API

```python
class AuditTrail:
    def __init__(self, run_log_dir: Path) -> None: ...

    def record(self, event: str, **fields: Any) -> str:
        """追加一条记录，返回它的 hash。"""

    @staticmethod
    def verify(path: Path) -> VerifyResult:
        """离线校验整条链。给 CLI 和事后调查用。"""
```

```python
@dataclass(frozen=True)
class VerifyResult:
    ok: bool
    records: int
    first_bad_line: int | None = None   # 链断在哪一行，没断就是 None
```

约定：

- `record()` 是同步写 + `flush()`。每次调用都 fsync 太贵，但只 flush 不 fsync 意味着掉电可能丢最后几条——接受这个代价，在文档里写明白。掉电丢记录 > 机器人等审计。
- 线程安全：`McpServer` 在 executor 线程调，加一把锁。临界区是一次 append，微秒级。
- `record()` 失败（磁盘满、权限问题）**抛异常**，让调用方决定怎么办。审计写不进去还假装没事，比没有审计更糟。L2 那边的约定是：审计失败时 deny 物理动作（fail-closed），但那是 L2 的策略，不是本模块的行为。

## 5. 续链与轮转

`__init__` 时如果文件已存在且非空：

1. 读最后一行，取出 `hash` 作为 `_prev_hash`；
2. 最后一行 JSON 解析失败（上次写到一半崩了）→ 不猜、不修，把 `_prev_hash` 设为 `"corrupt:<该行原文的 sha256>"`，从下一行继续写。坏行留在文件里，verify 会报出来，事后能查。

**不做自动轮转。** 单次 run 的审计量估算：每次工具调用一条记录 ~300B，就算 Agent 每分钟调 30 次跑一整天，也就 13MB 左右。为这个量级引入轮转逻辑（多文件、链跨文件怎么算）不划算。真有超长运行需求，按 run 切文件，run 结束链自然终止，下一个 run 新链从 `genesis` 开始。

## 6. CLI：`dimos log --audit`

```
dimos log --audit              # 最近一次 run 的审计，人类可读格式
dimos log --audit -r <run-id>  # 指定 run
dimos log --audit --verify     # 校验哈希链，输出 VerifyResult
dimos log --audit --json       # 原始 JSONL 透传
```

人类可读格式一行一条：

```
12:03:11  tool_call   move(x=0.5, duration=2.0)   trust=OPERATOR  -> allow
12:03:40  tool_call   jump()                      trust=USER_VOICE -> deny  trust insufficient for 'dynamic'
12:04:02  confirmation  jump()  id=9f2e...  approved by operator (cli)
12:04:05  tool_call   jump()                      trust=USER_VOICE -> allow  confirmation=9f2e...
```

`--verify` 的输出只有两种：`OK (1432 records)` 或者 `FAILED: chain broken at line 891`。调查场景下没有第三种有用的答案。

## 7. 不变量

1. **I1**：`record()` 返回后，记录一定已经 flush 到 OS。
2. **I2**：同一进程内，任意两条记录的 `prev`/`hash` 链严格连续——锁保证，不许并发交错。
3. **I3**：任何已写入的行，本模块不提供任何修改或删除的 API。想改审计？改文件去吧，verify 会抓到。
4. **I4**：`verify()` 是只读操作，绝不"顺手修复"坏链。
5. **I5**：记录里 `args` 原样落盘，不脱敏。脱敏策略（语音指令里可能有隐私内容）是 proposal 第 9 节的开放问题，定了再说，这层不预判。

## 8. 测试（`dimos/agents/safety/test_audit.py`）

| 用例 | 做法 | 期望 |
|------|------|------|
| 基本写入 | record 3 条 | 文件 3 行，每条有 ts/prev/event/hash |
| 链正确 | record N 条后 verify | ok=True, records=N |
| 续链 | 关掉重开同一目录再 record | 新记录的 prev == 旧文件最后一行的 hash |
| 空目录启动 | 新目录 | 第一条 prev == "genesis" |
| 篡改检测 | 写 5 条，手动改第 3 行的 args | verify 报 first_bad_line=3 |
| 截断检测 | 写 5 条，删掉最后 2 条再续写 1 条 | verify 在新记录处报断链 |
| 半行恢复 | 手动在文件尾写半行坏 JSON，再初始化 | 不抛异常，新记录 prev 以 "corrupt:" 开头；verify 报坏行 |
| 并发 | 8 线程各 record 100 条 | 800 条记录，链严格连续（I2） |
| 失败传播 | 把目录设为只读 | record() 抛异常，不吞 |
| schema | record 时漏传可选字段 | 落盘记录里对应 key 存在且为 null |
| CLI | `--verify` 对好链/坏链 | 输出格式符合 §6 |

## 9. 验收

1. 上面用例全过，进默认 fast tests。
2. `mypy --strict` 过。
3. 单线程连续 `record()` 10 万条无内存增长（文件句柄和 deque 之类没有泄漏累积）。
4. L2 集成时（下个 PR），`mcp_server.py` 里每次 `tools/call` 对应恰好一条 `tool_call` 记录——这条在 L2 的验收里再查一次。
