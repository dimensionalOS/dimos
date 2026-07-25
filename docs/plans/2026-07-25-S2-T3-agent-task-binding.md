# S2-T3 Agent TaskSpec 与持久化任务绑定计划

> **执行要求：** 使用 `executing-plans` 技能；先完成纯 TypeScript/Wrapper
> 失败测试，再实现；本任务不启动或移动真实机器狗。

**目标：** 朋友 Gateway 中的 product Agent 只把自然语言编译成受约束的
`go_to_place` 参数。Gateway 使用 `instruction_id` 确定性生成唯一 `task_id`，
持久化绑定，单次提交 `start_task`，并持续读取 `get_task_status`。只有同一任务进入
`completed`、`failed` 或 `cancelled` 且 `active=false` 后，才创建最终用户回复。

**边界：**

- Stage 2 只编译 `go_to_place + destination`，不提前实现视觉搜索、跟随或戒指。
- Agent session 不注册 MCP 或 coding tools；模型不生成 task ID、时间戳或终态。
- Gateway 是 instruction/task 绑定 owner；DimOS `MissionExecutor` 是物理任务状态
  owner。
- `start_task` 最多发送一次。进程重启后只恢复状态监控，不重新提交任务。
- 精确“停”/`stop` 继续绕过 Agent，直接调用 `stop_all`；原任务必须等到
  `cancelled + active=false` 后形成自己的终态回复。
- Wrapper product profile 必须转发任务生命周期工具，并继续隐藏旧低层动作。

---

## Task 1：任务契约、稳定 ID 与严格编译器

**Create**

- `/Users/johnsonmac/ai_completion/agent/components/agent-framework/agent-webhook-gateway/src/task-contract.ts`
- `/Users/johnsonmac/ai_completion/agent/components/agent-framework/agent-webhook-gateway/test/task-contract.test.ts`

**Modify**

- Gateway `types.ts`
- Gateway `agent-runtime.ts`
- Gateway `test/agent-runtime.test.ts`

**Tests**

1. 只接受精确 JSON `{"kind":"go_to_place","destination":"..."}`。
2. 空地点、额外字段、Markdown、未知 kind 全部 fail-closed。
3. 相同 `instruction_id` 始终生成相同合法 `task_id`。
4. `created_at` 由 Gateway 写入 UTC；模型无法覆盖。
5. product 编译 session 的 active tools 为空。

---

## Task 2：SQLite instruction/task binding

**Modify**

- Gateway `store.ts`
- Gateway `types.ts`
- Gateway `test/gateway.test.ts`

**Schema**

`instruction_task_bindings` 保存：

- `instruction_id`
- `task_id`
- canonical `task_json`
- `compile_status`
- `last_state`
- `last_snapshot_json`
- `created_at`
- `updated_at`

**Tests**

1. 一个 instruction 只能绑定一个 task。
2. 重复 instruction 不创建第二个 binding。
3. 进程恢复时，未绑定的旧 `processing` 指令仍 fail-closed。
4. 已绑定但未终态的指令保留为 recoverable work，不生成假完成回复。

---

## Task 3：单次提交与终态监控

**Create**

- Gateway `src/task-monitor.ts`
- Gateway `test/task-monitor.test.ts`

**Modify**

- Gateway `service.ts`
- Gateway `cli.ts`
- Gateway `config.ts`、`.env.example`
- Gateway focused tests

**Implementation**

1. `start_task` 只发送一次 canonical `task_json`。
2. 校验 start ack 的 accepted、task ID 和冲突。
3. 轮询 `get_task_status`，拒绝空任务、不同 task ID、非法 state 和超时。
4. terminal 但 `active=true` 时继续等待；只有 `active=false` 才可回复。
5. 重启恢复只调用 `get_task_status`，不得再调用 `start_task`。
6. completed/failed/cancelled 使用确定性用户文本，不使用 Agent 猜测。

---

## Task 4：Wrapper product mission surface

**Modify**

- Wrapper `dog_tools.py`、`module.py`、`server.py`
- Wrapper focused tests、README

**Tests**

- product profile 暴露：
  `start_task/pause_task/resume_task/cancel_task/get_task_status/`
  `list_semantic_places/stop_all` 和只读状态工具。
- product profile 不暴露 `relative_move`、`navigate_with_text`、exploration、
  patrol 或 sport action。
- 所有任务工具名称和参数原样单次转发。

---

## Task 5：跨组件 replay 与验收

**Replay**

使用 fake MCP 或本地 dry-run MCP，证明：

1. `去门口测试点` 只编译一次。
2. Gateway 产生并持久化一个 task ID。
3. accepted/navigating 不产生最终回复。
4. completed + inactive 后才回复完成。
5. 相同 instruction 重试不产生第二次 `start_task`。
6. 重启恢复监控不重复 `start_task`。
7. cancelled + inactive 后任务状态与最终回复一致。

**Verification**

```bash
cd /Users/johnsonmac/ai_completion/agent/components/agent-framework/agent-webhook-gateway
node node_modules/vitest/dist/cli.js --run \
  test/task-contract.test.ts test/task-monitor.test.ts \
  test/agent-runtime.test.ts test/gateway.test.ts

cd /Users/johnsonmac/ai_completion/agent/components/agent-framework/dimos-mcp-wrapper
PYTHONPATH=src /Users/johnsonmac/ai_completion/dimos/.venv/bin/python \
  -m unittest discover -s tests -v

cd /Users/johnsonmac/ai_completion/agent
npm run check
git diff --check
```

**通过标准：**

- 相同 instruction ID 只对应一个持久化 task ID，`start_task` 只调用一次。
- Agent 无 MCP/coding tools，只输出严格参数。
- Gateway 不把 accepted/navigating 当 completed。
- 重启只恢复监控，不重复物理任务。
- cancel 后必须看到同一 task `cancelled + active=false`。
- Wrapper product 面可达 mission tools 且无法绕过执行器调用旧运动工具。

**不在本任务中声明：**

- 不声明真实机器狗已导航、已连接或已到达。
- 不创建真实 `测试起点`/`门口测试点`。
- 不启动 S2-T4 UI 或 S2-R1 真机复查。
