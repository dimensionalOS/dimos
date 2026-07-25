# Go2 与朋友 Agent 融合式自主系统总实施计划

> **Execution scope reduced on 2026-07-25:** 当前可执行 Goal 已收敛为
> `docs/plans/2026-07-25-go2-three-stage-robot-validation-plan.md`。
> 本文件保留为长期架构与技术 backlog，不再作为当前阶段任务顺序。

> **For Codex:** REQUIRED SKILL: use `executing-plans`，每次只执行一个明确的
> `TASK_ID`；涉及两个仓库时仍必须保持一个任务、一个验收边界。

**Goal:** 把现有 DimOS Go2 能力与朋友提供的 Pi Agent、Webhook Gateway 和
MCP Wrapper 融合成一套系统：用户可通过 Studio、自然语言或智能戒指下达高层任务，
机器狗能够建图、理解语义地点、导航、检查目标、跟随人员并提供可追溯结果。

**Architecture:** DimOS/Python 是唯一机器人 Runtime 和唯一物理任务执行层；朋友的
Agent 只负责任务理解、`TaskSpec` 编译、任务状态查询和用户回复。朋友仓库的
`components/dimos-mcp` 作为唯一产品启动入口，组合 P1/P2/P3 模块，但不得再启动
第二个 Go2 Blueprint、第二个 `GO2Connection` 或第二个运动控制 owner。

**Tech Stack:** Python 3.12、DimOS `0.0.14b1`、Unitree Go2 WebRTC/DDS、
`ReplanningAStarPlanner`、Pydantic v2、MCP、FastAPI Studio、Rerun、
Node.js 22、TypeScript、Pi Agent、SQLite、可替换视觉模型、Pytest、Ruff、
Vitest、TypeScript check。

---

## 0. 权威性、仓库边界和当前状态

本文件是融合后的唯一产品总计划。它取代
`docs/plans/2026-07-25-go2-semantic-autonomy-master-plan.md` 作为后续任务状态、
依赖关系和验收标准的权威来源。旧计划保留为历史技术参考。

### 0.1 两个仓库的职责

| 仓库 | 绝对路径 | 唯一职责 |
|---|---|---|
| DimOS 产品与机器人能力 | `/Users/johnsonmac/ai_completion/dimos` | 任务契约、任务执行器、语义地图、感知、导航、恢复、证据、Studio 和实时数据流 |
| 朋友 Agent 平台 | `/Users/johnsonmac/ai_completion/agent` | 用户指令接入、Pi Agent、MCP 转发、指令/任务绑定、最终回复和产品启动组合 |

依赖方向必须保持：

```text
agent-framework
  -> HTTP MCP
  -> unified dimos-mcp product runtime
  -> installed dimos-go2-studio modules
  -> DimOS core navigation/perception
  -> Go2
```

`agent-framework` 不得导入底层 Python 内部模块。朋友的 `components/dimos-mcp`
可以把已安装的 `dimos-go2-studio` 作为产品模块依赖，但不得复制其中的
`MissionExecutor`、`SemanticWorld` 或恢复逻辑。

### 0.2 当前进度

| Phase | 产品结果 | 状态 |
|---|---|---|
| Phase 1 | 单一 Runtime、语义地点导航、朋友 Agent 全链路融合 | `IN_PROGRESS` |
| Phase 2 | 事件驱动视觉检查和未知目标主动搜索 | `NOT_STARTED` |
| Phase 3 | 障碍感知跟随、智能戒指、眼镜/地图只读映射 | `NOT_STARTED` |
| Phase 4 | 可配置的居家 Agent 任务平台 | `NOT_STARTED` |

| 已有任务 | 状态 | 当前真实边界 |
|---|---|---|
| P1-T1 Canonical mission contracts | `DONE` | 本地测试通过；没有机器人 I/O |
| P1-T2 Background `MissionExecutor` | `DONE` | 本地测试通过；生产 resolver 仍 fail-closed |
| P1-T3 Navigation recovery supervisor | `DONE` | 本地测试通过；没有真机阻塞验收 |
| P1-T4 Persistent semantic place store | `NOT_STARTED` | 下一项允许执行的任务 |

这些 P1 改动目前位于未提交工作树。`DONE` 表示当前工作树内的任务级验证通过，
不表示已经发布、安装到朋友 MCP 或通过真机阶段门。

### 0.3 每个任务的强制执行规则

1. 每次只选择一个 `TASK_ID`。
2. 先阅读本文件、`docs/PROJECT_CONTEXT.md`、任务列出的源文件和依赖。
3. 在 `docs/plans/` 创建对应 child plan，记录当前行为、失败测试、最小实现、
   回滚边界和实际验收边界。
4. 先写失败测试，再做最小实现。
5. 任何跨仓库任务都必须分别运行两个仓库的 focused tests。
6. 不把 MCP 返回 accepted、HTTP 200、模型文本或 UI 状态当作物理任务完成。
7. 真实机器人验收必须记录：运行 PID、唯一 Runtime、地图/姿态时间戳、任务 ID、
   导航状态、实际里程计轨迹、停止状态和证据 ID。
8. 未经用户明确授权，不提交 Git。

### 0.4 通用 Definition of Done

- 没有第二个 `ModuleCoordinator`、`GO2Connection`、MCP 端口 owner 或运动 owner。
- 错误、超时、取消、重复请求、陈旧数据和重启恢复都有测试。
- 任务完成必须由确定性执行器确认，不能由 Agent 猜测。
- `planned_path` 与 `actual_path` 分开记录。
- 云模型不可用时，本地停止、避障、导航取消和状态查询仍可用。
- 文档写明测试证明了什么，以及没有证明什么。

## 1. 产品需求与非功能指标

### 1.1 功能需求

系统最终必须支持：

1. Studio 或朋友 Agent 接收自然语言：
   - `去门口`
   - `去厨房看看有没有水瓶`
   - `来到我身边`
   - `跟着我`
2. 机器狗自主建图并保存地图版本。
3. 地点名称、别名、地图坐标和证据可持久化。
4. Agent 把语言转换为严格 `TaskSpec`，不直接控制速度。
5. `MissionExecutor` 负责执行、暂停、恢复、取消、超时和终态。
6. 未知目标搜索使用探索、候选视角、视觉判断和几何定位。
7. 人员跟随通过避障导航，不启用无避障的直接视觉伺服。
8. 智能戒指通过电脑提交同一套高层任务。
9. 电脑 UI 显示地图、机器人姿态、计划路线、实际轨迹、目标、任务和证据。
10. 后续可通过任务模板增加新居家任务，而不是为每句话修改导航代码。

### 1.2 非功能指标

| 类别 | Phase 1 指标 | 最终指标 |
|---|---|---|
| 单一控制权 | 只有一个机器人 Runtime | 持续满足 |
| 任务接收 | 本地 task ack p95 `< 1 s` | 戒指到 task ack p95 `< 1 s` |
| 取消 | 应用层 cancel-to-zero p95 `< 200 ms` | 持续满足 |
| 地图/姿态新鲜度 | UI 明示数据年龄；超过阈值拒绝完成 | 眼镜端同样标记 |
| 幂等 | 相同 `instruction_id` 只创建一个任务 | 戒指重试也只创建一个任务 |
| 云依赖 | 云故障不阻断本地停止和导航 | 持续满足 |
| 可追溯 | 每个终态有任务、状态、原因和证据 | 可导出完整任务回放元数据 |
| 隐私 | 默认不上传连续视频 | 只上传明确选中的候选帧 |
| 部署 | 单机、单用户、单 Go2 | 不为假设性规模增加微服务 |

### 1.3 明确不做

- 不让 LLM、VLM 或 Agent 直接输出 `Twist`、电机或运动帧。
- 不把两套 Go2 Runtime 同时运行作为“融合”。
- 不在 Phase 1 训练自定义视觉或控制模型。
- 不承诺开门、取物、搬运等机械操作。
- 不把原始相机、点云或高频里程计通过 MCP 传输。
- 不允许普通 Agent 在自主任务期间调用 raw velocity 或高风险 sport action。

## 2. 方案比较和架构决策

### 2.1 三种方案

| 方案 | 优点 | 问题 | 结论 |
|---|---|---|---|
| A. 两套 Runtime 同时运行 | 改动少 | 抢端口、抢 Go2、状态分裂、停止不可靠 | 拒绝 |
| B. 只运行原 `dimos-go2-studio`，朋友仓库只做 Agent | 边界最干净 | 用户指定的朋友 MCP 不再是启动入口 | 可行备选 |
| C. 朋友 `dimos-mcp` 作为唯一组合入口，加载 P1 模块 | 保留朋友入口；仍只有一个 DimOS Runtime | 需要明确依赖和工具权限 | 采用 |

### 2.2 ADR-008：朋友 MCP 是组合入口，不是第二个机器人实现

**Decision:** `components/dimos-mcp` 只组合一个官方 Go2 stack、一个
`MissionExecutor`、一个语义世界、一个恢复模块和一个 MCP Server。不得嵌套启动
完整 `go2_studio_agentic` Blueprint。

**Positive:** 保留朋友仓库的部署方式，同时复用 P1 源码。

**Negative:** 两个仓库必须通过明确版本/安装依赖同步。

### 2.3 ADR-009：Agent 是任务编译器，Python 是执行器

**Decision:** Pi Agent 只把用户语言编译成受约束的任务参数。稳定 ID、时间戳、
任务状态、重试、完成和取消由 Gateway 与 `MissionExecutor` 管理。

**Consequence:** 模型响应慢或错误不会直接改变实时运动控制。

### 2.4 ADR-010：一个用户指令对应一个物理任务

`instruction_id` 是上层幂等键。Gateway 生成或绑定唯一 `task_id`，持久化
`InstructionTaskBinding`。模型不得自行生成第二个 ID，也不得在重试时创建新任务。

### 2.5 ADR-011：`cancel_task` 与 `stop_all` 分工

- 正常停止：`cancel_task(task_id)`，让任务状态进入 `CANCELLED` 并等待导航 idle。
- 直接停止入口：`stop_all`，不依赖 Agent 队列。
- 产品停止路径：先发 `cancel_task`；无法确定活动任务或超时则立即发
  `stop_all`，并把状态差异记录为故障。

### 2.6 ADR-012：产品工具面与维护工具面分离

自主 Agent 只看到高层任务、状态、地点和停止工具。`move_forward`、
`relative_move`、`navigate_with_text`、探索、巡逻和 sport action 只在显式维护模式
或人工控制界面出现，不属于默认 Agent 工具集。

## 3. 目标系统

### 3.1 运行拓扑

```text
智能戒指 / Studio / 其他可信输入
                 |
                 v
Agent Webhook Gateway :8080
  - inbox/outbox
  - instruction_id 幂等
  - instruction_id <-> task_id
                 |
                 v
Pi Agent
  - 只做语言 -> 受约束任务参数
                 |
                 v
Product MCP Wrapper :9991
  - 高层工具白名单
  - 单次转发
                 |
                 v
Unified dimos-dog-mcp :9990
  - 唯一 ModuleCoordinator
  - 唯一 GO2Connection
  - MissionExecutor
  - SemanticWorld / EvidenceService
  - RecoverySupervisor
  - McpServer
                 |
                 v
DimOS Navigation / Perception / Map
                 |
                 v
Unitree Go2

Camera + LiDAR + odom + TF
  -> DimOS streams
  -> Studio/Rerun/Glasses read model
  -> 不经过高频 MCP
```

### 3.2 进程和端口所有权

| Surface | Owner | 用途 |
|---|---|---|
| `:8080` | Agent Webhook Gateway | 用户/戒指高层输入 |
| `:9991/mcp` | Product MCP Wrapper | Agent 高层工具入口 |
| `:9990/mcp` | 唯一 `dimos-dog-mcp` | 真正执行任务 |
| `:7779` | 同一个 DimOS Runtime | 地图/命令中心 |
| `:3030`、`:9877` | 同一个 DimOS Runtime/Viewer | 可视化数据 |

启动预检必须拒绝：

- 已存在第二个 Go2 Runtime；
- `9990` 被非目标进程占用；
- 同时加载两个 `McpServer`；
- 同时加载两个 `MissionExecutor`；
- 两个模块都能发布同一运动输出。

### 3.3 运行模式

| 模式 | Agent 工具 | 机器人运动 | 目的 |
|---|---|---|---|
| `replay` | 高层任务工具 | 无真实运动 | 开发和回归 |
| `hardware_readonly` | 状态、观察、地图 | 禁止 | 连接与传感器验收 |
| `product` | 高层任务、取消、状态 | 通过执行器 | 正式自主任务 |
| `maintenance` | 人工低层工具 | 显式人工控制 | 调试，不得与 product Agent 并行 |

## 4. 稳定契约和状态语义

### 4.1 核心契约

- `ExternalInstruction`
  - `instruction_id`
  - `text`
  - `received_at`
  - `source`
- `TaskSpec`
  - `task_id`
  - `kind`
  - `destination`
  - `target_description`
  - `question`
  - `priority`
  - `created_at`
- `InstructionTaskBinding`
  - `instruction_id`
  - `task_id`
  - `compile_status`
  - `created_at`
- `TaskSnapshot`
  - 当前状态、恢复前状态、结果、原因、证据
- `TaskEvent`
  - `task_id`
  - 单调序号
  - 状态
  - 时间戳
  - 结构化原因
- `SemanticEntity`
  - 地图版本、稳定 ID、名称、别名、姿态、置信度和证据
- `ObservationBundle`
  - 图像、相机信息、机器人地图姿态、点云引用、来源和时间戳

### 4.2 Agent-facing MCP 工具

Phase 1 默认产品工具：

```text
start_task
get_task_status
pause_task
resume_task
cancel_task
list_semantic_places
get_robot_summary
stop_all
```

Phase 2 增加：

```text
get_task_evidence
```

所有工具返回结构化 JSON 文本，至少包含 `ok`、`task_id`、`state`、
`reason` 和 `observed_at`。HTTP/MCP 成功与物理成功分开表达。

### 4.3 任务完成定义

| 任务 | 完成条件 |
|---|---|
| `go_to_place` | 目标已解析；导航 `is_goal_reached=true`；导航状态 `IDLE`；有到达姿态证据 |
| `inspect_place` | 到达完成；到达后采集新鲜证据；得到 found/not_found/uncertain |
| `find_target` | 有几何落点和多视角证据，或达到有界 not_found/uncertain 终态 |
| `come_to_user` | 用户位置来源有效；达到 stand-off；导航 idle |
| `follow_person` | 持续任务；只有取消、失败或明确结束才进入终态 |

## 5. 总依赖图和验收阶梯

```text
P1-T1 contracts [DONE]
  ├─ P1-T2 executor [DONE]
  ├─ P1-T3 recovery [DONE]
  └─ P1-T4 semantic world
       └─ P1-T5 map onboarding

P1-T2 + T3 + T4 + T5
  -> P1-T6A unified runtime composition
  -> P1-T6B product MCP profile
  -> P1-T6C Agent compiler and task binding
  -> P1-T6D unified UI/read model
  -> P1-T7 Phase 1 acceptance

P1 gate
  -> P2 observation/vision/grounding/evidence
  -> P2 inspection/search
  -> P2 Agent result integration
  -> P2 acceptance

P2 gate
  -> P3 person target/follow/recovery
  -> P3 ring adapter
  -> P3 glasses/trajectory read model
  -> P3 operations
  -> P3 home pilot acceptance

P3 gate
  -> P4 task profile platform
  -> P4 developer test kit
  -> P4 extensibility acceptance
```

每个阶段按下列顺序验证：

1. Contract/unit tests。
2. 两仓库集成测试。
3. recorded replay。
4. simulation。
5. real hardware read-only。
6. 一个短距离受监督动作。
7. 阶段统计验收。

任何一级失败，不得用更高一级“试一下真机”代替。

# Phase 1 — 可靠语义导航与 Agent 融合

## Phase 1 产品结果

用户从朋友 Agent 或 Studio 输入“去门口”，系统只启动一个 Go2 Runtime，Agent
生成一个任务，语义地点解析为已确认地图姿态，执行器导航并处理有界恢复，UI 显示
计划路线和实际轨迹，只有真正到达且导航 idle 后才回复完成。

## P1-T1 — Canonical mission contracts

**Status:** `DONE`

保留当前实现与 43-test 验证。进入融合前重新运行现有 focused tests，不增加新行为。

## P1-T2 — Background MissionExecutor

**Status:** `DONE`

保留当前实现与 57-test 验证。`MissionExecutor` 继续作为唯一任务生命周期 owner。

## P1-T3 — Bounded navigation recovery

**Status:** `DONE`

保留当前实现与 86-test 验证。真机恢复能力仍需 P1-T7 验收。

## P1-T4 — Persistent semantic place store

**Status:** `NOT_STARTED`

**Depends on:** P1-T1。

**Design:**

- `SemanticWorld` 保存 map ID/version、稳定 entity ID、名称、别名、地图姿态、
  置信度、确认状态、证据 ID 和更新时间。
- confirmed place、candidate 和 dynamic entity 分开。
- `DestinationResolver` 只返回 confirmed、地图版本匹配的地点。
- 低置信视觉候选不得自动成为导航目标。

**Files:**

- Create:
  `extensions/go2-studio-agent/src/dimos_go2_studio/semantic_world.py`
- Create:
  `extensions/go2-studio-agent/tests/test_semantic_world.py`
- Modify:
  `extensions/go2-studio-agent/src/dimos_go2_studio/mission_executor.py`
- Modify:
  `extensions/go2-studio-agent/src/dimos_go2_studio/blueprint.py`
- Modify only if the interface is insufficient:
  `dimos/perception/spatial_memory_spec.py`

**Implementation:**

1. 写稳定 ID、alias、map version、序列化、冲突和重启恢复失败测试。
2. 实现纯数据仓库和原子保存。
3. 实现 typed resolution result。
4. 把 resolver 注入现有 `MissionExecutor`。
5. 测试地图版本不匹配和 candidate fail-closed。

**Verification:**

```bash
cd /Users/johnsonmac/ai_completion/dimos
PYTHONPATH=extensions/go2-studio-agent/src \
  .venv/bin/python -m pytest \
  extensions/go2-studio-agent/tests/test_semantic_world.py \
  extensions/go2-studio-agent/tests/test_mission_executor.py -v
.venv/bin/ruff check \
  extensions/go2-studio-agent/src/dimos_go2_studio/semantic_world.py \
  extensions/go2-studio-agent/tests/test_semantic_world.py
git diff --check
```

**Acceptance:**

- 重启后 confirmed place 仍存在。
- `门口` 和配置 alias 指向同一个 entity。
- map version 不匹配时拒绝导航。
- candidate 不会被当成 confirmed。
- 本任务只证明存储和解析，不证明真机到达。

## P1-T5 — One-time map onboarding

**Status:** `NOT_STARTED`

**Depends on:** P1-T4。

**Design:**

状态为 `mapping -> proposing -> awaiting_confirmation -> saved`。系统可基于探索和
已有候选提出地点，但第一次必须由用户确认、改名、合并或拒绝。

**Files:**

- Create:
  `extensions/go2-studio-agent/src/dimos_go2_studio/map_onboarding.py`
- Create:
  `extensions/go2-studio-agent/tests/test_map_onboarding.py`
- Modify:
  `dimos/web/studio/app.py`
- Modify:
  `dimos/web/studio/static/app.js`
- Modify:
  `dimos/web/studio/static/index.html`
- Modify:
  `dimos/web/studio/test_studio.py`

**Implementation:**

1. 测试 onboarding 状态和 map ID。
2. 实现候选提案，不自动确认。
3. 实现 confirm/rename/merge/reject。
4. 将 confirmed entity 保存到 `SemanticWorld`。
5. UI 显示地图版本和地点状态。

**Acceptance:**

- 用户无需为每次任务重新点击坐标。
- 未确认地点不能用于任务。
- 相同地图重载能恢复地点。
- 不同地图不会静默使用旧坐标。

## P1-T6A — Unified product runtime composition

**Status:** `NOT_STARTED`

**Depends on:** P1-T2、P1-T3、P1-T4、P1-T5。

**Design:** 朋友 `components/dimos-mcp` 成为唯一产品启动入口。它组合当前官方
Go2 spatial/navigation 模块与 P1 模块，但不嵌套完整 Studio Blueprint。

**Files:**

- Modify:
  `/Users/johnsonmac/ai_completion/agent/components/dimos-mcp/pyproject.toml`
- Modify:
  `/Users/johnsonmac/ai_completion/agent/components/dimos-mcp/src/dimos_dog_mcp/blueprint.py`
- Modify:
  `/Users/johnsonmac/ai_completion/agent/components/dimos-mcp/tests/test_dimos_integration.py`
- Modify:
  `extensions/go2-studio-agent/tests/test_blueprint.py`
- Create:
  `/Users/johnsonmac/ai_completion/agent/components/dimos-mcp/tests/test_product_blueprint.py`

**Implementation:**

1. 先写 Blueprint 静态测试：一个 Go2 connection、一个 navigator、一个
   `MissionExecutor`、一个 MCP Server、没有 embedded LLM Agent。
2. 把 `dimos-go2-studio` 声明为明确依赖。
3. 只组合 P1 module atoms，不启动第二个 Blueprint。
4. 增加启动预检和重复 Runtime 拒绝。
5. 保留 `stop_all` 对所有底层活动的停止编排。

**Verification:**

```bash
cd /Users/johnsonmac/ai_completion/agent/components/dimos-mcp
PYTHONPATH=src .venv/bin/python -m unittest \
  tests.test_product_blueprint tests.test_dimos_integration -v

cd /Users/johnsonmac/ai_completion/dimos
PYTHONPATH=extensions/go2-studio-agent/src \
  .venv/bin/python -m pytest \
  extensions/go2-studio-agent/tests/test_blueprint.py -v
```

**Acceptance:**

- 构建结果只有一个机器人 Runtime 和一个任务执行器。
- replay 能发现 P1 高层任务工具。
- 没有真机运动声明。

## P1-T6B — Product MCP profile

**Status:** `NOT_STARTED`

**Depends on:** P1-T6A。

**Design:** 默认 Agent MCP 面只暴露高层任务工具。维护工具继续存在于显式
maintenance profile，但不注册给默认 Pi Agent。

**Files:**

- Modify:
  `/Users/johnsonmac/ai_completion/agent/components/agent-framework/dimos-mcp-wrapper/src/dimos_mcp_wrapper/dog_tools.py`
- Modify:
  `/Users/johnsonmac/ai_completion/agent/components/agent-framework/dimos-mcp-wrapper/src/dimos_mcp_wrapper/server.py`
- Modify:
  `/Users/johnsonmac/ai_completion/agent/components/agent-framework/dimos-mcp-wrapper/src/dimos_mcp_wrapper/config.py`
- Modify corresponding wrapper tests.

**Implementation:**

1. 写 product/maintenance allowlist 测试。
2. 增加五个任务生命周期工具和地点/机器人摘要工具。
3. 默认隐藏 raw movement、直接导航、探索、巡逻和 sport action。
4. 保持每个请求最多一次上游调用；不自动重试运动。
5. `stop_all` 始终可直接调用。

**Acceptance:**

- product `tools/list` 不出现 raw movement 或 sport action。
- `start_task` 参数原样单次转发。
- 上游错误不被包装成成功。
- Wrapper 不拥有任务状态和机器人连接。

## P1-T6C — Agent compiler and persistent task binding

**Status:** `NOT_STARTED`

**Depends on:** P1-T6B。

**Design:** Agent 输出任务参数，不输出任务 ID、时间戳或完成状态。Gateway 使用
`instruction_id` 创建稳定 `task_id`，持久化绑定并轮询终态。

**Files:**

- Create:
  `/Users/johnsonmac/ai_completion/agent/components/agent-framework/agent-webhook-gateway/src/task-contract.ts`
- Create:
  `/Users/johnsonmac/ai_completion/agent/components/agent-framework/agent-webhook-gateway/src/task-monitor.ts`
- Modify:
  `/Users/johnsonmac/ai_completion/agent/components/agent-framework/agent-webhook-gateway/src/agent-runtime.ts`
- Modify:
  `/Users/johnsonmac/ai_completion/agent/components/agent-framework/agent-webhook-gateway/src/service.ts`
- Modify:
  `/Users/johnsonmac/ai_completion/agent/components/agent-framework/agent-webhook-gateway/src/store.ts`
- Modify:
  `/Users/johnsonmac/ai_completion/agent/components/agent-framework/agent-webhook-gateway/src/types.ts`
- Add focused Vitest files under the existing `test/` directory.

**Implementation:**

1. 测试 instruction/task 一对一绑定、重复输入、重启和冲突。
2. 用 TypeBox 定义与 Pydantic 契约等价的 Agent tool schema。
3. Gateway 注入 `task_id` 和 UTC 时间。
4. `start_task` accepted 后保持事件未完成，低频查询 `get_task_status`。
5. 只有 terminal snapshot 才创建最终 reply。
6. “停”快速路径先取消活动 task；无法确认时调用 `stop_all`。
7. Agent 崩溃恢复不得重复提交已启动任务。

**Verification:**

```bash
cd /Users/johnsonmac/ai_completion/agent/components/agent-framework/agent-webhook-gateway
node node_modules/vitest/dist/cli.js --run \
  test/agent-runtime.test.ts \
  test/gateway.test.ts \
  test/mcp-client.test.ts \
  test/task-monitor.test.ts
npm run check
```

**Acceptance:**

- 相同 `instruction_id` 只能对应一个 `task_id`。
- Agent 不会把 start accepted 回复成“已完成”。
- Gateway 重启不会重复物理任务。
- 终态回复和 `TaskSnapshot` 一致。

## P1-T6D — Unified UI and read model

**Status:** `NOT_STARTED`

**Depends on:** P1-T6A、P1-T6C。

**Design:** Studio 直接消费 DimOS 数据流/API；Agent 只消费低频任务状态。UI 显示
同一个 task ID、地图、目标、计划路线、实际轨迹、恢复状态和终态原因。

**Files:**

- Create:
  `extensions/go2-studio-agent/src/dimos_go2_studio/actual_trajectory.py`
- Create:
  `extensions/go2-studio-agent/tests/test_actual_trajectory.py`
- Modify:
  `dimos/web/studio/app.py`
- Modify:
  `dimos/web/studio/service.py`
- Modify:
  `dimos/web/studio/static/app.js`
- Modify:
  `dimos/web/studio/static/index.html`
- Modify:
  `dimos/web/studio/test_studio.py`

**Implementation:**

1. 从 `odom` 采样 `actual_path`，与 planner `path` 分开。
2. 建立任务 read model，拒绝旧 task/event 覆盖新任务。
3. 显示数据时间戳、地图版本和 Runtime owner。
4. 停止按钮使用统一 cancel/stop 路径。
5. 不自动打开额外浏览器或 Viewer。

**Acceptance:**

- UI、Agent 回复和 MCP 状态显示同一个 task ID/state。
- 实际轨迹来自 odometry，不是 A* 计划路径。
- 数据陈旧时 UI 明确显示，不伪装实时。

## P1-T7 — Phase 1 acceptance

**Status:** `NOT_STARTED`

**Depends on:** P1-T1 至 P1-T6D。

**Artifacts:**

- Create: `tests/acceptance/go2_phase1_agent_fusion_matrix.md`
- Create: `scripts/run_go2_phase1_acceptance.sh`
- Create only with approval: `artifacts/go2-phase1/`

**Validation order:**

1. 两仓库 contract/unit tests。
2. replay：Agent -> Wrapper -> unified MCP -> MissionExecutor。
3. simulation：10 次已知地点任务。
4. hardware read-only：地图、姿态、地点、任务状态。
5. 一个短距离已知地点任务。
6. 10 次地点导航和 10 次受控临时阻挡。

**Phase 1 gate:**

- 全链路只有一个 Runtime、一个 `GO2Connection`、一个任务 owner。
- known-place 到达成功至少 `9/10`。
- blocked-path 有界恢复至少 `8/10`。
- 最终水平误差不超过 `0.6 m`，或符合地点 stand-off 定义。
- cancel-to-zero p95 `< 200 ms`。
- Agent 不出现 accepted-as-completed。
- 重复 `instruction_id` 不产生第二个任务。
- 地图和语义地点重启后恢复，且完成 relocalization。
- UI 保留任务、目标、计划路线、实际轨迹和最终姿态。

Phase 1 gate 未通过，不进入 Phase 2。

# Phase 2 — 视觉检查与未知目标主动搜索

## Phase 2 产品结果

用户说“去厨房看看有没有水瓶”或“找一扇门”。系统先使用语义地图和本地几何，
在候选事件上调用视觉 provider，把候选投影到地图并从多个视角确认，最后返回
found/not_found/uncertain 和证据，不让视觉模型直接控制运动。

## P2-T1 — Synchronized ObservationBundle

**Files:** create `dimos/perception/observation_bundle.py` and its tests；必要时
modify `dimos/robot/unitree/go2/connection.py`。

**Implementation:** 测试并实现 image、camera info、map pose、point-cloud ref、
source 和时间同步；陈旧或缺 TF 时 fail-closed。

**Acceptance:** 每个视觉请求都能追溯到新鲜图像和地图姿态；replay/live 同契约。

## P2-T2 — Provider-neutral vision gateway

**Files:** create `dimos/perception/vision_gateway.py` and tests；adapt current
`target_verification.py`。

**Implementation:** 结构化 verdict、置信度、区域、模型元数据、延迟、错误类型；
提供 local/no-upload 和 OpenAI-compatible provider；增加超时、取消和大小限制。

**Acceptance:** provider 离线不影响本地导航/停止；provider 不能输出速度或目标点。

## P2-T3 — Camera/LiDAR/pose grounding

**Files:** create `dimos/perception/semantic_grounding.py` and tests；复用现有
2D-to-3D 投影，不复用直接视觉伺服。

**Implementation:** 合成投影测试、有效深度过滤、map-frame 变换、不确定度和
collision-checked stand-off。

**Acceptance:** 视觉 label 单独不能成为导航目标；replay 中位定位误差
`<= 0.5 m`。

## P2-T4 — Multi-view evidence fusion

**Files:** create `dimos/perception/semantic_evidence.py` and tests；modify
`semantic_world.py`。

**Implementation:** stable candidate ID、重复帧拒绝、视角多样性、矛盾证据、
置信衰减和 TTL。

**Acceptance:** 重复同一帧不能抬高置信度；矛盾结果进入 `uncertain`。

## P2-T5 — Inspection mission

**Files:** modify `mission_executor.py`；create `evidence_service.py` 和
`test_inspection_mission.py`；update Studio evidence UI。

**Implementation:** resolve -> navigate -> idle -> settle -> post-arrival capture
-> verify -> report；支持 found/not_found/uncertain/provider_unavailable。

**Acceptance:** 到达前图像不能满足到达后检查；终态包含 evidence IDs。

## P2-T6 — Unknown-target active search

**Files:** create `semantic_search.py` and tests；复用 frontier goal selector。

**Implementation:** 先排名已知候选，再选择有信息量的 frontier/viewpoint；
验证前停止；拒绝后换视角；限制时间、距离、覆盖率和候选数。

**Acceptance:** 不无限探索；选择理由可追溯；最终语义判断前机器人静止。

## P2-T7 — Agent visual-task/result integration

**Files:** modify friend `task-contract.ts`、`agent-runtime.ts`、`task-monitor.ts`
和对应 tests。

**Implementation:** 允许 Agent 编译 `inspect_place`、`find_target`；最终回复只能
从结构化 `TaskResult` 生成，附带 found/not_found/uncertain 和证据引用。

**Acceptance:** Agent 不会根据模型对话自行宣布找到目标；回复与任务证据一致。

## P2-T8 — Phase 2 acceptance

**Artifacts:** semantic vision matrix、provider benchmark、replay evidence set。

**Phase 2 gate:**

- 至少 50 个多视角 target/non-target 场景。
- candidate recall `>= 90%`。
- false verified-target `< 5%`。
- 至少 95% 证据有新鲜 image、pose、source、provider metadata。
- grounded target 中位误差 `<= 0.5 m`。
- provider loss 不影响本地取消、避障或导航。
- 10 次 end-to-end inspection 至少 `9/10` 状态与证据正确。

# Phase 3 — 跟随、智能戒指与眼镜地图

## Phase 3 产品结果

机器人在已建图室内环境中通过导航跟随选定人员，遇障碍绕行，人员短时遮挡时有界
恢复；智能戒指通过电脑发送高层命令；电脑和智能眼镜读取同一份地图、实际轨迹、
任务和目标状态。

## P3-T1 — Dynamic person target

**Files:** create `dimos/perception/dynamic_target.py` and tests；只复用人员检测/
tracking，不启用直接视觉伺服。

**Acceptance:** tracker 不发布 `Twist`；身份不会在邻近人员间静默切换；陈旧目标触发
hold/search。

## P3-T2 — Obstacle-aware follow coordinator

**Files:** create `dimos/navigation/dynamic_follow.py` and tests；modify
`mission_executor.py` and product Blueprint。

**Implementation:** person map pose -> stand-off navigation goal；限频、dead band、
路径绕障、取消和目标丢失。

**Acceptance:** 所有运动通过 `NavigationInterfaceSpec`；小幅目标抖动不引起振荡。

## P3-T3 — Lost-target and blocked-follow recovery

**Files:** modify `dynamic_follow.py`、其 tests 和 `recovery_supervisor.py`。

**Acceptance:** 短遮挡可恢复；长时间丢失有界结束；身份不确定时 hold 并请求选择。

## P3-T4 — Smart-ring command adapter

**Design:** 戒指和 BLE 细节留在输入适配器；进入系统后转换为现有
`ExternalInstruction`，不得直接连接机器人 MCP。

**Files:**

- Create:
  `/Users/johnsonmac/ai_completion/agent/components/agent-framework/agent-webhook-gateway/src/ring-adapter.ts`
- Create corresponding tests.
- Modify Gateway HTTP/config only after ring transport is known.

**Implementation:** 定义 command ID、device ID、timestamp、payload、expiry；
测试去重、乱序、重试、断线和 cancel；转换到同一 TaskSpec 链路。

**Acceptance:** 重试只产生一个任务；过期/乱序命令不移动；戒指断线不影响本地取消。

## P3-T5 — Trajectory and glasses read model

**Design:** 智能眼镜只消费低频、只读、降采样状态，不承担实时控制。

**Files:**

- Extend `actual_trajectory.py`.
- Create:
  `extensions/go2-studio-agent/src/dimos_go2_studio/telemetry_read_model.py`
- Add tests and one documented read-only endpoint in Studio.

**Acceptance:** 眼镜看到的是 actual odometry path；数据带 task ID、map version 和
timestamp；断线不会影响机器人任务。

## P3-T6 — Persistence and operations

**Files:** create `task_store.py` and tests；modify semantic world、Studio service
and Gateway binding reconciliation。

**Implementation:** 持久化 task events、map version、semantic entities、evidence、
final result；加入 sensor age、nav state、recovery count、provider latency、
runtime owner。

**Acceptance:** 重启后中断任务标记 interrupted/failed，不显示 stale running；
导出记录可解释看到什么、决定什么、执行什么和为何结束。

## P3-T7 — Home pilot acceptance

**Phase 3 gate:**

- clear-path follow 中 90% 样本保持配置 stand-off。
- 20 次受控 follow 无直接视觉伺服碰撞行为。
- 短时遮挡恢复至少 `8/10`。
- 身份不确定时不跟随任意新人员。
- ring-to-task-ack p95 `< 1 s`。
- 重复 ring command 只创建一个任务。
- 重启、地图 reload、relocalization 连续通过三次。
- 电脑/眼镜显示同一 robot trajectory、target、task 和 map version。

# Phase 4 — 可配置的居家 Agent 平台

## Phase 4 产品结果

在不改导航、感知和运动底层的前提下，开发者可以增加一个新的居家任务模板，
定义它需要的能力、参数、状态和证据，并通过 replay 验证后再允许进入真机。

## P4-T1 — Mission profile registry

**Files:** create `mission_profiles.py`、schema 和 tests；增加只读 MCP 查询工具。

**Design:** profile 声明 task kind、所需 capabilities、参数、允许状态、完成证据和
超时；不允许 profile 直接发布运动。

**Acceptance:** 无效 profile 在启动前失败；新增 profile 不改 planner。

## P4-T2 — Deterministic policy rules

**Files:** create `policy_rules.py` and tests；由 `MissionExecutor` 调用纯规则结果。

**Design:** 规则只决定 pause/resume/fail/request_evidence/next_step，不直接控制
速度；规则输入必须是 typed snapshot。

**Acceptance:** 同一输入得到同一决定；规则异常 fail-closed；可以 replay。

## P4-T3 — Developer template and replay kit

**Files:** create task-profile template、fixture builder、focused test command 和
开发文档。

**Acceptance:** 一个新开发者能从模板创建“巡视客厅并汇报异常”任务，在不连接
Go2 的情况下跑通 contract、replay 和结果报告。

## P4-T4 — Extensibility acceptance

**Gate:**

- 至少新增三个不修改 planner 的任务 profile。
- 每个 profile 都有 contract、错误、取消、超时和 replay 测试。
- 新 profile 默认不获得 raw movement/sport 权限。
- 一次 profile 发布不会改变既有任务工具 schema。
- 只有通过阶段验收的 profile 才能进入 product mode。

## 6. 跨阶段故障矩阵

| 故障 | 必须行为 | 禁止行为 |
|---|---|---|
| 第二 Runtime | 拒绝启动并报告 owner | 两边同时发布运动 |
| Agent 重试 | 返回同一 task binding | 创建第二个物理任务 |
| start accepted | 显示 queued/resolving | 回复“已经完成” |
| Gateway 重启 | 恢复 binding 或标记 interrupted | 重新运行任务 |
| `stop_all` 成功但 task 未取消 | 记录状态分裂并对账 | 假装 `CANCELLED` |
| 地图版本不匹配 | relocalize/onboard | 使用旧坐标 |
| stale image/odom/TF | uncertain/fail | 猜测当前位置 |
| provider timeout | 本地系统继续运行 | 阻塞停止和避障 |
| no path | 有界恢复和 typed failure | 无限增加 retry |
| identity ambiguity | hold/request selection | 跟随最近的人 |
| ring duplicate | 幂等返回原 task | 新建任务 |

## 7. 统一验证命令

### DimOS/P1

```bash
cd /Users/johnsonmac/ai_completion/dimos
PYTHONPATH=extensions/go2-studio-agent/src \
  .venv/bin/python -m pytest \
  extensions/go2-studio-agent/tests \
  dimos/navigation/replanning_a_star/test_recovery_supervisor.py \
  dimos/web/studio/test_mission.py -v
.venv/bin/ruff check \
  extensions/go2-studio-agent/src \
  extensions/go2-studio-agent/tests
git diff --check
```

### 朋友 `dimos-mcp`

```bash
cd /Users/johnsonmac/ai_completion/agent/components/dimos-mcp
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -v
```

### MCP Wrapper

```bash
cd /Users/johnsonmac/ai_completion/agent/components/agent-framework/dimos-mcp-wrapper
PYTHONPATH=src python -m unittest discover -s tests -v
```

### Agent Webhook Gateway

```bash
cd /Users/johnsonmac/ai_completion/agent/components/agent-framework/agent-webhook-gateway
npm run check
node node_modules/vitest/dist/cli.js --run \
  test/agent-runtime.test.ts \
  test/gateway.test.ts \
  test/mcp-client.test.ts
```

实际执行时只运行当前 task 影响的 focused tests；阶段 gate 才运行跨仓库矩阵。

## 8. 每阶段证据包

每次阶段验收至少保存：

- Git commit/dirty state；
- 启动命令和实际 PID；
- Runtime/模块/工具清单；
- robot IP 与 map version，不保存密钥；
- task/instruction IDs；
- start/end pose；
- planned path 与 actual path；
- navigation/recovery 状态；
- cancel/zero-command 时间；
- evidence IDs、模型 metadata 和最终结果；
- 失败 trial 及其原因。

## 9. 当前下一步

下一项允许执行的任务是：

```text
P1-T4 — Persistent semantic place store
```

P1-T4 未完成前，不开始 P1-T5 或朋友 Agent 的运行时融合。P1-T6A 至 P1-T6D
完成前，不用朋友 Agent 对真机执行自主任务。P1-T7 gate 未通过前，不进入 Phase 2。
