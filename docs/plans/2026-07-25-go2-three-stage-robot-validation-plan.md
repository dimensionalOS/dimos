# Go2 三阶段真机闭环 Implementation Plan

> **For Codex:** REQUIRED SUB-SKILL: use `executing-plans`，按本计划一次只执行
> 一个 `S<stage>-T<task>`；每个阶段必须完成真机复查后才能进入下一阶段。

**Goal:** 用三个逐步扩大的真机闭环验证系统价值，而不是一次完成整套居家 Agent：
先证明朋友 Agent 能可靠控制和停止 Go2，再证明语义地点导航，最后证明视觉搜索并由
智能戒指触发。

**Architecture:** 朋友 Agent/Gateway 负责输入和语言理解；DimOS 是唯一机器人
Runtime。每阶段只增加一层能力，并保留前一阶段的真实机器人回归测试。

**Tech Stack:** Python 3.12、DimOS `0.0.14b1`、Unitree Go2、MCP、Pi Agent、
TypeScript、SQLite、FastAPI Studio、Rerun、Pytest、Ruff、Vitest。

---

## 0. 为什么改成三个阶段

此前的融合总计划适合作为长期架构和技术 backlog，但不适合作为当前 Goal：

- 一次包含语义地图、视觉、跟随、戒指、眼镜和任务平台，无法快速判断效果。
- 多数任务只有单元测试，要到很后面才接触真机。
- 如果底层控制闭环有问题，继续开发视觉和 Agent 只会放大排障成本。

本计划把“真机可验证”作为阶段定义：

| 阶段 | 只回答一个问题 | 真机复查 |
|---|---|---|
| Stage 1 | 朋友 Agent 能否可靠控制、返回和停止 Go2？ | 短距离移动、返回起点、中途停止 |
| Stage 2 | Go2 能否理解并到达已确认的地点？ | 两个命名点往返、重启恢复、临时阻挡 |
| Stage 3 | Go2 能否自主寻找目标，并由戒指发起？ | 探索、候选确认、几何接近、戒指重复请求 |

本轮明确不实现：

- 长时间人员跟随；
- 智能眼镜硬件适配；
- 通用 Mission Profile 平台；
- 多机器人或公网服务；
- 自定义模型训练；
- 操作门、拿取物体或搬运。

这些内容保留在
`docs/plans/2026-07-25-go2-agent-fusion-master-plan.md` 作为后续 backlog，
不属于当前三个阶段的完成条件。

## 1. 三阶段共同规则

### 1.1 单一 Runtime

任何真机复查前都必须确认：

- 一个 `ModuleCoordinator`；
- 一个 `GO2Connection`；
- 一个 MCP `:9990` owner；
- 一个 Viewer/地图数据源；
- 没有同时运行朋友 robot MCP 和另一个 Studio Go2 Blueprint。

### 1.2 证据而不是口头成功

每次真机复查必须保存：

- test/run ID；
- instruction ID、task ID 或 MCP call ID；
- 启动 PID 和模块清单；
- 开始/结束 odometry；
- actual path；
- navigation/motion final state；
- 停止确认；
- 失败原因和日志时间点。

MCP accepted、HTTP 200、模型回复和 UI 按钮变化都不单独算成功。

### 1.3 每阶段执行顺序

```text
失败测试
  -> 最小实现
  -> focused tests
  -> replay/dry-run
  -> hardware read-only
  -> 一次短真机动作
  -> 阶段复查
```

前一步失败，不得跳到下一步。

# Stage 1 — Agent 到 Go2 的最小控制闭环

## Stage 1 Goal

朋友 Agent 能通过一套 MCP/DimOS Runtime 让真实 Go2 执行一个短距离动作、返回本次
进程起点，并能在运动中停止。电脑能显示 actual odometry，而不是只显示命令已发送。

这个阶段不要求机器狗认识“门口”，也不调用视觉模型。

## S1-T1 — 单一 Runtime 启动和诊断

**Status:** `DONE`

**Files:**

- Modify:
  `/Users/johnsonmac/ai_completion/agent/components/dimos-mcp/src/dimos_dog_mcp/blueprint.py`
- Modify:
  `/Users/johnsonmac/ai_completion/agent/components/dimos-mcp/tests/test_go2_launcher.py`
- Modify:
  `/Users/johnsonmac/ai_completion/agent/components/dimos-mcp/tests/test_dimos_integration.py`
- Modify only if needed:
  `scripts/stop_dimos_native_conflicts.sh`

**Implementation:**

1. 写启动预检失败测试：端口被占用、已有 Go2 runtime、重复 MCP Server。
2. 实现明确的 runtime owner 信息。
3. `server_status` 返回 PID、mode、robot IP、module count 和启动时间。
4. 启动失败时不得连接或移动 Go2。

**Verification:**

```bash
cd /Users/johnsonmac/ai_completion/agent/components/dimos-mcp
PYTHONPATH=src .venv/bin/python -m unittest \
  tests.test_go2_launcher tests.test_dimos_integration -v
```

**Pass condition:**

- 第二个 Runtime 被拒绝。
- read-only 启动只产生一个 PID family。
- 地图、相机、odometry 都来自同一个 Runtime。

## S1-T2 — 最小验证工具面

**Status:** `DONE`

**Depends on:** S1-T1。

**Design:** 增加显式 `validation` profile，只用于这个阶段，允许：

```text
relative_move
return_to_start
motion_status
get_robot_summary
stop_all
```

默认 product Agent 不因此获得全部低层工具。

**Files:**

- Modify friend MCP wrapper `config.py`、`server.py`、`dog_tools.py`。
- Modify wrapper focused tests。
- Modify Gateway `agent-runtime.ts` and its focused tests。

**Implementation:**

1. 写 validation/product allowlist 差异测试。
2. 只给 validation profile 注册五个工具。
3. 保持单次转发，不自动重试运动。
4. Agent 回复明确区分 accepted 与 observed movement。

**Pass condition:**

- validation profile 精确发现五个工具。
- product profile 不暴露 `relative_move`。
- `stop_all` 不经过普通 Agent 队列。

## S1-T3 — Actual path 和停止状态

**Status:** `DONE`

**Depends on:** S1-T1。

**Implemented files:**

- Create:
  `/Users/johnsonmac/ai_completion/agent/components/dimos-mcp/src/dimos_dog_mcp/robot_summary.py`
- Create:
  `/Users/johnsonmac/ai_completion/agent/components/dimos-mcp/tests/test_robot_summary.py`
- Modify:
  `/Users/johnsonmac/ai_completion/agent/components/dimos-mcp/src/dimos_dog_mcp/blueprint.py`
- Modify:
  `/Users/johnsonmac/ai_completion/dimos/apps/DimOS Native/main.swift`
- Create:
  `/Users/johnsonmac/ai_completion/dimos/apps/DimOS Native/test_native_launcher.py`

Stage 1 的唯一运行入口在朋友 MCP，因此 actual trajectory 放在该 Runtime 内采样，
Native App 只负责显示，不在 Studio 中复制第二套轨迹状态。

**Implementation:**

1. 从 fresh odometry 采样 actual path。
2. 记录 start pose、latest pose、distance travelled 和 sample time。
3. 与 planned path 分开显示。
4. `motion_status` 增加 last command 和 final observed state，不伪造遥测。

**Pass condition:**

- replay 中 actual path 来自 odometry。
- 无 odometry 时显示 unavailable，不使用计划路径代替。
- stop 后 final state 可观察。

## S1-R1 — 第一次真机复查

**Status:** `PASSED`（2026-07-25）

**前置检查：**

- Go2 已连接同一局域网；
- 一个 Runtime；
- 地图、相机、odometry 新鲜；
- `stop_all` 可调用；
- 场地允许一次短距离动作。

**Test A：短距离动作**

1. 记录 session start pose。
2. 通过朋友 Agent 调用 validation profile，执行约 `0.3 m` 的相对前向任务。
3. 记录 actual displacement 和 final motion/navigation state。

**Test B：返回起点**

1. 调用 `return_to_start`。
2. 持续读取 odometry 和导航状态。
3. 只有距离起点不超过 `0.30 m` 且状态 idle 才算完成。

**Test C：中途停止**

1. 再启动一个短动作。
2. 在动作尚未结束时调用 `stop_all`。
3. 验证 actual path 不再增长，最终速度命令为零。

**Stage 1 Gate:**

- 一个 Runtime，全程没有 owner 变化。
- Test A 有明确 odometry 位移，不要求精确等于 `0.3 m`。
- Test B 回到起点 `<= 0.30 m`。
- Test C 在应用层观察到停止，目标暂定 `<= 1 s`。
- 所有调用和轨迹在同一个页面/记录中可查。

Stage 1 未通过，只修控制、状态、网络和停止，不开始语义地点开发。

**Live evidence:**

- Runtime owner 全程为 PID `69935`，mode `go2`，robot IP `192.168.12.1`；
  MCP worker PID `69969`，`9990`/`7779` 均属于同一 PID family。
- Wrapper `validation` profile 在 `9991` 精确发现五个工具；Gateway 使用无模型
  validation runtime。Pi 的 `zai/glm-5.1` 曾返回 429 余额不足，但不再阻塞本阶段。
- Test A `stage1-forward-001`：0.3 m 请求的测试基线为
  `(-0.3034,-0.1784)`，结束为 `(-0.4763,-0.1405)`，actual displacement
  `0.1770 m`，fresh odometry，最终 stationary。
- Test B `stage1-return-002`：到 Runtime 起点的距离从 `0.3262 m` 降至
  `0.1836 m`，改善 `0.1426 m`，阈值内连续静止超过 3 秒。
- Test C `stage1-stop-motion-002` / `stage1-stop-002`：0.384 秒时真实 odometry
  显示 moving 后发停；停止请求后 0.600 秒首次观测 stationary，2.318 秒后连续
  稳定，停止动作的实际位移约 `0.109 m`。
- Final stop `stage1-final-stop-001`：`motion_status.command_state=idle`，
  odometry age `0.037 s`，observed state `stationary`，speed `0.0006 m/s`。
- Boundary: 被停止的官方 `relative_move` 仍可能返回 `Navigation goal reached`；
  该字符串不是物理完成证据，后续阶段继续以 fresh odometry 和任务状态为准。

# Stage 2 — 已知语义地点导航

## Stage 2 Goal

用户只需确认一次“测试起点”和“门口测试点”。之后在朋友 Agent 中说“去门口测试点”
或“回到测试起点”，Go2 使用同一任务执行器和地图自主导航；重启后地点仍可用，
临时阻挡不会导致无限顶撞。

## S2-T1 — 最小 Persistent SemanticWorld

**Status:** `COMPLETE`（2026-07-25，软件验收）

**Depends on:** Stage 1 Gate。

**Files:**

- Create `semantic_world.py` and `test_semantic_world.py`。
- Modify `mission_executor.py` and product Blueprint。

**Scope:**

- 只实现人工确认的地点；
- 保存 name、alias、map ID/version、map pose、timestamp；
- 不做自动地点建议；
- 不接视觉模型。

**Implementation:**

1. 写保存、重启、alias、地图不匹配和重复名称测试。
2. 实现 confirmed-place repository。
3. 实现 `DestinationResolver`。
4. 注入现有 `MissionExecutor`。

**Pass condition:**

- 两个地点可保存并重载。
- map version 不匹配时 fail-closed。
- `start_task` 可从名称解析到一个 map-frame pose。

**Validation evidence:**

- 子计划：
  `docs/plans/2026-07-25-S2-T1-minimal-persistent-semantic-world.md`。
- 21 项聚焦测试通过；Ruff 和 `git diff --check` 通过。
- `SemanticWorld` 与 `DestinationResolverSpec` 的结构和注解兼容检查均通过。
- 本任务未移动真机；真机语义导航仍必须等待 S2-T2/T3/T4 和 S2-R1。

## S2-T2 — 朋友 MCP 加载唯一 MissionExecutor

**Status:** `COMPLETE`（2026-07-25，软件与 MCP replay 验收）

**Depends on:** S2-T1。

**Files:**

- Modify friend `components/dimos-mcp/pyproject.toml`。
- Modify friend `components/dimos-mcp/src/dimos_dog_mcp/blueprint.py`。
- Create `tests/test_product_blueprint.py`。

**Implementation:**

1. 测试一个 `MissionExecutor`、一个 navigator、一个 MCP Server。
2. 把 `dimos-go2-studio` 作为明确安装依赖。
3. 组合 module atoms，不嵌套启动第二个完整 Blueprint。
4. replay 验证 `start_task/get_task_status/cancel_task`。

**Pass condition:**

- 朋友 MCP 的同一个 Runtime 暴露 P1 工具。
- 旧 21 工具不能绕过 active mission。
- 没有 embedded conversational Agent。

**Validation evidence:**

- 子计划：
  `docs/plans/2026-07-25-S2-T2-friend-mcp-mission-composition.md`。
- 朋友 MCP 明确依赖同机 `dimos-go2-studio`，dry-run 和 Go2 Blueprint 均只
  组合一个 `SemanticWorld`、一个 `MissionExecutor`、一个 navigator 和一个
  `DogMcpServer`；Go2 仍只有一个 `GO2Connection`。
- product profile 包含任务生命周期、统一停止和只读状态工具，不注册旧低层运动
  工具；maintenance profile 保留人工调试面。
- 真实本地 MCP HTTP replay 以同一 task ID 完成
  `start -> navigating -> cancel -> cancelled/idle`；`stop_all` 先取消 mission。
- 朋友 MCP 62 项完整测试、DimOS 22 项聚焦测试、两个仓库 Ruff 和
  `git diff --check` 通过。macOS 上 DimOS 初始化的 Zenoh 后台线程会阻止
  `unittest` 进程自然退出，因此完整套件在取得明确 `OK` 后由隔离 harness 回收；
  没有残留测试 Runtime。
- 本任务没有重启真实 Go2 Runtime，也没有移动机器狗；真机链路仍等待
  S2-T3/T4 完成后由 S2-R1 验收。

## S2-T3 — Agent TaskSpec 和任务绑定

**Status:** `COMPLETE`（2026-07-25，软件验收）

**Depends on:** S2-T2。

**Files:**

- Create Gateway `task-contract.ts`、`task-monitor.ts` 和 tests。
- Modify `agent-runtime.ts`、`service.ts`、`store.ts`、`types.ts`。

**Implementation:**

1. Agent 只生成 kind/destination 等任务参数。
2. Gateway 用 `instruction_id` 生成稳定 `task_id` 和 UTC 时间。
3. 持久化 instruction/task binding。
4. accepted 后轮询任务状态。
5. terminal snapshot 后才投递最终回复。
6. 重启不得重复启动任务。

**Pass condition:**

- 相同 instruction ID 只创建一个 task。
- Agent 不会把 accepted 当 completed。
- cancel 后任务状态和导航状态一致。

**Validation evidence:**

- 子计划：
  `docs/plans/2026-07-25-S2-T3-agent-task-binding.md`。
- product Pi session 没有任何工具，只输出严格
  `go_to_place + destination` 参数；Gateway 生成稳定 task ID 和完整 TaskSpec。
- SQLite binding 在 `start_task` 前持久化；重复 instruction 不重新编译/提交，
  重启恢复只轮询 `get_task_status`。
- accepted/navigating 不产生回复；同一 task 只有 terminal 且 `active=false` 后
  形成确定性 completed/failed/cancelled 回复。
- Gateway 10 个文件 39 项定向测试和包级 TypeScript 检查通过；Wrapper 16 项
  测试与 Ruff 通过。全仓 `npm run check` 的本任务文件格式检查通过，但随后被
  上游 `packages/ai` 模型目录的既有全仓类型错误阻断。
- 本任务没有启动真实 Go2 Runtime，也没有移动机器狗。

## S2-T4 — 地点 UI、任务状态和恢复事件

**Status:** `COMPLETE`

**Depends on:** S2-T1、S2-T3。

**Files:**

- Modify Studio app/service/static files and focused tests。
- Reuse Stage 1 actual trajectory。

**UI 最少显示：**

- map ID/version；
- 两个 confirmed places；
- task ID/state；
- destination；
- planned path；
- actual path；
- recovery cause/attempt；
- cancel button。

## S2-R1 — 第二次真机复查

**Status:** `IN_PROGRESS`

**准备：**

1. 在同一张小范围室内地图确认两个点：
   - `测试起点`
   - `门口测试点`
2. 两点建议相距 `1–3 m`，保持同层平地。

**Test A：语义往返**

- Agent：`去门口测试点`
- Agent：`回到测试起点`
- 共执行三次单程任务。

**Test B：重启恢复**

- 正常停止 Runtime。
- 重新启动并完成 relocalization。
- 再执行一次 `去门口测试点`。

**Test C：临时阻挡**

- 在可绕行条件下加入一个临时障碍。
- 允许 planner 执行有界 replan/rotate-rescan。
- 记录完成或 typed failure，不要求强行通过。

**Stage 2 Gate:**

- 三次正常单程至少 `2/3` 到达。
- 到达误差 `<= 0.60 m`。
- 重启后地点可恢复，且 map version 正确。
- 临时阻挡不会无限推动或无限重试。
- Agent、MCP、UI 显示同一 task ID 和 terminal state。
- 任意一次 cancel 最终导航 idle。

Stage 2 未通过，不开始视觉和戒指接入。

**2026-07-25 preparation evidence:**

- `SemanticWorld` 已改为在确认地点时把当前 `world` 位姿转换到稳定 `map` 帧，
  解析地点时再转换回当前会话 `world`；变换缺失时 fail-closed。
- friend Go2 Runtime 已要求现有 `DIMOS_PREMAP_FILE` 并加载一个官方
  `RelocalizationModule`；机器人摘要暴露重定位 ready/reason。
- `192.168.12.1` 的只读 WebRTC 实测完成 ICE、peer 与 data-channel 验证，并
  收到新鲜 camera/LiDAR/odom；没有发送站立或运动指令。
- macOS 系统代理曾截获机器人 `/con_notify`；将机器人 IP 同时加入 `NO_PROXY`
  和 `no_proxy` 后恢复，修复已进入启动入口。
- 只读记录导出的预建图有 35,357 点；官方离线重定位按设计报告
  `n_pts < MIN_LOCAL_POINTS=50000`，因此当前仍没有可验收的 `world -> map`
  变换。不得降低阈值来制造通过。
- 下一步需要明确 `START GO2，场地已清空`，然后做短距离受控移动积累点云，
  验证重定位 ready 后再确认两个语义点并执行本节 A/B/C。S2-R1 仍未通过。
- friend MCP 已增加 `dimos-stage2-audit`：只读 preflight 在任何运动前核对
  Runtime/PID、fresh odometry、relocalization、地图地点与任务 idle；每个 trip
  只提交一次 `start_task`，机器记录同一 task ID、终态和稳定帧到达误差；独立
  cancel-check 要求 `cancelled + navigation_idle=true`。该工具不能替代
  Agent/Gateway/Studio 三个表面的 task ID 对照，也尚未运行真实 trip。

# Stage 3 — 自主视觉搜索与戒指触发

## Stage 3 Goal

用户或戒指发送“寻找门”。Go2 在限定区域内探索，发现候选时停止，使用视觉模型判断，
再结合相机/LiDAR/姿态生成可导航的 stand-off；系统能够接近目标或诚实返回
not_found/uncertain，并在电脑上显示证据。

这个阶段仍不实现人员跟随和智能眼镜硬件。

## S3-T1 — Synchronized ObservationBundle

**Status:** `NOT_STARTED`

**Files:** create `observation_bundle.py` and tests；必要时 modify Go2 connection。

**Pass condition:** 每个候选都有 fresh image、camera info、map pose、source 和
可选 point-cloud reference；陈旧数据进入 uncertain。

## S3-T2 — Candidate-only vision verifier

**Status:** `COMPLETE — SAVED-FRAME REPLAY ONLY`

**Depends on:** S3-T1。

> 2026-07-25 用户明确要求优先完善 S3-T2，因此只提前完成与 S3-T1 解耦的保存帧
> replay/provider gateway。实时候选接线仍依赖 S3-T1，且这不解除 S2-R1 或 Stage 3
> 真机门禁。

**Files:** reuse/adapt `target_verification.py`；create `vision_gateway.py` and tests。

**Implementation:**

1. 先在保存帧 replay 上运行。
2. provider 输出 yes/no/uncertain、region、confidence 和 metadata。
3. 不连续上传视频；每个候选最多选择少量帧。
4. provider timeout 不阻塞本地停止。

**Validation evidence:**

- 子计划：
  `docs/plans/2026-07-25-S3-T2-candidate-only-vision-gateway.md`。
- 新增非阻塞 `CandidateVisionGateway`、bounded provider slot、deadline typed
  uncertain、late-result suppression、candidate idempotency/conflict 和 sanitized
  provider metadata。
- 保存帧 replay/既有 verifier/CLI 联合回归 20/20，Ruff 和 diff check 通过。
- 未连接 provider、未上传真实帧、未启动或移动 Go2；live integration 继续等待
  S3-T1。

## S3-R1 — 候选发现即停止复查

**Robot test:**

1. 在限定距离/时间内开始探索。
2. 产生一个候选后停止 navigation。
3. 保存 fresh candidate frame、pose 和 task ID。
4. 不向候选移动。

**Pass condition:** 机器人确实移动探索、候选后确实停止、证据可查看；不要求确认门。

## S3-T3 — Camera/LiDAR/map grounding

**Status:** `NOT_STARTED`

**Depends on:** S3-T1、S3-T2。

**Files:** create `semantic_grounding.py` and tests。

**Implementation:** 2D region -> valid depth/LiDAR points -> map pose ->
uncertainty -> collision-checked stand-off。

**Pass condition:** 视觉 yes 不能单独产生目标；缺 TF/深度时返回 uncertain。

## S3-R2 — Grounding 和短距离接近复查

**Robot test:**

1. 使用已确认候选。
2. 生成目标前约 `1 m` stand-off。
3. 执行一个短、可取消的 approach。
4. 到达后停止并保存 final pose。

**Pass condition:**

- 没有几何落点时不移动。
- 有落点时目标误差满足 `1.0 m ± 0.6 m`。
- approach 可中途取消。

## S3-T4 — Active search mission and evidence result

**Status:** `NOT_STARTED`

**Depends on:** S3-T2、S3-T3。

**Files:** create `semantic_search.py`、`evidence_service.py` 和 tests；extend
`mission_executor.py`。

**Implementation:**

- 候选排序；
- viewpoint/frontier 选择；
- rejected candidate 换视角；
- 限制时间、距离、候选数；
- found/not_found/uncertain terminal result；
- evidence IDs。

## S3-T5 — Smart-ring input adapter

**Status:** `NOT_STARTED`

**Depends on:** S2-T3、S3-T4。

**Files:**

- Create friend Gateway `ring-adapter.ts` and tests。
- 只在戒指实际传输协议确认后增加 BLE/HTTP adapter。

**Implementation:**

1. command ID、device ID、timestamp、payload、expiry。
2. 转换为现有 `ExternalInstruction`。
3. 使用相同 instruction/task binding。
4. 测试 duplicate、retry、out-of-order 和 disconnect。

## S3-R3 — 第三次真机复查

**Test A：电脑 Agent 发起**

- 输入：`寻找门`
- 机器人：探索 -> candidate -> stop -> verify -> ground -> approach 或诚实终态。

**Test B：戒指发起同一任务**

- 戒指通过电脑发送同样的高层请求。
- 重复发送相同 command ID。
- 系统只能创建一个 task。

**Test C：视觉 provider 断开**

- 在任务期间让 provider 不可用。
- 验证本地 stop/cancel、地图和导航仍工作。
- 任务进入 provider_unavailable/uncertain，而不是继续盲走。

**Stage 3 Gate:**

- 三次限定搜索中没有 false verified target。
- 至少一次完整通过 candidate -> grounding -> approach。
- 没找到时正确返回 not_found/uncertain。
- 戒指重复请求只生成一个 task。
- provider 断开不影响本地停止。
- UI 保留 task、actual path、candidate frames、grounded pose 和 final result。

## 2. 每阶段统一测试命令

### DimOS

```bash
cd /Users/johnsonmac/ai_completion/dimos
PYTHONPATH=extensions/go2-studio-agent/src \
  .venv/bin/python -m pytest \
  extensions/go2-studio-agent/tests \
  dimos/web/studio/test_mission.py -v
.venv/bin/ruff check \
  extensions/go2-studio-agent/src \
  extensions/go2-studio-agent/tests
git diff --check
```

### 朋友 MCP

```bash
cd /Users/johnsonmac/ai_completion/agent/components/dimos-mcp
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -v
```

### Agent Gateway

```bash
cd /Users/johnsonmac/ai_completion/agent/components/agent-framework/agent-webhook-gateway
npm run check
node node_modules/vitest/dist/cli.js --run \
  test/agent-runtime.test.ts \
  test/gateway.test.ts \
  test/mcp-client.test.ts
```

每个 `Sx-Tx` child plan 只运行受影响的 focused tests；`Sx-Rx` 前运行该阶段的完整
回归。

## 3. 当前唯一 Goal

当前只执行：

```text
Stage 2 — 已知语义地点导航
```

当前下一项任务：

```text
S2-R1 — 第二次真机复查
```

Stage 1 已于 2026-07-25 通过 S1-R1 真机复查；S2-T1、S2-T2、S2-T3 和 S2-T4
已于同日通过各自软件验收。当前只执行 Stage 2 的 `S2-R1`；在 S2-R1 真机复查
通过前，不开始 Stage 3 视觉模型、戒指、人员跟随或平台化开发。
