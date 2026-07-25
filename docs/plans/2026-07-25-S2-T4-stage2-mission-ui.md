# S2-T4 地点、任务与轨迹 UI 计划

> **执行要求：** 使用 `executing-plans` 技能；先写失败测试再实现。本任务只做
> 软件和本地回放，不启动或移动真实机器狗。

**目标：** 让现有 DimOS Studio 直接监督朋友 product 控制栈：显示当前地图
ID/version、已确认地点、canonical task ID/state/destination、规划路线、真实
里程计轨迹和最近恢复事件；通过 Gateway 提交一个已知地点任务，并用 task ID
直接取消。

**边界：**

- Studio 是监督 UI，不创建第二个 robot Runtime。
- 新任务必须经 `Agent Webhook Gateway :8080`，不能从 UI 绕过
  `instruction_id -> task_id` binding 直接调用 `start_task`。
- 状态和取消经 product Wrapper `:9991/mcp`。
- `planned_path` 必须来自 planner 的 `path` stream；`actual_path` 必须来自
  odometry。两者不能互相替代。
- UI 收到 `202` 只显示“已受理”，不能显示“已到达”。
- 地点确认只能由操作者从 fresh odometry 发起，Agent 不得自行确认。
- 本任务不创建真实地图，不做 S2-R1 真机往返。

## Task 1：下游只读导航遥测

扩展已有 `get_robot_summary`，订阅 planner `path` 和 `recovery_event`，返回：

- `planned_path`
- `planned_path_frame_id`
- `recovery`（attempt/cause/action/outcome/reason/timestamp）
- 既有 `actual_path` 和 odometry freshness

dry-run 明确返回空路径和 `recovery=null`。

## Task 2：Studio Stage 2 控制适配器

创建严格、可测试的本地适配器：

1. 聚合 `list_semantic_places/get_task_status/get_robot_summary`。
2. 地点不存在或地图未配置时拒绝提交。
3. 用输入端提供的稳定 instruction ID 向 Gateway 提交“去<地点>”。
4. 持久化最近 submission 和 `agent.reply.completed`，重复 reply ID 幂等。
5. `cancel_task` 必须携带当前 canonical task ID。
6. Wrapper/Gateway 不可达时返回明确 unavailable，不伪造空闲或完成。

## Task 3：API 与最小可用界面

新增：

- `GET /api/stage2/status`
- `POST /api/stage2/navigate`
- `POST /api/stage2/cancel`
- `POST /api/stage2/reply`
- `POST /api/stage2/places/confirm-current`

Mission Control 显示：

- map ID/version 和 confirmed places；
- task ID/state/destination；
- planned/actual path 的独立折线和样本数；
- odometry freshness；
- recovery cause/attempt/action；
- “把机器狗当前位置确认成地点”（fresh odometry 才启用）；
- “前往选中地点”和“取消当前任务”。

## 验收

1. fake MCP 回放两个 confirmed places、navigating、planned/actual path 和 recovery。
2. UI 提交只到 Gateway 一次，并保留稳定 instruction ID。
3. 同一 reply ID 重投不产生第二条 UI 结果。
4. cancel 精确转发当前 task ID。
5. unavailable、idle、navigating、terminal 状态不会互相混淆。
6. Python focused tests、Ruff、JS syntax 和 `git diff --check` 通过。

## 实现结果（2026-07-25）

**Status:** `COMPLETE`

- `StageTwoControl` 聚合 product Wrapper/Gateway，并持久化 submission/reply
  read model。
- 当前点确认严格要求 fresh odometry、有限 pose/quaternion、显式名称和当前
  map ID/version；alias 规范化去重。该路径只暴露在 Studio operator API，不进入
  product Agent 参数编译器。
- UI 独立显示 planned/actual path、样本数、odometry freshness、recovery 和
  canonical task ID/state；active task 期间禁用地点确认。
- 验证：Studio 聚焦套件 29/29，Ruff、`node --check` 和相关
  `git diff --check` 通过。
- 真机边界：S2-T4 不证明 Go2 已到达；下一步只执行 S2-R1。

**不在本任务中声明：**

- 不声明当前 Go2 已连接、已移动或已到达。
- 不声明两个真实 confirmed places 已创建。
- 不把 iframe 地图或 planned path 当作真实运动证据。
