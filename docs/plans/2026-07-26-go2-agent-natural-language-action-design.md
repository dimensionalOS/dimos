# Go2 Agent 自然语言动作入口设计

日期：2026-07-26
状态：软件实现完成，真机闭环待验收

## 目标

让本地 Agent Console 的文字输入直接进入现有 Agent，并转换成 DimOS Product
Runtime 可执行的任务或受限动作。不得新增第二个 Agent、MCP、Planner、机器人连接
或任务状态 Owner。

## 方案

采用两条共享同一 Gateway、Wrapper 和 Runtime 的输入路径：

```text
低层全句口语
→ 本地确定性解析
→ priority queue
→ stop_all
→ relative_move
→ Wrapper :9991
→ Product Runtime :9990
→ 官方 NavigationInterface
→ 唯一 GO2Connection

高层任务语言
→ 现有无工具 taskCompiler
→ canonical TaskSpec
→ start_task
→ MissionExecutor
→ 官方导航
```

低层解析只处理停止、状态、暂停/继续/取消，以及前后左右和转向。允许有限礼貌外壳，
例如“请你”“帮我”“一下”“吧”，并且必须整句匹配。未指定参数时，平移固定为
`0.2 m`，转向固定为 `15°`；模型不能选择或覆盖参数。

“不要往前走”“往前走到门口”“往前走 2 米”“往前走然后左转”等文本不得命中
低层动作。它们继续进入高层分析，并在无法生成合法 canonical 任务时明确回复未执行。

高层语义继续只允许：

- `go_to_place`
- `mark_place`
- 有限 `visit_route`
- `follow_person`

## 错误语义

MCP 客户端将失败分为：

- `unavailable`：Wrapper 或 Runtime 未连接；
- `timeout`：结果未知，不允许自动重发；
- `protocol`：Gateway、Wrapper 与 Runtime 工具版本不一致；
- `rejected`：Runtime 明确拒绝动作。

原始 URL、堆栈和内部响应只写本机日志。Console 显示脱敏提示，并将失败显示为
“未执行”，不能再用“存在 reply”推断任务成功。

## 验收

软件验收：

1. “往前走”等口语不调用模型；
2. 调用顺序严格为 `stop_all -> relative_move`；
3. 参数固定且没有自动重试；
4. 否定、复合、目的地和距离负例不调用物理 MCP；
5. unavailable、timeout、protocol、rejected 可区分；
6. 高层地点与路线回归不变；
7. Console 正确显示失败状态。

真机验收：

1. 机器狗 LAN 可达；
2. 只有一个 Product Runtime 和一个 `GO2Connection`；
3. Runtime 与 Wrapper 各发现精确 20 个 product tools；
4. `get_robot_summary` 显示 fresh odometry；
5. Viewer `:9878` 在线；
6. 新建一条“往前走”指令，只执行一次；
7. 用动作前后 odometry 验证真实位移，不仅依赖字符串回复；
8. `stop_all` 后确认任务和机器人均不再运动。
