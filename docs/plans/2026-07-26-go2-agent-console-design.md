# Go2 Agent Console 最小设计

## 目标

用户只打开一个本地前台即可看到官方 DimOS 地图、输入自然语言任务、观察真实任务
状态并停止当前活动；日常操作不再依赖 Codex。

## 选择

采用现有朋友 Gateway 的同源前台：

```text
Agent Console :8080
  ├─ iframe → 官方 Rerun web viewer :9878（只读地图）
  ├─ POST /v1/instructions → 现有 Pi 参数编译器
  └─ GET /v1/instructions/:id → 现有 SQLite binding/outbox
                         ↓
Wrapper :9991 → 唯一 MCP Runtime :9990 → DimOS → Go2
```

Console 不连接机器人、不调用 MCP、不规划路线、不保存地图或机器人状态。地图继续由
现有 `RerunBridge` 发布；planned path、actual path、semantic places 和 current
target 仍只有一份。停止按钮提交精确文本“停”，由 Gateway 优先通道调用现有
`stop_all`。

## 备选方案

1. **采用：Gateway 同源静态前台。** 服务最少、无 CORS、直接复用持久化状态。
2. 修改旧 Studio。已有界面太重，且旧 Agent 文本入口不是朋友 Gateway。
3. 新建前台后端。会增加部署、回复存储和状态 Owner，当前没有必要。

## 失败处理

- Gateway 离线：输入按钮报告未进入 Agent，不声称任务已受理。
- Viewer 离线：地图区域明确显示 Runtime 未运行，文字链路仍可诊断。
- Agent/MCP 失败：只显示持久化的固定失败回复，不暴露内部异常。
- 页面刷新：从本地保存的最后 `instruction_id` 恢复只读轮询，不重新提交。

## 验收

- `/`、静态资源和 `/v1/ui-config` 可同源加载。
- 一次输入只生成一个 `instruction_id`，重复轮询不调用 Agent/MCP。
- 页面能显示 pending/processing/task state/final reply。
- 停止按钮只提交一次“停”，不直接调用 MCP。
- `RERUN_OPEN=none`、`RERUN_WEB=true` 时地图服务存在但不自动打开额外标签页。
- 单元测试、TypeScript 检查、仓库检查和视觉截图通过。
- 本轮软件验证不描述成真实机器狗运动成功。
