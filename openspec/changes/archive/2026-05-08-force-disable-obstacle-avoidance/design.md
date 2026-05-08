# 设计：强制关闭避障

## 概览

强制移动覆盖沿用现有 Command Center 手动控制链路：

```text
KeyboardControlPanel
  -> Connection.setForceMoveOverride()
  -> socket.io force_move_override event
  -> WebsocketVisModule.force_move_override Out[Bool]
  -> GO2Connection.force_move_override In[Bool]
  -> connection.set_obstacle_avoidance(False/configured)
```

该覆盖能力刻意绑定到手动移动。Web UI 只有在操作员同时按住 Ctrl+Shift 和移动键时才会请求启用。Go2 连接模块也识别双击移动手势，作为同一行为的辅助触发路径。

## UI 行为

`KeyboardControlPanel` 将 Ctrl+Shift+移动键视为强制覆盖模式。在该模式下，它使用降低后的速度倍率，而不是普通的 Shift 加速模式。面板只在覆盖处于启用状态，或需要发送回到禁用状态的跳变时，发送覆盖状态更新。

当键盘控制停止时，UI 会先显式清除覆盖状态，再停止移动。这样可以避免客户端状态过期后让机器人继续留在覆盖模式。

## 传输行为

`Connection` 发送带有 `enabled` boolean 字段的 `force_move_override` Socket.IO 事件。`WebsocketVisModule` 接收该事件，并通过新的 `force_move_override: Out[Bool]` stream 发布。这样覆盖能力仍然走 DimOS typed stream 连接方式，而不是在 Web 模块中直接加入 Go2 专用调用。

## Go2 行为

`GO2Connection` 订阅 `force_move_override: In[Bool]`。

启用时：

- 记录最近一次更新时间。
- 通过 `connection.set_obstacle_avoidance(False)` 关闭 Go2 避障。

禁用时：

- 发送零速度 twist 来停止运动。
- 将避障恢复为 `self.config.g.obstacle_avoidance` 中的配置值。

连接模块会启动 watchdog 线程。如果在 `FORCE_MOVE_OVERRIDE_TIMEOUT_SECONDS` 内没有收到刷新，它会自动禁用覆盖。`stop()` 也会清除覆盖状态，并等待 watchdog 线程退出。

## 安全说明

- 覆盖是临时状态，必须持续刷新。
- 释放手势或停止键盘控制会禁用覆盖。
- 超时兜底会恢复避障。
- 禁用路径会先停止机器人，再恢复避障。
- 该功能会有意绕过机器人安全行为，因此仍需要人工监督下的硬件验证。
