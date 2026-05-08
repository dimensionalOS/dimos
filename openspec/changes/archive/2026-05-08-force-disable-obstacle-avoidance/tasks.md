# 任务：强制关闭避障

- [x] 在 Command Center 中增加强制移动覆盖的键盘手势。
- [x] 通过 Socket.IO 从 Web 客户端发送强制覆盖状态。
- [x] 在 `WebsocketVisModule` 中将强制覆盖状态发布为 typed `Bool` stream。
- [x] 在 `GO2Connection` 中订阅强制覆盖状态。
- [x] 覆盖生效期间关闭 Go2 避障。
- [x] 覆盖禁用时停止运动，并恢复配置中的避障状态。
- [x] 增加 watchdog 超时，让过期的覆盖请求自动清除。
- [x] 在 Go2 连接关闭期间清除覆盖状态。
- [x] 在 OpenSpec artifacts 中记录剩余的硬件验证风险。
