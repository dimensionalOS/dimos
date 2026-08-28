# Host Daemon 设计

状态：设计大纲

## 定位

Host daemon 是每台执行机器上的常驻服务。它向 Controller 暴露本机的
身份和可用状态，并负责启动、观察和停止分配到本机的 Host fragment。

Host daemon 不负责全局放置，不重新编译完整 Blueprint，也不转发模块间
的数据流。

## 进程模型

每台机器运行一个 Host daemon。MVP 中，一个 Host 同时只运行一个
deployment。

Host daemon 收到已经解析完成的 Host fragment 后，为它启动一个独立的
子进程。子进程使用现有 `ModuleCoordinator` 部署模块和 workers。应用数据
通过 Zenoh 在模块之间直接传输，不经过 Host daemon。

Host daemon 与 deployment 子进程分离，避免模块异常、全局配置或 native
依赖影响常驻服务。

## Host 描述

Host 对外发布一个最小 descriptor：

| 字段            | 含义                                           |
| --------------- | ---------------------------------------------- |
| `host_id`       | 自动生成并持久化的稳定身份                     |
| `epoch`         | Host daemon 每次启动生成的新实例标识           |
| `name`          | 便于用户识别的名称                             |
| `tags`          | 用于 placement 的简单标签                      |
| `versions`      | Host 协议、fragment schema、DimOS 和应用版本   |
| `state`         | `available`、`starting`、`running` 或 `failed` |
| `active_run_id` | 当前运行的 application run ID，没有时为空      |

Zenoh liveliness 表示 Host 是否在线。Controller 发现 Host 后，通过
`describe` 获取最新 descriptor，不使用持久化的旧 descriptor 做放置决定。

## 控制接口

Host daemon 暴露四个幂等操作：

| 操作       | 含义                                            |
| ---------- | ----------------------------------------------- |
| `describe` | 返回当前 Host descriptor                        |
| `start`    | 在 Host 空闲时校验并启动一个 Host fragment      |
| `status`   | 返回当前 deployment 的状态、PID、日志位置和错误 |
| `stop`     | 停止当前 deployment 并释放 Host                 |

`start` 同时完成资源占用和启动。Host 在一个原子检查中从 `available`
进入 `starting`，因此多个 Controller 同时请求时最多只有一个成功，其余请求
返回 `RuntimeError`。

每个控制请求携带目标 `host_id` 和 Controller 观察到的 `epoch`。修改状态的
请求还携带全局唯一的 `run_id`、`generation` 和 `fragment_digest`，用于识别
重复请求和冲突请求。

## Host fragment

Host fragment 是 Controller 从完整应用图中切分出的本机部署单元，其中包含：

- 本机需要运行的模块；
- 已解析的模块配置和 GlobalConfig；
- 本机 stream 连接和跨 Host stream 的 Zenoh 配置；
- run ID、Host ID 和版本信息。

MVP 使用 Python fragment。Controller 序列化已经解析的 Blueprint fragment，
Host 子进程反序列化后交给现有 `ModuleCoordinator`。Host 不再根据原始
Blueprint 独立执行 placement 或全图解析。

## 状态模型

Host 的 deployment 状态为：

| 状态        | 含义                              |
| ----------- | --------------------------------- |
| `available` | 没有 deployment，可以接受 `start` |
| `starting`  | 子进程正在构建和启动 fragment     |
| `running`   | 模块已启动且初始健康检查通过      |
| `stopping`  | 正在停止子进程和 workers          |
| `failed`    | 启动失败或运行进程异常退出        |

正常状态转换为：

`available → starting → running → stopping → available`

启动失败时进入 `failed`。Host 完成残留进程清理后可以重新进入
`available`。重复的 `start`、`status` 和 `stop` 根据当前状态返回已有结果，
不会创建重复 deployment。

## 启动与回滚

Controller 完成全局编译、placement 和 fragment 切分后，并行调用各 Host 的
`start`。Host 只有在子进程完成模块启动和初始健康检查后才返回成功。

如果任一 Host 启动失败，Controller 对已经成功启动的 Host 调用 `stop`。
MVP 接受短暂的部分启动，不提供 prepare lease 或严格的分布式事务。

## 本地状态

Host daemon 只持久化两类信息：

- 稳定的 `host_id`；
- 当前 deployment 的 run ID、generation、状态、子进程 PID、fragment
  digest 和日志目录。

Host daemon 启动时先检查当前 deployment 记录，再决定是否发布
`available`。如果记录中的进程已经退出，Host 清理残留 workers 并记录失败；
如果进程仍然存在，则继续将 Host 视为已占用，避免同一物理设备被重复分配。

## 扩展边界

Host descriptor、控制请求和 Host fragment 从第一版开始携带版本。后续可以
在不改变 Host 基本职责的情况下增加 prepare lease、多 deployment slot、结构化
capabilities、资源限制、身份认证、更完整的进程恢复和其他 fragment 格式。
