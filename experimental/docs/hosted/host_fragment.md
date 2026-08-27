# Host Fragment 设计

状态：设计大纲

## 定位

Host fragment 是 Controller 将完整应用图完成解析和 placement 后，为某一个
Host 生成的本地部署描述。一个分布式 run 在每个参与的 Host 上最多有一个
fragment。

Host fragment 只描述“这个 Host 要运行什么以及如何连接”。它不包含全局
placement 逻辑，Host 也不根据 fragment 重新选择机器或重新编译完整应用图。

Host fragment 与 placement unit 不同。placement unit 是放置时不可拆分的一组
模块；多个 placement unit 如果被分配到同一个 Host，会合并为一个 Host
fragment。

## 最小结构

MVP 只支持 Python fragment。一个 fragment 由版本化 envelope 和 Blueprint
payload 组成。

| 字段 | 含义 |
| --- | --- |
| `schema_version` | Host fragment schema 版本，MVP 为 `1` |
| `format` | payload 格式，MVP 为 `python-blueprint` |
| `run_id` | 整个分布式 application 的唯一运行标识 |
| `generation` | 同一个 run 的 fragment 部署代数 |
| `host_id` | fragment 指定的目标 Host |
| `application_name` | 用于状态和日志展示的应用名称 |
| `application_revision` | Host 必须具备的代码版本 |
| `payload_digest` | 对实际 payload bytes 计算的摘要 |
| `blueprint_payload` | 序列化后的本地 Blueprint fragment |
| `config` | Controller 已解析的应用和模块配置 |

`run_id`、`host_id` 和 `generation` 共同标识一次 Host deployment。同一标识
重复提交时，`payload_digest` 必须相同；digest 不同表示冲突，而不是更新。

## Blueprint payload

`blueprint_payload` 是普通 Blueprint 的一个已解析子图，其中只保留目标 Host
需要的内容：

- 分配到这个 Host 的模块实例；
- 模块的唯一实例名和构造参数；
- 模块之间的本地 stream 连接；
- 跨 Host stream 在本机一侧的 endpoint；
- 已解析的 stream 名称、类型、remapping 和 transport；
- 只指向本 fragment 内模块的 module reference。

跨 Host stream 的两端分别存在于对应 Host 的 fragment 中，并使用相同的
run-scoped Zenoh topic。fragment 不包含远端模块实例，也不包含应用数据代理。

placement metadata 在 Controller 完成分配后不再参与 Host 执行，可以从
payload 中移除。Host 只验证 fragment 的目标 Host、版本和本地可执行性。

## 配置边界

Controller 放入 fragment 的配置包括：

- Blueprint 中固定的模块参数；
- Controller 解析后的应用级配置；
- run ID、Host ID 和 run-scoped topic namespace；
- 跨 Host stream 强制使用的 Zenoh transport 配置。

Host daemon 保留机器本地配置，例如 Zenoh 接入方式、设备路径、凭据和日志
目录。这些配置不由远端 fragment 任意覆盖。MVP 应明确一小组允许 Host 注入
或覆盖的字段，其他字段使用 Controller 已解析的值。

## 生成流程

Controller 按以下顺序生成 fragment：

1. 解析完整 Blueprint 的模块、stream、remapping 和 module reference；
2. 完成 placement，为每个模块确定唯一 Host；
3. 将连接分类为本地连接或跨 Host 连接；
4. 为每个 Host 提取本地子图和 boundary stream endpoint；
5. 固定配置、transport 和 run-scoped topic；
6. 序列化 Blueprint payload，并对实际 bytes 计算 digest；
7. 将 envelope 和 payload 交给目标 Host。

同一份已解析应用图是所有 fragment 的唯一来源。各 Host 不独立加载原始
Blueprint，以免因为环境变量、代码版本或本地 discovery 产生不同的图。

## Host 执行

Host daemon 收到 fragment 后先验证：

- `host_id` 与本机身份一致；
- schema、payload format 和应用版本兼容；
- payload digest 正确；
- Host 当前可以接受 deployment；
- fragment 不包含跨 Host module reference；
- boundary stream 使用 Zenoh，机器本地 transport 没有跨 Host。

验证通过后，Host 将原始 fragment 持久化并启动子进程。子进程合并允许的
Host-local 配置，反序列化 Blueprint payload，然后交给现有
`ModuleCoordinator` 构建和启动。

fragment 一旦被 Host 接受即保持不变。配置或模块发生变化时，Controller
生成新的 generation，而不是原地修改正在运行的 fragment。

## MVP 序列化

MVP 面向已经安装相同 Python 代码的可信网络，可以复用现有 Python
pickle/o3dpickle 能力传输 Blueprint fragment。`payload_digest` 对最终传输的
bytes 计算，不要求第一版实现 canonical serialization。

这一格式不用于代码分发，也不应被描述为适用于不可信网络。未来可以在
保留 envelope 的情况下增加安全、语言无关的 fragment format。

## 必须保持的约束

- 一个模块实例只出现在一个 Host fragment 中；
- 一个 Host 在同一个 run 中最多接收一个 fragment；
- 所有跨 Host stream 都使用相同的 run-scoped Zenoh topic 规则；
- 所有 module reference 都在 fragment 内闭合；
- Host 执行的代码 revision 与 fragment 要求一致；
- Host 不修改已经接受的 fragment。

## 扩展边界

后续版本可以增加资源需求、artifact 引用、secret 引用、非 Python payload、
签名和权限信息。多副本、动态迁移和重新 placement 仍由 Controller 产生新的
fragment，不改变 Host fragment 作为不可变本地部署描述的基本语义。
