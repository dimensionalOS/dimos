# 分布式 Host 架构

语言：[English](/experimental/docs/hosted/architecture.md) | 简体中文

状态：设计提案草案

范围：具备发现与约束式放置能力的 MVP

## 摘要

目前，一个 DimOS `Blueprint` 由单个 `ModuleCoordinator` 构建，其中的所有模块
都部署在同一台机器上。一些传输层，尤其是 Zenoh，已经可以在机器之间传递
数据流，但蓝图运行时还没有模块放置和多机协同生命周期的概念。

本提案增加一层轻量的分布式编排能力：

- 每个执行设备运行一个小型、常驻的 **Host 服务**。
- Host 加入 Zenoh fabric，并广播自动生成的身份、可发现名称、标签、能力、
  资源、兼容性和当前可用状态。
- 蓝图可以为模块或组合片段添加精确 Host 或标签约束。没有放置约束的模块
  默认继续在本机运行。
- `dimos run` 加入同一个 Zenoh fabric，发现可用 Host，解析放置关系，切分
  蓝图，并请求选中的 Host 启动各自的本地片段。
- 用户通过 router 连接或局域网发现加入 Zenoh fabric。
- 每个 Host 继续复用现有的 `ModuleCoordinator` 和 worker 实现进行本地部署。
- 跨 Host 的数据流直接通过 Zenoh 通信，控制器和 Host 服务不代理应用数据。

MVP 包含发现和简单的约束式放置，但有意不包含代码分发、透明的跨 Host 模块
引用、故障转移、迁移、副本或复杂的资源优化。

## 动机

蓝图描述的是一个逻辑应用，但目前它也隐式定义了一个部署单元：一台机器上的
一个协调器。这让以下应用很难自然表达：

- 在机器人板载计算机上运行驱动和控制模块；
- 在 GPU 服务器上运行感知模块；
- 在开发者工作站上运行 agent、导航栈或 UI；
- 在另一台机器上运行共享服务；
- 根据机器人形态选择一台可用机器人，例如 G1；
- 通过一个 Zenoh router 连接整个机群，router 后方有多个独立 Host 分别广播
  机器人和计算资源。

Zenoh 已经为这些模块提供了跨机器数据路径和共享发现网络。缺少的是一个能够
发现执行容量、绑定放置约束，并跨选定机器启动、观测和停止同一逻辑应用的
控制平面。

## 目标

第一版应当做到：

1. 每个 Host 在 Zenoh 上广播可由控制器发现的 descriptor。
2. 蓝图可以通过唯一的 Host 名称/ID 或一组 Host 标签约束模块或片段。
3. 如果没有在线且兼容的 Host 满足约束，在部署前失败并给出有用诊断。
4. 控制器加入相应 Zenoh fabric 后，通过一条 `dimos run` 命令启动完整应用。
5. 在每台机器上复用当前的蓝图编译器、`ModuleCoordinator`、worker、模块
   生命周期和数据流实现。
6. 使用 Zenoh 承载跨机器数据流。
7. 在模块启动前校验不受支持的分布式图。
8. 向用户提供一个 run ID、不可变的放置记录，以及聚合的 Host 和模块状态。
9. 保留现有单机蓝图的行为与 API。
10. 为发现机器人形态及其他类型化硬件能力打下基础，但不把机群副本扩展纳入
    MVP。

## MVP 非目标

以下能力有意不包含在 MVP 中：

- 最优装箱、成本感知调度、抢占或自动扩缩容；
- 请求多个副本或在所有匹配选择器的 Host 上运行；
- 向 Host 发送源代码、环境、模型或其他制品；
- 在不同 Host 上运行应用的不同版本；
- 跨 Host 模块引用或透明的分布式对象调用；
- 使用 LCM 作为分布式传输；
- 自动故障转移、重新调度或模块迁移；
- 不受信任或公网环境中的部署；
- WebTransport/WebRTC 连接或 NAT 穿透；
- 应用启动后因为 Host 加入或离开而改变放置；
- 将一个蓝图片段自动扩展到整个机器人机群。

以后可以增加这些能力，而无需改变 Host 发现、Host 控制平面和 Zenoh 应用
数据平面之间的基本分层。

## 当前架构

当前本地启动流程大致如下：

1. `dimos run` 加载一个 `Blueprint`。
2. 蓝图解析模块、配置、数据流连接和模块引用。
3. 一个 `ModuleCoordinator` 通过本地 worker manager 部署模块。
4. 协调器连接数据流和模块 RPC proxy，然后构建并启动模块。
5. 一个本地运行注册项负责进程及其日志。

重要的现有组件包括：

- [`core/coordination/blueprints.py`](/dimos/core/coordination/blueprints.py)，
  表示并组合逻辑模块图；
- [`core/coordination/module_coordinator.py`](/dimos/core/coordination/module_coordinator.py)，
  执行本地部署、连接和生命周期管理；
- [`core/coordination/worker_manager_python.py`](/dimos/core/coordination/worker_manager_python.py)，
  把 Python 模块部署到本地 worker 进程；
- [`core/coordination/coordinator_rpc.py`](/dimos/core/coordination/coordinator_rpc.py)，
  通过选定的 RPC 后端公开运行中的协调器；
- [`porcelain/remote_module_source.py`](/dimos/porcelain/remote_module_source.py)，
  已经展示了如何控制独立运行的协调器；
- [`core/transport_factory.py`](/dimos/core/transport_factory.py)，负责选择 LCM
  或 Zenoh 传输和 RPC 后端；
- [`protocol/service/zenohservice.py`](/dimos/protocol/service/zenohservice.py)，
  已经支持显式 Zenoh 连接端点和组播 scouting。

本设计保留本地运行时。Host 不是一种新的模块运行时，而是一个可被发现、
能创建并控制现有本地运行时的 supervisor。

## 术语

| 术语 | 含义 |
| --- | --- |
| 应用（Application） | 一个可能跨越多个 Host 的逻辑蓝图图。 |
| 控制器（Controller） | 负责编译、放置和协调应用的 `dimos run` 进程。 |
| Host | 一个对外广播的执行服务。通常每台机器一个，但同一个 router 后方或同一台机器上也可以有多个。 |
| Host ID | 稳定、不透明且自动生成的身份，用于协议 key 和运行记录。 |
| Host 名称 | 可发现、便于人阅读的标签，默认取机器 hostname，也可以覆盖。它不是网络地址。 |
| Host 标签 | 用户或能力 provider 提供的放置标签，例如 `gpu`、`g1` 或 `warehouse-a`。 |
| Host descriptor | Host 的身份、标签、能力、资源、兼容性、负载和可用状态。 |
| 放置约束 | 附加在蓝图元数据上的精确 Host 选择器和/或必需标签集合。 |
| 放置单元 | 必须共同部署在同一 Host 上的一组模块。 |
| Host 片段 | 一次运行中分配给某个 Host 的模块和本地连接。 |
| 本地协调器 | 用于运行一个 Host 片段的现有 `ModuleCoordinator`。 |
| 边界数据流 | publisher 和 subscriber 位于不同 Host 的数据流。 |
| 模块引用 | 由协调器以 RPC proxy 连接的类型化 `Spec`/模块依赖。 |

网络位置与执行身份相互分离。`tcp/some_host:7447` 这样的 locator 决定控制器
如何加入 Zenoh fabric；放置则根据该 fabric 中广播的 Host ID、名称、标签和
能力进行解析。

## 提议架构

系统包含三个彼此相关但相互分离的部分。

### 发现平面

Host 在 Zenoh 上广播实时执行容量。控制器首先加入 fabric，观察在线 Host
身份，并查询其 descriptor。一个 Zenoh router 旁边可以有一个 Host，后面可以
有五个 Host 或五台机器人，也可以完全没有 Host；路由拓扑不定义放置拓扑。

发现平面回答“什么地方可以运行这个应用？”。它不会启动任何内容，也不会因为
某个 Host 能广播自己就自动信任它。

### 控制平面

解析放置关系后，控制器与每个选中的 Host 服务通信，以完成：

- 校验身份、兼容性、可用性和已广播的 descriptor；
- 通过有时限的 prepare lease 预留放置；
- 提交 Host 片段和解析后的配置；
- prepare、启动、停止和检查本地部署；
- 报告启动失败和模块状态。

控制流量很小。初版应尽可能复用现有 RPC 抽象和 Zenoh 后端。RPC 身份按 Host
ID 和 run ID 划分作用域，使多个协调器可以共享同一个 Zenoh fabric。

### 数据平面

模块继续通过 DimOS transport 发布和订阅。边界数据流直接通过 Zenoh 在
publisher Host 与 subscriber Host 之间传输，控制器和 Host 服务都不接收、
中继或转换应用消息。

因此，即使控制器在启动后退出，应用仍然可以继续传输数据。不过 MVP 可以让
控制器保持运行，以聚合状态和协调停止操作。

### Host 服务

Host 服务是一个轻量、常驻的 supervisor。它为每个接受的部署创建隔离的本地
运行进程，该进程再创建现有的 `ModuleCoordinator`。隔离可以防止长期运行的
Host 服务与应用共享可变的全局配置、模块状态和日志。

初版可以只广播一个部署 slot，并限制每个 Host 同时只有一个活动的分布式部署。
descriptor 和协议仍应包含容量、`run_id` 以及 lease/generation 字段，以便以后
支持多运行 Host，而无需改变放置语义。

## Host 身份、发现与广播

### 启动 Host

Host 使用自动生成的身份和用户提供的标签启动：

```sh skip
ssh some_host 'dimos host serve --tag gpu --tag something'
```

首次启动时，服务生成并持久化一个不透明的 `host_id`。默认显示名称取机器的
hostname。需要稳定人类可读名称时，只需覆盖显示名称：

```sh skip
dimos host serve --name g1-01 --tag g1 --tag lab-a
```

Host 在 descriptor 中广播名称和标签。

名称用于方便阅读，而不是协议身份。两个在线 Host 可能意外广播相同名称；此时
基于精确名称的放置应因歧义而失败，可以使用不透明 Host ID 消除歧义。协议 key、
lease 和持久化的运行分配始终使用 `host_id`。

### 加入 fabric

控制器只需要一种加入包含目标 Host 的 Zenoh fabric 的方式。它可以连接一个
已知 peer/router：

```sh skip
dimos --zenoh-connect=tcp/some_host:7447 run app
```

也可以使用本地组播发现：

```sh skip
dimos --zenoh-autodiscovery run app
```

`--zenoh-autodiscovery` 在本地网络启用 Zenoh 组播 scouting。分布式放置使用
Zenoh 作为 transport backend。

每个远程 Host 必须通过本地 router、显式 connect 配置或组播 scouting 独立
加入同一个 fabric。控制器连接到 router 并不能为一个尚未联网的 Host 配置连接。

### 广播协议

发现协议使用 Zenoh liveliness 表示在线状态，并通过 queryable 返回当前
descriptor。概念上的 key 如下：

```text skip
dimos/hosts/<host-id>/live
dimos/hosts/<host-id>/describe
dimos/hosts/<host-id>/control/<operation>
```

具体 key expression 属于实现细节，但必须满足：

- Host 的 Zenoh liveliness token 丢失后，其在线状态随之消失；
- descriptor 从在线 Host 获取，而不是信任可能过期的 retained value；
- 发现能力可以穿过 router 和 gossip，而不只依赖本地组播；
- 控制操作以不透明 Host ID 寻址；
- 应用数据流使用独立的 run-scoped namespace；
- Host 每次重启都有可观察的 incarnation/epoch，避免把旧 lease 误认为当前 lease。

以后可以用 Zenoh storage 保留资产清单或历史资源数据，但持久化的 key/value
记录不能作为是否可调度的事实来源。在线 Host 的 query 响应和成功取得的
prepare lease 才是权威信息。

最小 Host descriptor 包括：

| 字段 | 用途 |
| --- | --- |
| `host_id`、`name`、`epoch` | 稳定身份、显示名称和当前进程实例。 |
| `tags` | 用户或能力 provider 配置的不透明放置标签。 |
| `capabilities` | 结构化硬件信息，例如机器人型号/序列号、GPU 类型、架构和连接设备。 |
| `resources_total`、`resources_available` | CPU、内存、加速器、部署 slot 及其他可调度容量。 |
| `protocol_version`、`plan_schema_version` | 控制协议和部署计划的兼容性。 |
| `dimos_version`、`application_revision` | 运行时和代码兼容性。 |
| `active_runs`、`leases` | 当前占用和待提交的预留。 |
| `health`、`last_error` | Host 是否可接收任务，以及不可用的原因。 |

MVP 中的标签保持简单。结构化 capabilities 可以避免将机器人序列号、GPU 内存
等信息压扁成一堆未校验标签。初期可以通过 `--tag g1` 实现机器人形态选择；
之后 Unitree capability provider 可以自动广播同一事实，并携带型号、序列号、
固件和安全状态。

descriptor 只是调度提示，可能在发现后立即变化。`prepare` 操作必须在获取有
时限的 lease 时重新校验兼容性、标签、设备和空闲资源。

## Blueprint 放置 API

应用仍然是普通 `Blueprint`。一个或多个 blueprint atom 或组合片段上的放置
元数据会启用分布式行为。

放置 API 使用 `placement()` modifier：

```python skip
unitree_go2_markers = autoconnect(
    unitree_go2,
    MarkerDetectionStreamModule.blueprint(
        marker_length_m=0.1,
        camera_info=GO2Connection.camera_info_static,
    ).placement(tags={"gpu"}),
)
```

这会让 `unitree_go2` 默认在控制机器上运行，并把 marker detection 放到一个已
发现、兼容且带有 `gpu` 标签的 Host 上。

按机器人形态或精确机器人名称放置时使用相同机制：

```python skip
multi_host_g1 = autoconnect(
    unitree_g1.blueprint().placement(tags={"g1"}),
    navigation.blueprint(),
    expensive.blueprint().placement(tags={"gpu"}),
)

pinned_g1 = autoconnect(
    unitree_g1.blueprint().placement(host="g1-01"),
    navigation.blueprint(),
    expensive.blueprint().placement(tags={"gpu"}),
)
```

第一个应用选择一个可用的 G1 Host，在本机运行 navigation，并选择一个可用的
GPU Host。第二个应用只把机器人片段固定到唯一广播的名称或 Host ID `g1-01`。

放置是通过 `placement()` 暴露的不可变蓝图元数据，不会传递给模块构造函数。
同一个 modifier 可以应用于单个模块或组合片段。

API 语义如下：

- `.placement(host=...)` 是精确选择器，匹配唯一广播的 Host 名称或不透明
  Host ID。
- `.placement(tags={...})` 要求 Host 包含列出的全部标签。
- 同时提供两者时，精确 Host 也必须满足所有标签。
- 对组合片段调用 `.placement(...)` 会使其成为一个放置单元，其中所有模块
  共置在同一 Host。
- 没有放置元数据的模块软偏好控制机器，从而保留当前行为。
- 必要时可通过 `.placement(local=True)` 表达硬性的本地约束。
- 一个模块实例只分配给一个 Host。副本或“所有匹配 Host”需要未来显式的
  cardinality API。
- 放置关系在启动时解析一次并以 Host ID 记录。之后 Host 的加入、改名或负载
  变化不会静默移动应用。

控制器自身的运行时在内部表示为一个合成的、run-scoped 本地 Host 身份。它参与
放置和聚合状态，但不会向其他控制器广播，也不能通过远程 Host 名称选择。

如果用户需要一个模块的两个副本，仍须像本地蓝图一样创建两个具名或 namespace
隔离的实例。放置不会复制 blueprint atom。

## 放置解析

放置针对在线 Host descriptor 求解约束。MVP 算法保持简单、可解释且确定：

1. 解析完整逻辑图及全部放置元数据。
2. 形成放置单元。显式要求共同放置的片段构成一个单元；通过仅支持本地的模块
   引用连接的模块也合并为一个单元，因为这些引用不能跨 Host。
3. 把控制机器作为无约束单元的优先候选。
4. 获取一次在线 Host descriptor 的发现快照。
5. 针对每个单元，按精确 Host 选择器、必需标签、平台与部署计划兼容性、必需
   代码 revision、已广播设备、可用资源和部署 slot 过滤候选。
6. 对每个共置单元取约束交集。互相冲突的精确 Host、显式本地约束与远程约束
   并存，或候选集合为空，均属于编译/放置错误。
7. 优先分配精确选择器。对于仅使用标签的选择器，优先选择已承诺负载最低的
   兼容 Host，再以稳定 Host ID 作为确定性的 tie-breaker。以后可以增加更复杂
   的评分，而无需改变选择器模型。
8. 打印或公开最终放置计划，然后获取 prepare lease。
9. 在运行记录中以 Host ID 固化成功的分配。

无约束模块保持本地运行，除非它因为模块引用与一个显式远程放置模块合并为
共置单元而被迫一起远程运行。放置解释应明确展示这种继承关系。

当选择器无法满足时，控制器必须清楚地失败。有效诊断应列出模块或片段、约束，
以及每个已发现 Host 被排除的原因，例如：

```text skip
没有 Host 满足 MarkerDetectionStreamModule 的放置要求
  必需标签：[gpu]
  已发现：
    g1-01 (4ec2...)：缺少标签 gpu
    gpu-lab (8aa1...)：忙，0/1 个部署 slot 可用
```

精确选择器不存在、重复、版本不兼容或繁忙时也应报错。默认情况下，控制器不会
无限等待匹配 Host；自动化场景可以增加显式 placement timeout。

发现结果会与其他控制器发生竞争，因此选择本身不代表资源已被占用。只有在
`prepare` 重新校验 descriptor 并创建 lease 后，Host 才接受任务。如果竞争
lease 失败，MVP 应报告放置状态已变化；以后可以增加有界重新规划，但必须对
用户可见，不能静默地在意外机器人上启动。

## 图编译与切分

控制器必须先解析完整逻辑图，再请求任何 Host 启动。随后，它为每个选中的 Host
编译一份不可变部署计划。

编译包含以下阶段：

1. 加载蓝图并应用应用级配置。
2. 在完整图上解析模块实例名、数据流自动连接、remapping、namespace、模块引用
   要求和放置约束。
3. 发现 Host，并把每个放置单元解析到唯一 Host ID。
4. 将每个连接分类为本地或跨 Host。
5. 根据下述规则校验 transport 和模块引用。
6. 创建 Host 片段，其中包含本地模块、本地连接、边界数据流端点、解析后的配置
   快照，以及资源/lease 要求。

Host 不应各自重新编译原始应用。环境变量、已安装 package、发现时序或机器本地
配置都可能导致图不一致。Host 接收解析完毕的计划，只进行本地校验和部署。

部署计划需要可序列化且有版本的表示。原型阶段发送任意存活 Python 对象虽然
方便，但 plan schema 更安全，因为它支持兼容性检查、诊断以及未来的非 Python
Host。

## 数据流传输规则

MVP 中的分布式运行要求使用 Zenoh 作为共享 transport backend。LCM 继续支持
现有单机运行，但当蓝图解析到多个 Host 时必须拒绝 LCM。

| 连接 | MVP 规则 |
| --- | --- |
| 使用 Zenoh 的本地数据流 | 支持。 |
| 使用 Zenoh 的跨 Host 数据流 | 支持。 |
| 多 Host 运行中的任意 LCM 数据流 | 拒绝。 |
| 使用共享内存的本地数据流 | 显式配置时支持。 |
| 使用共享内存的跨 Host 数据流 | 拒绝。 |
| 其他仅限单机的 transport | 仅当两个端点位于同一 Host 时支持。 |

为本地图像或点云路径保留显式共享内存可以避免性能倒退。但如果显式固定的单机
transport 跨越 Host 边界，编译器必须在部署前失败。

### Topic 身份与隔离

边界数据流两端必须独立推导出相同的 Zenoh key expression，同时还要与同一应用
更早或并行的运行隔离。因此，物理 topic 应包含 run-scoped 前缀，概念上为
`dimos/runs/<run-id>/streams/<logical-stream>/<message-type>`。

具体格式属于实现细节，但必须满足：

- 同一次运行中，相同逻辑数据流名和消息类型在所有 Host 上解析为相同 key；
- 不同 run ID 绝不共享物理 key；
- 消息类型不一致时在图校验阶段失败；
- topic 构造集中实现，不由每个 Host 重复实现；
- 发现与控制 key 使用独立于应用数据流的 namespace。

run scoping 是对当前 topic factory 的必要修改；当前 Zenoh 使用全局
`dimos/<name>` namespace。

## 模块引用规则

模块引用是由协调器解析并注入 RPC proxy 的类型化模块或 `Spec` 依赖。跨 Host
引用会对身份、可用性、timeout 行为和生命周期顺序提出额外要求。

因此 MVP 使用一条简单规则：

> 模块及其消费的所有模块引用必须属于同一个放置单元，并解析到同一个 Host。

图编译器在放置前合并通过引用连接的模块。如果一个模块有远程放置约束，另一个
没有约束，则二者一起远程放置；如果二者约束不兼容，则编译失败并报告 consumer、
provider、引用字段和冲突的选择器。

本地模块引用继续使用现有协调器行为，模块 API 不变。跨 Host 引用不在 MVP
范围内。

## Host 控制协议

协议应保持精简并带版本。最小逻辑 API 为：

| 操作 | 用途 |
| --- | --- |
| `describe` | 返回当前 Host descriptor。 |
| `prepare` | 重新校验约束、获取有时限的 lease，并暂存解析后的片段。 |
| `start` | 构建并启动已暂存的本地部署。 |
| `status` | 返回部署状态、lease 状态和模块摘要。 |
| `stop` | 优雅停止一次运行，并按现有本地策略升级终止方式。 |
| `logs` | 返回某次运行的日志元数据或有界数据流。 |

每个修改状态的请求至少包含：

- 由控制器一次生成的 `run_id`；
- 目标 `host_id` 以及控制器观察到的 Host `epoch`；
- 协议版本和 plan schema 版本；
- 用于幂等的 request ID 或 generation number；
- Host 必须重新校验的放置约束和资源；
- 已暂存但未提交工作的 lease deadline。

对于相同 generation 的重复 `prepare`、`start` 或 `stop` 请求，应返回当前结果，
而不是启动重复部署。Host 没有空闲部署 slot 时必须拒绝冲突运行。过期的
prepare lease 必须清理暂存状态，但不能终止已提交运行。

Host 服务使用按不透明 ID 划分的名称，例如 `Host/<host-id>`。在共享 Zenoh
fabric 上可见的本地模块 RPC 名称按 run 和 Host 划分作用域。

## 启动顺序

分布式启动应按以下顺序进行：

1. **编译：** 控制器解析完整逻辑应用和共置单元。
2. **加入和发现：** 加入配置的 Zenoh fabric，收集在线 Host descriptor，并
   加入自身的本地容量。
3. **放置：** 解析精确名称/ID 和标签，校验约束，并生成可解释的放置计划。
4. **Prepare 与 lease：** 控制器向每个选中 Host 发送解析后的片段。每个 Host
   在获取有时限的 prepare lease 时，重新校验身份、版本、代码、标签、设备、
   资源、可用性、import、配置和 transport 能力。
5. **提交：** 只有所有 Host 都 prepare 成功后，控制器才请求所有 Host 及本地
   协调器启动。
6. **观测：** 控制器等待所有本地协调器报告已启动，然后以解析后的 Host ID
   记录应用处于运行状态。
7. **失败回滚：** 如果任一 Host 在应用进入运行状态前失败，控制器停止所有已
   prepare 或已成功启动的 Host。

这是轻量级的两阶段启动，不是持久化分布式事务。回滚是 best effort，状态中
必须明确报告任何无法访问的 Host。

远程 subscriber 就绪前，模块可能已经开始发布。边界数据流必须遵循 Zenoh
配置的 delivery semantics；启动流程不能暗示 transport 并未提供的缓冲或回放。

## 生命周期与失败语义

分布式运行具有由所有 Host 状态聚合得到的状态：

| 状态 | 含义 |
| --- | --- |
| `placing` | 正在发现 Host 或解析约束。 |
| `preparing` | 至少一个 Host 正在获取 lease、校验或暂存片段。 |
| `starting` | 所有 Host 已 prepare，且至少一个正在启动。 |
| `running` | 所有必需 Host 都报告其本地部署正在运行。 |
| `degraded` | 应用已经启动，但后来丢失了某个 Host 或必需模块。 |
| `stopping` | 正在协调停止。 |
| `stopped` | 所有可访问 Host 都已停止该运行。 |
| `failed` | 放置或启动失败，或应用无法满足某个必需约束。 |

MVP 的失败行为有意保持保守：

- 不会自动把模块移动到其他 Host；
- Host 在放置期间消失后即不再是候选；
- 提交后丢失选中的 Host 会将应用标记为 `degraded`；
- Host 服务通过现有协调器状态报告本地模块失败；
- 停止应用时联系运行记录中的 Host ID，而不是当前恰好具有相同名称或标签的
  Host；
- 停止操作应报告无法访问的 Host，不能声称已干净停止；
- Host 重启后不会自动重建上次部署，除非以后加入明确的恢复设计；
- Host 在将相应容量重新广播为可用之前，必须清理部分启动的本地协调器和过期
  prepare lease。

控制器和 Host 应使用 liveliness 加 heartbeat 或 lease 来检测控制连接丢失。
在第一个 MVP 中，控制器 lease 过期只能作为状态信号；如果没有明确的机器人
安全策略，它不能终止仍然健康的机器人控制模块。

## 配置、代码与制品

第一版假设每个选中的 Host 已经具备：

- 相同且兼容的 DimOS 和应用代码；
- 所有必需的 Python 和 native 依赖；
- 必需的模型、校准文件及其他资产；
- 访问机器本地设备和凭据的权限。

控制器发送解析后的应用配置；设备路径或 secret 等 Host 本地设置可来自 Host
配置。应用配置和 Host 本地配置之间的优先级为：

1. 控制器提供的不可变放置和运行元数据；
2. 控制器解析的应用配置；
3. allowlist 中用于设备特定值和 secret 的 Host 本地覆盖。

preflight 应比较 release identifier 或 source revision，并在默认情况下拒绝不兼容
版本。自动分发代码和制品属于后续部署系统，不属于 Host 编排。

## 多机器人形态方向

标签提供了一个实用的第一步：

```python skip
unitree_g1.blueprint().placement(tags={"g1"})
```

它表示“将这个放置单元绑定到一个在线、兼容、可用且广播 `g1` 的 Host”，而不是
“在所有 G1 上运行”。选中的机器人必须以不透明 Host ID 记录；如果有机器人
序列号，也应同时记录。prepare lease 使这个物理机器人在该次运行中保持独占。

长期 capability model 应区分：

- **身份（identity）：** 某台具体机器人，例如 `g1-01` 或序列号；
- **类型（kind）：** 广义的机器人形态兼容性，例如 G1 或 Go2；
- **能力（capabilities）：** 手臂、夹爪、摄像头、运动方式、载荷、固件和安全
  状态；
- **资源（resources）：** CPU、GPU、内存、电量和部署 slot；
- **策略（policy）：** 控制器是否有权占用和控制该机器人。

同一种放置机制由此可以选择机器人、板载控制器、附近的边缘 GPU 或云端计算。
机群 fan-out、选择指定数量的机器人、anti-affinity 和多机器人协同安全仍属于
后续明确阶段。

## 可观测性

一次分布式运行只有一个应用 `run_id`。每条日志和状态记录还包含 `host_id`、
放置时发现的 Host 名称，以及适用时的 `module_name`。

聚合状态至少应展示：

- 应用名称、run ID 和聚合状态；
- 本次运行使用的 Zenoh 发现/连接模式；
- 每个选中 Host 的 ID、名称、匹配标签、版本、heartbeat 和状态；
- 每个放置单元选择该 Host 的原因；
- 每个 Host 上部署的模块及其生命周期状态；
- 每个 Host 的首个启动或运行时错误；
- 无法访问 Host 上尚未完成的清理工作。

有用的命令包括：

```sh skip
dimos host list
dimos run app --explain-placement
dimos status --run <run-id>
dimos log --run <run-id> --host gpu-lab
dimos stop --run <run-id>
```

`dimos host list` 查询实时 fabric。即使显示名称之后发生变化，status、logs
和 stop 仍使用运行记录中不可变的 Host ID 分配。

MVP 中日志可以保留在各个 Host 上，按需获取或 tail。无需中央日志存储，但用户
不应为了查找模块日志而了解 Host 的文件系统布局。

## 安全与网络假设

MVP 面向受控 Zenoh 部署下的可信机器人或开发网络。发现一个 Host 并不能证明
其身份；不能把本设计描述为适用于任意公网或不受信任的共享 router。

在用于更广泛部署前，控制平面需要经过认证的 Host 身份、控制器授权、加密传输、
防重放、lease ownership、secret 处理和审计日志。这些要求应隔离在 Host
client/service 协议之后，不影响模块 API。对具体物理机器人的精确选择最终应在
人类可读名称之外使用加密身份。

## 兼容性与破坏性变更

本设计在面向用户的模块和蓝图层面是增量式的：

- 现有本地 `Blueprint` 和 `dimos run <blueprint>` 保持当前行为。
- `Module`、`In`、`Out`、`@rpc` 和本地模块引用 API 不变。
- 每台机器继续使用现有本地协调器和 worker 实现作为运行时。
- 没有远程放置元数据的蓝图保持本地运行。
- 带有远程放置元数据的蓝图仍然是 `Blueprint`。
- 网络 locator 定义 Zenoh fabric membership，Host descriptor 定义放置候选。

必需的内部变更包括：

1. 为 `BlueprintAtom`/组合 `Blueprint` 增加不可变放置元数据，且不将其传给
   模块构造函数。
2. 增加共置分组、Host 发现、约束解析和图 partitioner。
3. 增加 Host service/client、持久化 Host 身份、descriptor、liveliness、lease
   和 Host CLI 命令。
4. 按 Host ID 和 run ID 划分 coordinator/control RPC 身份。
5. 增加 run-scoped Zenoh topic 和 RPC namespace。
6. 扩展运行注册表、status、log 和 stop 操作，记录不可变 Host ID 分配及放置
   解释。
7. 定义可序列化、有版本的 Host deployment plan 和 descriptor schema。

只有在一次运行解析到多个 Host 时，才会出现以下主要行为限制：

- 必须使用 Zenoh；
- 拒绝 LCM；
- 共享内存及其他本地 transport 不能跨 Host；
- 模块引用会形成共置约束，且不能跨 Host；
- 代码和依赖必须预先存在于每个选中的 Host。
