# 从零理解 DimOS：给小白看的项目教材

## 读者说明

这份文档写给第一次接触 DimOS 的读者。你不需要一开始就懂机器人、ROS、LCM、MCP、Agent 或多进程架构。你只需要知道一点 Python，愿意按照“先看整体，再看局部，最后串起来”的方式理解一个大型机器人项目。

这份教材重点回答四个问题：

1. 每个部分是干什么的？
2. 各个部分之间如何联系在一起？
3. 为什么要这样设计？
4. 如果从一条命令开始，DimOS 到底经历了哪些步骤？

你可以把 DimOS 想成一个“面向机器人和智能体的操作系统框架”。它不是只控制一个电机，也不是只跑一个大模型，而是把机器人系统里的感知、导航、控制、智能体、工具调用、可视化、配置、运行管理等部分组织成一套可组合的系统。

一句话记住 DimOS：

```text
Modules 做事情，Streams 传消息，Transports 负责底层通信，
Blueprints 组装系统，Skills 暴露能力，Agent 理解任务并调用能力。
```

## 第一章：先看总图，DimOS 到底是什么

### 1.1 DimOS 解决什么问题

机器人系统通常不是一个简单脚本。一个真实机器人可能同时需要：

- 从相机读取图像
- 从雷达读取点云
- 构建地图
- 识别物体
- 规划路线
- 控制底盘或机械臂
- 把状态显示出来
- 接收用户命令
- 让大模型决定下一步要做什么
- 在真实硬件、仿真和回放数据之间切换

如果所有逻辑都写在一个文件里，很快会变得无法维护。因为每个部分的频率、数据量、错误模式和依赖都不一样。相机图像可能很大，导航需要实时更新，Agent 可能几秒才思考一次，硬件控制又必须非常谨慎。

DimOS 的设计思路是：把复杂机器人拆成多个独立模块，让模块之间通过标准消息通信，再用 Blueprint 把这些模块组装成一个完整系统。

### 1.2 生活类比：机器人公司

可以把一个机器人系统想成一家公司：

- 相机模块像“眼睛部门”，负责看世界。
- 感知模块像“识别部门”，负责判断图像里有什么。
- 导航模块像“路线规划部门”，负责决定怎么走。
- 控制模块像“执行部门”，负责让机器人真的动起来。
- Agent 像“调度员”，负责理解用户目标并安排任务。
- Skill 像“标准工作单”，规定调度员能让执行部门做哪些事。
- Blueprint 像“组织架构图”，说明这家公司有哪些部门、谁和谁对接。
- CLI 像“前台入口”，用户通过它启动、停止和查看系统。

一个健康的公司不会让每个部门随便翻别人的资料，也不会让调度员直接拧电机螺丝。DimOS 也是这样：模块之间通过标准接口沟通，Agent 通过受控的 Skill 调用能力。

### 1.3 DimOS 的全局链路

从用户命令到机器人执行，大致可以理解为：

```text
用户输入
  ↓
CLI 或 Web API
  ↓
Blueprint 选择并组装系统
  ↓
多个 Module 启动
  ↓
Streams / Transports 连接数据流
  ↓
感知、导航、控制、可视化等模块并行运行
  ↓
Agent 通过 MCP 发现 Skills
  ↓
Agent 调用 Skill
  ↓
Skill 调用底层模块 RPC 或发布控制消息
  ↓
机器人、仿真器或回放系统执行
```

### 1.4 最小心智地图

先记住下面这张表，后面所有章节都会围绕它展开。

| 部分 | 它干什么 | 它和谁联系 | 为什么需要它 |
| --- | --- | --- | --- |
| CLI | 启动、停止、查看系统 | Blueprint、GlobalConfig、Run Registry | 给用户统一入口 |
| GlobalConfig | 管理运行参数 | CLI、Blueprint、Module | 同一套代码适配真机、仿真、回放 |
| Blueprint | 组装模块 | Module、Stream、Worker | 让机器人系统可组合 |
| Module | 封装一个功能单元 | Stream、RPC、Worker | 让复杂系统拆成小块 |
| Stream | 模块之间传数据 | In、Out、Transport | 避免模块强耦合 |
| Transport | 底层通信方式 | LCM、SHM、ROS、DDS | 适配不同数据和运行环境 |
| Worker | 运行模块进程 | Module Coordinator | 让模块并行运行 |
| Skill | 暴露机器人能力 | Agent、MCP、RPC | 让大模型安全调用能力 |
| Agent | 理解任务并决策 | Skill、Stream、Prompt | 把自然语言变成行动 |
| Perception | 识别世界 | Sensor、Navigation、Agent | 让机器人知道看到了什么 |
| Navigation | 地图和路径规划 | Perception、Robot Control | 让机器人知道怎么走 |
| Visualization | 显示状态 | Stream、Rerun、Web | 调试机器人必须看得见 |

## 第二章：从一条命令理解系统启动

### 2.1 典型命令

DimOS 常见启动方式如下：

```bash
dimos --replay run unitree-go2-agentic
```

这条命令可以拆成三部分：

- `dimos`：进入 DimOS CLI。
- `--replay`：使用回放数据，而不是真实机器人实时数据。
- `run unitree-go2-agentic`：运行名为 `unitree-go2-agentic` 的 Blueprint。

### 2.2 这条命令背后发生了什么

第一步，CLI 解析命令行参数。它会把 `--replay` 这类参数写入全局配置。

第二步，CLI 根据名字找到对应 Blueprint。Blueprint 是一个系统组合方案，里面声明了要启动哪些模块。

第三步，Blueprint 被 build。这个阶段会创建模块、连接数据流、准备 RPC 注入，并把模块分配给 Worker。

第四步，各个 Module 启动。比如机器人连接模块、感知模块、导航模块、Agent 模块、MCP 模块等。

第五步，模块之间开始通过 Stream 传消息。图像、点云、地图、位姿、目标、命令都可以作为流在模块之间传递。

第六步，如果这是 agentic Blueprint，MCP Server 会暴露 Skills，McpClient 或 Agent 会读取这些工具，让大模型知道自己可以调用哪些能力。

第七步，系统进入运行状态。此时可以通过 CLI、Web API、MCP 或 Agent 继续交互。

### 2.3 为什么不是直接运行一个 Python 文件

对小项目来说，一个 Python 文件确实更简单。但机器人系统有几个特点：

- 很多任务要并行运行。
- 不同模块可能需要不同依赖。
- 数据量可能很大，尤其是图像和点云。
- 硬件控制需要和上层逻辑隔离。
- 真机、仿真、回放需要共用大部分代码。
- 大模型调用能力必须有明确边界。

所以 DimOS 不是把所有逻辑堆在一起，而是建立一个可组合的运行体系。

## 第三章：Module，DimOS 的最小功能单元

### 3.1 Module 是什么

Module 是 DimOS 中最重要的概念之一。你可以把它理解为“一个独立运行的功能模块”。

一个 Module 可能负责：

- 读取相机
- 连接机器人
- 做目标检测
- 做路径规划
- 发布可视化数据
- 运行 Agent
- 暴露一组 Skills

Module 的核心思想是：每个模块只关心自己的职责，不直接把整个系统塞进自己内部。

### 3.2 Module 之间怎么联系

Module 主要通过两种方式联系：

第一种是 Stream，也就是持续的数据流。例如相机模块不断发布图像，感知模块订阅图像并输出检测结果。

第二种是 RPC，也就是一个模块调用另一个模块的方法。例如 Skill 模块收到 Agent 的工具调用后，可能通过 RPC 调用导航模块的 `set_goal()`。

可以这样理解：

```text
持续变化的数据 → 用 Stream
明确的一次性动作 → 用 RPC
```

### 3.3 为什么要用 Module

如果没有 Module，项目会遇到几个问题：

- 代码边界不清楚，谁都能调用谁。
- 难以单独替换一个算法。
- 很难把相机、导航、Agent、控制拆开测试。
- 系统并行运行会变复杂。
- 不同机器人复用代码会很困难。

Module 的价值在于让 DimOS 的每个能力都有清晰边界。

### 3.4 常见 Module 类型

在 DimOS 项目里，你会看到很多不同类型的模块：

- 机器人连接模块：负责和硬件通信。
- 传感器模块：负责读取图像、雷达、状态。
- 感知模块：负责检测、跟踪、理解环境。
- 导航模块：负责地图、路径和目标点。
- Agent 模块：负责语言理解和工具调用。
- Skill 容器：负责把机器人能力变成工具。
- 可视化模块：负责把状态显示出来。

## 第四章：Stream，模块之间的数据管道

### 4.1 Stream 是什么

Stream 是模块之间传递数据的管道。在 DimOS 中，模块通常不会直接拿到另一个模块的内部变量，而是通过 `In[T]` 和 `Out[T]` 通信。

简单理解：

- `Out[T]` 表示这个模块会发布某种类型的数据。
- `In[T]` 表示这个模块会订阅某种类型的数据。
- `T` 是数据类型，比如图像、位姿、地图、检测结果。

### 4.2 Stream 的例子

假设有三个模块：

```text
CameraModule
  Out[Image]
     ↓
ObjectDetectorModule
  In[Image]
  Out[Detections]
     ↓
AgentModule
  In[Detections]
```

相机模块不需要知道谁在使用图像。它只负责发布图像。检测模块只需要声明自己需要图像，Blueprint 会负责把它们接起来。

### 4.3 为什么要这样设计

如果相机模块直接调用检测模块，会有几个问题：

- 相机模块必须知道检测模块存在。
- 想换一个检测模块时要改相机代码。
- 想同时给两个模块使用图像会变麻烦。
- 模块之间会越来越耦合。

Stream 的好处是让生产数据的人和消费数据的人解耦。一个模块只要发布标准数据，其他模块就可以订阅。

### 4.4 Stream 适合什么场景

Stream 适合持续变化的数据，比如：

- 相机图像
- 雷达点云
- 机器人位姿
- 地图更新
- 目标检测结果
- 控制状态
- Agent 消息

只要数据是“不断产生、不断被消费”的，就很适合用 Stream。

## 第五章：Transport，消息真正怎么传过去

### 5.1 Transport 是什么

Stream 是概念层面的数据管道，Transport 是底层真正传输数据的方式。

可以类比成：

```text
Stream 是“我要寄快递”
Transport 是“用卡车、飞机、快递柜还是人工配送”
```

不同数据适合不同传输方式。小的结构化消息可以用一种方式，大图像和点云可能需要更高效的方式。

### 5.2 DimOS 中常见 Transport

DimOS 支持多种通信方式：

- LCM：默认通信方式，适合很多机器人消息。
- SHM：共享内存，适合图像、点云等大数据。
- pLCM：适合复杂 Python 对象。
- ROS Transport：用于和 ROS 系统交互。
- DDS Transport：用于 DDS 发布订阅系统。

### 5.3 为什么要抽象 Transport

如果系统只支持一种通信方式，会很难适配不同场景：

- 本机模块通信需要效率。
- 跨进程通信需要序列化。
- 大图像传输不能频繁复制。
- 和 ROS 或 DDS 系统交互需要兼容外部生态。

DimOS 把 Stream 和 Transport 分开，是为了让上层模块不用关心底层通信细节。

## 第六章：Blueprint，把模块组装成系统

### 6.1 Blueprint 是什么

Blueprint 是 DimOS 的“系统装配图”。它告诉 DimOS：这个机器人系统由哪些模块组成，这些模块之间应该如何连接。

如果 Module 是零件，那么 Blueprint 就是装配方案。

### 6.2 autoconnect 是什么

DimOS 中常见的组合方式是 `autoconnect()`。它会根据模块声明的输入输出名称和类型，自动把能匹配的 Stream 连起来。

这意味着你不需要手写大量连接代码。只要模块声明清楚自己发布什么、订阅什么，Blueprint 就能完成大部分连接。

### 6.3 Blueprint 为什么重要

同一个能力可以组合出不同机器人系统。

例如：

- Go2 真机系统
- Go2 回放系统
- Go2 agentic 系统
- G1 仿真系统
- xArm 感知操作系统
- Drone 视觉跟踪系统

它们可能共享感知、导航、Agent、可视化等模块，但组合方式不同。Blueprint 让这种组合变得明确。

### 6.4 小白如何读 Blueprint

读 Blueprint 时不要急着看每一行细节。先问三个问题：

1. 这个 Blueprint 最终要运行什么机器人或场景？
2. 它包含哪些 Module？
3. 它有没有加入 Agent、MCP、Skill、可视化这些上层能力？

理解 Blueprint 的关键不是背代码，而是看“系统由哪些能力拼起来”。

## 第七章：Worker 和运行时，为什么要多进程

### 7.1 为什么 Module 不都跑在一个地方

机器人系统往往有不同频率的任务：

- 控制循环可能需要很高频率。
- 图像处理计算量很大。
- Agent 推理可能比较慢。
- 可视化可能有自己的事件循环。
- 数据回放可能持续读取文件。

如果全部放在一个线程里，一个慢任务就可能拖住整个系统。

### 7.2 Worker 是什么

Worker 可以理解为运行 Module 的工作进程。Blueprint build 之后，Module Coordinator 会负责把模块部署到 Worker 中，并管理它们的生命周期。

### 7.3 为什么这样做

多进程运行有几个好处：

- 模块可以并行工作。
- 一个模块慢，不一定阻塞所有模块。
- 大型依赖可以隔离。
- 更接近真实机器人系统的运行方式。
- 便于部署、停止、重启和监控。

代价是系统复杂度更高，所以 DimOS 提供了 Module、Blueprint、Stream、Transport 这些抽象来管理复杂度。

## 第八章：GlobalConfig，一套代码适配不同环境

### 8.1 GlobalConfig 是什么

GlobalConfig 是 DimOS 的全局配置系统。它负责管理运行时参数，例如：

- 是否使用仿真
- 是否使用回放
- 机器人 IP
- viewer 类型
- worker 数量
- MCP 端口

### 8.2 配置从哪里来

配置可能来自：

- 默认值
- `.env` 文件
- 环境变量
- Blueprint 设置
- CLI 参数

CLI 参数通常优先级最高。例如你运行：

```bash
dimos --replay run unitree-go2-agentic
```

那么 `replay` 配置就会被打开。

### 8.3 为什么需要统一配置

没有统一配置时，同一个参数可能散落在很多地方。比如机器人 IP 可能在脚本里、模块里、环境变量里各写一份，最后很难知道哪个生效。

GlobalConfig 的作用是让系统在不同环境下保持一致：

- 本地开发
- 仿真
- 回放数据
- 真实机器人
- CI 测试
- 不同可视化后端

## 第九章：Robot 模块，DimOS 如何接入真实机器人

### 9.1 Robot 模块负责什么

Robot 模块负责把真实硬件或仿真机器人接入 DimOS。它通常处理：

- 和机器人 SDK 通信
- 读取机器人状态
- 发布传感器数据
- 接收控制指令
- 提供运动能力

这些模块是上层系统和真实世界之间的接口。

### 9.2 为什么硬件细节要封装起来

不同机器人差异很大：

- 四足机器人有步态和速度控制。
- 人形机器人有关节、姿态和平衡。
- 无人机有飞控、姿态、航点和安全边界。
- 机械臂有末端执行器、关节空间和轨迹规划。

如果 Agent 或导航模块直接依赖每种机器人 SDK，代码会很快失控。DimOS 会把硬件差异封装在机器人模块或技能容器里，让上层看到更稳定的能力接口。

### 9.3 Go2、G1、Drone、xArm 的位置

DimOS 中常见平台包括：

- Unitree Go2：四足机器人。
- Unitree G1：人形机器人。
- Drone：MAVLink、DJI 等无人机相关模块。
- xArm：机械臂感知、规划和控制。

它们不一定拥有完全相同的能力，所以对应的 Blueprint、Skill 和系统提示词也会不同。

### 9.4 为什么不同机器人需要不同 Prompt

Agent 依赖系统提示词理解自己能做什么。如果把 Go2 的 Prompt 用在 G1 上，Agent 可能以为自己拥有不存在的技能。这会导致工具调用错误，甚至在真实机器人上造成风险。

所以 DimOS 把机器人能力、Skill 和系统提示词关联起来。Agent 应该只知道当前系统真实暴露的能力。

## 第十章：Perception，机器人如何看见世界

### 10.1 Perception 是什么

Perception 是感知系统。它负责把传感器数据变成更有意义的信息。

传感器给出的通常是原始数据：

- 图像
- 点云
- 深度图
- 音频
- 位姿

感知模块要把这些数据变成机器人和 Agent 更容易使用的信息：

- 图像里有什么物体
- 物体在哪里
- 目标是否正在移动
- 当前空间有哪些障碍物
- 某个目标是否被看见

### 10.2 Perception 和其他模块的联系

典型链路如下：

```text
Camera / LiDAR
  ↓
Perception Module
  ↓
Detections / Tracks / Memory
  ↓
Navigation / Agent / Visualization
```

感知模块通常订阅传感器流，输出检测结果或空间信息。导航可以用这些结果避障，Agent 可以用这些结果回答问题或决定下一步行动。

### 10.3 为什么不能让 Agent 直接看所有原始数据

大模型可以理解图像，但它不适合直接承担所有实时感知工作。原因包括：

- 图像频率高，数据量大。
- 实时控制需要稳定低延迟。
- 检测、跟踪、投影等任务有专门算法。
- Agent 更适合做高层决策，而不是每帧图像都重新思考。

所以 DimOS 把实时感知交给 Perception 模块，把高层任务理解交给 Agent。

## 第十一章：Navigation，机器人如何行动

### 11.1 Navigation 是什么

Navigation 是导航系统。它关注机器人在空间中如何移动，包括：

- 地图表示
- 占据栅格
- 路径规划
- 路径平滑
- 目标点导航
- 障碍物处理
- 探索未知区域

### 11.2 Navigation 和 Perception 的关系

导航需要知道环境是什么样的。这个信息通常来自感知和传感器：

```text
LiDAR / Depth / Pose
  ↓
Mapping
  ↓
Occupancy Map
  ↓
Path Planning
  ↓
Robot Control
```

感知告诉系统哪里有障碍物，导航基于地图计算路径，控制模块让机器人沿路径移动。

### 11.3 为什么导航不能只靠一句自然语言

用户可以说“去桌子旁边”，但机器人真正执行时需要更具体的信息：

- 桌子在哪里？
- 当前机器人在哪里？
- 中间有没有障碍？
- 哪条路能走？
- 路径是否平滑？
- 到达后朝向哪里？

Agent 可以理解目标，但 Navigation 负责把目标变成可执行的空间路径。

## 第十二章：Skill，把机器人能力交给 Agent

### 12.1 Skill 是什么

Skill 是暴露给 Agent 的工具。它通常是一个 Python 方法，但通过装饰器和 schema 变成 Agent 可以发现和调用的能力。

例如，一个移动 Skill 可能表达为：

```text
move(x: float, duration: float) -> str
```

Agent 不需要知道底层怎么控制电机。它只需要知道：这个工具可以让机器人移动，参数是什么，调用后会返回什么结果。

### 12.2 Skill 和 RPC 的关系

在 DimOS 中，Skill 本质上也是 RPC 方法，但它额外暴露给 AI Agent。

可以这样理解：

```text
RPC：模块之间可以调用的方法
Skill：Agent 也可以调用的 RPC 方法
```

并不是所有 RPC 都应该变成 Skill。只有那些适合让 Agent 使用、边界清晰、安全可控的能力，才应该暴露成 Skill。

### 12.3 一个好 Skill 应该具备什么

一个适合 Agent 使用的 Skill 应该：

- 有清晰 docstring。
- 参数都有类型标注。
- 参数含义明确。
- 返回字符串结果。
- 不暴露危险的底层细节。
- 能告诉 Agent 调用结果或下一步状态。

### 12.4 为什么要用 Skill，而不是让 Agent 直接写代码

真实机器人系统不能让大模型随意执行任意代码来控制硬件。原因很简单：风险太高。

Skill 的意义是建立能力边界：

- Agent 可以选择工具。
- 工具参数可检查。
- 工具说明可读。
- 底层实现由工程代码控制。
- 危险动作可以限制、校验或拒绝。

这也是 agentic robot 系统最重要的安全设计之一。

## 第十三章：MCP，Agent 如何发现和调用工具

### 13.1 MCP 是什么

MCP 可以理解为工具发现和工具调用协议。DimOS 中的 McpServer 会把 Skills 暴露出去，McpClient 或 Agent 会读取这些工具。

### 13.2 MCP 在 DimOS 中的位置

典型关系如下：

```text
Skill Container
  ↓
McpServer
  ↓
McpClient
  ↓
Agent
  ↓
Tool Call
  ↓
Skill Method
```

Skill Container 定义能力，McpServer 把能力变成工具列表，McpClient 获取工具，Agent 决定什么时候调用。

### 13.3 为什么需要 MCP

没有 MCP，Agent 和工具之间可能会变成硬编码关系。每次加一个技能，都要手动改 Agent 逻辑。

MCP 的好处是：

- 工具可以被统一发现。
- 参数 schema 可以自动生成。
- Agent 不需要硬编码每个工具。
- 外部系统也可以用同一套协议调用工具。
- 调试工具列表更方便。

## 第十四章：Agent，DimOS 的智能决策层

### 14.1 Agent 负责什么

Agent 负责理解用户意图，并选择合适的工具或技能来完成目标。

例如用户说：

```text
去桌子旁边看看有没有杯子
```

Agent 可能需要拆成多个步骤：

1. 理解“桌子旁边”是一个空间目标。
2. 调用导航相关 Skill。
3. 到达后调用感知或观察相关 Skill。
4. 根据检测结果回答用户。

### 14.2 Agent 不负责什么

Agent 不应该直接负责：

- 每一帧图像处理
- 每一个控制周期
- 电机级控制
- 无边界硬件操作
- 绕过 Skill 的底层调用

Agent 是高层决策者，不是实时控制器。

### 14.3 Agent 为什么需要系统提示词

系统提示词告诉 Agent：

- 它是什么机器人。
- 它有什么能力。
- 应该如何使用工具。
- 不能做什么。
- 遇到不确定情况如何处理。

对于机器人系统来说，Prompt 不是普通文案，而是行为边界的一部分。

## 第十五章：Visualization，为什么机器人系统必须可视化

### 15.1 Visualization 负责什么

可视化模块负责把机器人系统中的状态展示出来，例如：

- 相机画面
- 点云
- 地图
- 路径
- 机器人轨迹
- 目标检测结果
- 坐标系
- 调试信息

### 15.2 为什么只看日志不够

普通后端服务出错时，看日志可能够用。但机器人系统生活在空间里。很多问题必须看图才能判断：

- 地图是不是偏了？
- 障碍物是不是识别错了？
- 路径是不是穿墙了？
- 相机画面是不是延迟？
- 机器人位姿是不是跳变？

所以 DimOS 提供可视化能力，让开发者能看到系统正在理解的世界。

## 第十六章：Web API，外部系统如何接入 DimOS

### 16.1 Web API 负责什么

Web API 让外部程序、网页界面或远程服务访问 DimOS 的能力。它可以用于：

- 查看机器人状态
- 发送命令
- 调用能力
- 集成前端界面
- 做远程监控

### 16.2 Web API 和 CLI 的区别

CLI 更适合开发者在终端操作，比如启动、停止、查看日志。

Web API 更适合系统集成，比如让前端页面、后台服务或远程工具控制和观察机器人。

两者都不是机器人能力本身，而是用户或外部系统进入 DimOS 的入口。

## 第十七章：Testing，为什么机器人项目特别需要测试

### 17.1 机器人项目的 bug 更危险

普通软件 bug 可能导致页面显示错误或服务失败。机器人 bug 可能导致：

- 机器人撞到东西
- 机械臂移动到危险位置
- 无人机执行错误动作
- Agent 调用不存在的技能
- 导航路径穿过障碍物

所以测试不是可选项。

### 17.2 DimOS 中测试关注什么

DimOS 的测试通常关注：

- Module 行为是否正确。
- Stream 和 Blueprint 是否能连接。
- 自动生成的 Blueprint registry 是否最新。
- Skill schema 是否能被正确生成。
- CLI 命令是否能工作。
- 导航、感知等核心算法是否稳定。

### 17.3 小白如何开始跑测试

常用命令是：

```bash
uv run pytest
```

如果只想跑某个文件，可以使用：

```bash
uv run pytest path/to/test_file.py -v
```

项目也可能提供更完整的验证脚本，例如：

```bash
bash scripts/verify.sh
```

## 第十八章：一次完整任务案例

现在把前面的概念串起来。假设用户对机器人说：

```text
走到桌子旁边，看看有没有杯子。
```

### 18.1 用户输入进入系统

用户可能通过 CLI、Agent 消息、Web API 或其他界面发送这句话。消息最终会进入 Agent。

### 18.2 Agent 理解任务

Agent 读取系统提示词，知道自己是什么机器人、有哪些 Skills、应该如何调用工具。

它不会直接控制电机，而是先思考任务结构：

```text
目标：去桌子旁边并观察杯子
需要能力：导航 + 感知
风险：需要知道桌子位置，不能盲目移动
```

### 18.3 Agent 调用 Skill

如果系统暴露了导航 Skill，Agent 会调用类似“导航到目标”的工具。这个 Skill 可能会通过 RPC 调用导航模块，或者发布目标点消息。

### 18.4 Navigation 规划路径

导航模块根据地图、当前位置和目标位置规划路线。如果地图来自回放或实时传感器，导航模块会持续更新环境理解。

### 18.5 Robot 模块执行

控制相关模块把路径或速度指令转成机器人能执行的动作。真实硬件、仿真或回放模式下，执行方式不同，但上层流程尽量保持一致。

### 18.6 Perception 观察杯子

到达桌子附近后，相机模块发布图像，感知模块检测物体。如果检测到杯子，结果会通过 Stream 传给 Agent 或其他模块。

### 18.7 Agent 回答用户

Agent 根据工具调用结果和感知结果回答：

```text
我已经到桌子旁边，看到一个杯子。
```

或者：

```text
我到了桌子旁边，但当前视野里没有检测到杯子。
```

### 18.8 这个案例串起了哪些部分

```text
用户命令
  ↓
Agent
  ↓
Skill / MCP
  ↓
Navigation
  ↓
Robot Control
  ↓
Camera / Sensors
  ↓
Perception
  ↓
Agent 回复
```

这就是 DimOS 的核心价值：把自然语言目标、安全工具调用、机器人运动、感知理解和系统运行管理连接成一条链。

## 第十九章：如何从源码角度阅读 DimOS

### 19.1 先读哪些目录

建议按这个顺序读：

1. `docs/usage/README.md`
2. `docs/usage/modules.md`
3. `docs/usage/blueprints.md`
4. `docs/usage/configuration.md`
5. `dimos/core/`
6. `dimos/agents/`
7. `dimos/robot/all_blueprints.py`
8. 某个具体平台目录，例如 `dimos/robot/unitree/go2/`

不要一开始就钻进某个复杂硬件驱动。先理解 DimOS 的抽象，再看具体平台。

### 19.2 看源码时的四个问题

每看到一个文件，都问：

1. 这个文件定义的是能力、连接方式、运行方式，还是具体平台？
2. 它输入什么数据？
3. 它输出什么数据？
4. 它是被 Blueprint 组装，还是被其他模块调用？

如果能回答这四个问题，大多数文件就不会显得混乱。

### 19.3 推荐阅读路径

先看核心概念：

- `dimos/core/module.py`
- `dimos/core/stream.py`
- `dimos/core/transport.py`
- `dimos/core/global_config.py`
- `dimos/core/coordination/blueprints.py`

再看 Agent 和 Skill：

- `dimos/agents/agent.py`
- `dimos/agents/annotation.py`
- `dimos/agents/mcp/`
- `dimos/agents/system_prompt.py`

最后看机器人平台：

- `dimos/robot/unitree/`
- `dimos/robot/drone/`
- `dimos/hardware/`
- `dimos/manipulation/`

## 第二十章：从零添加一个新能力的思路

### 20.1 先判断它是什么类型

你想添加的新能力，可能属于不同层：

- 是持续数据？可能应该做成 Stream。
- 是一个独立功能？可能应该做成 Module。
- 是 Agent 可以调用的动作？可能应该做成 Skill。
- 是一组模块组合？可能应该做成 Blueprint。
- 是运行参数？可能应该进入 GlobalConfig。
- 是外部交互入口？可能应该加入 CLI 或 Web API。

### 20.2 如果添加一个新 Skill

一般步骤是：

1. 找到合适的 Skill Container。
2. 添加带 docstring 和类型标注的方法。
3. 使用 `@skill` 暴露给 Agent。
4. 如果需要调用其他模块，使用 Spec 和 RPC。
5. 返回清晰的字符串结果。
6. 更新对应机器人系统提示词。
7. 确认 agentic Blueprint 包含该 Skill Container。
8. 跑测试或至少启动对应 Blueprint 验证工具列表。

### 20.3 如果添加一个新 Blueprint

一般步骤是：

1. 找出已有模块能复用哪些。
2. 定义新的模块组合。
3. 用 `autoconnect()` 连接模块。
4. 根据需要加入 McpServer、McpClient、Agent、Skills。
5. 暴露为模块级变量。
6. 更新或生成 Blueprint registry。
7. 运行对应测试确认 CLI 能找到它。

### 20.4 为什么先设计边界再写代码

在 DimOS 里，最重要的不是马上写一段函数，而是先决定它属于哪一层。层次放错会带来长期维护问题。

例如：

- 把硬件控制写进 Agent，会让安全边界变差。
- 把一次性动作做成 Stream，会让调用语义变模糊。
- 把高频控制做成大模型工具调用，会导致延迟和不稳定。
- 把平台专属能力写进通用模块，会降低复用性。

## 第二十一章：DimOS 的设计哲学

### 21.1 拆分，而不是堆叠

DimOS 把复杂系统拆成模块，不是为了显得复杂，而是因为机器人系统本来就复杂。拆分让每个部分更容易替换、测试和理解。

### 21.2 组合，而不是复制

不同机器人和场景可以复用很多能力。Blueprint 让系统通过组合形成差异，而不是每个平台复制一份完整代码。

### 21.3 数据流和调用分开

持续数据用 Stream，一次性动作或查询用 RPC。这种区分让系统语义更清楚。

### 21.4 Agent 有能力，但不能越界

Agent 负责理解任务和选择工具，但不应该绕开 Skill 直接控制硬件。DimOS 用 Skill、MCP、Prompt 和模块边界控制 Agent 的行动范围。

### 21.5 同一套代码服务多种运行模式

真实机器人、仿真和回放数据都很重要。DimOS 通过 GlobalConfig、Blueprint 和模块抽象，让同一套系统尽量适配不同运行模式。

## 第二十二章：最终总结

如果你是小白，只要记住下面这张图，就已经抓住 DimOS 的主线：

```text
CLI
  负责启动和管理
  ↓
GlobalConfig
  负责运行参数
  ↓
Blueprint
  负责选择和组装模块
  ↓
Module
  负责具体能力
  ↓
Stream / Transport
  负责模块通信
  ↓
Robot / Perception / Navigation / Visualization
  负责真实机器人能力
  ↓
Skill / MCP
  负责把能力安全暴露给 Agent
  ↓
Agent
  负责理解用户目标并调用工具
```

DimOS 的核心不是某一个文件，而是一套分层思想：

- 底层接硬件和数据。
- 中层组织模块通信和运行。
- 上层暴露能力给 Agent。
- 最外层通过 CLI、Web API、MCP 和可视化与用户交互。

当你读 DimOS 项目时，不要把它看成一堆目录。要把它看成一台机器人操作系统：

```text
它有身体：Robot Modules
它有眼睛：Perception
它有方向感：Navigation
它有神经系统：Streams / Transports
它有组织结构：Blueprints
它有工作单：Skills
它有大脑调度员：Agent
它有控制台：CLI / Web / Visualization
```

理解了这条主线，再去看具体代码，就不会迷路。

