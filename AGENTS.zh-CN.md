# Dimensional AGENTS.md

[English](AGENTS.md) | **中文**

## 什么是 DimOS

面向通用机器人的智能体操作系统。`Module`（模块）通过 LCM、ROS2、DDS 或其他传输协议上的类型化数据流进行通信。`Blueprint`（蓝图）将模块组合为可运行的机器人栈。`Skill`（技能）赋予智能体执行硬件物理功能的能力，例如 `grab()`、`follow_object()` 或 `jump()`。

---

## 快速开始

```bash
# 安装
uv sync --all-extras --no-extra dds

# 列出所有可运行的蓝图
dimos list

# --- Go2 四足机器人 ---
dimos --replay run unitree-go2                  # 感知 + 建图，回放数据
dimos --replay run unitree-go2 --daemon         # 同上，后台运行
dimos --replay run unitree-go2-agentic          # + LLM 智能体 (GPT-4o) + 技能 + MCP 服务器
dimos run unitree-go2-agentic --robot-ip 192.168.123.161  # 真实 Go2 硬件

# --- G1 人形机器人 ---
dimos --simulation run unitree-g1-agentic-sim   # MuJoCo 仿真中的 G1 + 智能体 + 技能
dimos run unitree-g1-agentic --robot-ip 192.168.123.161   # 真实 G1 硬件

# --- 检查与控制 ---
dimos status
dimos log              # 最近 50 行，可读格式
dimos log -f           # 实时跟踪日志
dimos agent-send "say hello"
dimos stop             # 优雅关闭：SIGTERM → SIGKILL
dimos restart          # 停止 + 使用相同参数重新运行
```

### 蓝图快速参考

| 蓝图 | 机器人 | 硬件 | 智能体 | MCP 服务器 | 备注 |
|-----------|-------|----------|-------|------------|-------|
| `unitree-go2-agentic` | Go2 | 真实硬件 | 通过 McpClient | ✓ | McpServer 运行中 |
| `unitree-g1-agentic-sim` | G1 | 仿真 | GPT-4o（G1 提示词） | — | 完整智能体仿真，无需真实机器人 |
| `xarm-perception-agent` | xArm | 真实硬件 | GPT-4o | — | 操控 + 感知 + 智能体 |
| `xarm7-trajectory-sim` | xArm7 | 仿真 | — | — | 轨迹规划仿真 |
| `teleop-quest-xarm7` | xArm7 | 真实硬件 | — | — | Quest VR 遥操作 |
| `dual-xarm6-planner` | xArm6×2 | 真实硬件 | — | — | 双臂运动规划器 |

运行 `dimos list` 查看完整列表。

---

## 可用工具（MCP）

**MCP 仅在蓝图包含 `McpServer` 时有效。** 所有已发布的智能体蓝图都使用 `McpServer` + `McpClient`。例如：`unitree-go2-agentic`。

```bash
# 先启动支持 MCP 的蓝图：
dimos --replay run unitree-go2-agentic --daemon

# 然后使用 MCP 工具：
dimos mcp list-tools                                              # 以 JSON 格式列出所有可用技能
dimos mcp call move --arg x=0.5 --arg duration=2.0               # 通过 key=value 参数调用
dimos mcp call move --json-args '{"x": 0.5, "duration": 2.0}'    # 通过 JSON 调用
dimos mcp status      # PID、模块列表、技能列表
dimos mcp modules     # 模块 → 技能映射

# 向正在运行的智能体发送消息（无 McpServer 也可使用）：
dimos agent-send "walk forward 2 meters then wave"
```

MCP 服务器运行在 `http://localhost:9990/mcp`（`GlobalConfig.mcp_port`）。

### 在蓝图中添加 McpServer

同时使用 `McpServer.blueprint()` 和 `McpClient.blueprint()`。

```python
from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer

unitree_go2_agentic = autoconnect(
    unitree_go2_spatial,   # 机器人栈
    McpServer.blueprint(), # HTTP MCP 服务器 -- 在端口 9990 暴露所有 @skill 方法
    McpClient.blueprint(), # LLM 智能体 -- 从 McpServer 获取工具
    _common_agentic,       # 技能容器
)
```

参考：`dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic.py`

---

## 仓库结构

```
dimos/
├── core/                    # 模块系统、蓝图、工作进程、传输
│   ├── module.py            # Module 基类、In/Out 数据流、@rpc、@skill
│   ├── blueprints.py        # 蓝图组合（autoconnect）
│   ├── global_config.py     # GlobalConfig（环境变量、CLI 标志、.env）
│   └── run_registry.py      # 每次运行的跟踪 + 日志路径
├── robot/
│   ├── cli/dimos.py         # CLI 入口点（typer）
│   ├── all_blueprints.py    # 自动生成的蓝图注册表（请勿手动编辑）
│   ├── unitree/             # Unitree 机器人实现（Go2、G1、B1）
│   │   ├── unitree_skill_container.py  # Go2 @skill 方法
│   │   ├── go2/             # Go2 蓝图与连接
│   │   └── g1/              # G1 蓝图、连接、仿真、技能
│   └── drone/               # 无人机实现（MAVLink + DJI）
│       ├── connection_module.py        # MAVLink 连接
│       ├── camera_module.py            # DJI 视频流
│       ├── drone_tracking_module.py    # 视觉目标跟踪
│       └── drone_visual_servoing_controller.py  # 视觉伺服
├── agents/
│   ├── agent.py             # Agent 模块（基于 LangGraph）
│   ├── system_prompt.py     # 默认 Go2 系统提示词
│   ├── annotation.py        # @skill 装饰器
│   ├── mcp/                 # McpServer、McpClient、McpAdapter
│   └── skills/              # NavigationSkillContainer、SpeakSkill 等
├── navigation/              # 路径规划、前沿探索
├── perception/              # 目标检测、跟踪、记忆
├── visualization/rerun/     # Rerun 桥接
├── msgs/                    # 消息类型（geometry_msgs、sensor_msgs、nav_msgs）
└── utils/                   # 日志、数据加载、CLI 工具
docs/
├── usage/modules.md         # ← 模块系统深入解析
├── usage/blueprints.md      # 蓝图组合指南
├── usage/configuration.md   # GlobalConfig + Configurable 模式
├── development/testing.md   # 快速/慢速测试、pytest 使用
├── development/dimos_run.md # CLI 使用、添加蓝图
└── agents/                  # 智能体系统文档
```

---

## 架构

### 模块

自治子系统。通过 `In[T]`/`Out[T]` 类型化数据流通信。在 forkserver 工作进程中运行。

```python
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.core import rpc
from dimos.msgs.sensor_msgs import Image

class MyModule(Module):
    color_image: In[Image]
    processed: Out[Image]

    @rpc
    def start(self) -> None:
        super().start()
        self.color_image.subscribe(self._process)

    def _process(self, img: Image) -> None:
        self.processed.publish(do_something(img))
```

### 蓝图

使用 `autoconnect()` 组合模块。数据流通过 `(名称, 类型)` 匹配自动连接。

```python
from dimos.core.blueprints import autoconnect

my_blueprint = autoconnect(module_a(), module_b(), module_c())
```

直接从 Python 运行蓝图：

```python
# build() 将所有模块部署到 forkserver 工作进程并连接数据流
# loop() 阻塞主线程，直到停止（Ctrl-C 或 SIGTERM）
autoconnect(module_a(), module_b(), module_c()).build().loop()
```

将蓝图暴露为模块级变量，以便 `dimos run` 发现。通过运行 `pytest dimos/robot/test_all_blueprints_generation.py` 添加到注册表。

### GlobalConfig

单例配置。值按优先级层叠：默认值 → `.env` → 环境变量 → 蓝图 → CLI 标志。环境变量前缀为 `DIMOS_`。关键字段：`robot_ip`、`simulation`、`replay`、`viewer`、`n_workers`、`mcp_port`。

### 传输

- **LCMTransport**：默认。组播 UDP。
- **SHMTransport/pSHMTransport**：共享内存 -- 用于图像和点云。
- **pLCMTransport**：序列化 LCM -- 用于复杂的 Python 对象。
- **ROSTransport**：ROS 话题桥接 -- 与 ROS 节点互操作（`dimos/core/transport.py`）。
- **DDSTransport**：DDS 发布/订阅 -- 在 `DDS_AVAILABLE` 时可用；使用 `uv sync --extra dds` 安装（`dimos/protocol/pubsub/impl/ddspubsub.py`）。

---

## CLI 参考

### 全局标志

每个 `GlobalConfig` 字段都是 CLI 标志：`--robot-ip`、`--simulation/--no-simulation`、`--replay/--no-replay`、`--viewer {rerun|rerun-web|foxglove|none}`、`--mcp-port`、`--n-workers` 等。标志会覆盖 `.env` 和环境变量。

### 核心命令

| 命令 | 描述 |
|---------|-------------|
| `dimos run <blueprint> [--daemon]` | 启动蓝图 |
| `dimos status` | 显示运行实例（运行 ID、PID、蓝图、运行时间、日志路径） |
| `dimos stop [--force]` | SIGTERM → 5 秒后 SIGKILL；`--force` = 立即 SIGKILL |
| `dimos restart [--force]` | 停止 + 使用原始参数重新执行 |
| `dimos list` | 列出所有非演示蓝图 |
| `dimos show-config` | 打印已解析的 GlobalConfig 值 |
| `dimos log [-f] [-n N] [--json] [-r <run-id>]` | 查看每次运行的日志 |
| `dimos mcp list-tools / call / status / modules` | MCP 工具（需要蓝图中包含 McpServer） |
| `dimos agent-send "<text>"` | 通过 LCM 向运行中的智能体发送文本 |
| `dimos lcmspy / agentspy / humancli / top` | 调试/诊断工具 |
| `dimos topic echo <topic> / send <topic> <expr>` | LCM 话题发布/订阅 |
| `dimos rerun-bridge` | 独立启动 Rerun 可视化 |

日志文件：`~/.local/state/dimos/logs/<run-id>/main.jsonl`
运行注册表：`~/.local/state/dimos/runs/<run-id>.json`

---

## 智能体系统

### `@skill` 装饰器

`dimos/agents/annotation.py`。设置 `__rpc__ = True` 和 `__skill__ = True`。

- `@rpc` 单独使用：可通过 RPC 调用，但不暴露给 LLM
- `@skill`：隐含 `@rpc` 并将方法作为工具暴露给 LLM。**不要同时使用两者。**

#### Schema 生成规则

| 规则 | 违反后果 |
|------|------------------------------|
| **文档字符串（docstring）必须提供** | 启动时抛出 `ValueError` -- 模块注册失败，所有技能消失 |
| **每个参数都必须有类型注解** | 缺少注解 → schema 中无 `"type"` -- LLM 没有类型信息 |
| **返回 `str`** | 返回 `None` → 智能体收到"已启动，稍后更新" |
| **完整文档字符串逐字写入 `description`** | 保持 `Args:` 块简洁 -- 它会出现在每次工具调用的提示词中 |

支持的参数类型：`str`、`int`、`float`、`bool`、`list[str]`、`list[float]`。避免使用复杂嵌套类型。

#### 最小正确技能示例

```python
from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module

class MySkillContainer(Module):
    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
    def move(self, x: float, duration: float = 2.0) -> str:
        """Move the robot forward or backward.

        Args:
            x: Forward velocity in m/s. Positive = forward, negative = backward.
            duration: How long to move in seconds.
        """
        return f"Moving at {x} m/s for {duration}s"

my_skill_container = MySkillContainer.blueprint

### 系统提示词

| 机器人 | 文件 | 变量 |
|-------|------|----------|
| Go2（默认） | `dimos/agents/system_prompt.py` | `SYSTEM_PROMPT` |
| G1 人形机器人 | `dimos/robot/unitree/g1/system_prompt.py` | `G1_SYSTEM_PROMPT` |

传入机器人特定提示词：`McpClient.blueprint(system_prompt=G1_SYSTEM_PROMPT)`。默认提示词针对 Go2；在 G1 上使用会导致技能幻觉。

### RPC 连接

要调用另一个模块的方法，声明一个 `Spec` Protocol 并用它注解属性。蓝图在构建时注入匹配的模块 -- 完全类型化、无字符串、在构建时（而非运行时）失败。

```python
# my_module_spec.py
from typing import Protocol
from dimos.spec.utils import Spec

class NavigatorSpec(Spec, Protocol):
    def set_goal(self, goal: PoseStamped) -> bool: ...
    def cancel_goal(self) -> bool: ...

# my_skill_container.py
class MySkillContainer(Module):
    _navigator: NavigatorSpec   # 蓝图在构建时注入

    @skill
    def go_to(self, x: float, y: float) -> str:
        """Navigate to a position."""
        self._navigator.set_goal(make_pose(x, y))
        return "Navigating"
```

如果多个模块匹配该 spec，使用 `.remappings()` 解决。源码：`dimos/spec/utils.py`、`dimos/core/blueprints.py`。

### 添加新技能

1. 选择合适的容器（机器人专用的或 `dimos/agents/skills/`）。
2. `@skill` + 必须提供文档字符串 + 所有参数的类型注解。
3. 如果需要其他模块的 RPC，使用 Spec 模式。
4. 返回描述性的 `str`。
5. 更新系统提示词 -- 添加到 `# AVAILABLE SKILLS` 部分。
6. 暴露为 `my_container = MySkillContainer.blueprint` 并包含在智能体蓝图中。

---

## 测试

```bash
# 快速测试（默认）
uv run pytest

# 包含慢速测试（CI）
./bin/pytest-slow

# 单个文件
uv run pytest dimos/core/test_blueprints.py -v

# Mypy
uv run mypy dimos/
```

`uv run pytest` 排除 `slow`、`tool` 和 `mujoco` 标记。CI（`./bin/pytest-slow`）包含 slow，排除 tool 和 mujoco。参见 `docs/development/testing.md`。

---

## Pre-commit 与代码风格

Pre-commit 在 `git commit` 时运行。包括 ruff 格式化/检查、许可证头、LFS 检查。

**提交前务必激活虚拟环境：** `source .venv/bin/activate`

代码风格规则：
- 导入语句放在文件顶部。除非存在循环依赖，否则不使用内联导入。
- HTTP 请求使用 `requests`（而非 `urllib`）。JSON 值使用 `Any`（而非 `object`）。
- 手动测试脚本以 `demo_` 为前缀，以排除在 pytest 收集之外。
- 不要硬编码端口/URL -- 使用 `GlobalConfig` 常量。
- 需要类型注解。Mypy 严格模式。

---

## `all_blueprints.py` 为自动生成

`dimos/robot/all_blueprints.py` 由 `test_all_blueprints_generation.py` 生成。添加或重命名蓝图后：

```bash
pytest dimos/robot/test_all_blueprints_generation.py
```

CI 会断言文件是最新的 -- 如果过时，CI 将失败。

---

## Git 工作流

- 分支前缀：`feat/`、`fix/`、`refactor/`、`docs/`、`test/`、`chore/`、`perf/`
- **PR 目标为 `dev`** -- 不要直接推送到 `main` 或 `dev`
- **不要强制推送**，除非在有冲突的 rebase 之后
- **减少推送次数** -- 每次推送都会触发 CI（自托管运行器上约 1 小时）。在本地批量提交，一次推送。

---

## 延伸阅读

- 模块系统：`docs/usage/modules.md`
- 蓝图：`docs/usage/blueprints.md`
- 可视化：`docs/usage/visualization.md`
- 配置：`docs/usage/configuration.md`
- 测试：`docs/development/testing.md`
- CLI / dimos run：`docs/development/dimos_run.md`
- LFS 数据：`docs/development/large_file_management.md`
- 智能体系统：`docs/agents/`
