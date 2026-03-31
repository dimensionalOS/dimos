<div align="center">

<img width="1000" alt="banner_bordered_trimmed" src="https://github.com/user-attachments/assets/64f13b39-da06-4f58-add0-cfc44f04db4e" />

<h2>物理空间的智能体操作系统</h2>

[![Discord](https://img.shields.io/discord/1341146487186391173?style=flat-square&logo=discord&logoColor=white&label=Discord&color=5865F2)](https://discord.gg/dimos)
[![Stars](https://img.shields.io/github/stars/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/stargazers)
[![Forks](https://img.shields.io/github/forks/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/fork)
[![Contributors](https://img.shields.io/github/contributors/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/graphs/contributors)
![Nix](https://img.shields.io/badge/Nix-flakes-5277C3?style=flat-square&logo=NixOS&logoColor=white)
![NixOS](https://img.shields.io/badge/NixOS-supported-5277C3?style=flat-square&logo=NixOS&logoColor=white)
![CUDA](https://img.shields.io/badge/CUDA-supported-76B900?style=flat-square&logo=nvidia&logoColor=white)
[![Docker](https://img.shields.io/badge/Docker-ready-2496ED?style=flat-square&logo=docker&logoColor=white)](https://www.docker.com/)

<a href="https://trendshift.io/repositories/23169" target="_blank"><img src="https://trendshift.io/api/badge/repositories/23169" alt="dimensionalOS%2Fdimos | Trendshift" style="width: 250px; height: 55px;" width="250" height="55"/></a>

<big><big>

[硬件](#硬件) •
[安装](#安装) •
[Agent CLI 与 MCP](#agent-cli-与-mcp) •
[蓝图](#蓝图) •
[开发](#开发)

⚠️ **预发布测试版** ⚠️

[English](README.md) | **中文**

</big></big>

</div>

# 简介

Dimensional 是面向通用机器人的现代操作系统。我们正在建立下一代 SDK 标准，并与主流机器人制造商深度集成。

只需简单安装，无需 ROS，即可完全使用 Python 构建运行在任何人形机器人、四足机器人或无人机上的物理应用。

Dimensional 天生支持智能体 -- 使用自然语言为您的机器人"氛围编程"（vibecode），构建（本地和托管的）多智能体系统，与您的硬件无缝协作。智能体作为原生模块运行 -- 可订阅任何嵌入式数据流，从感知（激光雷达、相机）和空间记忆到控制回路和电机驱动程序。
<table>
  <tr>
    <td align="center" width="50%">
      <a href="docs/capabilities/navigation/native/index.md"><img src="assets/readme/navigation.gif" alt="导航" width="100%"></a>
    </td>
    <td align="center" width="50%">
      <img src="assets/readme/perception.png" alt="感知" width="100%">
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <h3><a href="docs/capabilities/navigation/native/index.md">导航与建图</a></h3>
      SLAM、动态避障、路径规划和自主探索 -- 同时支持 DimOS 原生和 ROS<br><a href="https://x.com/stash_pomichter/status/2010471593806545367">观看视频</a>
    </td>
    <td align="center" width="50%">
      <h3>感知</h3>
      检测器、3D 投影、视觉语言模型（VLM）、音频处理
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <a href="docs/capabilities/agents/readme.md"><img src="assets/readme/agentic_control.gif" alt="智能体" width="100%"></a>
    </td>
    <td align="center" width="50%">
      <img src="assets/readme/spatial_memory.gif" alt="空间记忆" width="100%">
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <h3><a href="docs/capabilities/agents/readme.md">智能体控制、MCP</a></h3>
      "嘿，机器人，去找厨房"<br><a href="https://x.com/stash_pomichter/status/2015912688854200322">观看视频</a>
    </td>
    <td align="center" width="50%">
      <h3>空间记忆</a></h3>
      时空 RAG、动态记忆、目标定位与持久性<br><a href="https://x.com/stash_pomichter/status/1980741077205414328">观看视频</a>
    </td>
  </tr>
</table>


# 硬件

<table>
  <tr>
    <td align="center" width="20%">
      <h3>四足机器人</h3>
      <img width="245" height="1" src="assets/readme/spacer.png">
    </td>
    <td align="center" width="20%">
      <h3>人形机器人</h3>
      <img width="245" height="1" src="assets/readme/spacer.png">
    </td>
    <td align="center" width="20%">
      <h3>机械臂</h3>
      <img width="245" height="1" src="assets/readme/spacer.png">
    </td>
    <td align="center" width="20%">
      <h3>无人机</h3>
      <img width="245" height="1" src="assets/readme/spacer.png">
    </td>
    <td align="center" width="20%">
      <h3>其他</h3>
      <img width="245" height="1" src="assets/readme/spacer.png">
    </td>
  </tr>

  <tr>
    <td align="center" width="20%">
      🟩 <a href="docs/platforms/quadruped/go2/index.md">Unitree Go2 pro/air</a><br>
      🟥 <a href="dimos/robot/unitree/b1">Unitree B1</a><br>
    </td>
    <td align="center" width="20%">
      🟨 <a href="docs/platforms/humanoid/g1/index.md">Unitree G1</a><br>
    </td>
    <td align="center" width="20%">
      🟨 <a href="docs/capabilities/manipulation/readme.md">Xarm</a><br>
      🟨 <a href="docs/capabilities/manipulation/readme.md">AgileX Piper</a><br>
    </td>
    <td align="center" width="20%">
      🟧 <a href="dimos/robot/drone/README.md">MAVLink</a><br>
      🟧 <a href="dimos/robot/drone/README.md">DJI Mavic</a><br>
    </td>
    <td align="center" width="20%">
      🟥 <a href="https://github.com/dimensionalOS/openFT-sensor">力/扭矩传感器</a><br>
    </td>
  </tr>
</table>
<br>
<div align="right">
🟩 稳定 🟨 测试版 🟧 预览版 🟥 实验性

</div>

> [!IMPORTANT]
> 🤖 将您喜欢的 Agent（OpenClaw、Claude Code 等）指向 [AGENTS.md](AGENTS.md) 和我们的 [CLI 与 MCP](#agent-cli-与-mcp) 接口，即可开始构建强大的 Dimensional 应用。

# 安装

## 交互式安装

```sh
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
```

> 有关非交互式和高级选项，请参阅 [`scripts/install.sh --help`](scripts/install.sh)。

## 手动系统安装

如需设置系统依赖项，请参阅以下指南之一：

- 🟩 [Ubuntu 22.04 / 24.04](docs/installation/ubuntu.md)
- 🟩 [NixOS / 通用 Linux](docs/installation/nix.md)
- 🟧 [macOS](docs/installation/osx.md)

> 完整的系统要求、测试配置和依赖层级：[docs/requirements.md](docs/requirements.md)

## Python 安装

### 快速开始

```bash
uv venv --python "3.12"
source .venv/bin/activate
uv pip install 'dimos[base,unitree]'

# 回放已录制的四足机器人会话（无需硬件）
# 注意：首次运行时会显示黑色的 rerun 窗口，同时从 LFS 下载约 75 MB 数据
dimos --replay run unitree-go2
```

```bash
# 安装仿真支持
uv pip install 'dimos[base,unitree,sim]'

# 在 MuJoCo 仿真中运行四足机器人
dimos --simulation run unitree-go2

# 在仿真中运行人形机器人
dimos --simulation run unitree-g1-sim
```

```bash
# 控制真实机器人（通过 WebRTC 连接 Unitree 四足机器人）
export ROBOT_IP=<YOUR_ROBOT_IP>
dimos run unitree-go2
```

# 精选运行文件

| 运行命令 | 功能说明 |
|-------------|-------------|
| `dimos --replay run unitree-go2` | 四足机器人导航回放 -- SLAM、代价地图、A* 路径规划 |
| `dimos --replay --replay-dir unitree_go2_office_walk2 run unitree-go2-temporal-memory` | 四足机器人时序记忆回放 |
| `dimos --simulation run unitree-go2-agentic` | 四足机器人智能体 + 仿真中的 MCP 服务器 |
| `dimos --simulation run unitree-g1` | MuJoCo 仿真中的人形机器人 |
| `dimos --replay run drone-basic` | 无人机视频 + 遥测回放 |
| `dimos --replay run drone-agentic` | 无人机 + LLM 智能体与飞行技能（回放） |
| `dimos run demo-camera` | 摄像头演示 -- 无需硬件 |
| `dimos run keyboard-teleop-xarm7` | 使用模拟 xArm7 进行键盘遥操作（需要 `dimos[manipulation]` 扩展） |
| `dimos --simulation run unitree-go2-agentic-ollama` | 使用本地 LLM 的四足机器人智能体（需要 [Ollama](https://ollama.com) + `ollama serve`） |

> 完整蓝图文档：[docs/usage/blueprints.md](docs/usage/blueprints.md)

# Agent CLI 与 MCP

`dimos` CLI 管理完整的生命周期 -- 运行蓝图、检查状态、与智能体交互以及通过 MCP 调用技能。

```bash
dimos run unitree-go2-agentic --daemon   # 后台启动
dimos status                              # 检查运行状态
dimos log -f                              # 实时跟踪日志
dimos agent-send "explore the room"       # 向智能体发送命令
dimos mcp list-tools                      # 列出可用的 MCP 技能
dimos mcp call relative_move --arg forward=0.5  # 直接调用技能
dimos stop                                # 关闭
```

> 完整 CLI 参考：[docs/usage/cli.md](docs/usage/cli.md)


# 使用方法

## 将 DimOS 作为库使用

以下是一个简单的机器人连接模块示例，它向机器人发送连续的 `cmd_vel` 数据流，并将 `color_image` 接收到一个简单的 `Listener` 模块中。DimOS 模块是机器人上的子系统，使用标准化消息与其他模块通信。

```py
import threading, time, numpy as np
from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs import Twist
from dimos.msgs.sensor_msgs import Image, ImageFormat

class RobotConnection(Module):
    cmd_vel: In[Twist]
    color_image: Out[Image]

    @rpc
    def start(self):
        threading.Thread(target=self._image_loop, daemon=True).start()

    def _image_loop(self):
        while True:
            img = Image.from_numpy(
                np.zeros((120, 160, 3), np.uint8),
                format=ImageFormat.RGB,
                frame_id="camera_optical",
            )
            self.color_image.publish(img)
            time.sleep(0.2)

class Listener(Module):
    color_image: In[Image]

    @rpc
    def start(self):
        self.color_image.subscribe(lambda img: print(f"image {img.width}x{img.height}"))

if __name__ == "__main__":
    autoconnect(
        RobotConnection.blueprint(),
        Listener.blueprint(),
    ).build().loop()
```

## 蓝图

蓝图是关于如何构建和连接模块的指令。我们使用 `autoconnect(...)` 组合蓝图，它通过 `(名称, 类型)` 自动连接数据流并返回一个 `Blueprint`。

蓝图可以被组合、重映射，并且在 `autoconnect()` 因变量名或 `In[]` 和 `Out[]` 消息类型冲突而失败时，可以覆盖传输方式。

以下蓝图示例将机器人的图像流连接到基于 MCP 的 LLM 智能体，用于推理和动作执行。
```py
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs import Image
from dimos.robot.unitree.go2.connection import go2_connection
from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer

blueprint = autoconnect(
    go2_connection(),
    McpServer.blueprint(),
    McpClient.blueprint(),
).transports({("color_image", Image): LCMTransport("/color_image", Image)})

# 运行蓝图
if __name__ == "__main__":
    blueprint.build().loop()
```

## 库 API

- [模块](docs/usage/modules.md)
- [LCM](docs/usage/lcm.md)
- [蓝图](docs/usage/blueprints.md)
- [传输](docs/usage/transports/index.md) -- LCM、SHM、DDS、ROS 2
- [数据流](docs/usage/data_streams/README.md)
- [配置](docs/usage/configuration.md)
- [可视化](docs/usage/visualization.md)

## 演示

<img src="assets/readme/dimos_demo.gif" alt="DimOS 演示" width="100%">

# 开发

## 在 DimOS 上开发

```sh
export GIT_LFS_SKIP_SMUDGE=1
git clone -b dev https://github.com/dimensionalOS/dimos.git
cd dimos

uv sync --all-extras --no-extra dds

# 运行快速测试套件
uv run pytest dimos
```


## 多语言支持

Python 是我们的胶水语言和原型开发语言，但我们通过 LCM 互操作支持多种语言。

查看我们的语言互操作示例：
- [C++](examples/language-interop/cpp/)
- [Lua](examples/language-interop/lua/)
- [TypeScript](examples/language-interop/ts/)
