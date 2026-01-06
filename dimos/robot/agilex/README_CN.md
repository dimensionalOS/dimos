# DIMOS 机械臂机器人开发指南

本指南介绍如何创建机器人类、集成智能体（Agent）以及使用 DIMOS 模块系统和 LCM 传输。

## 目录
1. [机器人类架构](#机器人类架构)
2. [模块系统与 LCM 传输](#模块系统与-lcm-传输)
3. [智能体集成](#智能体集成)
4. [完整示例](#完整示例)

## 机器人类架构

### 基本机器人类结构

DIMOS 机器人类应遵循以下模式：

```python
from typing import Optional, List
from dimos import core
from dimos.types.robot_capabilities import RobotCapability

class YourRobot:
    """您的机器人实现。"""

    def __init__(self, robot_capabilities: Optional[List[RobotCapability]] = None):
        # 核心组件
        self.dimos = None
        self.modules = {}
        self.skill_library = SkillLibrary()

        # 定义能力
        self.capabilities = robot_capabilities or [
            RobotCapability.VISION,
            RobotCapability.MANIPULATION,
        ]

    async def start(self):
        """启动机器人模块。"""
        # 初始化 DIMOS，指定工作线程数
        self.dimos = core.start(2)  # 需要的工作线程数

        # 部署模块
        # ... (参见模块系统章节)

    def stop(self):
        """停止所有模块并清理资源。"""
        # 停止模块
        # 关闭 DIMOS
        if self.dimos:
            self.dimos.close()
```

### 关键组件说明

1. **初始化**：存储模块、技能和能力的引用
2. **异步启动**：模块必须异步部署
3. **正确清理**：在关闭 DIMOS 之前始终停止模块

## 模块系统与 LCM 传输

### 理解 DIMOS 模块

模块是 DIMOS 机器人的构建块。它们：
- 处理数据流（输入）
- 产生输出
- 可以相互连接
- 通过 LCM（轻量级通信和编组）进行通信

### 部署模块

```python
# 部署相机模块
self.camera = self.dimos.deploy(
    ZEDModule,                    # 模块类
    camera_id=0,                  # 模块参数
    resolution="HD720",
    depth_mode="NEURAL",
    fps=30,
    publish_rate=30.0,
    frame_id="camera_frame"
)
```

### 设置 LCM 传输

LCM 传输实现模块间通信：

```python
# 启用 LCM 自动配置
from dimos.protocol import pubsub
pubsub.lcm.autoconf()

# 配置输出传输
self.camera.color_image.transport = core.LCMTransport(
    "/camera/color_image",        # 主题名称
    Image                         # 消息类型
)
self.camera.depth_image.transport = core.LCMTransport(
    "/camera/depth_image",
    Image
)
```

### 连接模块

将模块输出连接到输入：

```python
# 将操作模块连接到相机输出
self.manipulation.rgb_image.connect(self.camera.color_image) # ROS set_callback
self.manipulation.depth_image.connect(self.camera.depth_image)
self.manipulation.camera_info.connect(self.camera.camera_info)
```

### 模块通信模式

```
┌──────────────┐  LCM    ┌────────────────┐  LCM    ┌──────────────┐
│   相机模块    │────────▶│   操作模块      │────────▶│  可视化输出   │
│              │  消息    │                │  消息    │              │
└──────────────┘         └────────────────┘         └──────────────┘
     ▲                          ▲
     │                          │
     └──────────────────────────┘
              直接连接(RPC指令)
```

## 智能体集成

### 设置智能体与机器人

运行文件的智能体集成模式：

```python
#!/usr/bin/env python3
import asyncio
import reactivex as rx
from dimos.agents_deprecated.claude_agent import ClaudeAgent
from dimos.web.robot_web_interface import RobotWebInterface

def main():
    # 1. 创建并启动机器人
    robot = YourRobot()
    asyncio.run(robot.start())

    # 2. 设置技能
    skills = robot.get_skills()
    skills.add(YourSkill)
    skills.create_instance("YourSkill", robot=robot)

    # 3. 设置响应式流
    agent_response_subject = rx.subject.Subject()
    agent_response_stream = agent_response_subject.pipe(ops.share())

    # 4. 创建 Web 界面
    web_interface = RobotWebInterface(
        port=5555,
        text_streams={"agent_responses": agent_response_stream},
        audio_subject=rx.subject.Subject()
    )

    # 5. 创建智能体
    agent = ClaudeAgent(
        dev_name="your_agent",
        input_query_stream=web_interface.query_stream,
        skills=skills,
        system_query="您的系统提示词",
        model_name="claude-3-5-haiku-latest"
    )

    # 6. 连接智能体响应
    agent.get_response_observable().subscribe(
        lambda x: agent_response_subject.on_next(x)
    )

    # 7. 运行界面
    web_interface.run()
```

### 关键集成点

1. **响应式流**：使用 RxPy 进行事件驱动通信
2. **Web 界面**：提供用户输入/输出
3. **智能体**：处理自然语言并执行技能
4. **技能**：将机器人能力定义为可执行动作

## 完整示例

### 步骤 1：创建机器人类（`my_robot.py`）

```python
import asyncio
from typing import Optional, List
from dimos import core
from dimos.hardware.camera import CameraModule
from dimos.manipulation.module import ManipulationModule
from dimos.skills.skills import SkillLibrary
from dimos.types.robot_capabilities import RobotCapability
from dimos_lcm.sensor_msgs import Image, CameraInfo
from dimos.protocol import pubsub

class MyRobot:
    def __init__(self, robot_capabilities: Optional[List[RobotCapability]] = None):
        self.dimos = None
        self.camera = None
        self.manipulation = None
        self.skill_library = SkillLibrary()

        self.capabilities = robot_capabilities or [
            RobotCapability.VISION,
            RobotCapability.MANIPULATION,
        ]

    async def start(self):
        # 启动 DIMOS
        self.dimos = core.start(2)

        # 启用 LCM
        pubsub.lcm.autoconf()

        # 部署相机
        self.camera = self.dimos.deploy(
            CameraModule,
            camera_id=0,
            fps=30
        )

        # 配置相机 LCM
        self.camera.color_image.transport = core.LCMTransport("/camera/rgb", Image)
        self.camera.depth_image.transport = core.LCMTransport("/camera/depth", Image)
        self.camera.camera_info.transport = core.LCMTransport("/camera/info", CameraInfo)

        # 部署操作模块
        self.manipulation = self.dimos.deploy(ManipulationModule)

        # 连接模块
        self.manipulation.rgb_image.connect(self.camera.color_image)
        self.manipulation.depth_image.connect(self.camera.depth_image)
        self.manipulation.camera_info.connect(self.camera.camera_info)

        # 配置操作输出
        self.manipulation.viz_image.transport = core.LCMTransport("/viz/output", Image)

        # 启动模块
        self.camera.start()
        self.manipulation.start()

        await asyncio.sleep(2)  # 允许初始化

    def get_skills(self):
        return self.skill_library

    def stop(self):
        if self.manipulation:
            self.manipulation.stop()
        if self.camera:
            self.camera.stop()
        if self.dimos:
            self.dimos.close()
```

### 步骤 2：创建运行脚本（`run.py`）

```python
#!/usr/bin/env python3
import asyncio
import os
from my_robot import MyRobot
from dimos.agents_deprecated.claude_agent import ClaudeAgent
from dimos.skills.basic import BasicSkill
from dimos.web.robot_web_interface import RobotWebInterface
import reactivex as rx
import reactivex.operators as ops

SYSTEM_PROMPT = """您是一个有用的机器人助手。"""

def main():
    # 检查 API 密钥
    if not os.getenv("ANTHROPIC_API_KEY"):
        print("请设置 ANTHROPIC_API_KEY")
        return

    # 创建机器人
    robot = MyRobot()

    try:
        # 启动机器人
        asyncio.run(robot.start())

        # 设置技能
        skills = robot.get_skills()
        skills.add(BasicSkill)
        skills.create_instance("BasicSkill", robot=robot)

        # 设置流
        agent_response_subject = rx.subject.Subject()
        agent_response_stream = agent_response_subject.pipe(ops.share())

        # 创建 Web 界面
        web_interface = RobotWebInterface(
            port=5555,
            text_streams={"agent_responses": agent_response_stream}
        )

        # 创建智能体
        agent = ClaudeAgent(
            dev_name="my_agent",
            input_query_stream=web_interface.query_stream,
            skills=skills,
            system_query=SYSTEM_PROMPT
        )

        # 连接响应
        agent.get_response_observable().subscribe(
            lambda x: agent_response_subject.on_next(x)
        )

        print("机器人就绪，访问 http://localhost:5555")

        # 运行
        web_interface.run()

    finally:
        robot.stop()

if __name__ == "__main__":
    main()
```

### 步骤 3：定义技能（`skills.py`）

```python
from dimos.skills import Skill, skill

@skill(
    description="执行一个基本动作",
    parameters={
        "action": "要执行的动作"
    }
)
class BasicSkill(Skill):
    def __init__(self, robot):
        self.robot = robot

    def run(self, action: str):
        # 实现技能逻辑
        return f"已执行：{action}"
```

## 最佳实践

1. **模块生命周期**：在部署模块之前始终先启动 DIMOS
2. **LCM 主题**：使用带命名空间的描述性主题名称
3. **错误处理**：用 try-except 块包装模块操作
4. **资源清理**：确保在 stop() 方法中正确清理
5. **异步操作**：使用 asyncio 进行非阻塞操作
6. **流管理**：使用 RxPy 进行响应式编程模式

## 调试技巧

1. **检查模块状态**：打印 module.io().result() 查看连接
2. **监控 LCM**：使用 Foxglove 可视化 LCM 消息
3. **记录一切**：使用 dimos.utils.logging_config.setup_logger()
4. **独立测试模块**：一次部署和测试一个模块

## 常见问题

1. **"模块未启动"**：确保在部署后调用 start()
2. **"未收到数据"**：检查 LCM 传输配置
3. **"连接失败"**：验证输入/输出类型是否匹配
4. **"清理错误"**：在关闭 DIMOS 之前停止模块

## 高级主题

### 自定义模块开发

创建自定义模块的基本结构：

```python
from dimos.core import Module, In, Out, rpc

class CustomModule(Module):
    # 定义输入
    input_data: In[DataType]

    # 定义输出
    output_data: Out[DataType]

    def __init__(self, param1, param2, **kwargs):
        super().__init__(**kwargs)
        self.param1 = param1
        self.param2 = param2

    @rpc
    def start(self):
        """启动模块处理。"""
        self.input_data.subscribe(self._process_data)

    def _process_data(self, data):
        """处理输入数据。"""
        # 处理逻辑
        result = self.process(data)
        # 发布输出
        self.output_data.publish(result)

    @rpc
    def stop(self):
        """停止模块。"""
        # 清理资源
        pass
```

### 技能开发指南

技能是机器人可执行的高级动作：

```python
from dimos.skills import Skill, skill
from typing import Optional

@skill(
    description="复杂操作技能",
    parameters={
        "target": "目标对象",
        "location": "目标位置"
    }
)
class ComplexSkill(Skill):
    def __init__(self, robot, **kwargs):
        super().__init__(**kwargs)
        self.robot = robot

    def run(self, target: str, location: Optional[str] = None):
        """执行技能逻辑。"""
        try:
            # 1. 感知阶段
            object_info = self.robot.detect_object(target)

            # 2. 规划阶段
            if location:
                plan = self.robot.plan_movement(object_info, location)

            # 3. 执行阶段
            result = self.robot.execute_plan(plan)

            return {
                "success": True,
                "message": f"成功移动 {target} 到 {location}"
            }

        except Exception as e:
            return {
                "success": False,
                "error": str(e)
            }
```

### 性能优化

1. **并行处理**：使用多个工作线程处理不同模块
2. **数据缓冲**：为高频数据流实现缓冲机制
3. **延迟加载**：仅在需要时初始化重型模块
4. **资源池化**：重用昂贵的资源（如神经网络模型）

希望本指南能帮助您快速上手 DIMOS 机器人开发！
