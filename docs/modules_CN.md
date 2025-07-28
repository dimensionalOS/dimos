# Dimensional 模块系统

DimOS 模块系统使用 Dask 进行计算分布和 LCM（轻量级通信和编组）进行高性能进程间通信，实现分布式、多进程的机器人应用。

## 核心概念

### 1. 模块定义
模块是继承自 `dimos.core.Module` 的 Python 类，定义输入、输出和 RPC 方法：

```python
from dimos.core import Module, In, Out, rpc
from dimos.msgs.geometry_msgs import Vector3

class MyModule(Module): # ROS Node
    # 将输入/输出声明为初始化为 None 的类属性
    data_in: In[Vector3] = None # ROS Subscriber
    data_out: Out[Vector3] = None # ROS Publisher

    def __init__():
        # 调用父类 Module 初始化
        super().__init__()

    @rpc
    def remote_method(self, param):
        """使用 @rpc 装饰的方法可以远程调用"""
        return param * 2
```

### 2. 模块部署
使用 `dimos.deploy()` 方法在 Dask 工作进程中部署模块：

```python
from dimos import core

# 启动具有 N 个工作进程的 Dask 集群
dimos = core.start(4)

# 部署模块时可以传递初始化参数
# 在这种情况下，param1 和 param2 被传递到模块初始化中
module = dimos.deploy(Module, param1="value1", param2=123)
```

### 3. 流连接
模块通过使用 LCM 传输的响应式流进行通信：

```python
# 为输出配置 LCM 传输
module1.data_out.transport = core.LCMTransport("/topic_name", MessageType)

# 将模块输入连接到输出
module2.data_in.connect(module1.data_out)

# 访问底层的 Observable 流
stream = module1.data_out.observable()
stream.subscribe(lambda msg: print(f"接收到: {msg}"))
```

### 4. 模块生命周期
```python
# 启动模块以开始处理
module.start()  # 如果定义了 @rpc start() 方法，则调用它

# 检查模块 I/O 配置  
print(module.io().result())  # 显示输入、输出和 RPC 方法

# 优雅关闭
dimos.shutdown()
```

## 实际示例：机器人控制系统

```python
# 连接模块封装机器人硬件/仿真
connection = dimos.deploy(ConnectionModule, ip=robot_ip)
connection.lidar.transport = core.LCMTransport("/lidar", LidarMessage)
connection.video.transport = core.LCMTransport("/video", Image)

# 感知模块处理传感器数据
perception = dimos.deploy(PersonTrackingStream, camera_intrinsics=[...])
perception.video.connect(connection.video)
perception.tracking_data.transport = core.pLCMTransport("/person_tracking")

# 开始处理
connection.start()
perception.start()

# 通过 RPC 启用跟踪
perception.enable_tracking()

# 获取最新的跟踪数据
data = perception.get_tracking_data()
```

## LCM 传输配置

```python
# 用于简单类型（如激光雷达）的标准 LCM 传输
connection.lidar.transport = core.LCMTransport("/lidar", LidarMessage)

# 用于复杂 Python 对象/字典的基于 pickle 的传输
connection.tracking_data.transport = core.pLCMTransport("/person_tracking")

# 自动配置 LCM 系统缓冲区（在容器中必需）
from dimos.protocol import pubsub
pubsub.lcm.autoconf()
```

这种架构使得能够将复杂的机器人系统构建为可组合的分布式模块，这些模块通过流和 RPC 高效通信，从单机扩展到集群。

# Dimensional 安装指南
## Python 安装（Ubuntu 22.04）

```bash
sudo apt install python3-venv

# 克隆仓库（dev 分支，无子模块）
git clone -b dev https://github.com/dimensionalOS/dimos.git
cd dimos

# 创建并激活虚拟环境
python3 -m venv venv
source venv/bin/activate

sudo apt install portaudio19-dev python3-pyaudio

# 如果尚未安装，请安装 torch 和 torchvision
# 示例 CUDA 11.7，Pytorch 2.0.1（如果需要不同的 pytorch 版本，请替换）
pip install torch==2.0.1 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### 安装依赖
```bash
# 仅 CPU（建议首先尝试）
pip install .[cpu,dev]

# CUDA 安装
pip install .[cuda,dev]

# 复制并配置环境变量
cp default.env .env
```

### 测试安装
```bash 
# 运行标准测试
pytest -s dimos/

# 测试模块功能
pytest -s -m module dimos/

# 测试 LCM 通信
pytest -s -m lcm dimos/
```

# Unitree Go2 快速开始

要快速测试模块系统，您可以直接运行 Unitree Go2 多进程示例：

```bash
# 确保设置了所需的环境变量
export ROBOT_IP=<your_robot_ip>

# 运行多进程 Unitree Go2 示例
python dimos/robot/unitree_webrtc/multiprocess/unitree_go2.py
```

## 模块系统的高级特性

### 分布式计算
DimOS 模块系统建立在 Dask 之上，提供了强大的分布式计算能力：

- **自动负载均衡**：模块自动分布在可用的工作进程中
- **容错性**：如果工作进程失败，模块可以在其他工作进程上重新启动
- **可扩展性**：从单机到集群的无缝扩展

### 响应式编程模型
使用 RxPY 实现的响应式流提供了：

- **异步处理**：非阻塞的数据流处理
- **背压处理**：自动管理快速生产者和慢速消费者
- **操作符链**：使用 map、filter、merge 等操作符进行流转换

### 性能优化
LCM 传输针对机器人应用进行了优化：

- **零拷贝**：大型消息的高效内存使用
- **低延迟**：微秒级的消息传递
- **多播支持**：一对多的高效通信 