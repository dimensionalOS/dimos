# TRON 1 接入 DimOS 迁移实施文档

本文档描述如何将当前 DimOS 项目移植/适配到 LimX TRON 1 平台。目标是把 TRON 1 的 SDK（上层应用接口 + lowlevel 接口）映射到 DimOS 的 `Module / Blueprint / Skill / MCP` 体系，使你可以通过 `dimos run tron1-*` 运行同一套导航、感知、agentic 能力。

参考资料：

- TRON 1 SDK 开发指南（上层应用接口 + lowlevel 目录）：https://limx.cn/zh/documents/799585387524788224
- limxsdk-lowlevel（Python/C++ 低层接口与安装方式）：https://github.com/limxdynamics/limxsdk-lowlevel
- tron1-mujoco-sim（MuJoCo 仿真、RobotCmd/RobotState/ImuData 对象模型）：https://github.com/limxdynamics/tron1-mujoco-sim
- tron1-gazebo-ros2（ROS2 仿真环境，可选）：https://github.com/limxdynamics/tron1-gazebo-ros2

---

## 1. 迁移目标与边界

### 1.1 你要“迁移”的是什么

DimOS 核心（模块系统、蓝图、调度、MCP、agent 框架）不需要改。需要新增的是“机器人适配层”：

- TRON 1 控制与状态连接模块（高层 + 后续 lowlevel/仿真）
- TRON 1 协议编码/解码
- TRON 1 技能容器（`@skill`）
- TRON 1 的蓝图（basic/perceptive/agentic）
- TRON 1 的系统提示词（system_prompt）

### 1.2 两条接入路线（必须先定）

- 路线 A：上层应用接口优先（推荐第一阶段）
  - 目标：尽快跑通 `move/cmd_vel + odom + imu + emergency_stop + recover`，上线 `tron1-basic`/`tron1-agentic`
  - 风险：控制粒度较粗，但最稳
- 路线 B：lowlevel / MuJoCo / RL（推荐第二阶段）
  - 目标：关节级控制、策略部署、仿真-真机一致
  - 依赖：`limxsdk-lowlevel` 与其对象模型（RobotCmd/RobotState/ImuData）

---

## 2. DimOS 现有接入链路（你要对齐的结构）

建议直接对齐 `Unitree G1` 的分层写法（高层连接 + spec 注入 + skill 容器 + agentic 蓝图），避免重复发明结构：

- G1 连接模块（高层）：[G1Connection](file:///mnt/SSD/dimos/dimos/robot/unitree/g1/connection.py#L72-L115)
- G1 Spec（用于注入）：[G1ConnectionSpec](file:///mnt/SSD/dimos/dimos/robot/unitree/g1/connection_spec.py#L21-L23)
- G1 SkillContainer：[UnitreeG1SkillContainer](file:///mnt/SSD/dimos/dimos/robot/unitree/g1/skill_container.py#L66-L124)
- G1 basic/perceptive/agentic 蓝图层级：
  - [unitree_g1_basic](file:///mnt/SSD/dimos/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_basic.py#L18-L27)
  - [unitree_g1](file:///mnt/SSD/dimos/dimos/robot/unitree/g1/blueprints/perceptive/unitree_g1.py#L18-L27)
  - [unitree_g1_agentic](file:///mnt/SSD/dimos/dimos/robot/unitree/g1/blueprints/agentic/unitree_g1_agentic.py#L18-L25)
- G1 primitive 蓝图已经把关键流名固定为 `/cmd_vel /odom /state_estimation /lidar /color_image` 等：[unitree_g1_primitive_no_nav](file:///mnt/SSD/dimos/dimos/robot/unitree/g1/blueprints/primitive/unitree_g1_primitive_no_nav.py#L125-L156)

结论：TRON1 最佳实践是“复制 G1 的分层方式”，只替换底层连接与传感器来源。

---

## 3. TRON 1 SDK 与 DimOS 的接口映射表

### 3.1 上层应用接口（推荐第一阶段）

以下接口来自 TRON 1 SDK 第 5 章（协议接口定义），建议作为 `TRON1HighLevelConnection` 的第一阶段能力：

- 运动控制
  - `request_twist`：对应 DimOS 的 `cmd_vel: In[Twist]` → `TRON1Connection.move(twist, duration)`
  - `request_walk_mode`：建议在第一次 `request_twist` 前确保进入 walk mode
  - `request_stand_mode`：建议在 `start()` 阶段执行（可配置）
  - `request_sitdown`：用于停机收尾/安全姿态
- 安全与恢复
  - `request_emgy_stop`：对应 `emergency_stop()`
  - `request_recover`：对应 `recover()`
- 关键状态输入
  - `request_enable_odom` + `notify_odom`：发布 `state_estimation: Out[Odometry]` 与 `odom: Out[PoseStamped]`
  - `request_enable_imu` + `notify_imu`：发布 `imu: Out[Imu]`
- 可选控制
  - `request_base_height`：对应 `set_base_height(height)`
  - `request_light_effect`：对应 `set_light_effect(effect)`

### 3.2 lowlevel 接口（推荐第二阶段）

来自文档第 4 章（limxsdk-lowlevel）的关键能力：

- `init`：连接初始化
- `subscribeImuData`：对接 `imu: Out[Imu]`
- `subscribeRobotState`：对接 odom/关节/姿态等（按你能拿到的字段决定）
- `publishRobotCmd`：关节级命令（不建议第一阶段用）

### 3.3 DimOS 消息与流

建议 TRON1 连接模块至少实现并发布这些流（对齐现有导航/感知模块）：

- `cmd_vel: In[Twist]`
- `state_estimation: Out[Odometry]`
- `odom: Out[PoseStamped]`
- `imu: Out[Imu]`（消息类型：[Imu](file:///mnt/SSD/dimos/dimos/msgs/sensor_msgs/Imu.py#L26-L76)）
- 如果要跑视觉与 VLM：
  - `color_image: Out[Image]`
  - `camera_info: Out[CameraInfo]`
- 如果要跑导航/建图：
  - `pointcloud: Out[PointCloud2]`

---

## 4. 代码结构与新增文件（建议目录）

建议新增目录（与现有 `dimos/robot/unitree/*` 并列）：

```text
dimos/robot/limx/tron1/
├── protocol.py
├── high_level_client.py
├── connection_spec.py
├── connection.py
├── skill_container.py
├── system_prompt.py
├── sim/
│   └── mujoco_connection.py
└── blueprints/
    ├── primitive/
    │   └── tron1_primitive_no_nav.py
    ├── basic/
    │   └── tron1_basic.py
    ├── perceptive/
    │   └── tron1.py
    └── agentic/
        ├── _agentic_skills.py
        └── tron1_agentic.py
```

每个文件职责如下：

- `protocol.py`
  - 把 TRON1 SDK 的“请求/响应/通知”的具体字段编码与解析集中在这里
  - 目标：`connection.py` 不出现任何供应商字段细节
- `high_level_client.py`
  - 维护上层接口通信连接（例如 WebSocket）
  - 目标：提供 `call(request_name, payload)` 与 `register_notify_handler(...)`
- `connection.py`
  - DimOS 的 `Module`，把控制与状态桥接进 DimOS Streams 与 RPC
- `connection_spec.py`
  - `Spec + Protocol`，用于 skill container 注入连接模块（对齐 G1 模式）
- `skill_container.py`
  - `@skill` 封装：move/stand/recover/emergency_stop/base_height/light_effect
- `system_prompt.py`
  - `TRON1_SYSTEM_PROMPT`，描述可用技能与安全原则
- `sim/mujoco_connection.py`
  - 第二阶段：用 `limxsdk` + MuJoCo 做仿真连接
- `blueprints/*`
  - 用 `autoconnect(...)` 组合成可运行的 `tron1-basic / tron1 / tron1-agentic`

---

## 5. TRON1 高层连接模块设计（第一阶段核心）

### 5.1 Spec 设计

建议最小 Spec（用于 SkillContainer 注入）：

- `move(twist, duration)`
- `publish_request(request_name, data)`
- `stand()/sitdown()/recover()/emergency_stop()`

参考 `G1ConnectionSpec`：[connection_spec.py](file:///mnt/SSD/dimos/dimos/robot/unitree/g1/connection_spec.py#L21-L23)

### 5.2 `TRON1Connection` 的模块接口（建议）

模块接口（Streams）：

- `cmd_vel: In[Twist]`
- `state_estimation: Out[Odometry]`
- `odom: Out[PoseStamped]`
- `imu: Out[Imu]`

模块生命周期（`start()`）应完成：

1. 建立高层连接（client.connect）
2. 注册通知回调：`notify_odom` / `notify_imu`（以及可选的其他 notify）
3. 发送 `request_enable_odom` / `request_enable_imu`（建议做成配置可关）
4. 可选发送 `request_stand_mode`
5. 订阅 `cmd_vel` 并转成 `request_twist`

### 5.3 状态桥接细节（不可省略）

- `notify_odom` 需要同时产出：
  - `Odometry` → `state_estimation`
  - `PoseStamped` → `odom`
  - 原因：现有导航底座已经按这两个流名接线：[unitree_g1_primitive_no_nav.py](file:///mnt/SSD/dimos/dimos/robot/unitree/g1/blueprints/primitive/unitree_g1_primitive_no_nav.py#L136-L145)
- `notify_imu` 转成 `dimos.msgs.sensor_msgs.Imu`，至少填 orientation/gyro/acc（covariance 可用零）

### 5.4 `cmd_vel -> request_twist` 的策略

建议实现策略（避免“上来就跑不动”）：

- 在第一次发 `request_twist` 前，确保 `walk_mode` 已启用（调用 `request_walk_mode`）
- `duration` 的处理：
  - 如果 TRON1 `request_twist` 是“持续速度直到下一条命令”，则 `duration` 在 connection 层实现为“定时发 0”
  - 如果 TRON1 `request_twist` 自带持续时间字段，则直接透传

---

## 6. SkillContainer 与 MCP/Agent 接入（第一阶段可交付）

### 6.1 需要实现的技能列表

建议第一版最小技能（映射 SDK 上层接口）：

- `move(x, y, yaw, duration)`
- `stand()`
- `sitdown()`
- `recover()`
- `emergency_stop()`
- `set_base_height(height)`（可选）
- `set_light_effect(effect_name)`（可选）

写法参考 `UnitreeG1SkillContainer`：

- `move`：构造 `Twist` 并调用 `_connection.move(...)`：[skill_container.py](file:///mnt/SSD/dimos/dimos/robot/unitree/g1/skill_container.py#L77-L94)
- 其他动作：通过 `_connection.publish_request(...)` 或专用 RPC

### 6.2 Agentic 蓝图结构（对齐 G1）

G1 的 agentic skills 组合为：

- `McpServer + McpClient + NavigationSkillContainer + SpeakSkill + UnitreeG1SkillContainer`：[g1 _agentic_skills](file:///mnt/SSD/dimos/dimos/robot/unitree/g1/blueprints/agentic/_agentic_skills.py#L18-L32)

TRON1 建议完全对齐：

- `TRON1SkillContainer` 替换 `UnitreeG1SkillContainer`
- `system_prompt` 替换为 `TRON1_SYSTEM_PROMPT`

---

## 7. 蓝图（Blueprint）落地方案

### 7.1 primitive 层（复用导航底座接线）

建议先复制 `unitree_g1_primitive_no_nav.py`，保留所有 transports 绑定（/cmd_vel、/odom、/state_estimation、/lidar、/color_image 等），再逐步替换“数据来源模块”为 TRON1 的传感器模块。

关键点：不要改流名，否则现有导航与感知模块需要跟着改。

参考：G1 primitive 的 transports 显式绑定：[unitree_g1_primitive_no_nav.py](file:///mnt/SSD/dimos/dimos/robot/unitree/g1/blueprints/primitive/unitree_g1_primitive_no_nav.py#L125-L156)

### 7.2 basic 层（primitive + TRON1Connection）

对齐 G1 basic：

- G1：`primitive_no_nav + G1Connection`：[unitree_g1_basic](file:///mnt/SSD/dimos/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_basic.py#L18-L27)
- TRON1：`tron1_primitive_no_nav + TRON1Connection`

### 7.3 perceptive 层（basic + perception_and_memory）

对齐 G1 perceptive：

- G1：`unitree_g1_basic + _perception_and_memory`：[unitree_g1](file:///mnt/SSD/dimos/dimos/robot/unitree/g1/blueprints/perceptive/unitree_g1.py#L18-L27)
- TRON1：`tron1_basic + _perception_and_memory`

### 7.4 agentic 层（perceptive + agentic_skills）

对齐 G1 agentic：

- G1：`unitree_g1 + _agentic_skills`：[unitree_g1_agentic](file:///mnt/SSD/dimos/dimos/robot/unitree/g1/blueprints/agentic/unitree_g1_agentic.py#L18-L25)
- TRON1：`tron1 + _tron1_agentic_skills`

---

## 8. GlobalConfig 与 CLI 参数（必须落地）

### 8.1 为什么要加 GlobalConfig 字段

`dimos run` 会把 `GlobalConfig` 所有字段动态映射成 CLI 参数，并在构建 Blueprint 之前执行 `global_config.update(...)`，确保模块在构造时读到正确值：[dimos run](file:///mnt/SSD/dimos/dimos/robot/cli/dimos.py#L250-L291)

### 8.2 建议新增字段

- `tron1_robot_type`（PF_TRON1A / SF_TRON1A / WF_TRON1A）
- `tron1_connection_mode`（highlevel/mujoco/lowlevel）
- `tron1_ws_url` 或 `tron1_host`
- `tron1_enable_odom`
- `tron1_enable_imu`
- `tron1_auto_stand`
- `tron1_auto_walk_mode`

同时保留当前已有的通用字段（`robot_ip/simulation/replay/robot_model` 等）：[GlobalConfig](file:///mnt/SSD/dimos/dimos/core/global_config.py#L33-L109)

---

## 9. 仿真接入（第二阶段）

### 9.1 MuJoCo 仿真对象模型（已验证）

`tron1-mujoco-sim` 的 `simulator.py` 清晰展示了 lowlevel 对象模型与仿真接口：

- `datatypes.RobotCmd / RobotState / ImuData`
- `robot.subscribeRobotCmdForSim(...)`（仿真接收控制）
- `robot.publishRobotStateForSim(...)`
- `robot.publishImuDataForSim(...)`
- 通过 `ROBOT_TYPE` 加载不同模型与关节列表（PF/SF/WF）  
参考：`simulator.py`（仓库原始文件）

### 9.2 `TRON1MujocoConnection` 的建议实现

实现思路：

- 连接端不再是 WebSocket，而是 `limxsdk.robot.Robot`（仿真模式）
- 输入仍然是 DimOS 的 `cmd_vel`
- 输出仍然发布 `odom/imu`
- 若需要关节级控制，再额外增加 `JointState`/`RobotCmd` 流

---

## 10. 蓝图注册与运行

### 10.1 注册机制（不要手改 all_blueprints.py）

只要你在模块顶层声明 `tron1_basic / tron1 / tron1_agentic` 这种变量（且不以下划线开头），生成测试会扫描并注册到 `all_blueprints.py`：[扫描与生成逻辑](file:///mnt/SSD/dimos/dimos/robot/test_all_blueprints_generation.py#L160-L224)

运行：

```bash
uv run pytest dimos/robot/test_all_blueprints_generation.py
```

### 10.2 目标运行命令

- 最小（先跑运动与状态）：

```bash
dimos run tron1-basic --robot-ip 10.192.1.2
```

- Agentic：

```bash
dimos run tron1-agentic --daemon --robot-ip 10.192.1.2
dimos agent-send "stand up and move forward slowly"
```

---

## 11. 测试与验收（必须可操作）

### 11.1 必须新增的测试

- 协议层测试（不依赖真机）
  - `build_request_twist`、`parse_notify_odom`、`parse_notify_imu`
- 连接模块测试（mock client）
  - `start()` 会 enable odom/imu、注册 notify handler
  - `move()` 会确保 walk mode + 发 twist
- SkillContainer 测试
  - `move()` 构造 Twist 并调用 `_connection.move(...)`
- 蓝图注册测试
  - 运行 `test_all_blueprints_generation.py` 使 CI 可通过

### 11.2 分阶段验收点

- 阶段 1（上层接口）
  - `tron1-basic` 可启动
  - `cmd_vel` 生效（TRON1 能走/停）
  - `notify_odom` 与 `notify_imu` 能稳定发布到 DimOS
  - `emergency_stop/recover` 可用
- 阶段 2（仿真）
  - `TRON1MujocoConnection` 可用
  - 同一套上层蓝图在真机/仿真切换无需改业务模块
- 阶段 3（感知/导航）
  - 相机/雷达模块接入后跑通 perception + mapping + navigation

---

## 12. 实施顺序（建议）

第一阶段（推荐按此顺序实现）：

1. `protocol.py`：先把 request/notify 的编码与解析定下来
2. `high_level_client.py`：通信层 + notify 分发
3. `connection.py`：把 `cmd_vel/odom/imu` 桥接起来
4. `connection_spec.py`：给 SkillContainer 注入做准备
5. `skill_container.py`：封装 move/stand/recover/stop
6. `tron1_basic.py`：primitive + connection
7. 蓝图注册：跑生成测试
8. `tron1_agentic.py`：加 MCP 与 agent skills

第二阶段：

1. `sim/mujoco_connection.py`
2. `tron1` perceptive 蓝图（接视觉/雷达后再做）

