# ARISE SLAM — CATL 项目技术简报

**编写：** dimos/crew/ace
**日期：** 2026-03-29
**状态：** 已在 M20 硬件上验证（2026-03-25）

---

## 1. 算法概述

ARISE SLAM 是 **LOAM**（实时激光雷达里程计与建图）算法的升级实现，由 **卡内基梅隆大学机器人研究所** Ji Zhang 的 Field Robotics Center 实验室开发。

- **基础算法：** LOAM（RSS 2014, J. Zhang & S. Singh）—— 现代 LiDAR SLAM 系统（LIO-SAM/FAST-LIO 系列）的奠基算法。
- **相较 LOAM 的改进：** 基于 GTSAM 的 IMU 预积分、Ceres 求解器用于基于特征的扫描匹配（点到线/点到面优化）、基于位移的去畸变、ROS2 生命周期管理节点
- **许可证：** GPLv3
- **作者：** Ji Zhang（CMU Field Robotics Center）、Guofei Chen（CMU 机器人硕士 2025）

### 架构（3 节点 ROS2 系统）

```
激光雷达 ──► 特征提取 ──► 激光建图 ──► /state_estimation（6自由度位姿）
               ▲              ▲                    │
               │              │                    ▼
IMU ──────► IMU 预积分 ───────┘              /registered_scan（世界坐标系点云）
```

1. **特征提取节点** —— 从原始点云中提取边缘和平面关键点。使用 TBB 并行化 PCA 曲率计算。
2. **激光建图节点** —— 通过 Ceres 求解器进行基于特征的扫描-地图配准（点到线/点到面优化）。维护具有 Octree 索引特征的 3D 体素地图。支持建图和重定位模式。
3. **IMU 预积分节点** —— 基于 GTSAM 因子图的紧耦合 IMU 融合。以约 50Hz 输出融合后的 6 自由度位姿和速度（每 4 次 IMU 回调发布一次）。

---

## 2. 支持的传感器

| 传感器类型 | 型号 | 接口 | 备注 |
|-----------|------|------|------|
| **Livox** | Mid-360 | `livox_ros_driver2::CustomMsg` | 原生支持 |
| **Velodyne** | VLP-16, VLP-32, VLP-64 | `sensor_msgs/PointCloud2` | 标准 LOAM 环形处理 |
| **Ouster** | OS1, OS2 系列 | `sensor_msgs/PointCloud2` | 通过 Ouster 转换路径 |
| **RoboSense** | RSAIRY（192通道） | `sensor_msgs/PointCloud2`（经桥接） | 在 DDS 桥接中重映射为 64 个 ring bin |
| **IMU** | 任意 6 轴 | `sensor_msgs/Imu` | 可配置噪声模型 |

### 已验证配置（M20 机器人）

- **激光雷达：** 2x RoboSense RSAIRY（每个 192 通道，合并，10Hz，~77K 点/帧）
- **IMU：** Yesense（200Hz，6 轴）
- **桥接：** 自定义 POSIX 共享内存桥接（drdds → ROS2 Humble）

---

## 3. 建图能力

### 在线建图模式
- **3D 体素网格：** 21×21×11 块（共 4851 块），动态尺寸 —— 工作空间随体素分辨率设置缩放
- **特征存储：** 每个体素独立存储边缘和平面点云（通过 nanoflann Octree 索引）
- **动态平移：** 当机器人移动超出边界时，地图网格自动平移（滑动窗口）
- **实时输出：** `/registered_scan`（世界坐标系去畸变扫描）+ `/state_estimation`（6 自由度里程计）

### 重定位模式
- **地图加载：** PCD 或 TXT 格式点云地图（地图保存通过外部 PCD 导出完成，未内置于 ARISE 中）
- **启动参数：** `local_mode: true` + `relocalization_map_path: <路径>.pcd`
- **初始位姿：** 可配置 `init_x/y/z/roll/pitch/yaw` 用于引导启动
- 如未找到地图文件，自动回退至建图模式

### 多楼层支持
- **垂直范围：** 11 个体素块深度 —— 足以覆盖多层建筑
- **3D 体素平移：** Z 维度与 X/Y 处理方式相同 —— 机器人在楼层间导航时网格垂直平移
- **垂直盲区：** 可配置 `blindDiskLow/High/Radius` 用于车体自遮挡
- **推荐方案：** 每层楼独立地图文件以获得最佳精度；楼层切换时使用重定位模式

### 局限性
- **尚无回环检测 / PGO：** 漂移随时间累积（尤其是航向约 2-3°/分钟）
- **无在线楼层检测：** 楼层通过点云数据中的间隔自然分离
- **单次建图：** 无法合并不同运行的地图

---

## 4. 性能特征

### M20 硬件实测（ARM64 RK3588，4 核，16GB 内存）

| 指标 | 数值 | 备注 |
|------|------|------|
| **特征提取** | ~50ms/帧 | TBB 并行化 |
| **扫描配准** | 50-150ms/帧 | Ceres 基于特征优化，5 次迭代 |
| **端到端吞吐量** | ~1-2 Hz | 输入 10Hz；ARM64 瓶颈 |
| **位置漂移** | <0.5m/100m | 含 IMU 融合 |
| **航向漂移** | ~2-3°/分钟 | 无回环检测时 |
| **内存（局部地图）** | ~500MB | 4851 个体素块，稀疏存储 |
| **IMU 融合频率** | ~50 Hz | GTSAM 预积分输出（每 4 次 IMU 回调发布一次） |

### x86 桌面/服务器预期性能

| 指标 | 预期值 |
|------|--------|
| **端到端吞吐量** | ~10 Hz（实时） |
| **特征提取** | <10ms/帧 |
| **扫描配准** | <30ms/帧 |

### ARM64 性能优化路径
仅通过配置更改即可使 ARM64 更接近实时：
- 将 `max_range` 从 30m 降至 15m（减少约 30-40% 的远距离点）
- 将 `scan_line` 从 64 降至 32（特征提取工作量减半）
- 设置 `mapping_skip_frame: 2`（每隔一帧处理 → 有效 5Hz）
- 将 `max_iterations` 从 5 降至 3（ICP 时间减少 40%）

---

## 5. 与 CMU 自主导航栈的集成

ARISE SLAM 是 Ji Zhang 完整的 **CMU 自主探索与导航** 系统中的一个模块，该系统包括：

| 模块 | 用途 | 论文 |
|------|------|------|
| **ARISE SLAM** | 激光雷达-惯性里程计 + 建图 | LOAM（RSS 2014） |
| **FAR Planner** | 基于可见性图的路径规划 | IROS 2022 最佳学生论文奖 |
| **TARE Planner** | 分层探索规划 | RSS 2021 最佳论文奖 + 最佳系统论文奖 |
| **Base Autonomy** | 地形可通行性分析、避障、航点跟踪 | — |
| **Local Planner** | 实时避障 + 路径跟踪 | — |

### 输出话题

| 话题 | 类型 | 频率 | 用途 |
|------|------|------|------|
| `/state_estimation` | `nav_msgs/Odometry` | ~50 Hz | 6 自由度位姿 + 速度（主要里程计输出） |
| `/registered_scan` | `sensor_msgs/PointCloud2` | ~10 Hz | 世界坐标系去畸变扫描（供下游规划使用） |
| TF: `map` → `sensor` | 坐标变换 | 持续 | 坐标系链 |

### 下游规划集成

```
ARISE SLAM ──► /state_estimation ──► Local Planner（避障、路径跟踪）
           ──► /registered_scan  ──► Terrain Analysis（可通行性评估）
                                 ──► Terrain Analysis Ext（连通性检查）
                                 ──► VoxelGrid Mapper（占据栅格用于代价建图）
```

---

## 6. 配置参数

### 关键可调参数

| 参数 | 范围 | M20 值 | 影响 |
|------|------|--------|------|
| `scan_line` | 4, 16, 32, 64 | 64 | 扫描线数 —— 影响特征提取速度 |
| `sensor` | livox/velodyne/ouster | velodyne | 传感器处理管线 |
| `min_range` / `max_range` | 米 | 0.5 / 30.0 | 点云过滤范围 |
| `mapping_line_resolution` | 米 | 0.1 | 边缘特征体素尺寸 |
| `mapping_plane_resolution` | 米 | 0.2 | 平面特征体素尺寸 |
| `max_iterations` | 1-10 | 5 | ICP 收敛迭代次数 |
| `max_surface_features` | ≥100 | 2000 | 平面关键点下采样上限 |
| `acc_n` / `gyr_n` | 噪声标准差 | 0.4 / 0.002 | IMU 测量噪声 |
| `acc_w` / `gyr_w` | 偏置随机游走 | 0.006 / 0.00004 | IMU 偏置随机游走 |

### M20 机器人配置

- **车体尺寸：** 820 × 430 × 570 mm
- **激光雷达高度：** 47cm（敏捷姿态）
- **盲区：** 前后 ±0.5m，左右 ±0.3m（车体遮挡）
- **最大自主速度：** 0.3 m/s（室内保守设置）
- **机器人宽度参数：** 0.45m（430mm + 每侧 10mm 余量）

---

## 7. 部署架构（M20）

```
NOS 主机（RK3588 ARM64）
├── drdds_recv（主机进程） ──► POSIX 共享内存环形缓冲区
│   └── 通过 drdds（FastDDS 2.14）订阅 rsdriver + yesense
│
├── Docker 容器（dimos-nav）
│   ├── ros2_pub ──► /bridge/LIDAR_POINTS + /bridge/IMU（ROS2 Humble, FastDDS 2.6）
│   ├── ARISE SLAM（3 个节点） ──► /state_estimation + /registered_scan
│   ├── 局部规划器 + 地形分析 ──► /way_point 命令
│   └── dimos RPC 服务器（桥接至主机端 dimos）
│
└── dimos（原生 Python 3.10）
    ├── M20Connection（通过 UDP 控制机器人）
    ├── VoxelGridMapper（占据栅格）
    ├── CostMapper（地形代价分析）
    ├── ROSNav（通过 Docker RPC 桥接 DDS 话题）
    └── WebSocket 可视化（端口 7779）
```

---

## 8. 相关工作与背景

### CMU 自主导航栈部署平台

同一导航栈（含 ARISE SLAM）已部署在以下平台：
- **Diablo**（轮腿机器人）—— [github.com/jizhang-cmu/autonomy_stack_diablo_setup](https://github.com/jizhang-cmu/autonomy_stack_diablo_setup)
- **麦克纳姆轮平台** —— [github.com/jizhang-cmu/autonomy_stack_mecanum_wheel_platform](https://github.com/jizhang-cmu/autonomy_stack_mecanum_wheel_platform)
- **Unitree Go2**（使用 Point-LIO 而非 ARISE）—— [github.com/jizhang-cmu/autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2)

### 仿真环境

CMU 提供基于 Unity 的仿真环境，包括 **多层停车场**（140m × 130m，5 层），用于测试多楼层环境下的自主探索。

网站：[cmu-exploration.com](https://www.cmu-exploration.com/)

### 关键参考文献

1. **LOAM：** Zhang, J. & Singh, S. "LOAM: Lidar Odometry and Mapping in Real-time." RSS 2014. [链接](https://www.ri.cmu.edu/publications/loam-lidar-odometry-and-mapping-in-real-time/)
2. **FAR Planner：** 基于可见性图的自主导航。IROS 2022 最佳学生论文奖。
3. **TARE Planner：** 分层探索规划。RSS 2021 最佳论文奖 + 最佳系统论文奖。
4. **Ji Zhang 实验室：** [frc.ri.cmu.edu/~zhangji/](https://frc.ri.cmu.edu/~zhangji/)

---

## 9. CATL 项目总结

**ARISE SLAM 提供：**
- 基于 LOAM 的实时激光雷达-惯性 SLAM（现代 LiDAR SLAM 的奠基算法）
- 通过 GTSAM 进行紧耦合 IMU 融合，实现稳健的 6 自由度位姿估计
- 多传感器支持（Livox、Velodyne、Ouster、RoboSense）
- 地图加载及重定位模式，支持重复作业
- 动态尺寸 3D 体素工作空间（满足多层工厂需求）
- 与 CMU 成熟的自主导航栈集成（FAR Planner、TARE Explorer、地形分析）
- 已在 M20 四足机器人上验证，配备双 192 通道 RoboSense RSAIRY 激光雷达

**CATL 工厂部署优势：**
- 30m 室内有效距离足以覆盖工厂车间建图
- 重定位模式使机器人能够从预建地图恢复导航，支持跨班次作业
- 0.3 m/s 保守速度确保在设备和人员周围安全运行
- 地形分析 + 代价建图实现智能避障路径选择
- WebSocket 可视化提供机器人状态实时监控

**当前局限性（含解决路径）：**
- **尚无回环检测** —— 长时间连续运行（>30 分钟）可能累积漂移，需要刷新地图。**但是：** CMU 自主导航栈已包含 **位姿图优化（PGO）模块**，具备基于 GTSAM iSAM2 的回环检测和轨迹校正功能。该模块目前在 Jeff 的 `rosnav4` 分支中已与 FAST-LIO 对接，正在集成到 ARISE 管线中。接入后，系统将通过 ICP 扫描匹配自动检测重访位置，并校正整条轨迹的累积漂移。这项工作已列入近期开发计划。
- ARM64 吞吐量为 ~1-2 Hz（已确定优化路径；x86 可达完整 10Hz）
- 多楼层切换需要每层独立地图文件（无自动楼层检测）
