# M20 V1.1.8.5 OTA 后 drdds PointCloud2 接收问题技术说明

日期：2026-04-25

## 摘要

M20 升级到 Deep Robotics 系统软件 V1.1.8.5 后，我们在 NOS 板上的
PointCloud2 消费进程无法再通过公开/可见的 `libdrdds` 订阅接口从
`rsdriver` 接收激光雷达点云。

这个问题的表现很具体：

- `rsdriver` 仍然在发布有效的 PointCloud2 数据。
- DDS endpoint discovery 成功。
- `DrDDSSubscriber` 和 `DrDDSChannel` 都报告 endpoint 已匹配。
- 但是没有任何 PointCloud2 sample 被传递到 libdrdds callback。
- 使用同一套 Deep Robotics 生成的 IDL 类型，raw FastDDS subscriber
  可以正确接收同一个数据流，前提是禁用 built-in transports，并显式配置
  UDPv4 transport。

这强烈说明：OTA 后的 `rsdriver` / `libdrdds` / FastDDS 栈在接收
PointCloud2 时选择了某种 built-in local DataSharing/shared-memory
传输路径。这个路径可以完成 endpoint discovery/matching，但无法把大的
PointCloud2 sample 交给普通的 `DrDDSSubscriber` 或 `DrDDSChannel`。

我们当前的生产 workaround 是：传感器接收路径绕过 `libdrdds`，直接使用
raw FastDDS + 显式 UDPv4 订阅。

## 我们希望 Deep Robotics 协助确认的问题

1. 请确认 V1.1.8.5 是否修改了 `rsdriver`、`libdrdds`、`drqos.xml`、
   FastDDS transport 默认行为，或者 PointCloud2 的接收要求。
2. 请提供第三方进程通过 `libdrdds` 从 `rsdriver` 接收
   `/LIDAR/POINTS` 和 `/LIDAR/POINTS2` PointCloud2 的官方支持方式。
3. 请确认 PointCloud2 consumer 是否需要禁用 FastDDS built-in transports，
   或者禁用本地 DataSharing/SHM。
4. 请确认 `DrDDSSubscriber` / `DrDDSChannel` 的 `use_shm` 参数在
   V1.1.8.5 后是否足以接收 PointCloud2，还是还需要其他 API 或配置步骤。
5. 如果这是 `libdrdds` 的 bug，请提供修复后的 `libdrdds`，或者提供针对大
   PointCloud2 stream 的已文档化 transport override。

## 系统背景

机器人：Deep Robotics M20

系统软件：V1.1.8.5 OTA

测试板卡：NOS

发布进程：`rsdriver` / `rslidar`

受影响 topic：

- `/LIDAR/POINTS` -> DDS topic `rt/LIDAR/POINTS`
- `/LIDAR/POINTS2` -> DDS topic `rt/LIDAR/POINTS2`

消息类型：

- `sensor_msgs::msg::dds_::PointCloud2_`

当前已安装组件指纹：

```text
e476b1f6b39a372b5be9114341189fdc7f785ae16eada95a9299ae019ce72d8c  /usr/local/lib/libdrdds.so.1.1.7
0c0dec4e719aab80709984947bc250abfd352bef8aa1d5009f085906f89e547b  /usr/local/lib/libfastrtps.so.2.14.2
b36b17aa799b15f31dd60f2fb1862b0e01f360b6bee1d877b94bfe0e155f4f56  /usr/local/lib/libfastcdr.so.2.2.5
cb2dc759da63b41bb614be9d33eabca6d5665e52cd996b53f1b036dc3fb00b4e  /opt/drdds/dr_qos/drqos.xml
f036085a5ed60b1e89309fd1e54be1e985262a400f491f4a5a4040b4c78b589c  /opt/robot/share/node_driver/bin/rslidar
```

当前 `rslidar` 二进制中仍然包含预期的 topic 字符串：

```text
/LIDAR/IMU201
/LIDAR/IMU202
/LIDAR/POINTS
/LIDAR/POINTS2
/LIDAR/STATUS
```

该二进制中也仍然包含 `DrDDSManager::Init` 使用的 `eth0/eth1` 网络字符串。

## OTA 前后对比

我们有一份 V1.1.8.5 OTA 前立即生成的备份。该备份包含：

- `rsdriver_config.yaml`
- 进程/网络状态
- `/dev/shm/fastrtps_*` 状态
- systemd unit / drop-in 状态

该备份没有包含：

- `/usr/local/lib/libdrdds.so*`
- `/opt/drdds/dr_qos/drqos.xml`
- `/opt/robot/share/node_driver/bin/rslidar`

当前恢复后的 `rsdriver_config.yaml` 与 OTA 前备份逐字节一致。因此，这个问题
不能用当前残留的 `rsdriver` 配置差异来解释。

当前 active config 仍然包含我们期望的双雷达配置：

```yaml
send_separately: true
lidar_type: RSAIRY
use_lidar_clock: true
dense_points: true
ts_first_point: false
dds_send_point_cloud_topic: /lidar_points
```

注意：虽然 YAML 中写的是 `/lidar_points`，当前 `rslidar` 二进制实际发布的
是 hardcoded 的 Deep Robotics topic：`/LIDAR/POINTS` 和
`/LIDAR/POINTS2`。对应的 FastDDS DDS topic name 是
`rt/LIDAR/POINTS` 和 `rt/LIDAR/POINTS2`。

## 观察到的故障现象

OTA 后使用 `libdrdds`：

- `DrDDSSubscriber<sensor_msgs::msg::PointCloud2PubSubType>` 可以匹配
  publisher。
- `DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>` 可以匹配
  publisher。
- 两者都收不到 sample。
- callback 从未被调用。

最小化 libdrdds probe 的代表性输出：

```text
[drdds_probe] Init domain=0 network=eth0/eth1 mode=channel topic=/LIDAR/POINTS use_shm=false pub_use_shm=false prefix=rt seconds=20
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] done msgs=0
```

普通 `DrDDSSubscriber` 也有同样现象：

```text
[drdds_probe] Init domain=0 network=eth0/eth1 mode=subscriber topic=/LIDAR/POINTS use_shm=false pub_use_shm=false prefix=rt seconds=15
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] done msgs=0
```

启用 `use_shm=true` 也无法修复：

```text
[drdds_probe] Init domain=0 network=eth0/eth1 mode=subscriber topic=/LIDAR/POINTS use_shm=true pub_use_shm=false prefix=rt seconds=10
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] done msgs=0
```

使用空 topic prefix 无法匹配，确认当前 active DDS topic 是
`rt/LIDAR/POINTS`：

```text
[drdds_probe] Init domain=0 network=eth0/eth1 mode=subscriber topic=/LIDAR/POINTS use_shm=false pub_use_shm=false prefix=<empty> seconds=10
[drdds_probe] status matched=0 pub_matched=-1 msgs=0
[drdds_probe] status matched=0 pub_matched=-1 msgs=0
[drdds_probe] done msgs=0
```

## Raw FastDDS 行为

使用 built-in transports 的 raw FastDDS 也有同样故障：

```text
[fastdds_pc2_probe] domain=0 topic=rt/LIDAR/POINTS type=sensor_msgs::msg::dds_::PointCloud2_ reliability=BEST_EFFORT custom_udp=false keep_builtin=false interface=<all> recv_buffer=8388608 custom_shm=false data_sharing=auto seconds=10
[fastdds_pc2_probe] matched=1
[fastdds_pc2_probe] matched=2
[fastdds_pc2_probe] status matched=2 msgs=0
[fastdds_pc2_probe] status matched=2 msgs=0
[fastdds_pc2_probe] done msgs=0
```

说明：在这个 probe 中，`keep_builtin` 只控制添加 custom transport 时是否保留
built-in transports。当 `custom_udp=false` 时，participant 使用的是 FastDDS
默认 built-in transports。

禁用 built-in transports 并显式配置 UDPv4 后，raw FastDDS 可以正常接收：

```text
[fastdds_pc2_probe] domain=0 topic=rt/LIDAR/POINTS type=sensor_msgs::msg::dds_::PointCloud2_ reliability=BEST_EFFORT custom_udp=true keep_builtin=false interface=<all> recv_buffer=8388608 custom_shm=false data_sharing=auto seconds=10
[fastdds_pc2_probe] matched=1
[fastdds_pc2_probe] matched=2
[fastdds_pc2_probe] msg #1 stamp=1777093036.399991274 width=34641 height=1 point_step=26 bytes=900666
[fastdds_pc2_probe] msg #2 stamp=1777093036.499986172 width=34903 height=1 point_step=26 bytes=907478
[fastdds_pc2_probe] msg #3 stamp=1777093036.599982262 width=34755 height=1 point_step=26 bytes=903630
[fastdds_pc2_probe] msg #4 stamp=1777093036.699985266 width=34786 height=1 point_step=26 bytes=904436
[fastdds_pc2_probe] msg #5 stamp=1777093036.799975157 width=34631 height=1 point_step=26 bytes=900406
[fastdds_pc2_probe] msg #10 stamp=1777093037.299981117 width=34737 height=1 point_step=26 bytes=903162
[fastdds_pc2_probe] msg #20 stamp=1777093038.299987078 width=34902 height=1 point_step=26 bytes=907452
[fastdds_pc2_probe] status matched=2 msgs=20
[fastdds_pc2_probe] msg #30 stamp=1777093039.299987078 width=34728 height=1 point_step=26 bytes=902928
[fastdds_pc2_probe] msg #40 stamp=1777093040.299985170 width=34777 height=1 point_step=26 bytes=904202
[fastdds_pc2_probe] msg #50 stamp=1777093041.299983263 width=34612 height=1 point_step=26 bytes=899912
[fastdds_pc2_probe] msg #60 stamp=1777093042.299984217 width=34733 height=1 point_step=26 bytes=903058
[fastdds_pc2_probe] msg #70 stamp=1777093043.299988270 width=34973 height=1 point_step=26 bytes=909298
[fastdds_pc2_probe] status matched=2 msgs=70
[fastdds_pc2_probe] done msgs=70
```

显式 UDP 使用 RELIABLE reader 也可以正常接收：

```text
[fastdds_pc2_probe] domain=0 topic=rt/LIDAR/POINTS type=sensor_msgs::msg::dds_::PointCloud2_ reliability=RELIABLE custom_udp=true keep_builtin=false interface=<all> recv_buffer=268435456 custom_shm=false data_sharing=auto seconds=15
[fastdds_pc2_probe] matched=1
[fastdds_pc2_probe] matched=2
[fastdds_pc2_probe] msg #1 stamp=1777092933.999992132 width=34788 height=1 point_step=26 bytes=904488
...
[fastdds_pc2_probe] status matched=2 msgs=148
[fastdds_pc2_probe] done msgs=148
```

如果保留 built-in transports，即使同时添加显式 UDP，也会再次失败：

```text
[fastdds_pc2_probe] domain=0 topic=rt/LIDAR/POINTS type=sensor_msgs::msg::dds_::PointCloud2_ reliability=BEST_EFFORT custom_udp=true keep_builtin=true interface=<all> recv_buffer=8388608 custom_shm=false data_sharing=auto seconds=10
[fastdds_pc2_probe] matched=1
[fastdds_pc2_probe] matched=2
[fastdds_pc2_probe] status matched=2 msgs=0
[fastdds_pc2_probe] status matched=2 msgs=0
[fastdds_pc2_probe] done msgs=0
```

这是最关键的观察：同一个 reader、同一个 publisher，只有在禁用 built-in
transports 后才能收到数据。只要保留 built-in transports，就会出现 endpoint
已匹配但没有 sample delivery 的状态。

## Probe 矩阵

下表所有测试都针对同一个 `rsdriver` 实例发布的 live `/LIDAR/POINTS`。

| Probe | Transport/QoS | 结果 |
| --- | --- | --- |
| `drdds_probe --mode channel --use-shm 0` | libdrdds `DrDDSChannel`，prefix `rt` | `matched=2`，20 s 内 `msgs=0` |
| `drdds_probe --mode subscriber --use-shm 0` | libdrdds `DrDDSSubscriber`，prefix `rt` | `matched=2`，15 s 内 `msgs=0` |
| `drdds_probe --mode subscriber --use-shm 1` | libdrdds subscriber，启用 SHM | `matched=2`，10 s 内 `msgs=0` |
| `drdds_probe --mode subscriber --prefix __empty__` | libdrdds，无 `rt` prefix | `matched=0`，`msgs=0` |
| `fastdds_pc2_probe` | raw FastDDS，built-in transports | `matched=2`，`msgs=0` |
| `fastdds_pc2_probe --custom-udp 1 --recv-buffer 8388608` | raw FastDDS，禁用 built-ins，显式 UDP | `matched=2`，10 s 内 `msgs=70` |
| `fastdds_pc2_probe --custom-udp 1 --recv-buffer 268435456` | raw FastDDS，禁用 built-ins，显式 UDP | `matched=2`，15 s 内 `msgs=130` |
| `fastdds_pc2_probe --custom-udp 1 --keep-builtin 1` | raw FastDDS，显式 UDP 但保留 built-ins | `matched=2`，`msgs=0` |
| `fastdds_pc2_probe --custom-udp 1 --reliable 1` | raw FastDDS，显式 UDP，RELIABLE reader | `matched=2`，15 s 内 `msgs=148` |

## 最小代码形态

### 会匹配但收不到数据的 libdrdds subscriber 形态

```cpp
DrDDSManager::Init(0, "eth0/eth1");

DrDDSSubscriber<sensor_msgs::msg::PointCloud2PubSubType> sub(
    callback,
    "/LIDAR/POINTS",
    0,
    "rt",
    false  // use_shm；也测试过 true，仍然收不到数据
);
```

### 可以正确接收的 raw FastDDS 形态

```cpp
using namespace eprosima::fastdds::dds;
using eprosima::fastdds::rtps::UDPv4TransportDescriptor;

DomainParticipantQos pqos;
pqos.name("pc2_raw_probe");
pqos.transport().use_builtin_transports = false;

auto udp = std::make_shared<UDPv4TransportDescriptor>();
udp->maxMessageSize = 65500;
udp->receiveBufferSize = 8 * 1024 * 1024;
udp->sendBufferSize = 8 * 1024 * 1024;
udp->non_blocking_send = false;
pqos.transport().user_transports.push_back(udp);

auto* participant =
    DomainParticipantFactory::get_instance()->create_participant(0, pqos);

TypeSupport pc2_type(new sensor_msgs::msg::PointCloud2PubSubType());
pc2_type->setName("sensor_msgs::msg::dds_::PointCloud2_");
pc2_type.register_type(participant);

auto* topic = participant->create_topic(
    "rt/LIDAR/POINTS",
    pc2_type->getName(),
    TOPIC_QOS_DEFAULT);

DataReaderQos rqos = DATAREADER_QOS_DEFAULT;
rqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
rqos.durability().kind = VOLATILE_DURABILITY_QOS;
rqos.history().kind = KEEP_LAST_HISTORY_QOS;
rqos.history().depth = 8;

auto* reader = subscriber->create_datareader(topic, rqos, &listener);
```

同样的显式 UDP 方案也可以配合 `RELIABLE_RELIABILITY_QOS` 正常工作。

## 已排除的原因

### Topic name 或 prefix 不匹配

已排除。

`rt/LIDAR/POINTS` 可以匹配。空 prefix 无法匹配。当前 `rslidar` 二进制包含
`/LIDAR/POINTS` 和 `/LIDAR/POINTS2`，active DDS topic name 是常规的
`rt/` prefix 形式。

### PointCloud2 类型不匹配

已排除。

raw FastDDS 使用同一套 Deep Robotics 生成的 IDL 类型
`sensor_msgs::msg::dds_::PointCloud2_` 可以解码完整点云。frame 大小正常：

- 前雷达：约 34k 到 35k points，每帧约 0.9 MB
- 后雷达：约 47k 到 48k points，每帧约 1.23 MB

### RELIABLE vs BEST_EFFORT 不匹配

已排除。

raw explicit UDP 使用 BEST_EFFORT 和 RELIABLE reader 都可以接收。

### socket receive buffer 太小

基本排除。

explicit UDP 使用 8 MiB receive buffer 也可以工作。生产环境中我们仍然使用更大
buffer 作为安全余量，但复现出来的失败变量是 built-in transport selection，
不是 buffer size。

### rsdriver YAML 配置漂移

对当前已恢复的系统来说已排除。

当前 `rsdriver_config.yaml` 与 OTA 前备份逐字节一致。

## 当前生产 workaround

我们已经把 bridge 中的 libdrdds receive path 替换为一个 fork 出来的 raw
FastDDS 子进程。该子进程对所有 sensor topics 使用显式 UDPv4，并禁用
built-in transports：

- `rt/LIDAR/POINTS`
- `rt/LIDAR/POINTS2`
- `rt/IMU`
- `rt/LIDAR/IMU201`
- `rt/LIDAR/IMU202`

bridge 随后把这些数据写入我们已有的 POSIX shared-memory buffer，供下游模块
读取。

验证结果：

```text
[drdds_recv] status: lidar_front_matched=2 lidar_front_msgs=241 lidar_rear_matched=1 lidar_rear_msgs=240
[drdds_recv] imu_status: imu_yesense_matched=1 imu_yesense_msgs=4665 imu_airy_front_matched=3 imu_airy_front_msgs=9375 imu_airy_rear_matched=3 imu_airy_rear_msgs=9375
...
[drdds_recv] status: lidar_front_matched=2 lidar_front_msgs=941 lidar_rear_matched=1 lidar_rear_msgs=940
[drdds_recv] imu_status: imu_yesense_matched=1 imu_yesense_msgs=18666 imu_airy_front_matched=3 imu_airy_front_msgs=37377 imu_airy_rear_matched=3 imu_airy_rear_msgs=37377
```

这表示前/后 PointCloud2 均以约 10 Hz 稳定接收，Yesense IMU 约 200 Hz，
Airy IMU 约 400 Hz，这些速率都是 receive callback 侧观察到的。

我们也在 dock-safe no-motion 模式下运行了 native navigation stack。
FAST-LIO 可以正确消费 PC2 path：

```text
[fastlio2] stationary preroll complete after 2.00s - releasing lidar scans to FAST-LIO
[fastlio2] frame #0  path=pc2 ... npts=31646
...
[fastlio2] frame #29  path=pc2 ... npts=34088
[drdds_bridge] lidar #301 pts=34263 bytes=890838
[nav_cmd_pub] #351 x=0.000 y=0.000 yaw=0.000 matched=1
```

## 当前 root-cause hypothesis

V1.1.8.5 后，Deep Robotics 的 `rsdriver` / `libdrdds` 栈会发布可以匹配的
PointCloud2 endpoint，但默认 FastDDS/libdrdds receive path 会选择一个本地
built-in/DataSharing/SHM transport。该 transport 能完成 matching，但不会把大
PointCloud2 sample 交付给普通的 `DrDDSSubscriber` 或 `DrDDSChannel`。

证据如下：

- built-in transports enabled -> `matched=2`，`msgs=0`
- built-in transports disabled 且显式配置 UDPv4 ->
  以 10 Hz 收到完整 PointCloud2 samples
- 添加显式 UDP 但保留 built-ins 仍然失败

这说明问题点更可能是 built-in transport selection 或 local DataSharing/SHM
行为，而不是 topic naming、IDL type support、reliability QoS 或 buffer size。

## 请求 Deep Robotics 调查的问题

请重点检查 V1.1.8.5 中以下问题：

1. `rsdriver` 发布 PointCloud2 时，是否有 DataSharing/SHM-only 的要求，而普通
   `DrDDSSubscriber` 不再满足该要求？
2. `libdrdds` 是否会为本地 reader 强制或优先使用 FastDDS built-in transports，
   导致 endpoint 能匹配但大 sample 无法接收？
3. `DrDDSSubscriber(..., use_shm=true)` 是否应该加入与 `rsdriver` 相同的
   PointCloud2 transport path？
4. Deep Robotics 的生产服务是否使用了某个未文档化 API 来 attach 到
   `rsdriver` 的 shared-memory/DataSharing path？
5. `libdrdds` 是否应该暴露一种方式，让 subscriber 禁用 built-in transports 或
   强制使用 UDP transport？
6. 2026 年 3 月构建的 `libdrdds.so.1.1.7` 是否对 `DrDDSChannel` constructor
   或 transport defaults 做过 ABI/API 变更？

## 建议的修复方向

以下任一方式都可以让第三方集成更容易支持：

1. 让 V1.1.8.5 后的普通 `DrDDSSubscriber<PointCloud2PubSubType>` 能正常接收
   `/LIDAR/POINTS` 和 `/LIDAR/POINTS2`。
2. 文档化 PointCloud2 consumer 所需的 `libdrdds` 初始化方式和 constructor
   flags。
3. 提供一个等价于 `use_builtin_transports=false` + 显式 UDPv4 的 `libdrdds`
   transport 选项。
4. 提供一个 M20 V1.1.8.5 上针对 `rsdriver` 的官方 PointCloud2 subscriber
   示例。
5. 明确第三方 PointCloud2 consumer 是否支持 local SHM/DataSharing；如果支持，
   请说明所需 API 和配置。
