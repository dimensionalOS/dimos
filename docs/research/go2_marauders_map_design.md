# Go2 Marauder's Map — 方案设计与启动指南

> 本文档是项目当前实现的"全景图"。另起一个会话时，先把这份文档读一遍，就有足够上下文去帮我启动 / 调试 Unitree Go2 + 活点地图。
>
> 仓库路径：`/Users/fengzhou/Code/DimOS`
> 当前分支：`danvi/experimental/route-replay-through-SHM`
> 平台：macOS（Apple Silicon），断网现场使用
> 标准启动命令：`uv run dimos --robot-ip 192.168.12.1 --rerun-open native run go2-marauders-map`

---

## 1. 这是什么

一个 dimos blueprint —— `go2-marauders-map` —— 让 Unitree Go2 一边巡逻一边把识别到的人画在一张"哈利波特活点地图"风格的羊皮纸网页上。每个人被映射成一个固定的 HP 角色（哈利、邓布利多、海格…共 100 个，每人 10 句台词，全部本地化，断网可用）。同时支持：

- **网页 teleop**：地图页右下角的 Q/W/E/A/S/D + STOP 按钮，按住 = 移动，松开 = 停
- **网页选人**：点击地图上的角色脚印 → 狗距离跟随
- **Rerun 选人**：在 Rerun 相机画面里点黄框 → 同一套距离跟随（和网页共享内部状态）
- **离线**：socket.io、HP 角色和台词、YOLOE 权重、osnet 权重、mobileclip_blt.ts 全部本地，现场无网络也能跑

---

## 2. 总体架构

```
┌─────────────────┐    ┌──────────────────┐
│ Unitree Go2     │    │ macOS 笔记本     │
│ (WiFi AP host)  │◄───┤ 通过 WebRTC      │
│ 192.168.12.1    │    │ + LCM 控制        │
└─────────────────┘    └──────────────────┘
                              │
                              ├── Rerun viewer  (localhost:9876)
                              └── 活点地图 web  (localhost:7782)
```

每只狗自己广播 WiFi AP（如 `dimair13`，密码 `88888888`）。每台笔记本连一只狗的 AP。多狗场景下，每台笔记本各自看自己的视角，**SSID 名称会自动作为视角标签写在活点地图角落**（用 `networksetup -getairportnetwork` 探测）。

---

## 3. Blueprint 模块组成（dimos 风格的 dataflow 拓扑）

文件：[dimos/apps/marauders_map/blueprint.py](../../dimos/apps/marauders_map/blueprint.py)

```python
autoconnect(
    unitree_go2_basic,            # WebRTC 连接 + 摄像头 + lidar + odom + 基础 viewer
    VoxelGridMapper,              # 世界坐标系下的体素地图
    CostMapper,                   # 2D 占用栅格 → 活点地图羊皮纸"墙"
    _marauders_vis,               # Rerun 视图（相机 + 3D + bbox 叠加）
    YoloeTrackingModule,          # YOLOE 开放词汇检测 + BoT-SORT 跟踪
    BBoxSelectionModule,          # ⭐ 选择的"单一权威"——见第 4 节
    TargetLockModule,             # 目标锁定状态机（locked/searching/lost）
    BBoxDistanceBehaviorModule,   # 距离跟随：根据 bbox 估距，靠近到 0.2m
    MovementManager,              # cmd_vel mux：tele 优先 + nav 兜底
    ReidMapModule,                # ⭐ 活点地图 Web 服务（端口 7782）
)
```

**KeyboardTeleop（pygame 窗口）已剔除** —— 在 macOS 上 pygame 的 Cocoa 驱动从 worker 线程调 UI 会触发 NSException 直接杀进程。我们把 teleop 改成了网页按钮，等价能力但不依赖 pygame。Linux 上想用 pygame 版的话跑 `yoloe-keyboard-teleop` blueprint。

---

## 4. 选择的"单一权威"设计（重点）

**问题**：网页和 Rerun 都能选人。如果两边各自往 `/user_selected_bbox` 写数据，就会 race —— 后到的覆盖先到的，下游 TargetLockModule 看到流断断续续 → 狗在多个目标间反复横跳 → 物理表现就是"原地摇晃"。这个 bug 在 22:37 那次 run 的日志里非常明显（`target_id` 在 97/122/104/276/297 之间每秒切几十次）。

**解法**：让 BBoxSelectionModule 当唯一的"选择权威"，两个入口都只是**写入它的内部状态**，由它统一对外 republish。

### 数据流

```
Rerun 相机画面点击 ─┐                                            ┌─► TargetLockModule
                  ├─► BBoxSelectionModule (_selected_id 单源) ──┤   (locked_bbox)
活点地图角色点击 ─┘   topic: /user_selected_bbox                  └─► BBoxDistanceBehaviorModule
   ▲                                ▲                                (nav_cmd_vel)
   │                                │
   │                                └── 通过 external_selection 输入端口
   │
   └── ReidMapModule 写 topic /web_selected_bbox
       （网页点击 → socket "select"/"deselect" 事件 → 发 Detection2DArray）
```

### 关键端口

| 模块 | 端口 | 方向 | 主题 | 角色 |
|---|---|---|---|---|
| ReidMapModule | `web_selected_bbox` | Out | `/color_image/web_selected_bbox` | 网页"我想选谁"的请求 |
| BBoxSelectionModule | `external_selection` | In | `/color_image/web_selected_bbox` | 接受外部选择请求 |
| BBoxSelectionModule | `clicked_point` | In | viewer click stream | Rerun 像素点击 |
| BBoxSelectionModule | `selected_bbox` | Out | `/color_image/selected_bbox` (→ remap → `/user_selected_bbox`) | 权威输出 |
| TargetLockModule | `selected_bbox` | In | `/color_image/user_selected_bbox` | 消费权威输出 |

**好处**：
- `/user_selected_bbox` 永远只有一个 writer（BBoxSelectionModule），下游永远不会被覆盖
- 网页和 Rerun 是 peer 的输入源，没有谁 override 谁
- 以后加第三个选择面板（手机 app、对讲机等）只要往 `external_selection` 写就行
- BBoxSelectionModule 7 个单元测试全部通过，原有 Rerun 点击行为零修改

---

## 5. Teleop 信号链

网页按住 W → 浏览器 emit socket 事件 → ReidMapModule 发 Twist → MovementManager → GO2Connection → 狗

```
网页按钮 (20Hz 心跳，按下时连续发；松开时发一次零)
  ▼
ReidMapModule.cmd_vel (remap → /tele_cmd_vel)
  ▼
MovementManager.tele_cmd_vel
  ├─► cmd_vel.publish(scaled twist)  → GO2Connection → 狗
  └─► stop_movement.publish(True)    → (remap → /teleop_active) → BBoxDistanceBehaviorModule 暂停跟随
```

- **teleop 优先于 nav**：任何 `tele_cmd_vel` 到来都会让 MovementManager 在 1.0 秒冷却内丢弃 `nav_cmd_vel`
- **松开按钮后约 1 秒**，距离跟随重新接管（如果还有锁定目标）
- 默认线速度 0.5 m/s，角速度 0.8 rad/s，可在 `marauders_map.html` 顶部的 `TELE_LIN` / `TELE_ANG` 改

---

## 6. 启动指令（断网现场）

四步，每步都要做：

```bash
# 1. 连狗的 WiFi（不同狗 SSID 不同）
#    macOS 系统偏好 → WiFi → 选 dimair13 → 密码 88888888
#    连上后狗的 IP 总是 192.168.12.1

# 2. 修 macOS 组播路由（每次切 WiFi 都必须重做）
sudo route delete -net 224.0.0.0/4 2>/dev/null
sudo route add  -net 224.0.0.0/4 -interface lo0
netstat -rn | grep "224.0.0/4"   # 必须显示 lo0，不是 en0；不对则 LCM 会段错误

# 3. 进项目
cd /Users/fengzhou/Code/DimOS

# 4. 启动活点地图 blueprint
uv run dimos --robot-ip 192.168.12.1 --rerun-open native run go2-marauders-map
```

**狗会主动避障**（`global_config.obstacle_avoidance = True`，固件级）。室内有墙/桌椅/人时，狗看到"前方有障碍"会拒绝执行前进指令，腿在原地踏 + 抬头放头反复切换 = 看起来在颤抖。若要验证 teleop / 距离跟随是不是真的能走，需要先关闭避障：

```bash
uv run dimos --robot-ip 192.168.12.1 --rerun-open native run go2-marauders-map \
    -o robot.obstacle_avoidance=false
```

关掉避障后狗 100% 听 cmd_vel，操控者自己负责别撞墙。

---

## 7. 启动后能做什么

1. 浏览器开 [http://localhost:7782/](http://localhost:7782/)
2. Rerun viewer 自动开 (`localhost:9876`)
3. 让狗动起来：右下角 teleop pad 按 W/A/S/D/Q/E
4. 让狗跟人：等画面里出现人 → 点活点地图上的脚印 / 或在 Rerun 相机画面里点黄色 bbox
   - 选中的角色周围会出现脉动虚线圈 + 十字标
   - 顶部出现 "tracking <角色名>" 的小条，旁边有 release 取消
   - 角色掉出 roster（超过 8 秒没出现）会自动取消选择
5. STOP 按钮 = 紧急停 teleop
6. 点角色脚印还会飘一段该角色的 HP 台词（5.5 秒淡出）

---

## 8. 文件索引

| 用途 | 路径 |
|---|---|
| blueprint 入口 | `dimos/apps/marauders_map/blueprint.py` |
| 活点地图后端 (Socket.IO + Starlette) | `dimos/apps/marauders_map/module.py` |
| 活点地图前端 (Canvas + 角色化 + teleop) | `dimos/apps/marauders_map/templates/marauders_map.html` |
| 100 个 HP 角色 × 10 句台词 | `dimos/apps/marauders_map/templates/hp_characters.js` |
| Socket.IO 离线 vendor | `dimos/apps/marauders_map/templates/socketio.min.js` |
| 选择权威 | `dimos/robot/custom/modules/bbox_selection_module.py` |
| 目标锁定 FSM | `dimos/robot/custom/modules/target_lock_module.py` |
| 距离跟随任务 | `dimos/robot/custom/tasks/bbox_distance_behavior_module.py` |
| YOLOE 检测 + BoT-SORT 跟踪 | `dimos/robot/custom/modules/yoloe_tracking_module.py` |
| Twist mux | `dimos/navigation/movement_manager/movement_manager.py` |
| 调研背景 | `docs/research/reid_capability_summary.md` |

---

## 9. 日志在哪 / 怎么诊断

每次 run 会写一份 jsonl 到 `logs/<时间戳>-<blueprint>/main.jsonl`，例如：

```
logs/20260527-225901-go2-marauders-map/main.jsonl
```

CLI 也能看：

```bash
.venv/bin/dimos log -n 200                          # 最近一次 run 最后 200 行
.venv/bin/dimos log -r 20260527-225901-go2-marauders-map -n 200
```

**调"狗为什么不走"的 checklist**：

1. 看 [movement_manager.py](../../dimos/navigation/movement_manager/movement_manager.py)：grep `Ignored out-of-range click` —— 像素坐标被当世界坐标拒掉了
2. 看 [bbox_distance_behavior_module.py](../../dimos/robot/custom/tasks/bbox_distance_behavior_module.py)：grep `task started` / `task ended`
   - 如果反复闪烁（毫秒级 started → ended）→ 选择 race（不应该再发生，单源设计后）
   - 如果完全没有 `task started` → 用户没成功选中任何人
3. 看 [unitree connection.py](../../dimos/robot/unitree/connection.py:262)：obstacle_avoidance 是不是开着
4. cmd_vel_timeout 是 0.2s —— 必须 > 5Hz 才能持续走，网页 teleop 是 20Hz heartbeat 没问题

---

## 10. 已知陷阱

| 陷阱 | 症状 | 对策 |
|---|---|---|
| macOS 切 WiFi 后组播路由跑到 en0 | `_lcm.so` SEGV 启动崩 | `sudo route add -net 224.0.0.0/4 -interface lo0` |
| 默认避障开着 | 狗原地颤抖不前进 | `-o robot.obstacle_avoidance=false` |
| pygame Cocoa 在 worker 线程调 UI | NSException 直接杀进程，狗趴下 | 已剔除 KeyboardTeleop，用网页 teleop 代替 |
| YOLOE 首次需下 `mobileclip_blt.ts`（572MB） | 启动卡一会儿 | 仓库根已经预下好，断网现场不影响 |
| sysctl maxsockbuf 上限 16MB | 启动日志会有 warning | 不影响功能，忽略 |
| WebSocket 连不上 6Gbps 流 | 视频帧丢 | 在 macOS 是已知极限，缩小 Rerun 窗口或降帧率 |

---

## 11. 关于 git / 提交

**重要**：用户明确要求"不要 commit / 我需要控制分支"。所有改动都只改工作区文件，**永远不要 `git commit` / `git push` / `git reset` / `git branch -D`**。

需要看本地改动：

```bash
git status
git diff dimos/apps/marauders_map/blueprint.py
```

---

## 12. 下一步可能要做的事

按优先级：

1. **真正验证 teleop**：关掉 obstacle_avoidance 跑一遍，确认按 W 狗真的走 0.5 m/s
2. **真正验证选人**：识别到人后点活点地图，确认 BBoxDistanceBehaviorModule 日志出现稳定的 `task started target_id=...`（不再闪烁）
3. **多狗多视角**：两台笔记本各连一只狗，活点地图 SSID 标签应该不同 → 可以并排开两个窗口看不同视角
4. **如果有 part-based reID 需求**：参考 [reid_capability_summary.md](reid_capability_summary.md) 的 P3 阶段
