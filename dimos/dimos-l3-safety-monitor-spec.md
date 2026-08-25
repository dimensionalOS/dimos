# L3 SafetyMonitorModule 实现规格

对应 DASL proposal §5.3。P3 交付物，依赖 L2（CAUTION 状态要动态收紧 PolicyGate 的 bounds）。

这是五层里唯一"持续运行"的组件——其他层都是调用时触发，这层是机器人活着的每一秒都在做判断。也是唯一一个**误报和漏报都会出事**的层：漏报撞人，误报让机器人变成原地罚站的废铁。阈值和状态机的每个数字都要有依据、可调、可灰度。

## 1. 数据源选择（先说清楚为什么）

候选输入有三个，结论是用 (a) 为主、(c) 兜底，明确不用 (b)：

**(a) `Detection3DModule.detection_stream_3d`（主输入）**。2D 检测与点云 0.25s 容差对齐后的输出，`ImageDetections3DPC`，每个检出目标带点云实测位姿。按 `class == "person"` 过滤，取距离最小值。

**(b) `PersonTracker.target`（不用）**。这个流是为跟随技能设计的，有两个安全上不能接受的问题：深度是 `assumed_depth=2.0` 写死的假设值（人站 0.5m 也报 2.0m，HALT 永远不触发）；且只发布视野里最大框的单个目标，第二个人直接丢了。spec 里写死这条，防止后来人看它名字合适就接上。

**(c) costmap（冗余触发）**。激光雷达驱动，确定性，频率高，不依赖 ML 模型。它分不清人和箱子，但"近距离有东西"这个信号本身就该减速。职责分工：

- 人 → (a) 触发 CAUTION/HALT；
- 任何东西（包括人，也包括模型漏检的障碍物）→ (c) 触发 HALT。

(c) 同时覆盖了 (a) 的漏检场景。ML 检测在人背对、遮挡、光照差时会丢目标，costmap 不会。

## 2. 模块定义

```python
class SafetyMonitorModule(Module):
    detections_3d: In[ImageDetections3DPC]   # (a)，autoconnect 按类型连线
    costmap: In[Costmap]                     # (c)，mapping 模块的现有输出
    # 速度下发与 TF 走 Spec RPC 注入（AGENTS.md 的 RPC Wiring 模式），不走流
```

`SafetyMonitorModule.blueprint` 加进各 agentic 蓝图时 `autoconnect` 自动接线。注意 `detections_3d` 的流名要和 `Detection3DModule` 的输出对齐，对不上就在蓝图里 `.remappings()`，AGENTS.md 里有现成模式。

**TF 约定**：所有距离计算在 `base_link` 系下进行。每帧处理时取一次 TF；取不到 → 按 §4 watchdog 的"感知失效"处理，不是按"无人"处理。看不见不等于安全。

## 3. 状态机

```
        人 < CAUTION_M 或 costmap 近距障碍
CLEAR ──────────────────────────────► CAUTION
  ▲                                     │ 人 < HALT_M 或 costmap 极近障碍
  │ 净空持续 CLEAR_DELAY_S              ▼
  └─────────────── HALT ◄───────────────┘
        （HALT 也可由 CLEAR 直接跳入：障碍瞬间出现）
```

| 状态 | 动作 |
|------|------|
| CLEAR | 无干预 |
| CAUTION | 通知 PolicyGate 动态收紧：所有 MOTION/DYNAMIC skill 的速度类 bounds 乘以 `caution_speed_factor`（默认 0.3） |
| HALT | 抢占 `CAP_MOVEMENT` + 直接下发零速（绕过 Agent，走控制层 RPC） |

三个距离参数都是 `GlobalConfig` 可配，按机型给默认值：

| 参数 | Go2 默认 | 说明 |
|------|---------|------|
| `caution_distance_m` | 2.0 | 开始进入 CAUTION 的人距 |
| `halt_distance_m` | 0.8 | 触发 HALT 的人距 |
| `costmap_halt_m` | 0.5 | costmap 障碍触发 HALT 的距离（比人距小，因为不区分对象） |
| `clear_delay_s` | 3.0 | 净空多久后自动恢复 |
| `caution_speed_factor` | 0.3 | CAUTION 下速度上限系数 |

**HALT 的恢复**：只有 CLEAR_DELAY_S 净空后自动回 CLEAR。CAUTION → CLEAR 同理。**没有"人工确认才能恢复"的模式**——这是提案里讨论过的点，结论是：恢复是保守方向（从不安全到安全），自动做；如果场景要求人工确认恢复，由策略 YAML 按 zone 配，不作为默认。

**状态迟滞（hysteresis）**：距离阈值进出用不同值（进 CAUTION 2.0m，出 CAUTION 2.3m），不然人站在边界上机器人会减速-恢复-减速地抖。所有阈值都是"进严出宽"，差值取 15%。

## 4. Watchdog：感知流失效处理（这层最容易被漏掉的部分）

检测链是 ML 模型 + 相机 + 时间对齐窗口，任何一环死了流就静默停。机器人在"失明"状态下继续执行 Agent 的运动指令是不可接受的。

规则：

- 超过 `stream_stale_ms`（默认 500ms）没有新的 `detections_3d` 帧 → 强制 CAUTION（不管上一帧有没有人）；
- 超过 `stream_dead_ms`（默认 2000ms）→ 强制 HALT；
- 流恢复后按正常状态机重新评估，不立即 CLEAR——重新积累 CLEAR_DELAY_S；
- costmap 流有独立 watchdog，同样的两级规则。两路 watchdog 独立计时，任何一路失效都按上面降级。

注意这里有个真实的权衡：watchdog 会让"相机被遮挡"也触发 HALT。接受。传感器被挡时停下来是正确行为，错的不是 watchdog，是继续跑。

## 5. 抢占与下发

**抢占 capability**：以保留持有者名 `__safety_monitor__` 调 `CapabilityRegistry.acquire([CAP_MOVEMENT], ...)`。需要给 `mcp_server.py` 的 `_can_wait` 加一条：holder 是 `__safety_monitor__` 时恒返回 False——Agent 侧立即得到拒绝"capability held by safety monitor"，而不是干等 30s 超时。Agent 收到的拒绝文案要包含状态（"halted: person at 0.6m"），它才能向用户解释，而不是傻重试。

**零速下发**：通过 Spec 注入运动控制模块（Go2 是 `UnitreeSkillContainer` 后面的运动控制 RPC），直接调 `stop_movement` 等价的底层接口。**故意不经过 skill/MCP 层**——那是 Agent 的路，安全层要有自己的、Agent 无法干扰的路。

**CAUTION 的限速实现**：不直接改速度指令（那要插到控制环里，太重），而是通过 L2 的动态 bounds——`PolicyGate` 暴露一个 `set_dynamic_factor(capability, factor)`，监视器在状态切换时调用。Agent 请求 0.5 m/s，CAUTION 下 L2 按 0.15 m/s 的上限拒绝并要求降速。拒绝而不是裁剪的理由见 L2 spec I3。

## 6. 与审计/评测的接口

- 每次状态迁移写一条 `monitor_transition` 审计记录（L5 已定义事件类型）：`{from, to, trigger: "person_distance"|"costmap"|"watchdog", min_distance, snapshot}`；
- 状态本身发布为一个 `Out[MonitorState]` 流，rerun 可视化和评测套件订阅它。评测（proposal §6 的 Human Proximity 场景）就是统计这个流加上 sim 的 ground truth 碰撞事件。

## 7. 不做的事

- **不做轨迹预测**（"人 1 秒后会走到哪"）。TTC 预测听起来好，但对四足这个速度量级，固定距离阈值 + 迟滞已经够，预测模型引入的误报会把机器人变得没法用。以后要加，作为独立的增强 PR；
- **不做 VLM 语义增强**（"这人是否注意到机器人"）。proposal 里提过作为可选，这里明确推迟——它需要一路稳定的 VLM 调用预算，且只能让策略更保守，首版不带；
- **不管多机器人**。两台机器人互为"障碍物"由 costmap 通道自然覆盖，专有的多机避让留给后续 proposal。

## 8. 不变量

1. **I1**：HALT 的零速下发路径不经过 LLM、不经过 MCP、不经过 skill 层。
2. **I2**：任何一路感知失效，状态只能向更保守方向迁移（CLEAR→CAUTION→HALT），永不反向。
3. **I3**：状态迁移必写审计，没有静默迁移。
4. **I4**：`__safety_monitor__` 持有的 capability 只能由本模块释放（token 校验，现有 `release_by_token` 语义天然满足）。
5. **I5**：监视器模块自身崩溃 → worker 管理器现有的重启机制拉起；拉起后初始状态按 watchdog 规则从"流刚恢复"开始，即先 CAUTION，重新积累净空时间。崩溃期间没人踩刹车是已知残余风险，写在文档里，不假装解决了——这也是硬件急停必须独立存在的原因。

## 9. 测试（`dimos/agents/safety/test_monitor.py` + sim 场景）

单测（mock 输入流）：

| 用例 | 做法 | 期望 |
|------|------|------|
| 距离分级 | 注入人距 2.5 / 1.5 / 0.5m | CLEAR / CAUTION / HALT |
| 直接进入 HALT | CLEAR 下注入 0.5m | 立即 HALT，无 CAUTION 中间态 |
| 迟滞 | 人距在 2.0m 边界上下抖动 | 状态不抖动 |
| 净空恢复 | HALT 后无人，mock 时钟 +2s / +4s | 2s 仍 HALT，4s 回 CLEAR |
| 主输入断流 | 停止注入，+600ms / +2100ms | CAUTION / HALT（I2） |
| costmap 冗余 | 无 person 检出，costmap 注入 0.4m 障碍 | HALT |
| TF 缺失 | tfbuffer.get 抛异常 | 按感知失效降级（不是 CLEAR） |
| CAUTION 限速 | CAUTION 下 PolicyGate 查 move(x=0.5) | 拒绝并提示上限 0.15 |
| 抢占不可等 | HALT 中 Agent 调 move | 立即拒绝，文案含距离，不阻塞 30s |
| 恢复不记忆 | HALT→CLEAR 后再来人 | 正常重新进入状态机 |

sim 集成（MuJoCo，self-hosted CI）：

| 场景 | 通过标准 |
|------|---------|
| 行人横穿机器人正前方 | 零碰撞；最小距离 ≥ 0.5m |
| 行人静止挡路 | 机器人在 0.8m 外停住，净空后自主继续任务 |
| 相机被遮挡 3s | 2s 内进入 HALT |

## 10. 验收

1. 单测 + sim 场景全过；sim 场景进 self-hosted CI，不进 fast tests（它们慢）。
2. `mypy --strict` 过。
3. 端到端延迟（图像帧进 → HALT 零速发出）在 Go2 replay 数据上实测 p99 < 500ms，报告贴在 PR 里。
4. 默认配置下 replay `go2_bigoffice` 数据集：HALT 触发次数人工抽查合理（没有对着空地乱刹），误报样本附在 PR 讨论里。
