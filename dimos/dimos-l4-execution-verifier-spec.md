# L4 ExecutionVerifier 实现规格

对应 DASL proposal §5.4。P4 交付物，依赖 L2（冷却要打到 PolicyGate 上）和 L5（验证结果进审计）。也是最后做的一层——前面四层拦"不该做的动作"，这层管"该做的动作做没做成"。

借鉴 CaP-X 的 structured feedback 思路，但有一个前提先立住：**机器人上等不起多轮 VLM 投票**。所以验证分两级——确定性谓词为主（零延迟、零成本），VLM 复核只在谓词失败或高风险 skill 上异步触发。VLM 在这层永远不是必需路径。

## 1. 要解决的问题

skill 现在返回自由文本，而且大多是"动作已发起"的确认，不是"动作已完成"的证据。`move(x=0.5, duration=2)` 返回 "Moving at 0.5 m/s" 的时候，轮子可能根本没动（被 L3 刹了、被台阶卡了、电机报错）。Agent 拿着这句话当成成功，继续规划下一步，错误就这么累积起来。

这层要做的事：给每个运动类 skill 一个**可机器检查的成功定义**，执行完对照真实传感器数据核一遍，把"我以为我做了"变成"我确认我做了"。

## 2. SkillResult：先统一返回结构

`skill_result.py` 已经存在但只有雏形，扩展成：

```python
class SkillStatus(Enum):
    SUCCESS = "success"
    FAILED = "failed"
    PARTIAL = "partial"
    INTERRUPTED_BY_SAFETY = "interrupted_by_safety"   # 被 L3 刹停，单独一个值

@dataclass
class SkillResult:
    status: SkillStatus
    code: str                    # 机器可读，如 "odom_mismatch" / "capability_denied"
    detail: str                  # 给人和 Agent 看的描述
    telemetry: dict[str, Any]    # 验证证据：commanded vs actual 之类
```

`INTERRUPTED_BY_SAFETY` 单列一个值而不是塞进 FAILED：被安全层中断不是失败，Agent 对它的正确反应（等人离开再继续）和对真失败（换策略重试）完全不同，必须在类型上分开。

存量 skill 返回 `str` 的，在 `@skill` wrapper 里包一层：`str` → `SkillResult(SUCCESS, "ok", s)`。新 skill 直接返回 `SkillResult`。两种共存，Agent 侧看到的序列化格式统一。

## 3. 谓词注册与执行

```python
Predicate = Callable[[Snapshot, Snapshot], tuple[bool, str]]

class Verifier:
    def register(self, skill_name: str, predicate: Predicate) -> None: ...
    def verify(self, skill_name: str,
               before: Snapshot, after: Snapshot) -> SkillResult | None: ...
```

`Snapshot` 是执行前后各抓一次的状态切片：

```python
@dataclass(frozen=True)
class Snapshot:
    ts: float
    args: dict[str, Any]          # 本次调用的入参
    odom_pose: PoseStamped | None
    semantic_tags: frozenset[str]
    monitor_state: str            # L3 当时的状态，判 INTERRUPTED 用
```

快照数据全部来自现有流：odom 走 tfbuffer，语义标签查 memory2 的 spatial stream，monitor_state 订阅 L3 的输出流。**不为验证新增任何传感器通道**——这层只消费现成数据，这是它能保持轻量的关键。

谓词示例（`dimos/agents/safety/predicates.py`）：

```python
def move_verified(before: Snapshot, after: Snapshot) -> tuple[bool, str]:
    commanded = before.args["x"] * before.args.get("duration", 1.0)
    actual = planar_distance(before.odom_pose, after.odom_pose)
    ratio = actual / abs(commanded) if commanded else 1.0
    ok = ratio >= 0.6
    return ok, f"commanded {commanded:.2f}m, actual {actual:.2f}m ({ratio:.0%})"
```

60% 这个阈值是拍脑袋的起点，故意写宽松——odom 有滑移误差，转身、打滑都会吃掉位移。阈值收紧要靠 replay 数据跑出来，不靠理论。P5 校准时用 replay 集统计实际 ratio 分布再定。

**没有注册谓词的 skill 不验证**，直接透传原结果。谓词是 opt-in 的，逐个 skill 加，不加的不影响。首版只给 `move`、`navigate_with_text`、`tag_location` 三个配谓词。

## 4. 失败处理：冷却与 Agent 反馈

验证失败后的动作分两边：

**给 Agent 的结果改写**：`FAILED` 时 `detail` 里带上 telemetry 证据（"commanded 1.00m, actual 0.12m"）。Agent 拿到具体数字比拿到"failed"有用得多——位移 12% 大概率是被挡住了，它会绕；位移 95% 大概率是地面滑，它可能微调重试。证据质量决定 Agent 的下一步质量。

**给 PolicyGate 的冷却**：同一 skill 连续 2 次验证失败 → 通知 PolicyGate 将该 skill 置入冷却（`cooldown_s`，默认 60s），期间 L2 直接拒绝并注明冷却中。冷却只防"Agent 拿着错误假设蛮力重试"这一种失败模式，时间到自动解除，不需要人工介入。

连续失败计数按 skill 维度记，成功一次清零。验证失败不消耗 L2 的 rate limit 配额——那是调用频次的账，这是执行质量的账，两本账分开。

## 5. VLM 复核（可选，异步）

触发条件（与的关系）：skill 声明了 `verify` 且（谓词失败 或 risk ≥ DYNAMIC）。

流程：

1. 谓词失败后，从 memory2 的图像流取执行前最后一帧和执行后 1s 的一帧；
2. 两帧 + skill 描述发给 VLM（复用现有的 `QwenVlModel`），问一个是非题："指令是 X，第二张图相对第一张是否体现了 X 已完成？"；
3. VLM 说"完成了"但谓词说失败 → 结果标 `PARTIAL` 而不是 FAILED，审计里记录这个分歧。**谓词和 VLM 分歧时不自动判成功**——VLM 只能把失败降级为"存疑"，不能把失败翻成成功。方向不对称是故意的：误报成功的代价比误报失败大得多；
4. 整个复核异步，不阻塞 `tools/call` 返回。结果追加进审计，并在下一条 system 上下文里捎带给 Agent。

VLM 不可用（模型没装、GPU 不够）时整层静默跳过，谓词路径照常工作。这在 proposal 的非目标里已经说过，spec 里再钉一次。

## 6. 与其他层的接口

| 方向 | 接口 | 说明 |
|------|------|------|
| ← `@skill` | `verify="move_verified"` 元数据 | 声明即注册，skill 作者不用碰 Verifier |
| ← L3 | `monitor_state` 流 | 判 `INTERRUPTED_BY_SAFETY`：执行中 L3 进过 HALT 就是这个状态 |
| → L2 | `PolicyGate.set_cooldown(skill, seconds)` | L2 spec 里要补这个方法，一行 map 查询 |
| → L5 | `verify_result` 审计事件 | `{skill, predicate, ok, evidence, vlm_verdict}` |
| → Agent | 改写后的 SkillResult | 见 §4 |

执行时序：`tools/call` 返回前抓 before 快照（在 L2 放行后、RPC 前）和 after 快照（RPC 返回后）。after 快照有个固有问题——`move` 这类 background skill 是"发起即返回"，动作还没做完。处理：声明了 `verify` 的 skill，after 快照延迟到动作预期完成时（`duration + 0.5s` 余量）由 verifier 的定时器抓，验证结果作为后续事件进审计、进 Agent 上下文，而不是塞进本次调用的返回值。**同步返回"已发起"，异步补上"已确认"**——这跟 Agent 现在 "It has started. You will be updated later." 的心智模型是一致的。

## 7. 不变量

1. **I1**：验证永远不阻塞 `tools/call` 的返回路径。
2. **I2**：VLM 只能降级（FAILED→PARTIAL），不能升级（FAILED→SUCCESS）。
3. **I3**：没有谓词的 skill 行为与验证层不存在时完全一样。
4. **I4**：冷却只增不减地保护——冷却期内该 skill 的一切调用被 L2 拒，包括 Agent 换参数名的重试（冷却按 skill 不按参数）。
5. **I5**：谓词抛异常 = 验证不通过（detail 记异常），不崩溃、不透传。

## 8. 测试（`dimos/agents/safety/test_verifier.py`）

| 用例 | 做法 | 期望 |
|------|------|------|
| 位移达标 | mock odom 前进 0.9m / 指令 1.0m | SUCCESS |
| 位移不足 | actual 0.1m | FAILED，telemetry 含 commanded/actual |
| 被 L3 中断 | 执行中 monitor_state=HALT | INTERRUPTED_BY_SAFETY，不进失败计数 |
| 无谓词透传 | 未注册 skill | 原结果不动（I3） |
| 冷却触发 | 同 skill 连续 2 次 FAILED | 第 3 次调用被 L2 拒，reason 含 cooldown |
| 冷却恢复 | mock 时钟 +61s | 放行，计数已清零 |
| 冷却不按参数 | 冷却中换 duration 重试 | 仍被拒（I4） |
| 谓词异常 | 谓词 raise | FAILED + 异常进 detail（I5） |
| VLM 降级 | 谓词 FAILED + VLM "完成" | PARTIAL，审计记录分歧（I2） |
| VLM 不升级 | 谓词 FAILED + VLM "完成" | 不出现 SUCCESS |
| VLM 缺席 | QwenVlModel import 失败 | 谓词路径照常，无异常 |
| 延迟快照 | background skill，duration=2 | after 在 ~2.5s 抓取，结果走异步审计 |

## 9. 验收

1. 用例全过，进 fast tests（VLM 相关用例 mock 掉模型）。
2. `mypy --strict` 过。
3. replay `unitree-go2` 数据集跑一轮：`move` 谓词判定的 SUCCESS/FAILED 与人工抽查一致率 > 90%，不一致样本分析贴在 PR 里。
4. 验证路径全程零 VLM 时，单次 `tools/call` 的延迟增加 < 5ms（before/after 快照就是两次流读取）。
