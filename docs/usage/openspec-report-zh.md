# OpenSpec 使用汇报

## 一句话结论

OpenSpec 的价值不是“多写文档”，而是把从想法到代码的过程产品化：先用规格约束需求和设计，再让人或 AI 按同一份任务清单实现，最后通过验证和归档形成可追踪闭环。

在 DimOS 这种机器人系统仓库里，改动经常跨模块、蓝图、agent skills、CI 和真实硬件路径。OpenSpec 可以帮助团队在写代码前先对齐目标、边界、验收标准和风险，减少需求漂移、隐藏假设和 AI 直接开写带来的返工。

## 当前仓库状态

- 当前 OpenSpec schema：`spec-driven`
- 当前 active changes：`0`
- 配置文件：`openspec/config.yaml`
- 推荐与现有流程结合：OpenSpec change → 实现 → `bash scripts/verify.sh` → commit / PR → archive

## 为什么需要 OpenSpec

### 1. 解决需求对齐问题

传统开发里，需求经常散落在聊天、issue、commit 和 PR 描述里。等到代码写完再评审时，大家才发现对范围、边界或验收标准理解不一致。

OpenSpec 把这些内容前置到 change artifacts 里：

- `proposal.md` 说明为什么做、做什么、不做什么。
- `design.md` 记录方案、权衡、风险和迁移思路。
- `specs/` 沉淀能力级要求和验收标准。
- `tasks.md` 把方案拆成可执行步骤。

这样评审可以先评审“该不该这么做”，再评审“代码写得对不对”。

### 2. 解决 AI 协作的上下文问题

AI agent 如果只看局部代码，很容易按当前文件猜意图。对于 DimOS 这种模块化系统，真正重要的上下文可能在蓝图、transport、agent skill、系统 prompt、测试规则或硬件约束里。

OpenSpec 给 agent 一个明确入口：实现前先读 proposal、design、spec 和 tasks。这样 agent 不是凭聊天记忆写代码，而是按团队确认过的规格执行。

### 3. 解决变更追踪问题

很多工程决策不是代码本身能完全表达的，例如：

- 为什么选这个方案而不是另一个方案。
- 哪些行为是明确不做的。
- 哪些风险需要人工或硬件验证。
- 后续如何扩展。

OpenSpec change 完成后可以归档，保留当时的决策记录。后续回看时，不只能看到“代码变了什么”，还能看到“为什么这样变”。

## 日常使用流程

### 1. Explore：先把问题想清楚

适合场景：

- 需求还模糊。
- 方案有多种可能。
- 需要先读代码理解现状。
- 改动可能影响多个模块或公共接口。

常用命令：

```bash
openspec list --json
```

目标不是马上写代码，而是澄清问题、约束范围、识别风险。如果讨论成熟，就进入 proposal。

### 2. Propose：创建一个 change

适合场景：

- 已经知道要做什么。
- 需要让人或 AI 后续按同一份规格实现。
- 变更需要评审范围、设计或验收标准。

常用命令：

```bash
openspec new change "<change-name>"
openspec status --change "<change-name>" --json
```

建议 change name 使用 kebab-case，例如：

```bash
openspec new change "add-go2-mcp-skill"
```

一个 change 最好对应一个清晰目标，并且自然落到一个小 PR。不要把多个不相关目标塞进同一个 change。

### 3. Apply：按 tasks 实现

实现前应该读取 OpenSpec 给出的上下文文件，而不是直接从代码开始猜。

常用命令：

```bash
openspec instructions apply --change "<change-name>" --json
```

实现时的原则：

- 按 `tasks.md` 逐项推进。
- 完成一个任务就勾选一个任务。
- 如果实现中发现设计假设不成立，先更新 OpenSpec artifacts，再继续实现。
- 不要为了让代码通过而削弱 spec、测试或验证流程。

### 4. Verify：与 DimOS 验证流程对齐

DimOS 当前推荐的本地验证入口是：

```bash
bash scripts/verify.sh
```

如果完整验证太慢或被环境阻塞，应运行最窄但有意义的检查，并在最终汇报或 PR 中说明没有覆盖的部分。

### 5. Archive：完成后归档

当实现、验证和 PR 流程完成后，应归档 change，避免 active changes 长期堆积。

归档的意义：

- 保留设计和需求记录。
- 让当前工作区保持干净。
- 方便后续追踪历史决策。

## Artifacts 分工

| Artifact | 作用 |
| --- | --- |
| `proposal.md` | 回答为什么做、做什么、不做什么，是对齐范围的入口。 |
| `design.md` | 记录方案、权衡、风险和迁移思路，避免只在聊天里口头决定。 |
| `specs/` | 沉淀能力级要求和验收标准，是后续回归、评审、归档的依据。 |
| `tasks.md` | 把方案拆成可执行步骤，implementation 阶段逐项勾掉。 |

## 最佳使用模式

### 适合使用 OpenSpec 的改动

- 跨多个模块、蓝图或系统边界的改动。
- 用户可见行为变化。
- API、协议、配置或数据结构变化。
- 机器人运动控制、硬件路径或安全相关改动。
- CI、发布、验证流程变化。
- 需要多人评审方案的功能。
- AI agent 需要较长上下文才能正确实现的任务。

### 可以跳过 OpenSpec 的改动

- 拼写修复。
- 注释补充。
- 单文件小重构。
- 格式化调整。
- 已经完全明确且风险很低的局部修复。

### 推荐粒度

最佳粒度是：

> 一个 OpenSpec change 对应一个清晰工程目标，最好能自然映射到一个小 PR。

不推荐：

- 一个 change 包含多个不相关功能。
- 一个 change 覆盖太多模块，导致 proposal 和 tasks 失去清晰边界。
- 只为了走流程而给很小的改动创建复杂规格。

### 与 AI agent 的最佳配合

推荐模式：

1. 先用 Explore 澄清需求和方案。
2. 再用 Propose 生成 change artifacts。
3. 实现时让 agent 先读取 artifacts。
4. agent 按 tasks 实现，并及时更新任务状态。
5. 如果实现中发现需求或设计问题，先修改 OpenSpec，再继续写代码。
6. 完成后运行验证、提交 PR、归档 change。

这样可以把 agent 从“凭上下文猜测”变成“按规格执行”。

## 建议在 DimOS 中采用的标准流程

### 变更前

对于中等以上复杂度的改动，先创建 OpenSpec change。

`proposal.md` 应写清：

- 背景和动机。
- 目标。
- 非目标。
- 影响范围。
- 风险和未知点。

`design.md` 应写清：

- 方案结构。
- 关键取舍。
- 与现有模块、蓝图、transport、agent skill 或 CLI 的关系。
- 测试和验证思路。

`tasks.md` 应拆到可以执行和检查的粒度。

### 实现中

实现时遵循：

- 先读 OpenSpec artifacts，再改代码。
- 保持改动范围和 proposal 一致。
- 每完成一个任务就更新 `tasks.md`。
- 如果设计需要变化，先更新 design 或 specs。
- 避免把无关重构混入当前 change。

### 合并前

合并或提交 PR 前：

- 运行 `bash scripts/verify.sh`。
- 如果验证未完整运行，说明原因和替代检查。
- PR 描述中概括 OpenSpec change 的背景、实现范围和风险。
- 确认没有提交 secrets、`.env`、本地缓存或无关生成文件。

### 完成后

完成后归档 change，保留历史记录。

## 汇报口径

可以这样向团队介绍：

> OpenSpec 是我们给复杂变更加上的“工程规格层”。它不替代代码、不替代 PR，而是在代码和 PR 之前先把需求、设计、验收和任务拆清楚。这样人和 AI 都能按同一份规格工作，评审先看方向和边界，再看实现细节。对于 DimOS 这种机器人系统仓库，它能显著降低跨模块改动、硬件风险和 AI 协作中的上下文不一致问题。

## 推荐落地规则

1. 小改动可以直接做，不强制 OpenSpec。
2. 中等以上复杂度、跨模块、影响用户行为或硬件路径的改动，先建 OpenSpec change。
3. 每个 change 保持单一目标，尽量对应一个小 PR。
4. 实现前必须读 artifacts，不能只靠聊天上下文。
5. 实现中发现假设不成立，先更新 OpenSpec，再继续写代码。
6. 完成后运行验证并归档 change。

