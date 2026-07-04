# Go2 Agent 架构 Review 说明

日期: 2026-07-05
分支: `refactor/go2-architecture-layers`
合并上游前的本地 HEAD: `646e7ec5`
已合并的上游目标: `upstream/main` at `6e813a72`
共同祖先: `1f544d05`

这份文档是当前分支完整 review 的中文入口。它把上游同步内容和本地新增功能分开说明，并重点解释每个新增功能的实现逻辑、数据流、存储位置、判断主体、风险点和验证情况。

## 当前 Git 状态

合并前，本分支有 16 个本地提交不在 upstream，上游 `main` 有 160 个提交不在本分支。用 merge base 精确比较后，真正与本地改动重叠的文件有 15 个，实际产生冲突的文件有 7 个:

- `dimos/agents/mcp/mcp_server.py`
- `dimos/agents/mcp/test_mcp_server.py`
- `dimos/perception/spatial_perception.py`
- `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic.py`
- `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_temporal_memory.py`
- `dimos/robot/unitree/go2/blueprints/smart/unitree_go2_spatial.py`
- `dimos/robot/unitree/go2/test_connection.py`

冲突解析原则:

- 保留上游基础设施更新，尤其是 Zenoh transport、MCP capability gating、Unitree AES key 转发、文档和打包更新。
- 保留本地 Go2 分层架构作为 Go2 agentic blueprint 的权威 wiring。
- 使用上游新的 perception 模块路径，同时保留本地基于 `STATE_DIR` 的 spatial memory 默认持久化路径。
- 两边新增的测试如果覆盖不同风险点，则合并保留，而不是二选一删除。

环境注意事项:

- 上游现在在 macOS 上默认使用 `zenoh` transport。这个后端需要 `eclipse-zenoh`，本地 `.venv` 已安装 `eclipse-zenoh==1.9.0`。
- 当前机器已经有一个 live `dimos` 进程监听 MCP 端口 `9990`。跑 MCP 测试时要换端口，例如 `MCP_PORT=9991`，否则测试客户端会连到 live 服务。
- `git diff --cached --check -- ':(exclude)*.patch'` 已通过。完整 `git diff --check` 会报上游 `.patch` 文件内的 trailing whitespace，这些是 patch 文件的 context 行，应单独处理，不能随意改动以免破坏补丁语义。

## Review 粒度标准

review 每个本地功能时，建议都按下面这些问题看，而不是只看文件名是否合理:

- 入口: 第一个对外入口是 blueprint、module、MCP skill、RPC method，还是 helper 函数？
- 输入: 它读取哪些 JSON 字符串、Spec provider、memory provider 或 runtime state？
- 输出: 它返回哪种 `SkillResult`、dict schema、Spec return value、file artifact 或 dashboard payload？
- 判断主体: 判断是确定性代码做的，LLM 做的，小型在线统计模型做的，还是人类 reviewer 做的？
- 状态: 它是无状态、只在内存、写 JSON、写 Git，还是写已有 RAG/vector database？
- 写边界: 哪个方法会写入？哪些方法是纯 read-only？
- 失败形态: 缺数据时返回 `unavailable`、`uncertain`、clarifying question、failed `SkillResult`，还是 proposal artifact？
- 安全边界: 它能执行机器人运动，还是只给上层建议？
- Review artifact: 哪个 schema 是未来工具要依赖的稳定接口？
- 测试: 哪个测试证明主数据流？哪个测试证明失败路径？

这个分支最重要的区分是三件事:

- Context selection: LLM 看到上下文前，由确定性代码做过滤和摘要。
- Agent judgment: LLM 读取已选上下文和工具后决定下一步。
- Self-evolution artifact: 产出可 review 的 JSON event/proposal，不自动改代码。

## 架构图

![DimOS Go2 Agent 自进化架构](2026-07-05-go2-agent-architecture.png)

## 接口输入/输出速查

review API 边界时优先看这一节。这里列的是本地 Go2 架构改动新增或改动过的稳定 public surface；没有跨 module 边界消费的内部 helper 不列入。

| 接口 | 输入 | 输出 |
| --- | --- | --- |
| Go2 blueprint factories (`unitree_go2_agentic`、`unitree_go2_spatial`、`unitree_go2_temporal_memory`) | CLI blueprint name，以及 DimOS 已解析好的 `GlobalConfig`。lazy layer import 本身没有 runtime 入参。 | `autoconnect()` 组合出的 DimOS `Blueprint`。输出的是 module wiring、stream wiring 和 Spec injection path，不是面向用户的 JSON。 |
| `RobotBodyStateSpec.get_robot_body_snapshot()` | 无参数；读取 Layer 6 近期 odom/image/lidar observation，以及 connection/local-policy summary。 | `dict`，包含 connection state、sensor state、local policy state、safety state、freshness/availability markers。 |
| `RobotBodyStateSpec.get_connection_state()` | 无参数。 | 描述 connection availability 和 config-derived mode 的 `dict`。 |
| `RobotBodyStateSpec.get_sensor_state()` | 无参数。 | `dict`，包含 image、lidar、odom 等 sensor counters/freshness。 |
| `RobotBodyStateSpec.get_local_policy_state()` | 无参数。 | `dict`，描述上层 preflight 使用的 local policy/readiness evidence。 |
| `WorldStateSpec.get_world_snapshot(task, spatial_limit)` | `task: str`、`spatial_limit: int`；读取 Layer 6 body state，以及已接线 spatial/temporal memory provider。 | `dict` world snapshot，包含 `robot_state`、`runtime`、`memory_state`、`semantic_temporal_map`、`sources` 和 snapshot-storage metadata。 |
| `WorldStateSpec.get_robot_state()` | 无参数。 | 上层 preflight 使用的 robot/body state `dict`。 |
| `WorldStateSpec.get_runtime_state()` | 无参数。 | `dict` runtime mode/config summary，例如 replay/simulation/hardware、MCP/viewer 设置。 |
| `WorldStateSpec.get_memory_state(task, spatial_limit)` | `task: str`、`spatial_limit: int`。 | `dict` memory section，包含 spatial/temporal availability、matches/summaries，以及 provider probe failure 的 errors。 |
| `WorldStateSpec.get_snapshot_storage_policy()` | 无参数。 | `dict`，说明 snapshot 是 transient、写 memory，还是持久化到其他位置。 |
| `SemanticTemporalMapSpec.query_semantic_temporal_map(query, spatial_limit)` | `query: str`、`spatial_limit: int`；读取 spatial 和 temporal memory provider。 | `dict`，包含 spatial section、temporal section、fused evidence entries、confidence/location/time summaries 和 source errors。 |
| `SkillInterfaceSpec.get_skill_interface_snapshot(domain)` | 可选 `domain: str` filter。 | `dict`，包含 `available`、`version`、`source`、`domain_filter`、`domains`、`skill_count` 和静态 contract 列表 `skills`。 |
| `SkillInterfaceSpec.get_skill_contract(skill_name)` | `skill_name: str`。 | 匹配的 skill-contract `dict`；没有 contract 时返回 `None`。 |
| `SkillInterfaceSpec.validate_skill_request(skill_name, args_json)` | `skill_name: str`、`args_json: str` JSON object。 | `dict`，包含 `valid`、`errors`、`warnings`、匹配的 `contract`；未知 skill 返回 `valid=False`。 |
| `SkillInterfaceSpec.compare_mcp_tools(tools_json)` | `tools_json: str`，MCP `tools/list` 风格 payload 或 list。 | `dict`，包含 `valid`、parser errors、contract/MCP counts、missing contracts、unregistered MCP tools、known internal tools。 |
| `route_task(task, context)` MCP skill | `task: str`，可选 `context: str`。 | `SkillResult` metadata: `domain`、`confidence`、`matched_keywords`、`recommended_tools`、`needs_context`、`reason`、`context_used`。 |
| `get_context(task, focus, spatial_limit)` MCP skill | `task: str`、可选 `focus: str`、`spatial_limit: int`；读取已接线 Layer 4、memory、skill、feedback、outcome、world-model provider。 | `SkillResult` message 加 metadata，包含 `sources`、`runtime`、`robot_state`、`world_state`、`skill_state`、`context_feedback`、`causal_state`、`world_model_state`、`context_evidence` 和 provider `errors`。 |
| `build_context_evidence(metadata, policy)` helper | ContextProvider 生成的 metadata dict，以及 `ContextEvidencePolicy` 阈值。 | `context_evidence.v1` dict，描述 selected evidence、dropped/low-confidence evidence、selected sources 和 policy effect。 |
| `memory_backend_status()` MCP skill | 无参数；对可选 provider 做 read probe。 | `SkillResult` metadata，schema 是 `go2_memory_backend_status.v1`，包含每个 provider 的 wired/available/probe 字段和 warnings。 |
| `record_evolution_event(event_type, task, payload_json, commit)` MCP skill | `event_type: str`、可选 `task: str`、`payload_json: str` JSON object、`commit: bool`。 | `SkillResult` metadata，包含 `schema`、`event_type`、`task`、`ledger_dir`、`event_path`、`commit_requested`、可选 `commit_sha`、`warnings` 和完整 `event`。 |
| `record_skill_proposal(proposal_json, commit)` MCP skill / ledger RPC | `proposal_json: str`，使用 `dimos.skill_proposal.v1`；`commit: bool`。 | `SkillResult` metadata，包含 `schema`、`proposal_id`、`ledger_dir`、`proposal_path`、`commit_requested`、可选 `commit_sha`、`warnings` 和校验后的 `proposal`。 |
| `EvolutionLedgerSpec.write_evolution_event(event_type, task, payload, commit)` | 其他 Layer 3 module 传入的结构化 payload `dict`。 | 和 `record_evolution_event` 相同的 ledger event record dict，但不经过 MCP JSON parsing。 |
| `EvolutionLedgerSpec.write_skill_proposal(proposal, commit)` | 已校验的 proposal `dict`。 | 和 `record_skill_proposal` 相同的 proposal record dict，但不经过 MCP JSON parsing。 |
| `evaluate_task_feasibility(task, context_json)` MCP skill | `task: str`、`context_json: str` JSON object，通常来自 `get_context` metadata；已接线时读取 Layer 5 contracts。 | `SkillResult` metadata: `feasible` (`yes`、`no`、`uncertain`)、`missing_context`、`required_skills`、`available_skills`、`missing_skills`、`safety_risks`、`recommended_next_action`、`clarifying_question`、`evidence_sources` 和 warnings。 |
| `record_context_feedback(task, context_evidence_json, selected_skill, outcome_json, helpful_sources_json, ignored_risks_json)` MCP skill | task text、`context_evidence.v1` JSON、可选 selected skill、outcome JSON object、helpful-source JSON list、ignored-risk JSON list。 | `SkillResult` metadata，包含一条 `go2_context_feedback.v1` feedback record、`total_feedback` 和可选 ledger warnings。 |
| `ContextFeedbackSpec.get_recent_context_feedback(limit, source)` | `limit: int`，可选 source filter。 | newest-first 的 `go2_context_feedback.v1` feedback dict 列表。 |
| `ContextFeedbackSpec.get_context_feedback_summary(limit)` | `limit: int`。 | aggregate `dict`，包含 success/failure/unknown outcome counts，以及 helpful/harmful source counters。 |
| `record_skill_outcome(skill_name, success, domain, error_code, message, risk, recovery)` MCP skill | skill name、success boolean、可选 domain/error/message/risk/recovery 字符串。 | `SkillResult` metadata，包含 recorded outcome dict 和 `total_outcomes`。 |
| `summarize_skill_outcomes(limit, skill_name, domain)` MCP skill | limit，以及可选 skill/domain filters。 | `SkillResult` metadata，包含过滤后的 newest-first `outcomes` list。 |
| `SkillOutcomeStoreSpec.get_recent_outcomes(limit, skill_name, domain)` | limit，以及可选 exact skill/domain filters。 | newest-first outcome dict 列表: timestamp、skill name、success、domain、error code、message、risk、recovery。 |
| `predict_skill_outcome(skill_name, args_json, context)` MCP skill | skill name、planned args JSON object、可选 context text；已接线时读取 SkillOutcomeStore 和 CausalWorldModel。 | `SkillResult` metadata，包含 `risk`、`predicted_success`、`failure_reasons`、`recovery_suggestions`、recent outcomes/transitions、可选 world-model prediction 和 provider availability flags。 |
| `record_causal_transition(...)` MCP skill | task、skill name、args JSON、before/after context text、prediction JSON、outcome JSON、domain、before/after state JSON。 | `SkillResult` metadata，包含 transition dict、`total_transitions`、`autosave_error`、`dashboard_error`。 |
| `predict_world_transition(snapshot_json, action_json, goal, horizon)` MCP skill / `predict_next_state(...)` RPC | Layer 4 snapshot JSON、包含 `skill_name` 和 `args` 的 action JSON、可选 goal、bounded horizon。 | `dimos.world_model_prediction.v1` dict，包含 action、snapshot summary、risk、predicted success、score、confidence、predicted symbolic delta、failure modes、reasons、model output、causal attribution、SCM explanation 和 intervention evidence。 |
| `score_action(snapshot_json, action_json, goal)` RPC | 与 prediction 相同的 snapshot/action/goal 输入。 | compact dict，包含 score、risk、confidence、predicted success、failure modes、reasons、model/causal/SCM/intervention evidence。 |
| `summarize_causal_patterns(skill_name, domain, limit)` MCP skill | 可选 skill/domain filters 和 limit。 | `SkillResult` metadata，包含 repeated causal `patterns` 和过滤后的 `transitions`。 |
| `record_intervention(...)` MCP skill | task、intervention name、target variable、before/after value JSON、action JSON、before/after snapshot JSON、outcome JSON、causal hypothesis。 | `SkillResult` metadata，包含 intervention record、`total_interventions`、`autosave_error`、`dashboard_error`。 |
| `save_world_model_state(path)` / `load_world_model_state(path)` MCP skills | 可选 path；如果未设置 `DIMOS_GO2_WORLD_MODEL_STATE`，则必须传 path。 | `SkillResult` summary，包含保存/加载的 model-state counts，以及可选 dashboard error。 |
| `CausalWorldModelSpec.get_recent_transitions(limit, skill_name, domain, cause)` | limit，以及可选 exact filters。 | newest-first transition dict 列表。 |
| `CausalWorldModelSpec.get_intervention_log(limit, target_variable, intervention_name)` | limit，以及可选 exact filters。 | newest-first intervention dict 列表。 |
| `CausalWorldModelSpec.get_model_state()` | 无参数。 | `dict`，包含 online model sample/weights summary、provider contract、causal estimator snapshot、intervention log snapshot、SCM snapshot 和 persistence config。 |
| `CausalWorldModelSpec.get_provider_contract()` | 无参数。 | `dimos.world_model_provider.v1` dict，声明 provider、model type、capabilities 和 output schemas。 |
| `propose_skill_interface(task, failure_context_json)` MCP skill | task text，以及带 missing-capability evidence 的 failure-context JSON object；已接线时读取 Layer 5 contracts/outcomes。 | `SkillResult` metadata: `proposal_created`、创建时的 `dimos.skill_proposal.v1` `proposal`、`existing_skill_matches`、`recommended_next_action`、可选 `proposal_path` 或 warnings。 |
| MCP `tools/list` / `tools/call` runtime surface | JSON-RPC requests；`tools/call` 携带 MCP arguments 和可选 `_mcp_context` progress/capability token。 | 带 capability metadata 的 tool schema；tool-call text/content result；background tool 的 SSE progress frames；instant return 或 stopped frame 触发 capability release。 |
| `WebsocketVisSpec.set_world_model_state(state)` | `state: dict`，预期使用 `dimos.world_model_dashboard_state.v1`。 | `dict` acknowledgement/current display state，供 dashboard clients 读取。 |

## 需要 Review 的上游更新

这些不是本地自进化功能，但会影响本分支。

### 1. Zenoh Transport 集成

相关文件:

- `dimos/core/transport_factory.py`
- `dimos/core/transport.py`
- `dimos/protocol/pubsub/impl/zenohpubsub.py`
- `dimos/protocol/service/zenohservice.py`
- `dimos/protocol/rpc/pubsubrpc.py`
- `dimos/protocol/tf/tf.py`
- `dimos/core/global_config.py`

实现逻辑:

- `GlobalConfig.transport` 现在可以是 `lcm` 或 `zenoh`。
- macOS 上游默认值改为 `zenoh`，其他平台默认仍是 `lcm`。
- `make_transport()` 根据当前后端把逻辑 channel name 映射成 LCM topic 或 Zenoh key expression。
- Zenoh key expression 会放在 `dimos/` namespace 下。
- RPC 和 TF 后端分别通过 `rpc_backend()`、`tf_backend()` 选择。
- 高频 sensor channel 使用 latest-wins/best-effort QoS，human/agent 低频消息使用 reliable/blocking QoS。

Review 风险:

- macOS 默认行为变了。如果本地运行仍假设 LCM，需要设置 `DIMOS_TRANSPORT=lcm`。
- Zenoh 会创建 native pyo3 callback 线程，现有 pytest thread leak 检测可能在 cleanup 延迟时误报或暴露真实清理问题。
- 长生命周期服务测试必须避开已经运行的 MCP server 端口。

### 2. MCP Tool Capability Gating

相关文件:

- `dimos/agents/capabilities.py`
- `dimos/agents/mcp/mcp_server.py`
- `dimos/agents/mcp/tool_stream.py`
- `dimos/agents/annotation.py`
- `dimos/agents/test_capabilities.py`

实现逻辑:

- `SkillInfo` 增加 capability 元数据，例如 `uses` 和 `lifecycle`。
- `tools/list` 在相关工具上暴露 `_meta.dimos/uses` 和 `_meta.dimos/lifecycle`。
- `tools/call` 在真正调用 skill 前先申请 capability lock。
- instant skill 的占用可以等待；background skill 的占用需要先调用对应 stop tool。
- background tool 的 capability release 绑定到 tool-stream stopped frame。

Review 风险:

- 长时间运动或控制类 skill 必须正确声明 capability，否则 MCP server 无法串行化冲突行为。
- background tool 必须稳定发出 stopped frame，否则 capability 可能被永久占用。

### 3. Coordinator RPC 和动态 Blueprint 加载

相关文件:

- `dimos/core/coordination/coordinator_rpc.py`
- `dimos/core/coordination/module_coordinator.py`
- `dimos/core/coordination/worker_manager_python.py`

实现逻辑:

- `ModuleCoordinator` 可以暴露一个 singleton coordinator RPC service。
- 客户端可以通过 coordinator RPC 查询模块、加载 blueprint。
- 动态加载和 restart 期间，已部署 module proxy 由 lock 保护。

和本地改动的关系:

- 本地 runtime plumbing 保留了 `McpClient` 的 non-blocking system-module notification，避免 agent 启动被直接 tool registration 阻塞。
- 在 `DIMOS_TRANSPORT=lcm` 下，`ModuleCoordinator` 测试全部通过。

### 4. Memory 和 Perception 路径迁移

相关文件:

- `dimos/perception/image_embedding.py`
- `dimos/perception/spatial_vector_db.py`
- `dimos/perception/visual_memory.py`
- `dimos/perception/spatial_perception.py`
- `dimos/memory2/*`

实现逻辑:

- 上游把 visual/spatial memory helper 从 `dimos.agents_deprecated.memory` 迁移到 `dimos.perception`。
- 旧的 deprecated agent memory 目录被删除。
- 上游新增了 memory2 TF service、replay 和 tooling。

本地解析:

- `spatial_perception.py` 使用上游新的 `dimos.perception.*` 模块路径。
- 仍保留本地默认持久化路径 `STATE_DIR / "spatial_memory"`，并支持 `DIMOS_SPATIAL_MEMORY_DIR` 覆盖，而不是改成上游 project assets output 路径。

### 5. Control、Manipulation、Learning、Docs、LFS 更新

概要:

- control tasks 被拆成带 registry 的 package 目录。
- 新增 roboplan、learning、data-prep 相关模块。
- navigation 文档按 Mintlify 结构重组。
- 新增 LFS 数据包和 native build 脚本。

Review 风险:

- 这些是广泛的上游变更，不属于 Go2 自进化核心逻辑。主要审查 dependency、packaging、blueprint registry、CI 和运行环境影响。

## 本地新增功能清单

### 1. Go2 分层 Blueprint 架构

相关文件:

- `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic.py`
- `dimos/robot/unitree/go2/blueprints/smart/unitree_go2_spatial.py`
- `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_temporal_memory.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/__init__.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/__init__.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/__init__.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_6_robot_body/__init__.py`

实现逻辑:

- `unitree_go2_agentic` 现在由四层明确组成: `_go2_robot_body`、`_go2_spatial_world_state`、`_go2_skill_interface`、`_go2_agent_brain`。
- 各 layer package 使用 lazy `__getattr__` 构建，避免 import 某一层时提前拉起重型 perception、VLM 或 robot stack。
- `unitree_go2_spatial` 是非 agentic 的空间栈，由 Layer 6、Layer 4 spatial world state、Layer 5 spatial skills 组成。
- `unitree_go2_temporal_memory` 通过 Layer 4 的 `_go2_temporal_memory_world_state()` 懒加载 temporal memory，确保 CLI config 已经生效后再创建 TemporalMemory blueprint。

为什么重要:

- reviewer 可以分别审查 robot body、world state、skill interface、agent reasoning。
- 防止 agent brain 变成无结构的大型 import hub。
- 仍保留 DimOS blueprint 语义: 通过 `autoconnect()` 组合模块，通过 Spec 注入 RPC refs。

主要风险:

- 分层不能掩盖漏接模块。`test_all_blueprints_generation.py` 和 Layer 4 wiring test 是主要保护。

组成部分:

- `unitree_go2_agentic`: 完整 Go2 agentic stack，组合 physical body、structured world state、skill interface registry/containers、Layer 3 MCP/LLM brain。
- `unitree_go2_spatial`: 没有 LLM brain 的 perception/world-state stack，用来单独验证 Layer 4 和 Layer 5。
- `unitree_go2_temporal_memory`: 包住 agentic stack，并通过 Layer 4 factory 追加 temporal memory。
- 各 layer package 的 `__init__.py`: lazy blueprint factories。它们不是普通 import 便利层，而是架构边界的一部分，避免过早 import 重型 perception/model 依赖。

构建数据流:

1. CLI 通过 registry 懒加载命名 blueprint。
2. blueprint 只 import 需要的 layer handle。
3. 每个 layer handle 构建自己的 `autoconnect()` blueprint。
4. 最终仍是普通 DimOS blueprint 组合，stream wiring 和 Spec-based RPC injection 仍由 `ModuleCoordinator` 完成。
5. 测试验证 Layer 3 能通过预期 contract 访问 Layer 4/5/6，并且 blueprint registry 是最新的。

状态和持久化:

- blueprint layer 本身不保存 robot 数据。
- 状态由各层内部 module 持有: Layer 6 body state、Layer 4 memory/world snapshot、Layer 5 skill contracts、Layer 3 context/outcome/causal state。
- 持久化属于具体 module，不属于 blueprint 文件。

安全边界:

- blueprint 决定哪些 module 部署在一起。
- blueprint 不决定调用哪个 skill，也不执行 motion。
- wiring 错误应该按安全风险看待，因为 LLM 可能看到错误工具或错误上下文。

### 2. Layer 6 Robot Body State

相关文件:

- `dimos/robot/unitree/go2/blueprints/layers/layer_6_robot_body/robot_body_state.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_6_robot_body/robot_body_spec.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_6_robot_body/test_robot_body_state.py`

实现逻辑:

- `_Go2RobotBodyState` 是一个小模块，通过 spec-facing API 暴露 robot body 状态。
- 它贴近真实 Go2 connection stack，但不替代真实连接模块。
- 上层通过 typed spec 消费 robot state，而不是直接访问硬件 connection module。

Review 重点:

- 是否提供了 agent preflight 足够使用的安全相关状态。
- 是否避免重复持有或篡改真实控制权。

组成部分:

- `_Go2RobotBodyState`: 把低层 robot status 适配成紧凑 Layer 6 provider 的 module。
- `robot_body_spec.py`: 上层消费的 typed RPC contract。
- Layer 6 package factory: 把底层 Go2 robot blueprint 和 body-state module 接在一起。

数据流:

1. 底层 Go2 connection 和 robot modules 持有真实硬件/仿真通信。
2. `_Go2RobotBodyState` 通过 spec 暴露 read-oriented body-state summary。
3. Layer 4/Layer 3 把这些状态用于 safety preflight 和 context evidence。
4. agent brain 看到的是摘要状态，不是直接硬件 handle。

判断主体:

- body-state module 不做任务决策。
- 它是 evidence provider。Layer 3 确定性 preflight 和 LLM 会使用它的输出判断等待、澄清或调用 skill。

状态和写边界:

- 它应该被视为 current-state adapter。
- 不应该持久化 memory，也不应该发控制命令。
- 未来如果增加 body-state cache，必须有明确 bound 和 timestamp。

失败形态:

- 如果底层 robot state 缺失，应明确返回 unavailable/unknown，不应伪造 ready。
- motion-sensitive preflight 应把缺失 robot body state 视为风险。

### 3. Layer 4 Structured World State

相关文件:

- `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/structured_world_state.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/semantic_temporal_map.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/world_state_spec.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/test_world_state.py`

实现逻辑:

- `_Go2StructuredWorldState` 通过 typed spec 暴露紧凑的 robot/world snapshot。
- `_Go2SemanticTemporalMap` 把 spatial memory 和 temporal memory 摘要合并成更适合 agent 使用的 world-state surface。
- Layer 3 通过 RPC spec 注入读取这些信息，不直接 import 具体 memory 实现。

这里的 "有效上下文" 是什么:

- Layer 4 不让 LLM 直接判断 memory 相关性。
- 它把可用 world/memory source 归一化成结构化 provider surface。
- Layer 3 再负责打分、过滤、组合 evidence，最后交给 LLM 使用。

Review 重点:

- memory backend 缺失时是否明确返回 unavailable，而不是静默空值。
- snapshot schema 是否足够稳定，能被 prompt 和 dashboard 长期消费。

组成部分:

- `_Go2StructuredWorldState`: 把 robot/world 信息规范化成 compact snapshot provider。
- `_Go2SemanticTemporalMap`: 融合 spatial memory 和 temporal memory summary，产出 agent context 使用的 semantic evidence。
- `world_state_spec.py`: Layer 3 依赖的 RPC contract。
- `_go2_temporal_memory_world_state()`: lazy factory，延迟 TemporalMemory 构建，确保 `--new-memory` 等 CLI flags 已经生效。

数据流:

1. SpatialMemory 和 TemporalMemory 仍是 storage/search 系统。
2. Layer 4 查询或接收这些系统的 summary。
3. Layer 4 把 summary 整形成稳定 world-state object，并明确 source availability。
4. Layer 3 通过 spec 请求 world state，然后根据当前 task 选择 evidence。

状态和持久化:

- Layer 4 可以读取持久化 memory backend，但它自身的 structured snapshot 是 derived view。
- Temporal memory 持久化由 TemporalMemory module 和 CLI config 控制。
- Spatial memory 持久化由 `SpatialConfig` 和 `spatial_perception.py` 的 state-dir 路径逻辑控制。

判断主体:

- Layer 4 决定如何规范化 provider output。
- Layer 4 不判断上下文是否足够行动。这个判断属于 Layer 3 preflight policy 和 LLM。

失败形态:

- provider unavailable 应在 snapshot 里明确表示。
- search result empty 不等于 backend failure，review 时要确认二者有区别。

### 4. Layer 5 Skill Interface Registry

相关文件:

- `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/skill_interface_registry.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/skill_interface_spec.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/test_skill_interface_registry.py`

实现逻辑:

- skill contract 是显式静态记录，包含 skill name、domain、module、required args、optional args、是否 motion sensitive、context requirement、推荐 preflight、risk class、outcome shape。
- registry 把这些 contract 作为数据暴露给 Layer 3。
- 已知非 Layer 5 MCP tool 被排除在 contract mismatch 检查之外，避免 status/preflight 内部工具污染 skill coverage。

和自进化的关系:

- 机器人不会自动修改 skill 代码。
- 当任务失败且有明确 missing capability 证据时，可以生成待 review 的 skill interface proposal。
- 现有 contract 是 duplicate suppression 的判断基准。

Review 重点:

- 每个 MCP action skill 是否都有准确 contract。
- `risk_class`、`requires_context`、`requires_robot_state` 应按 safety-critical metadata 审查。

组成部分:

- 静态 `_SkillContract` 记录: 每个 action skill 的人工维护接口描述。
- `_Go2SkillInterfaceRegistry`: 通过 RPC/MCP-facing 方法把 contracts 暴露给 Layer 3。
- `skill_interface_spec.py`: 消费 skill metadata 的 typed contract。
- Layer 5 blueprint factory: 接入 action skill containers、perception/security skills 和 registry。

数据流:

1. action modules 通过 MCP 暴露 `@skill` 方法。
2. registry 提供并行 contract view，包含比 MCP JSON schema 更丰富的 safety/context metadata。
3. ContextProvider 和 TaskFeasibility 读取 contracts，判断需要哪些 context、arguments、preflight checks。
4. SkillProposal 读取同一份 contracts，避免提重复 skill。

状态和写边界:

- contracts 是代码/静态数据，不是 learned state。
- registry 运行时只读。
- self-evolution proposal 不会直接修改这个文件。开发者 review 后才编辑 contract。

判断主体:

- 人类维护 source-of-truth contract 文案和风险元数据。
- Layer 3 确定性代码消费这些数据。
- LLM 可以使用 contract 信息，但不应发明 registry/MCP list 之外的隐藏能力。

失败形态:

- 暴露的 skill 没有 contract 是 review failure，因为 preflight 无法知道 risk/context 需求。
- contract 存在但 MCP tool 缺失也是 review failure，除非该 blueprint 有意禁用它。

### 5. Layer 3 Expert Router

相关文件:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/expert_router.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/prompt_policy.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_expert_router.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_prompt_policy.py`

实现逻辑:

- router 根据 task domain 推荐应该使用的 expert/tool family。
- prompt policy 把 Layer 3 自进化约束注入 MCP client system prompt，但不完全替代机器人原有 prompt。
- 输出是确定性的 routing metadata，不是额外 LLM call。

判断主体:

- 第一层 routing 和 evidence packaging 由确定性代码完成。
- LLM 仍是读取最终上下文并决定如何执行、是否澄清、是否调用工具的主体。

Review 重点:

- routing label 是否和实际 skill contract 对齐。
- prompt 文案是否过度承诺自治或自动改代码能力。

组成部分:

- `_Go2ExpertRouter`: 确定性 task/domain router。
- prompt policy helpers: 把 layer-specific instructions 合并进 agent system prompt。
- router tests: 固定预期 label，避免行为无意扩大。

数据流:

1. user task 或 subtask 进入 router。
2. router 把任务分类到少量 domain，例如 navigation、perception、person follow、类似 manipulation 但当前不支持的任务、general conversation。
3. 返回 routing metadata 和 recommended next tools/preflight。
4. ContextProvider 可以把 routing output 放进 context。
5. LLM 把 routing 当作提示，而不是不可逆 planner。

判断主体:

- domain classification 是确定性的、可检查的。
- tool choice 仍是 LLM/tool-calling decision，除非 deterministic preflight 明确拒绝或要求澄清。

状态和持久化:

- router 无状态。
- 它自己不写 ledger event 或 feedback。

失败形态:

- 未知任务应该 route 到 uncertainty/general handling，不应伪造能力。
- safety-sensitive domain 应偏向 preflight 和 clarification。

### 6. Context Provider 和 Evidence Selection

相关文件:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_provider.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_evidence.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_provider.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_evidence.py`

实现逻辑:

- `get_context()` 汇总 task text、runtime metadata、robot state、RAG/spatial memory、temporal memory、skill contracts、skill outcomes、causal world model、context feedback。
- `context_evidence.py` 提供纯函数策略 `build_context_evidence(metadata, ContextEvidencePolicy)`。
- policy 字段包括 `min_relevance_score`、`max_entries`、`include_low_confidence`、`require_robot_state_for_motion`。
- 返回结果包含 compact context 和 `context_evidence.v1`，说明选了哪些 evidence、为什么选。

现在 "有效" 的标准:

- 有效表示被确定性 policy 从可用 provider 里选中。
- 主要依据包括 relevance score、confidence、source availability、task type、safety requirement。
- 这还不是基于 replay metric 学出来的策略。
- LLM 会消费这些上下文，并且仍可以判断上下文不足。

组成部分:

- `_Go2ContextProvider`: MCP-facing context bundle provider。它是聚合器，读取可选 Layer 4/5/3 providers，并把结果整理成 agent 可用的 compact context。
- `context_evidence.py`: 纯 evidence selector。它负责确定性的排序/过滤策略，可以在没有 robot、MCP server、memory backend 或 LLM 的情况下单测。
- 可选 provider refs: world state、spatial/RAG memory、temporal memory、skill interface registry、skill outcome store、causal world model、context feedback store。
- `context_evidence.v1`: 嵌在返回结果里的 review artifact。它记录哪些 evidence 被选中、丢弃或标成 low-confidence。

输入:

- 调用方传入的 task text 和 runtime metadata。
- 已接线 DimOS modules 提供的 provider snapshot/summary。
- policy 参数，例如 relevance 阈值、最大条数、low-confidence 处理方式、motion task 是否必须有 robot state。
- 已接线时的历史 feedback、outcome、world-model summary。

数据流:

1. 调用方带着 task/runtime metadata 调用 `get_context()`。
2. ContextProvider 向已接线 provider 请求 world state、memory summary、skill contract、outcome summary、causal-world-model state、context feedback。provider 没接线时，应贡献明确的 unavailable 状态，而不是静默丢掉该 source。
3. metadata 被传给 `build_context_evidence()`。这个 helper 是纯函数: 给定 metadata 和 `ContextEvidencePolicy`，返回被选 evidence，不发 RPC、不写 memory、不调用 LLM。
4. ContextProvider 格式化 compact context，并附加 `context_evidence.v1`，让 reviewer 能看到为什么选了这些 evidence。
5. LLM 仍然是最终消费上下文、做用户可见推理和工具选择的主体。ContextProvider 本身不决定执行 skill。

状态和写边界:

- ContextProvider 本身基本是 read-mostly，不写 RAG/vector memory、temporal memory、skill contracts 或 robot state。
- 唯一的 durable side channel 是间接的: 它可能包含来自 feedback/outcome/ledger-aware modules 的摘要，但写入由那些 module 自己负责。
- 被选中的 context 是临时 prompt artifact，除非另一个调用方显式记录 feedback 或 evolution event。

判断主体:

- 确定性代码决定哪些 evidence 进入 context bundle。
- LLM 决定如何消费这个 bundle、是否追问、是否调用已暴露 tool。
- 人类 reviewer 判断确定性 policy 对当前机器人安全姿态是否足够保守。

失败形态:

- 缺失 provider 应显示为明确 unavailable metadata。
- provider payload 格式错误时，应降级该 source，而不是清空整个 context bundle。
- 低相关或低置信 evidence 应按 policy 丢弃或标注，不能静默升级为可信 evidence。

没有实现的边界:

- 还没有用 replay log 训练 learned ranker。
- ContextProvider 不写 RAG/vector database。
- context feedback 只是一个摘要信号，不会单独覆盖 provider data。

Review 重点:

- relevance/confidence 默认阈值是否过松。
- motion task 是否必须要求 robot state。
- low-confidence evidence 应该隐藏，还是保留但显式标注。

### 7. Memory Backend Status

相关文件:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/memory_backend_status.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_memory_backend_status.py`

实现逻辑:

- 新增 MCP skill `memory_backend_status()`。
- 报告 world state、spatial memory、temporal memory、skill outcomes、causal world model、skill interface 是否已接线。
- 能 probe 的 backend 会做小型 read probe。
- 只返回诊断元数据，不创建新的 memory database。

Review 重点:

- 这是诊断工具，不是新的 memory scheduler。
- 它不应该写入 RAG、spatial memory、temporal memory 或 Git。

组成部分:

- `_Go2MemoryBackendStatus`: 作为 MCP skill 暴露的诊断 module。
- 可选 provider specs: world state、spatial memory、temporal memory、outcome store、causal world model、skill interface registry。
- probe helpers: 小型 read attempt，用来报告 availability 和 error。

数据流:

1. skill 检查哪些可选 provider reference 已注入。
2. 对每个 provider 记录 `wired`、`available` 和 probe result。
3. probe failure 被捕获成 status metadata，而不是作为 hard crash 抛给 agent。
4. 结果以 JSON/text 返回给 LLM 或 human operator。

状态和写边界:

- 它是 read-only。
- 不持久化 status snapshot。
- 不初始化缺失 database。如果 database 没运行或没接线，status 应明确说明。

判断主体:

- 这个 skill 不决定使用哪个 memory。
- 它只回答“现在接了什么、能读什么”。ContextProvider 和 LLM 决定如何响应。

失败形态:

- probe failure 应标明 provider 和 error string。
- missing provider 应足够明确地区分 blueprint wiring 问题和 memory result empty。

### 8. Git-backed Evolution Ledger

相关文件:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/evolution_event.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/evolution_ledger.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/evolution_proposal.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_evolution_ledger.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_evolution_proposal.py`

实现逻辑:

- event 使用 schema `dimos.evolution_event.v1`。
- proposal 由 `evolution_proposal.py` 做 schema 验证。
- 默认写入本地仓库路径:
  `.dimos/evolution/events/YYYY/MM/DD/*.json`
  和 `.dimos/evolution/proposals/*.json`。
- `DIMOS_EVOLUTION_LEDGER_DIR` 可以覆盖存储目录。
- 写入内容是 JSON 文件，目标是低频、可 review、可 diff 的控制面记录。
- 默认 `commit=False`。
- 如果 `commit=True`，只把该 event/proposal 文件提交到本地 Git。

组成部分:

- `evolution_event.py`: 低层自进化 observation 的 schema 和校验，例如 feasibility assessment、context feedback、runtime review note。
- `evolution_proposal.py`: 更高层 proposed change 的 schema 和校验。这些 proposal 必须先经人工 review，才可能变成代码或配置。
- `_EvolutionLedger`: filesystem 和可选 Git writer。它负责路径选择、JSON serialization、commit mode 下的 staging/commit 行为。
- 测试: 覆盖 schema validation、路径形状、commit 隔离和 proposal 持久化。

输入:

- Layer 3 MCP tool 传入的 event/proposal payload。
- 可选的 caller-controlled commit flag。
- 可选的 `DIMOS_EVOLUTION_LEDGER_DIR` 覆盖。
- commit mode 打开时的当前 Git repository context。

Git 内容会提交到哪里:

- 文件先进入当前本地仓库工作区。
- commit 只进入本地 `.git` 历史。
- 不会自动 push 到 GitHub。
- 只有人类后续运行 `git push` 或开 PR，GitHub 才会收到这些记录。

写入数据流:

1. 调用方调用 ledger-facing MCP skill，例如 `record_evolution_event()` 或 `record_skill_proposal()`。
2. 模块验证 schema，并规范化 payload。
3. 模块选择存储 root。默认是 repo-local `.dimos/evolution`，`DIMOS_EVOLUTION_LEDGER_DIR` 可以覆盖。
4. 写入 JSON 文件，作为可 review artifact。
5. 如果 `commit=False`，除了工作区文件外不碰 Git。
6. 如果 `commit=True`，只 stage 并提交该 event/proposal 文件到本地 Git，不 push。

状态和写边界:

- ledger 是 durable filesystem state，不是内存学习模型。
- 它只在配置的 ledger root 下面写 JSON artifact。
- 它不修改 skill 代码、prompt、contracts、memory database 或 model weights。
- commit mode 有意保持 local-only 且范围很窄: stage 该 artifact，创建本地 commit，然后停止。

判断主体:

- 调用方 module 决定何时值得记录 event/proposal。
- ledger 只判断 artifact 是否符合 schema，以及应该存到哪里。
- 人类 reviewer 决定这些 ledger artifact 后续是否变成代码改动、PR、被 ignore 的本地轨迹，或训练/评估数据。

失败形态:

- schema 无效应在写入前失败。
- 不安全路径或路径穿越应在 filesystem access 前被拒绝。
- Git commit 失败时，应让 JSON artifact 仍然在 worktree 可见，而不是假装已经提交。

隐私和 review 边界:

- 这些文件可能包含 task text、outcome message、context summary、proposal rationale。真实机器人运行前，要先决定 `.dimos/evolution` 是否应该进入正常 Git 历史。
- ledger 是控制面审计轨迹，不替代 SpatialMemory、TemporalMemory、SkillOutcomeStore 或 causal model state。

Review 重点:

- event/proposal 字段是否可能造成路径穿越。
- commit mode 是否只 stage 新 ledger 文件。
- 根据隐私策略决定 `.dimos/evolution` 应该 ignore、commit，还是导出到别的存储。

### 9. Task Feasibility Preflight

相关文件:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/task_feasibility.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_task_feasibility.py`

实现逻辑:

- 新增 MCP skill `evaluate_task_feasibility(task, context_json)`。
- 读取 skill contracts、world/robot context 和可选 JSON context。
- 返回确定性结果: feasibility status、missing context、required skills、available skills、safety risks、recommended next action、clarifying question、evidence sources。
- 如果 evolution ledger 已接线，可以写入 ledger event。

自进化含义:

- 这不是模型自训练。
- 它是 agent self-assessment: 在行动前，agent 可以调用确定性 evaluator 判断任务是否具备上下文和能力支持。

组成部分:

- `_Go2TaskFeasibility`: deterministic preflight 的 MCP skill surface。
- Skill contract reader: 消费 Layer 5 skill metadata，例如 required arguments、context requirements、risk class、motion sensitivity。
- Context parser: 把显式 `context_json` 和已接线 world/robot provider summary 合并。
- Ledger integration: 可选的 review trace event writer。

输入:

- `task`: 用户请求或 subtask 的自然语言文本。
- `context_json`: 调用方可选传入的结构化 context。
- Layer 5 skill contracts 和 availability metadata。
- 已接线时的 Layer 4 robot/world state。
- context bundle 中可用的 outcome/context feedback summary。

决策/数据流:

1. 解析 task 和可选 `context_json`。
2. 把 task 匹配到已知 skill contract 和 expert domain。
3. 根据 Layer 5 contract 检查 required args 和 context requirements。
4. 对 motion-sensitive 或高风险 skill 检查 robot/world-state 是否可用。
5. 输出三类粗粒度结果之一: `feasible`、`uncertain`、not feasible。
6. 如果缺数据，推荐澄清问题或 context-gathering action。
7. 如果缺能力，报告 missing capability evidence。这个 evidence 后续可以成为 skill-interface proposal 的依据。

状态和写边界:

- 它对 robot state、memory 和 skill contracts 是 read-only。
- 它不会调用目标 skill。
- 它不会创建新 skill。
- 如果 ledger 已接线，它可以写一条描述 preflight result 的 audit event，但不是代码改动。

判断主体:

- feasibility classification 是确定性代码。
- LLM 可以调用这个工具并消费结果，但 LLM 不是这个工具内部的 classifier。
- 人类 review 判断这些确定性规则对真实硬件是否足够保守。

失败形态:

- 缺少 required arguments 应产生 clarification，而不是新 skill proposal。
- 缺少 context 应产生 context-gathering 或 uncertainty，而不是误报 feasible。
- missing capability evidence 要足够具体，方便 SkillProposal 把它和 transient runtime failure 区分开。

Review 重点:

- motion-sensitive task 是否会误判为可行。
- 缺少 context 时是否优先澄清，而不是生成 skill proposal。

### 10. Context Feedback Store

相关文件:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_feedback.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_feedback.py`

实现逻辑:

- 新增 MCP skill `record_context_feedback(...)`。
- 内存里维护最多 100 条 recent feedback 的 bounded deque。
- schema 为 `go2_context_feedback.v1`。
- 如果 evolution ledger 已接线，会把反馈写成 ledger event。
- ContextProvider 会把 compact feedback summary 纳入后续 context。

Review 重点:

- feedback 默认是 session memory，除非 ledger 已接线。
- 它不能无界增长，也不应该静默覆盖 RAG 或 temporal memory。

组成部分:

- `_Go2ContextFeedbackStore`: bounded in-memory store 和 MCP skill surface。
- feedback entry schema: task/context identifiers、rating 或 usefulness signal、source labels、optional explanation、timestamp。
- optional evolution ledger connection: 已接线时提供 durable audit trail。
- ContextProvider integration: compact aggregate feedback summary。

数据流:

1. agent 或 human-facing process 在 context bundle 有帮助、不完整、过期或误导后调用 `record_context_feedback(...)`。
2. store 验证并规范化 feedback fields。
3. entry append 到最多 100 条的 deque。
4. 如果 ledger 已接线，feedback 也写成 evolution event。
5. ContextProvider 读取 summary，而不是完整 raw feedback list，避免 prompt 膨胀。

状态和持久化:

- 没有 ledger 时，feedback 是 process-local，重启丢失。
- 有 ledger 时，feedback 成为可 review JSON，但不会自动训练 ranker。

判断主体:

- feedback recording 是输入信号。
- evidence policy 和 LLM 决定如何使用该信号。没有自动 prompt rewrite 或 memory deletion。

失败形态:

- malformed feedback 应 validation fail。
- feedback 超量时淘汰最旧 entry，不应无界增长。

### 11. Skill Outcome Store 和 Predictor

相关文件:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/skill_outcome_store.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/skill_outcome_predictor.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_skill_outcomes.py`

实现逻辑:

- 记录近期 skill outcome，存储是 bounded 的。
- 按 skill 汇总成功、失败和模式。
- predictor 根据近期历史和 skill contract metadata 估计风险或可能结果。
- ContextProvider 可以把这些 summary 作为 task evidence。

Review 重点:

- predictor 是启发式，不是学习模型。
- 旧失败不应永久阻止可用 skill。

组成部分:

- `_Go2SkillOutcomeStore`: bounded recent outcome history。
- `_Go2SkillOutcomePredictor`: 基于近期 history 和 skill contract metadata 的 heuristic scorer。
- outcome summary methods: 按 skill 聚合 successes、failures、repeated errors、recent messages。

数据流:

1. skill call 后可以记录 skill name、success、error code、message、metadata。
2. outcome store 把记录 append 到 bounded memory。
3. summary 按 skill 和 error pattern 分组。
4. predictor 把近期 outcome 和 skill contract risk/context metadata 合并，返回 risk 和 rationale。
5. ContextProvider 可以把 summary 作为后续调用的 evidence。

状态和持久化:

- 当前 store 是 bounded runtime memory，除非未来接入其他持久化。
- 它和 Git ledger 分离。ledger 记录 review/audit event；outcome store 支持即时 runtime adaptation。

判断主体:

- predictor 给出 heuristic risk hint。
- 它不应单独 block 一个 skill；TaskFeasibility 或 LLM 应把它作为 evidence source 之一。

失败形态:

- unknown skill 应返回 low-confidence 或 no-history output。
- repeated failure 应提高谨慎度，但不应阻止 recovery/stop skills。

### 12. Causal World Model 和 Dashboard Contract

相关文件:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/causal_world_model.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/causal_effect_estimator.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/online_transition_model.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/structural_causal_model.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/intervention_log.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/world_model_contract.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_causal_world_model.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_world_model_contract.py`

实现逻辑:

- 记录 causal transition 和 intervention。
- 维护近期 world-state change 的 online transition model。
- 基于记录事件估计 causal effect。
- 输出 versioned contract:
  `dimos.world_model_prediction.v1`、
  `dimos.world_model_provider.v1`、
  `dimos.world_model_dashboard.v1`。
- 支持 save/load model state。

它是不是训练了模型:

- 严格说: 有一个非常小的在线模型，但没有离线训练流程，也没有神经网络世界模型。
- `OnlineTransitionOutcomeModel` 是 lightweight logistic-style linear model。它把 feature weight 存在 dict 里，每次 `record_causal_transition()` 收到一次已观测 success/failure outcome 时增量更新。
- 如果还没有记录 transition，它的 `sample_count` 是 0，此时预测只能视为低置信先验行为，不是已学习的机器人知识。
- symbolic SCM 是手写的，不是从数据学出来的。
- causal effect estimator 是观测统计估计，只比较 feature active/inactive 的平滑成功率差异，不控制 hidden confounders。

组成部分:

- `OnlineTransitionOutcomeModel`: 在线 success-probability scorer。feature 包括 bias、skill name、domain、是否有 odom、navigation state、spatial memory 是否可用、spatial match count、temporal 是否可用、action argument 是否存在和部分 argument value。更新方式是一阶 gradient adjustment，带 L2 shrinkage。
- `CausalEffectEstimator`: bounded observation store，对 symbolic feature 做 treated/control success-rate difference。它能给出 risk factor、supporting factor 和 intervention suggestion，但输出里明确声明 unobserved confounders are not controlled。
- `StructuralCausalModel`: 手写因果图，变量包括 `spatial_has_matches`、`target_resolvability`、`odom_ready`、`motion_safety`、`navigation_success`。它还会生成 counterfactual suggestion，例如语义导航前先 tag 或 perceive target。
- `InterventionLog`: 记录显式 intervention，并让 prediction 能引用历史 intervention evidence。
- `_WorldTransition`: compact event record，把 task、skill、before/after state、predicted risk、outcome、inferred cause、recovery suggestion 绑定起来。

记录数据流:

1. `record_causal_transition()` 接收 task、skill name、可选 args、before/after Layer 4 snapshot、prediction metadata、outcome metadata。
2. 输入会按 JSON object 解析和验证。缺少 outcome 时，如果 SkillOutcomeStore 已接线，可以回退到同 skill 的最新 outcome。
3. 模块根据 context、prediction reasons、outcome success、error code、message 推断粗粒度 cause 和 recovery suggestion。
4. 模块计算 before/after snapshot 的 symbolic state delta。
5. 调用 `_outcome_model.update(...)` 和 `_causal_estimator.update(...)`。
6. 把 transition append 到内存 deque，最多保留 200 条。
7. 只有设置了 `DIMOS_GO2_WORLD_MODEL_STATE` 时才 autosave。
8. 如果 WebsocketVis 已接线，会 publish dashboard state。

预测数据流:

1. `predict_next_state()` 解析 snapshot 和 candidate action。
2. 读取同 skill 的 recent transitions，提取 repeated failure modes。
3. 根据当前 snapshot 和 action 计算 rule-based risk reasons。
4. 调用在线模型得到 success probability、score、risk、confidence、top weighted feature contribution。
5. 调用 causal estimator 得到 observational risk/support factors。
6. 调用手写 SCM 得到 variables、edges、counterfactuals。
7. 调用 intervention log 查找匹配的 intervention evidence。
8. 合并 rule risk 和 model risk，得到最终 risk 和 score。
9. 返回 `dimos.world_model_prediction.v1`，并可选 publish dashboard contract。

可信边界:

- 输出只是 advisory evidence，用来帮助 LLM 选择 preflight、perception、clarification 或更安全的 skill path。
- 它不能直接命令机器人运动。
- 当前 high confidence 需要同 skill 足够样本；否则应当视为低置信启发式。

Review 重点:

- 当前 causal estimate 只能作为 advisory evidence。
- prediction schema 要对 dashboard 和 LLM prompt consumer 保持稳定。
- persistence 不应意外混合不同 run 的数据。
- UI/prompt 文案不能暗示这是完整训练好的 physical world model。
- 在依赖它做自主 action selection 前，应补 replay 或 simulation evaluation。

### 13. Skill Interface Proposal Generator

相关文件:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/skill_proposal.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_skill_proposal.py`

实现逻辑:

- 新增 MCP skill `propose_skill_interface(task, failure_context_json)`。
- 只有在 failure context 里有明确 missing capability evidence 时，才生成 `dimos.skill_proposal.v1` review artifact。
- 对 missing arguments、missing context 不生成 proposal。
- 如果现有 contract 已覆盖能力，则抑制重复 proposal。
- 如果 ledger 已接线，通过 evolution ledger 写入 proposal 文件。
- 不修改 Python 代码、不新增 `@skill`、不改 blueprint wiring。

决策树:

1. 解析 `failure_context_json`。
2. 查找明确 missing capability evidence，而不是只看执行失败。
3. 查询现有 Layer 5 contracts，避免 proposal 重复。
4. 如果只是缺 argument、缺 context、provider 暂时 unavailable，则拒绝生成 proposal。
5. 构造 proposal artifact，包含 task、evidence、suggested interface shape、expected inputs、expected output、review rationale。
6. 如果 ledger 已接线，通过 evolution ledger 写入 proposal。

人工边界:

- 这个工具故意只做 proposal。开发者仍然需要写代码、选择 container、加 `@skill`、更新 contract/prompt，并测试行为。

Review 重点:

- 这是最安全的自进化边界: 只生成 proposal，必须人工 review。
- evidence requirement 要足够严格，避免用户表达不清时产生大量噪声 proposal。

组成部分:

- `_Go2SkillProposalGenerator`: proposal generation 的 MCP skill surface。
- 现有 Layer 5 contracts: duplicate suppression 的 source of truth。
- Evolution ledger: 可选 proposal writer。
- Proposal schema: 描述 missing capability 和 proposed interface 的 review artifact，不是可执行代码。

输入:

- `task`: 失败的 user/subtask text。
- `failure_context_json`: 描述为什么现有 skills 失败或不足的结构化 evidence。
- 现有 contracts 和已知 MCP tool names。

接受的 evidence:

- 明确 missing capability。
- repeated failure 显示没有可用 skill 能完成目标操作。
- 现有 contracts 没覆盖的 domain/action gap。

拒绝的 evidence:

- 只是缺少现有 skill 的 required argument。
- 只是缺 context 或 memory/provider unavailable。
- 临时 runtime failure。
- 能力已被现有 contract 覆盖。

输出边界:

- 输出是 JSON proposal 和可选 ledger 文件。
- 不 import、不写入、不生成 Python code。
- 不自动更新 prompt 或 registry。

### 14. MCP Runtime Plumbing

相关文件:

- `dimos/agents/mcp/mcp_client.py`
- `dimos/agents/mcp/mcp_server.py`
- `dimos/agents/mcp/tool_stream.py`
- `dimos/agents/mcp/test_mcp_client_unit.py`
- `dimos/agents/mcp/test_mcp_server.py`
- `dimos/agents/mcp/test_tool_stream.py`

实现逻辑:

- `McpServer.on_system_modules()` 避免通过 RPC 查询自己的 proxy。
- 对自身 skill 和带 actor class 的 proxy，server 本地收集 schema。
- `McpClient` 在 RPC path 外延迟 direct tool registration，避免启动阶段互相等待。
- tool-stream progress frame 可以通过 persistent SSE 发送，并带 progress-token metadata。
- background capability release 绑定 stopped frame。
- `agent_send()` 使用 `make_transport()`，因此会尊重当前 backend。

Review 重点:

- 启动路径不能等待依赖自身初始化的工具。
- MCP 测试端口必须和 live robot session 隔离。

组成部分:

- `McpServer`: 暴露 MCP JSON-RPC endpoints、tool list/call handling、SSE progress stream、server status tools、capability gating、agent-send。
- `McpClient`: LLM agent module，负责发现 tools 并暴露给 agent runtime。
- `tool_stream.py`: 长运行 skill 的 progress/log notification path。
- capability registry: server-side lock manager，用于 mutually exclusive tool capabilities。

启动数据流:

1. ModuleCoordinator 部署 worker 并启动 modules。
2. `McpServer.start()` 启动 HTTP/SSE serving，并订阅 tool-stream frames。
3. `on_system_modules()` 注册 tool schemas。对 server 自身和 actor-class backed modules，本地收集 schema，避免 RPC self-deadlock。
4. `McpClient` 初始化 agent，但不在 RPC path 外阻塞等待 direct tool registration。

Tool call 数据流:

1. JSON-RPC `tools/call` 到达。
2. server 解析 `SkillInfo` 和 RPC call target。
3. 如果 skill 声明 `uses`，先申请 capability lock。
4. progress token 和 capability token 通过保留 `_mcp_context` 注入。
5. RPC call 在 executor 里运行。
6. instant skill 返回后释放 capability；background skill 把 release 交给 tool-stream stopped frame。

状态和失败边界:

- server state 是 process-local: skills、rpc calls、SSE queues、capability registry。
- port conflict 是外部环境失败。live server 存在时，测试必须用隔离 MCP port。
- tool not found 应返回结构化 MCP text result，而不是 crash server。

### 15. Runtime Hardening

相关文件:

- `dimos/core/coordination/module_coordinator.py`
- `dimos/core/coordination/worker_manager_python.py`
- `dimos/simulation/mujoco/mujoco_process.py`
- `dimos/robot/unitree/mujoco_connection.py`
- `dimos/robot/unitree/go2/connection.py`
- `dimos/robot/unitree/go2/test_connection.py`

实现逻辑:

- coordinator 的 system-module notification 避免等待已知 agent client。
- worker manager/coordinator 变更保留 module restart、reload、stream rewiring 行为。
- MuJoCo helper 可以读取当前可用 stderr，避免阻塞。
- macOS headless MuJoCo 在需要时使用当前 Python executable。
- Go2 WebRTC connection 会从 `GlobalConfig` 转发 AES key。

Review 重点:

- 这些是运行时稳定性修复，重点审查 process lifecycle 和 stream rewiring，而不是 agent reasoning。

组成部分:

- ModuleCoordinator notification path: 控制 build/start/system-module notification 顺序。
- WorkerManagerPython: 在 workers 中部署 modules，并管理 worker pool lifecycle。
- MuJoCo process helpers: 处理 subprocess executable 选择和 stderr draining。
- Go2 connection config: replay stop 行为和 WebRTC AES key forwarding。

数据流:

1. build/deploy 通过 ModuleCoordinator 和 worker managers 完成。
2. modules 收到 system-module notification 前，stream transports 和 RPC refs 已经 wiring。
3. 部分 notification 使用 nowait，避免等待正在通过 MCP 初始化的 agent client。
4. restart/reload paths 必须保留 transports 并重新 wiring module refs。
5. simulator helper code 把平台特定 process 问题隔离在机器人栈外层。

状态和写边界:

- Coordinator 持有 deployed module proxy maps、transport registry、module transports、reload aliases。
- 这些改动不改变 skill semantics。
- 它们影响 module 是否能干净 start、stop、reload、reconnect。

失败形态:

- build/start 卡住时应 stop managers 并暴露原始异常。
- restart 不应遗留 old transports，也不应让 consumer 连在 dead proxy 上。
- ReplayConnection stop 应该是 no-op，而不是 exception。

### 16. Rerun/Websocket Dashboard World-model State

相关文件:

- `dimos/web/websocket_vis/websocket_vis_module.py`
- `dimos/web/websocket_vis_spec.py`
- `dimos/web/templates/rerun_dashboard.html`
- `dimos/web/websocket_vis/test_websocket_vis_module.py`

实现逻辑:

- websocket visualization 除已有 payload 外，还可以携带 world-model/dashboard state。
- dashboard payload shape 与 `world_model_contract.py` 对齐。

Review 重点:

- dashboard state 应保持展示用途。
- dashboard consumer 不应依赖未 version 的内部 Python object。

组成部分:

- `websocket_vis_spec.py`: visualization update 的 typed surface。
- `websocket_vis_module.py`: 存储/publish dashboard state 的 runtime module。
- `rerun_dashboard.html`: browser-facing display surface。
- `world_model_contract.py`: world-model dashboard payload 的 schema source。

数据流:

1. CausalWorldModel 使用 `dimos.world_model_dashboard.v1` 生成 dashboard payload。
2. 如果 WebsocketVis 已接线，CausalWorldModel 调用 `set_world_model_state(...)`。
3. WebsocketVis 保存并向 dashboard clients 提供 latest state。
4. dashboard 把 state 渲染成人类 inspection data。

状态和持久化:

- dashboard state 是 latest display snapshot，不是 source of truth。
- 如果配置了 durable state，它属于 CausalWorldModel save/load JSON。
- browser/dashboard consumer 只应依赖 versioned payload fields。

安全边界:

- dashboard update 不能触发机器人动作。
- UI 不应把 advisory prediction 展示成 guaranteed outcome。

## 推荐 Review 顺序

1. 先看 Go2 blueprint wiring:
   `unitree_go2_agentic.py`、Layer 3/4/5/6 `__init__.py`、`test_world_state.py`。
2. 再看 MCP runtime plumbing:
   `mcp_server.py`、`mcp_client.py`、`tool_stream.py` 和对应测试。
3. 再看自进化写入边界:
   `evolution_ledger.py`、`evolution_proposal.py`、`skill_proposal.py`。
4. 再看 context 质量:
   `context_provider.py`、`context_evidence.py`、`context_feedback.py`。
5. 再看 action safety:
   `skill_interface_registry.py`、`task_feasibility.py`、`skill_outcome_predictor.py`。
6. 再看 advisory world-model:
   `causal_world_model.py` 和 `world_model_contract.py`。
7. 最后单独 review 上游 transport 变化，尤其是 macOS 默认 Zenoh。

## 本次同步后的验证结果

已通过:

- `.venv/bin/python -m pytest dimos/agents/mcp/test_mcp_server.py -q`
  - `10 passed, 1 warning`
- `.venv/bin/python -m pytest dimos/robot/unitree/go2/test_connection.py -q`
  - `5 passed`
- `.venv/bin/python -m pytest dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain -q`
  - `62 passed`
- `.venv/bin/python -m pytest dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/test_world_state.py -q`
  - `3 passed, 1 warning`
- `.venv/bin/python -m pytest dimos/perception/test_spatial_perception_config.py -q`
  - `2 passed`
- `DIMOS_TRANSPORT=lcm .venv/bin/python -m pytest dimos/core/coordination/test_module_coordinator.py -q`
  - `35 passed`
- `DIMOS_TRANSPORT=lcm MCP_PORT=9991 .venv/bin/python -m pytest dimos/agents/mcp/test_mcp_client_unit.py dimos/agents/mcp/test_tool_stream.py -q`
  - `44 passed, 1 warning`
- `.venv/bin/python -m pytest dimos/robot/test_all_blueprints_generation.py -q`
  - `1 passed`
- `git diff --cached --check -- ':(exclude)*.patch'`
  - passed

已知 caveat:

- 本机有 live `dimos-live` 进程占用默认 MCP 端口 `9990`，所以默认端口跑 MCP 测试会失败。
- 上游在 macOS 默认 `zenoh` backend 后，部分测试暴露 native pyo3 thread 生命周期问题。同样 coordinator 和 MCP 测试在 `DIMOS_TRANSPORT=lcm` 下通过。
- 上游 `.patch` 文件包含会触发完整 `git diff --check` 的 whitespace；当前验证排除了 `.patch`，避免破坏补丁语义。
