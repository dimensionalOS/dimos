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

Git 内容会提交到哪里:

- 文件先进入当前本地仓库工作区。
- commit 只进入本地 `.git` 历史。
- 不会自动 push 到 GitHub。
- 只有人类后续运行 `git push` 或开 PR，GitHub 才会收到这些记录。

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

Review 重点:

- 当前 causal estimate 只能作为 advisory evidence。
- prediction schema 要对 dashboard 和 LLM prompt consumer 保持稳定。
- persistence 不应意外混合不同 run 的数据。

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

Review 重点:

- 这是最安全的自进化边界: 只生成 proposal，必须人工 review。
- evidence requirement 要足够严格，避免用户表达不清时产生大量噪声 proposal。

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
