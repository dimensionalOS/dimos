# S2-T2 朋友 MCP 单 Runtime 任务层组合计划

> **执行要求：** 使用 `executing-plans` 技能按测试、实现、replay、验收顺序执行。本任务不下发真机移动指令。

**目标：** 让朋友仓库的 `components/dimos-mcp` 成为唯一产品组合入口，在同一个 DimOS Runtime 中加载一个 `SemanticWorld`、一个 `MissionExecutor`、一个 navigator 和一个 MCP Server；replay 能发现并调用任务生命周期工具，默认产品工具面不能用旧低层动作绕过任务执行器。

**边界：**

- 复用 `dimos-go2-studio` 源码，不复制任务、语义世界或恢复逻辑。
- 只组合 module atoms，不嵌套启动完整 `go2_studio_agentic` Blueprint。
- `dry-run` 使用同一任务层和一个无硬件 replay navigator。
- `go2` 必须显式配置语义地图 ID 和版本；缺失时启动前失败。
- 产品 MCP 只暴露任务生命周期、停止和只读状态/观察工具；旧低层运动工具只留在显式 maintenance profile。
- `stop_all` 必须同时取消活动 mission，并继续执行全部底层停止动作。
- 集成测试前停止现有陈旧 Runtime，避免 LCM/端口/模块图冲突；测试结束不自动移动机器狗。

---

## Task 1：先写组合、依赖和产品工具面失败测试

**Files**

- Create:
  `/Users/johnsonmac/ai_completion/agent/components/dimos-mcp/tests/test_product_blueprint.py`
- Modify:
  `/Users/johnsonmac/ai_completion/agent/components/dimos-mcp/tests/test_dimos_integration.py`

**测试：**

1. `pyproject.toml` 明确声明 `dimos-go2-studio`。
2. dry-run Blueprint 恰好有一个 `SemanticWorld`、一个 `MissionExecutor`、一个 navigator、一个 `DogMcpServer`。
3. Go2 Blueprint 恰好有一个 `GO2Connection`、一个 `MissionExecutor`、一个 `SemanticWorld`、一个 navigator、一个 MCP Server。
4. 没有 embedded LLM/`McpClient`/第二套完整 Studio Blueprint。
5. 产品工具面包含 `start_task/pause_task/resume_task/cancel_task/get_task_status/list_semantic_places/stop_all`，不包含旧低层移动、探索、巡逻和运动指令。
6. maintenance profile 仍保留现有人工调试工具。
7. Go2 缺 map ID/version 时在构建模块前失败。

**RED 命令：**

```bash
cd /Users/johnsonmac/ai_completion/agent/components/dimos-mcp
PYTHONPATH=src:/Users/johnsonmac/ai_completion/dimos/extensions/go2-studio-agent/src \
  /Users/johnsonmac/ai_completion/dimos/.venv/bin/python -m unittest discover \
  -s tests -p test_product_blueprint.py -v
```

---

## Task 2：组合 P1/S2 模块并增加显式运行配置

**Files**

- Modify: `components/dimos-mcp/pyproject.toml`
- Modify: `components/dimos-mcp/src/dimos_dog_mcp/config.py`
- Modify: `components/dimos-mcp/src/dimos_dog_mcp/blueprint.py`
- Modify: `components/dimos-mcp/src/dimos_dog_mcp/navigation.py`
- Modify: `components/dimos-mcp/src/dimos_dog_mcp/tool_contract.py`
- Modify: `components/dimos-mcp/src/dimos_dog_mcp/server.py`
- Modify: `components/dimos-mcp/config/go2.env.example`

**实现：**

- 声明 `dimos-go2-studio==0.1.0` 和本地开发 source。
- 增加 `SemanticRuntimeConfig` 与 `ToolProfile`。
- dry-run navigator 实现 `NavigationInterfaceSpec`，只模拟 task active/cancel，不发布运动。
- 两种 Runtime 都组合一个 `SemanticWorld` 和一个 `MissionExecutor`。
- 产品/maintenance 工具 allowlist 明确分离。

---

## Task 3：统一停止活动 mission 并完成 MCP replay

**Files**

- Modify:
  `extensions/go2-studio-agent/src/dimos_go2_studio/mission_executor.py`
- Modify:
  `extensions/go2-studio-agent/tests/test_mission_executor.py`
- Modify friend `stop.py`、`go2_stop.py`、`stop_actions.py` 和对应 tests。
- Modify friend `tests/test_dimos_integration.py`。

**实现与 replay：**

1. `MissionExecutor` 增加非 Agent 工具的 `cancel_active_task()` RPC。
2. `stop_all` 先取消任务，再继续导航、探索、巡逻、视觉和零速度停止。
3. 在临时语义库中确认 `测试点`。
4. 通过真实本地 MCP HTTP 链路调用：
   - `start_task`
   - `get_task_status`
   - `cancel_task`
   - `get_task_status`
5. 验证相同 task ID 从 queued/navigating 进入 cancelled，replay navigator 回到 idle。

---

## Task 4：聚焦验收、文档与 Runtime 边界

**验证：**

```bash
cd /Users/johnsonmac/ai_completion/agent/components/dimos-mcp
PYTHONPATH=src:/Users/johnsonmac/ai_completion/dimos/extensions/go2-studio-agent/src \
  /Users/johnsonmac/ai_completion/dimos/.venv/bin/python -m unittest discover \
  -s tests -p test_product_blueprint.py -v
PYTHONPATH=src:/Users/johnsonmac/ai_completion/dimos/extensions/go2-studio-agent/src \
  /Users/johnsonmac/ai_completion/dimos/.venv/bin/python -m unittest discover \
  -s tests -p test_dimos_integration.py -v
PYTHONPATH=src:/Users/johnsonmac/ai_completion/dimos/extensions/go2-studio-agent/src \
  /Users/johnsonmac/ai_completion/dimos/.venv/bin/python -m unittest discover \
  -s tests -p test_stop_all.py -v

cd /Users/johnsonmac/ai_completion/dimos
uv run pytest \
  extensions/go2-studio-agent/tests/test_semantic_world.py \
  extensions/go2-studio-agent/tests/test_mission_executor.py \
  extensions/go2-studio-agent/tests/test_blueprint.py -q

uv run ruff check \
  extensions/go2-studio-agent/src/dimos_go2_studio \
  extensions/go2-studio-agent/tests

cd /Users/johnsonmac/ai_completion/agent
/Users/johnsonmac/ai_completion/dimos/.venv/bin/ruff check \
  components/dimos-mcp/src components/dimos-mcp/tests
git diff --check
```

**通过标准：**

- 两种模式都只有一个任务执行器和一个语义世界。
- Go2 模式仍只有一个连接、一个 navigator、一个 MCP Server。
- replay 经 MCP 完成 start/status/cancel/status，且没有硬件 I/O。
- 默认产品面无法调用旧低层动作绕过 active mission。
- `stop_all` 会取消 mission，最终 replay navigator idle。
- 聚焦测试、Ruff 和两个仓库 `git diff --check` 全部通过。
- DIMOS 当前初始化的 Zenoh 后台线程在 macOS 上会阻止部分 `unittest`
  进程自然退出；验收时必须在看到明确 `OK` 后回收独立测试子进程，不能让测试
  Runtime 残留并占用后续通信资源。

**不在本任务中声明：**

- 不声明机器狗已完成语义导航。
- 不声明里程计/相机已经重新变 fresh。
- 不创建真实地点，不执行 S2-R1。
