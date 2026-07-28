# Agent 官方中央人员跟随 Implementation Plan

**Goal:** 用户说“跟着我”后，朋友仓库的 Agent 启动 DimOS 官方
`PersonFollowSkillContainer`，以启动画面中央的人作为首次视觉目标，并可用统一
`stop_all` 停止。

**Architecture:** 不实现第二套检测、跟踪或运动控制。SiliconFlow
`Qwen/Qwen3-VL-8B-Instruct` 只替换官方 Qwen VL provider，用于首次人物 bbox；
DimOS 官方 EdgeTAM 随后以本地 20Hz 跟踪，官方 `VisualServoing2D` 直接发布
`cmd_vel`。朋友 `components/dimos-mcp` 仍是唯一 Go2 Runtime，Wrapper 只做单次
转发，Gateway 只把明确的“跟着我”编译为严格 `follow_person` 意图并调用官方工具。

**明确边界:** 这是快速复用官方能力的 MVP，不是 canonical `TaskSpec`，不创建
instruction/task binding，不提供地图轨迹、自动重识别或到达终态。官方实现明确不做
障碍物避让，假设路径清空；因此软件通过不等于在人流或杂物环境中真机跟随通过。

---

### Task 1: Agent 严格跟随意图

**Files:**

- `agent/components/agent-framework/agent-webhook-gateway/src/agent-runtime.ts`
- `agent/components/agent-framework/agent-webhook-gateway/src/task-contract.ts`
- `agent/components/agent-framework/agent-webhook-gateway/src/service.ts`
- 对应 Vitest

**Acceptance:**

1. 只接受严格 `{"kind":"follow_person"}`，不接受模型生成的人物描述或 task ID。
2. Gateway 固定调用 `follow_person(query="the person closest to the center of the image")`。
3. 只有官方响应包含 `Starting to follow` 才回复已启动。
4. 跟随不伪装成已完成的 canonical 任务，也不创建 task binding。

### Task 2: 官方 Qwen provider 适配

**Files:**

- `dimos/models/vl/qwen.py`
- `dimos/models/vl/test_qwen_provider_config.py`
- `dimos/navigation/visual/query.py`
- `dimos/navigation/visual/test_query.py`

**Acceptance:**

1. 支持 `DIMOS_QWEN_VL_API_KEY`、`DIMOS_QWEN_VL_BASE_URL`、
   `DIMOS_QWEN_VL_MODEL`，并保留原 Alibaba 默认兼容。
2. bbox 提示要求严格 JSON 和 0–1000 标准坐标；解析后统一转换为真实图像像素。
3. 缺 key、非 JSON、越界或反向 bbox 均 fail-closed。
4. 使用非机器人测试图对官方 `QwenVlModel + get_object_bbox_from_image` 做一次
   真实 SiliconFlow smoke test，不记录 key 或图片。

### Task 3: 在唯一 Go2 Blueprint 中恢复官方跟随

**Files:**

- `agent/components/dimos-mcp/src/dimos_dog_mcp/blueprint.py`
- `agent/components/dimos-mcp/src/dimos_dog_mcp/tool_contract.py`
- `agent/components/dimos-mcp/pyproject.toml`
- 对应 Blueprint/tool profile 测试

**Acceptance:**

1. Go2 Blueprint 只组合一个 `PersonFollowSkillContainer`，摄像头内参来自同一个
   `GO2Connection.camera_info_static`。
2. 不添加第二个 Agent、第二个 Go2 connection 或自研 tracker。
3. product profile 公开 `follow_person`；maintenance 额外保留官方
   `stop_following`。
4. Go2 extra 安装官方跟随所需 `misc/perception` 依赖，并通过本地 DimOS editable
   source 使用 provider 适配。

### Task 4: Wrapper 与统一停止

**Files:**

- `agent/components/agent-framework/dimos-mcp-wrapper/`
- `agent/components/dimos-mcp/src/dimos_dog_mcp/go2_stop.py`
- `agent/components/dimos-mcp/src/dimos_dog_mcp/stop_actions.py`
- 对应 unittest

**Acceptance:**

1. Wrapper product profile 单次、原样转发 `follow_person(query)`。
2. `stop_all` 调用官方 `stop_following`，即使其他停止项失败也继续发布本地停止。
3. 不向 product profile 单独公开 `stop_following`；用户统一发送“停”。

### Task 5: 配置、检查与真机边界

**Files:**

- `agent/components/dimos-mcp/config/go2.env.example`
- `agent/components/dimos-mcp/scripts/run-go2-mcp.sh`
- `agent/USAGE.md`
- `agent/CONTEXT.md`
- `dimos/docs/PROJECT_CONTEXT.md`

**Acceptance:**

1. 仓库只记录 SiliconFlow base URL 与 model ID；真实 key 只在私有配置或 Keychain。
2. Python、TypeScript 定向测试、Ruff、`git diff --check` 和密钥扫描通过，或明确记录
   既有无关阻塞。
3. 本轮不启动 Go2、不发运动命令。真机验收必须另行确认：
   初始画面单人中央锁定、短距离直线路径、目标丢失停止、用户“停”立即停止。
4. 杂物、人流、交叉人员、遮挡重识别属于后续“避障感知 + 动态导航”升级，不能用
   该官方 MVP 的软件结果代替。
