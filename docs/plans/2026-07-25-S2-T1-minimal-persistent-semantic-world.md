# S2-T1 最小持久化语义地点实施计划

> **执行要求：** 使用 `executing-plans` 技能按任务逐项实现和验证。本计划只覆盖 Stage 2 的 S2-T1，不触发真机运动。

**目标：** 在 DimOS Go2 Studio 扩展中增加一个只保存“人工确认地点”的持久化 `SemanticWorld`，并让唯一的 `MissionExecutor` 通过 DimOS `Spec` 自动连接它，把 `go_to_place` 的语义名称解析成当前地图坐标。

**边界：**

- 只保存人工确认地点；本任务不接视觉模型、不自动识别地点。
- 每个地点保存稳定 ID、名称、别名、地图 ID/版本、地图坐标、确认与更新时间。
- 当前地图 ID 或版本不匹配时必须 fail-closed，不给导航发布目标。
- 同名更新保留稳定 ID；名称或别名冲突必须拒绝。
- JSON 使用同目录临时文件和 `os.replace` 原子写入；损坏文件不能被静默覆盖。
- 不启动第二套 Runtime，不向当前已连接的机器狗下发移动指令。

---

## Task 1：先写契约与持久化失败测试

**Files**

- Create: `extensions/go2-studio-agent/tests/test_semantic_world.py`

**测试：**

1. 保存两个确认地点，重新构造 `SemanticWorld` 后仍可读。
2. 中文名称和别名都能解析到同一稳定地点。
3. 地图 ID/版本不匹配时解析失败。
4. 同名更新保留实体 ID；名称/别名冲突被拒绝。
5. 损坏 JSON 启动失败且原文件不被覆盖。
6. 未配置地图身份时确认和解析均 fail-closed。

**RED 命令：**

```bash
uv run pytest extensions/go2-studio-agent/tests/test_semantic_world.py -q
```

预期：因 `dimos_go2_studio.semantic_world` 尚不存在而失败。

---

## Task 2：实现最小 SemanticWorld

**Files**

- Create: `extensions/go2-studio-agent/src/dimos_go2_studio/semantic_world.py`

**实现：**

- 定义严格、不可变的 `SemanticPlacePose`、`SemanticPlace`、`SemanticWorldDocument`。
- 定义 `DestinationResolverSpec(Spec, Protocol)`。
- `SemanticWorld(Module)` 提供：
  - `confirm_place(...)`：内部/测试使用的强类型确认接口；
  - `confirm_semantic_place(place_json)`：RPC 边界；
  - `list_semantic_places()`：只读 Agent skill；
  - `resolve(task)`：只为当前地图解析 `TaskSpec.destination`；
  - `resolve_detail(task)`：提供 resolved/unresolved/map_mismatch 诊断。
- 使用 NFKC、去首尾空格和 `casefold` 做索引规范化。
- 使用 `RLock` 和原子 JSON 落盘。

**GREEN 命令：**

```bash
uv run pytest extensions/go2-studio-agent/tests/test_semantic_world.py -q
```

---

## Task 3：接入唯一 MissionExecutor 与产品 Blueprint

**Files**

- Modify: `extensions/go2-studio-agent/src/dimos_go2_studio/mission_executor.py`
- Modify: `extensions/go2-studio-agent/src/dimos_go2_studio/blueprint.py`
- Modify: `extensions/go2-studio-agent/tests/test_mission_executor.py`
- Modify: `extensions/go2-studio-agent/tests/test_blueprint.py`

**实现：**

- `MissionExecutor` 增加 `_destination_resolver: DestinationResolverSpec` 注入引用。
- 保留构造器 `resolver=` 测试覆盖；生产路径优先使用自动连接的 `SemanticWorld`。
- Blueprint 中只增加一个 `SemanticWorld`，且仍保持一个 `MissionExecutor`、一个官方 Go2 栈。

**测试：**

1. `MissionExecutor.start_task()` 能把已知语义地点解析为准确的 map-frame `PoseStamped` 并交给导航。
2. 未知地点、地图不匹配不调用 `set_goal()`。
3. Blueprint 恰好包含一个 `SemanticWorld`，执行器引用顺序为 `_navigation`、`_destination_resolver`。

---

## Task 4：S2-T1 聚焦验收与文档更新

**验证命令：**

```bash
uv run pytest \
  extensions/go2-studio-agent/tests/test_semantic_world.py \
  extensions/go2-studio-agent/tests/test_mission_executor.py \
  extensions/go2-studio-agent/tests/test_blueprint.py -q
uv run ruff check \
  extensions/go2-studio-agent/src/dimos_go2_studio/semantic_world.py \
  extensions/go2-studio-agent/src/dimos_go2_studio/mission_executor.py \
  extensions/go2-studio-agent/src/dimos_go2_studio/blueprint.py \
  extensions/go2-studio-agent/tests/test_semantic_world.py \
  extensions/go2-studio-agent/tests/test_mission_executor.py \
  extensions/go2-studio-agent/tests/test_blueprint.py
git diff --check
```

**S2-T1 通过标准：**

- 两个确认地点可保存、重启后可恢复。
- 别名解析正确，稳定 ID 不因同名更新变化。
- 地图身份不匹配、未知地点、损坏存储均 fail-closed。
- `start_task` 能把已知地点解析成准确 map-frame goal。
- Blueprint 只有一个 `SemanticWorld` 和一个 `MissionExecutor`。
- 聚焦测试、Ruff 和 `git diff --check` 全部通过。

**回滚：**

- 移除 Blueprint 中 `SemanticWorld.blueprint()`。
- `MissionExecutor` 恢复默认不可用 resolver。
- 保留存储文件不删除，避免用户确认地点丢失。
