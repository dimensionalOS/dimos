# SeatGuide 机器狗空位引导 Step-by-Step 计划

目标：让用户通过浏览器麦克风或文字对 Go2 说“帮我找一个空位”，系统用真实相机识别椅子和人，判断空位，给导航下发目标，并通过网页/手机反馈结果。没有连接 G2 时，所有能本地验证的模块都必须有单测或 smoke 验证；连接 G2 后只跑硬件验收，不再临场拼功能。

## 总体模块拆分

| 模块 | 负责什么 | 输入 | 输出 | 是否可并行 | 当前验证方式 |
| --- | --- | --- | --- | --- | --- |
| 1. 基础语音/文字控制入口 | 接收浏览器麦克风、浏览器文字、普通 agent text，先识别普通运动/姿态命令，再识别找座位请求 | WebInput `/submit_query`、`/upload_audio`、Whisper 文本、agent text | 普通 agent tool call，或 SeatGuide preview/live 请求 | 是 | MCP tool 验收、WebInput 单测、HTTP TestClient、硬件验收脚本 |
| 2. 场景感知 | 用 Go2 RGB 图像 + odom + Moondream2/VLM 识别椅子和人，并投影成 map 坐标 | `color_image`、`odom`、本地 Moondream2 模型缓存 | `SeatSceneObservation` | 是 | Camera provider 单测、`camera_seat_provider_status` |
| 3. 空位规划 | 判断哪些椅子被占用，选择最近空位，生成机器人应到达的引导点 | 椅子位姿、人员位置、机器人位置 | 选中椅子、导航目标 pose | 是 | Planner 单测、`preview_empty_seat_goal` |
| 4. 导航执行 | 把目标 pose 发给已有导航模块，并读取完成状态 | SeatGuide goal pose | `set_goal()`、`goal_reached` | 部分并行 | fake navigator 单测、`seat_guide_navigation_status` |
| 5. 手机/网页反馈 | 告诉用户找到哪个位置、是否需要跟随、失败原因 | SeatGuide 结果文本 | web response text、手机扬声器 relay | 是 | `web_input_status`、可选 `phone_speaker_test` |
| 6. 验收脚本 | 把 no-motion、真实语音、真实导航串起来，保存 transcript | 当前 DimOS stack | 通过/失败原因、验收日志 | 是 | `bin/demo_seat_guide_*` |

## 阶段 1：基础语音控制验收

目的：先证明“人说一句话或输入一句话 -> 系统识别意图 -> 下发到 Go2 -> Go2 执行动作”这条最小闭环成立。这个阶段不做找空位，不依赖 VLM，不依赖座椅识别。

要做的工作：

1. 用 MCP 直接调用姿态/移动工具，证明 Go2 控制工具本身可用。
2. 用浏览器文字输入普通运动命令，证明文字能进入 agent 并触发 Go2 tool。
3. 用浏览器麦克风说普通运动命令，证明麦克风 -> Whisper -> agent -> Go2 tool 链路可用。
4. 验证停止/安全命令，确保每次小距离动作后可以停下。
5. 只验收低风险动作：站立、恢复站立、小距离前进/后退、小角度转向；不要在第一阶段测试跳跃、翻滚等动态动作。

### 阶段 1 的可验收路径拆分

| 路径 | 入口 | 是否会让 Go2 移动 | 验收命令/动作 | 通过标准 |
| --- | --- | --- | --- | --- |
| 1A. MCP 姿态命令 | MCP tool | 可能改变姿态，不走位 | `dimos mcp call execute_sport_command --json-args '{"command_name":"BalanceStand"}'` | tool 返回成功，Go2 进入稳定站立/平衡状态 |
| 1B. MCP 小距离移动 | MCP tool | 是，小距离 | `relative_move` 前进 0.3m、后退 0.3m、左转 30 度 | Go2 按命令小幅移动或导航状态显示目标完成 |
| 1C. 浏览器文字 -> agent -> Go2 tool | Web 页面文字框或 `/submit_query` | 是，小距离 | 输入 `walk forward 30 centimeters`、`walk backward 30 centimeters` | 日志显示 WebInput 收到文本，非找座位请求进入普通 agent path，agent 调用 `relative_move` |
| 1D. 浏览器麦克风 -> Whisper -> agent -> Go2 tool | 电脑浏览器麦克风 | 是，小距离 | 对浏览器说 `walk forward 30 centimeters` 或中文等价命令 | 日志显示 Whisper 识别文本，agent 调用对应 Go2 tool，Go2 执行动作 |
| 1E. 停止/安全 | MCP tool 或 agent tool | 停止当前导航/动作 | `dimos mcp call stop_navigation` | 导航状态回到停止/空闲，不再继续移动 |

推荐验收命令：

```bash
dimos mcp call execute_sport_command --json-args '{"command_name":"BalanceStand"}'
dimos mcp call relative_move --json-args '{"forward":0.3,"left":0,"degrees":0}'
dimos mcp call relative_move --json-args '{"forward":-0.3,"left":0,"degrees":0}'
dimos mcp call relative_move --json-args '{"forward":0,"left":0,"degrees":30}'
dimos mcp call stop_navigation
```

通过标准：

- MCP 直接调用能让 Go2 执行姿态和小距离移动。
- 浏览器文字命令能触发普通 agent tool，而不是误进 SeatGuide。
- 浏览器麦克风命令能完成语音识别，并触发同一个 Go2 tool。
- 任何一次动作失败时，可以定位失败点是控制工具、agent tool selection、Whisper、WebInput，还是 Go2 连接。
- 当前默认输入设备是 **电脑浏览器麦克风**，不是 Go2 机身麦克风；如果要使用 Go2 自带麦克风，需要后续单独增加输入模块。

## 阶段 2：SeatGuide 基础模块开发和本地单测

目的：不接机器狗也能证明核心逻辑正确。

要做的工作：

1. 实现 SeatGuide 数据模型：椅子、人员、场景、规划结果、语音意图。
2. 实现空位判断：人在椅子附近 0.75m 内则认为占用。
3. 实现最近空位选择：从机器人当前位置选最近的空椅子。
4. 实现引导点生成：目标点在椅子旁边的过道方向，而不是椅子中心。
5. 实现 preview 和 live 两种路径：preview 不移动，live 才下发导航。

验收方式：

```bash
uv run pytest dimos/agents/skills/test_seat_guide.py -q -k 'planner or find_empty_seat or preview_empty_seat_goal'
```

通过标准：

- 能选出正确空位。
- 被占用椅子不会被选中。
- preview 不调用导航。
- live 只在可导航且场景来源可信时调用导航。

## 阶段 3：SeatGuide 语音和 WebInput 链路

目的：用户可以通过浏览器输入或麦克风触发 SeatGuide，而不是必须手动 MCP call。

要做的工作：

1. WebInput 文字输入 `/submit_query` 直接路由 SeatGuide 请求。
2. 浏览器音频上传 `/upload_audio` 推入 `audio_subject`。
3. Whisper 自动识别语言，不强制英文。
4. 中文 preview 语句：`预检帮我找一个空位` 只做 no-motion 检查。
5. 中文 live 语句：`帮我找一个空位` 才触发导航。
6. WebInput 把 SeatGuide 返回结果推到 `agent_responses`，浏览器可见。

### 阶段 3 的可验收路径拆分

| 路径 | 入口 | 是否会让 Go2 移动 | 验收命令/动作 | 通过标准 |
| --- | --- | --- | --- | --- |
| 3A. 模块状态检查 | MCP tool | 否 | `dimos mcp call web_input_status` | 输出包含 `web=started`、`voice_upload=connected`、`stt=connected`、`seat_route=seat_guide_direct` |
| 3B. 浏览器文字 preview | Web 页面文字框或 `/submit_query` | 否 | 在 WebInput 页面输入 `预检帮我找一个空位`，或用硬件脚本自动 POST | `agent_responses` 出现 `SeatGuide preflight ready` 或明确 no-go 原因；导航目标不会下发 |
| 3C. 浏览器麦克风 preview | 电脑浏览器麦克风 | 否 | 打开 WebInput URL，允许麦克风，点击麦克风后说 `预检帮我找一个空位` | Whisper 识别文本后，WebInput 日志包含 `WebInput received text` 和 `WebInput routing text to SeatGuide preview` |
| 3D. 普通 agent text fallback | `/human_input` agent path | 否，除非 agent 后续显式调用工具 | 输入非找座位文本，例如 `what time is the meeting` | WebInput 不调用 SeatGuide，文本进入普通 agent path |
| 3E. 浏览器麦克风 live | 电脑浏览器麦克风 | 是 | 只在 no-motion 通过且现场安全后，说 `帮我找一个空位` | WebInput 日志包含 `WebInput routing text to SeatGuide live request`，SeatGuide 返回 `Navigating to ...` |

推荐验收顺序：

1. 先跑 3A，确认 WebInput/STT/浏览器音频上传都在线。
2. 再跑 3B，确认文字入口和 SeatGuide preview 直连。
3. 再跑 3C，确认电脑浏览器麦克风 -> Whisper -> SeatGuide preview 直连。
4. 最后才跑 3E，因为它会下发真实导航目标。

注意：当前方案默认使用 **电脑浏览器麦克风**，不是直接使用 Go2 机身麦克风。用户对电脑浏览器说话，电脑把音频上传到 DimOS，Whisper 识别文本，然后 SeatGuide 给 Go2 下发导航。

验收方式：

```bash
uv run pytest dimos/agents/skills/test_seat_guide.py -q -k 'web_input or upload_audio or submit_query'
```

通过标准：

- `/submit_query` 能触发 SeatGuide preview。
- `/upload_audio` 能产生 `AudioEvent`。
- 未配置音频入口或音频解码失败时不能误报成功。
- `web_input_status` 必须包含：
  - `web=started`
  - `thread=running`
  - `seat_route=seat_guide_direct`
  - `responses=connected`
  - `voice_upload=connected`
  - `stt=connected`
  - `human_transport=connected`

## 阶段 4：真实相机/VLM/odom 感知

目的：不能用假的 mock 当最终结果，硬件验收必须证明来自真实 camera source。

要做的工作：

1. `CameraSeatObservationProvider` 订阅 `color_image` 和 `odom`。
2. 使用 Qwen/VLM 分别检测 `chair` 和 `person`。
3. 根据图像框中心和 odom 估算 map-frame 椅子/人员位置。
4. 检查 stale image、stale odom、missing key、missing camera 等 no-go 状态。
5. 保留 `set_seat_scene` 作为 fallback/calibration，但硬件验收不接受 fallback。

验收方式：

```bash
uv run pytest dimos/agents/skills/test_seat_guide.py -q -k 'camera_observation_provider or camera_seat_provider_status'
```

硬件前检查：

```bash
dimos mcp call camera_seat_provider_status
dimos mcp call seat_guide_status
```

通过标准：

- `camera_seat_provider_status` 显示：
  - `image=<width>x<height>`
  - `image_fresh=true`
  - `odom=(...)`
  - `odom_fresh=true`
  - `credential=present`
  - `override=inactive`
  - `configured_fallback_seats=0`
  - `configured_fallback_people=0`
- `seat_guide_status` 必须以 `SeatGuide scene source=camera:` 开头。

## 阶段 5：导航和手机/网页反馈

目的：找到空位以后，Go2 能真正下发导航目标，并通过网页或绑在机器狗上的手机给用户可见/可听反馈。

要做的工作：

1. SeatGuide 注入 `NavigationInterfaceSpec`。
2. live request 时调用 `set_goal(PoseStamped)`。
3. 如果导航忙，拒绝覆盖当前任务。
4. 读取 `navigation_state` 和 `goal_reached`。
5. SeatGuide 返回明确的结果文本，并让 WebInput 发布到 `agent_responses`。
6. 如果需要可听反馈，让手机打开 speaker relay 页面并用 `phone_speaker_test` 验证。

验收方式：

```bash
uv run pytest dimos/agents/skills/test_seat_guide.py -q
```

硬件前检查：

```bash
dimos mcp call seat_guide_preflight
dimos mcp call preview_empty_seat_goal
dimos mcp call seat_guide_navigation_status
```

通过标准：

- preflight 显示 `navigation=IDLE`。
- preview 有 `selected=...` 和 `goal=(...)`。
- WebInput response stream 可见；需要声音时手机 relay 可播放测试消息。
- live 后 `seat_guide_navigation_status` 最终显示新的 `goal_sequence` 且 `goal_reached=true`。

## 阶段 6：Mac replay / SHM 视频流修复

目的：Mac 上 replay 的高带宽视频/点云/地图流不再因为 UDP/LCM 大包路径缺失导致看不到视频。

要做的工作：

1. 集成 `dimensionalOS/dimos#2245` / `danvi/experimental/route-replay-through-SHM`。
2. 将 Go2 replay 的高带宽流 route 到 `pSHMTransport`：
   - `color_image`
   - `lidar`
   - `pointcloud`
   - `global_map`
   - `merged_map`
   - `global_costmap`
   - `navigation_costmap`
3. Rerun bridge 接收 SHM visual transports。

验收方式：

```bash
uv run pytest dimos/protocol/pubsub/test_registry.py dimos/visualization/rerun/test_viewer_integration.py -q
bin/demo_seat_guide_replay_smoke
```

通过标准：

- replay stack 能启动。
- 日志里高带宽流显示 `transport=pSHMTransport`。
- `bin/demo_seat_guide_replay_smoke` 能完整跑完并停止 stack。

## 阶段 7：真实 Go2 bring-up

目的：你接上机器狗后只跑一个入口，不需要手动拼命令。

你需要手动准备：

1. 机器狗上电，和 Mac 在同一网络。
2. 确认 Go2 IP，默认示例是 `192.168.123.161`。
3. 可选准备普通 agent 的 API key。SeatGuide 直连语音/MCP 路径不需要 LLM key；如果选择 Qwen 作为找座位 VLM，仍然需要 Alibaba/Qwen：

```bash
export OPENROUTER_API_KEY="你的 OpenRouter key"
export OPENROUTER_MODEL="openai/gpt-4o-mini"
```

启动一键 bring-up：

```bash
bin/demo_seat_guide_hardware_bringup --robot-ip 192.168.123.161
```

这个脚本会自动执行：

1. 检查本地 Moondream2 模型缓存；如果没有 agent key，普通 agent chat 会禁用，但 SeatGuide 直连路径仍然可用。
2. 启动 `unitree-go2-seat-guide-agentic`。
3. 跑 `bin/demo_seat_guide_smoke` 做 no-motion 检查。
4. 跑 `bin/demo_seat_guide_hardware_acceptance` 做真实浏览器语音和导航验收。

你在脚本过程中需要手动做：

1. 打开脚本打印的 WebInput URL。
2. 允许浏览器麦克风权限。
3. no-motion 阶段对浏览器说：`预检帮我找一个空位`。
4. 确认 Go2 周围安全后，在终端输入 `LIVE`。
5. live 阶段对浏览器说：`帮我找一个空位`。

通过标准：

- no-motion 阶段所有 gate 通过。
- live 阶段 WebInput 日志包含中文识别文本。
- SeatGuide 返回 `Navigating to ...`。
- 最终 `seat_guide_navigation_status` 显示新的 `goal_sequence` 和 `goal_reached=true`。
- `bin/demo_seat_guide_verify_acceptance_log <log>` 通过。

## 阶段 8：失败时怎么分模块排查

| 失败位置 | 看什么命令 | 常见原因 | 处理方式 |
| --- | --- | --- | --- |
| WebInput 未启动 | `dimos mcp call web_input_status` | 端口占用、WebInput 模块未启动 | 检查 `dimos status`、重启 stack |
| 麦克风没进来 | `web_input_status`、浏览器权限 | `voice_upload=missing`、浏览器拒绝麦克风 | 允许麦克风权限，刷新 WebInput 页面 |
| STT 不工作 | `web_input_status` | Whisper/faster-whisper 初始化失败 | 看 DimOS log，确认依赖安装 |
| 没有图像 | `camera_seat_provider_status` | Go2 camera/replay stream 没到 | 转向桌子，确认 replay/SHM 流 |
| odom 缺失或过期 | `camera_seat_provider_status` | localization 没启动或 stale | 等待 odom，检查 Go2/replay stack |
| VLM 失败 | `seat_guide_status` | 本地 Moondream2 模型缺失、模型加载失败，或远程 VLM key 缺失 | 拉取模型或重新 export 对应 key，并重启 stack |
| 找不到椅子 | `seat_guide_status` | 摄像头没朝向桌子、光照/识别失败 | 调整机器人视角；只调试时可 fallback |
| 导航忙 | `seat_guide_preflight` | `navigation=FOLLOWING_PATH` 或 `RECOVERY` | 等任务结束或停止导航后重跑 |
| 手机反馈不可用 | `web_input_status` / `phone_speaker_test` | 手机没有打开 relay 页面或网络不可达 | 先确认 web response stream；需要声音时让手机访问可用的 relay 页面 |

## 当前已完成状态

已完成：

- SeatGuide planner / scene / intent / navigation integration。
- WebInput 中文语音和文字直连 SeatGuide。
- Camera/VLM/odom provider。
- WebInput response stream 和可选手机 speaker relay。
- Go2 SeatGuide blueprints。
- macOS replay SHM route 集成。
- 一键硬件 bring-up 脚本。
- no-motion smoke、hardware acceptance、acceptance log verifier。

已验证：

- SeatGuide/MCP 相关测试通过。
- pubsub/Rerun SHM 相关测试通过。
- `bin/demo_seat_guide_replay_smoke` 在 Mac 上完整跑完。

未完成：

- 真实 Go2 硬件 transcript。最终完成标准必须包含真实浏览器麦克风输入、真实 camera/VLM/odom、真实导航和 `goal_reached=true`。
