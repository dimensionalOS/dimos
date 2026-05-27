# Custom Robot Blueprints

## yoloe-tracking-test

`yoloe-tracking-test` 是一个只用于验证 YOLOE tracking 效果的 Go2 replay blueprint。它不接距离控制、不接 Agent/MCP/skill，只把 Go2 `color_image` 输入到 `YoloeTrackingModule`，并把 YOLOE 的 `Detection2DArray` 发布到 `/color_image/yoloe_detections`。

### 组成

- `unitree_go2_basic`
- `YoloeTrackingModule.blueprint()`
- 专用 Rerun viewer overlay

数据流：

```text
unitree_go2_basic.color_image
  -> YoloeTrackingModule.color_image
  -> YoloeTrackingModule.detections
  -> /color_image/yoloe_detections
  -> Rerun Camera overlay
```

### 离线模型准备

运行 `yoloe-tracking-test` 时不会拉取或解压 YOLOE 模型。需要先在有网络的环境准备模型：

```bash
git lfs pull --include data/.lfs/models_yoloe.tar.gz
uv run python -c 'from dimos.utils.data import get_data; print(get_data("models_yoloe"))'
ls -lh data/models_yoloe/yoloe-11s-seg-pf.pt
```

如果 `data/models_yoloe/yoloe-11s-seg-pf.pt` 不存在，blueprint 会在部署前失败并打印同一组预处理命令。真机或离线 replay 环境只运行 blueprint，不做模型下载。

### 启动

```bash
.venv/bin/dimos --replay run yoloe-tracking-test
```

Rerun Camera 视图会显示 `world/color_image/yoloe_detections`。bbox 标签使用 detection id；当 YOLOE 返回 tracking id 时，可以直接观察同一个目标跨帧 id 是否稳定。

## bbox-distance-follow

`bbox-distance-follow` 是一个最小可启动的 Go2 blueprint，用于验证 “检测多个 bbox -> 用户选择一个 bbox -> 根据 selected bbox + lidar + camera_info 控制 Go2 距离” 这条链路。

这个 blueprint 只保留 DimOS Module/RPC，不接 `@skill`、Agent、Prompt、MCP 或新的 UI 模块。它可以通过 `dimos run bbox-distance-follow` 启动，也可以在 replay 下通过 viewer 观察 selected bbox overlay。

### 组成

- `unitree_go2_basic`
- `Detection2DModule.blueprint(camera_info=GO2Connection.camera_info_static, publish_detection_images=False)`
- `BBoxSelectionModule.blueprint()`
- `BBoxDistanceBehaviorModule.blueprint()`

全局配置：

```python
.global_config(n_workers=6, robot_model="unitree_go2")
```

### 数据流

```text
unitree_go2_basic.color_image
  -> Detection2DModule.color_image
  -> Detection2DModule.detections
  -> BBoxSelectionModule.detections
  -> BBoxSelectionModule.selected_bbox
  -> BBoxDistanceBehaviorModule.selected_bbox
  -> BBoxDistanceBehaviorModule.cmd_vel
  -> GO2Connection.cmd_vel

unitree_go2_basic.lidar
  -> BBoxDistanceBehaviorModule.lidar

unitree_go2_basic.camera_info
  -> BBoxDistanceBehaviorModule.camera_info

dimos-viewer Camera click
  -> RerunWebSocketServer.clicked_point
  -> BBoxSelectionModule.clicked_point
  -> BBoxSelectionModule.selected_bbox
```

`Detection2DModule.detections` 明确发布到 LCM topic `/color_image/detections`，`selected_bbox` 明确发布到 LCM topic `/color_image/selected_bbox`。RerunBridge 会把它们映射到 `world/color_image/detections` 和 `world/color_image/selected_bbox`，本 blueprint 对这两个实体路径配置了专用 `visual_override`，把 YOLO 候选 bbox 显示为黄色框，把 selected bbox 显示为绿色框。Go2 viewer 的 Camera view origin 是 `world/color_image`，所以这些 bbox 会作为相机图像子实体显示在 Camera 视图上。

`publish_detection_images=False` 是有意设置：`Detection2DModule.detected_image_0/1/2` 当前是 cropped detection images，不是原始相机图像上的 bbox overlay。这个 blueprint 直接在 Rerun Camera 视图里渲染 bbox，避免 cropped images 出现在 3D view 中并触发无 Pinhole 的 2D visualizer warning。

### BBoxSelectionModule

职责：

- 消费 `Detection2DModule.detections` 的多 bbox。
- 消费 dimos-viewer 发回的 `clicked_point`，把 Camera 视图里的像素点击映射到最新一帧 bbox。
- 保存最新一帧 detections。
- 通过 RPC 保存用户选择的 `index` 或 `id`。
- 通过 viewer 点击保存用户选择的 `index`；如果点击在 Camera 视图但没有命中任何 bbox，会清空当前选择。
- 每帧只转发当前选中的 detection。
- 如果当前帧找不到选中 bbox，发布空 `Detection2DArray`，避免下游或 viewer 复用旧 bbox。

它不创建 detector、不跑 YOLO、不调用 VLM、不做 ReID/EdgeTAM，也不负责目标丢失恢复。

RPC：

- `list_candidates() -> list[dict[str, Any]]`
- `select_bbox(index: int | None = None, id: str | None = None) -> str`
- `clear_selection() -> str`

候选字段：

- `index`
- `id`
- `bbox: [x1, y1, x2, y2]`
- `confidence`
- `class_id`

### BBoxDistanceBehaviorModule

职责：

- 消费 `selected_bbox + lidar + camera_info`。
- 输出 `cmd_vel` 和 `behavior_status`。
- 不做人识别、不选择目标、不做目标丢失恢复。

RPC：

- `start_bbox_distance_behavior(hold_seconds=None, hold_distance=None, approach_distance=None) -> str`
- `stop_bbox_distance_behavior() -> str`

默认参数：

- `command_hz = 20.0`
- `hold_seconds = 3.0`
- `hold_distance = 1.5`
- `approach_distance = 0.8`
- `depth_percentile = 25.0`
- `max_linear_speed = 0.45`
- `max_angular_speed = 0.8`

状态机：

```text
idle -> holding_distance -> approaching -> done
```

`hold_seconds` 从第一次拿到有效 bbox + lidar distance 后开始计时，不从 RPC 调用瞬间开始。bbox 为空、camera_info 缺失、lidar 距离无效时发布 `Twist.zero()` 并等待，不重新识别。完成或停止时发布 `Twist.zero()`。

lidar 距离 MVP 直接用 camera intrinsics 将点云投影到 bbox，取 `depth_percentile` 深度；如果实际 lidar 坐标系没有和相机对齐，后续再补 TF 修正。

### 启动

`bbox-distance-follow` 复用 `Detection2DModule` 的默认 YOLO detector。真机测试通常断网，所以需要先在有网络的环境预下载 `yolo11n.pt`：

```bash
mkdir -p data/models_yolo
curl -L -o data/models_yolo/yolo11n.pt https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n.pt
```

确认文件存在：

```bash
ls -lh data/models_yolo/yolo11n.pt
```

如果这个文件缺失，blueprint 会在部署模块前直接报错并打印同一条预下载命令，避免 Ultralytics 在真机断网时隐式下载失败。

CLI 全局参数必须放在 `run` 前面，例如 `--robot-ip`、`--replay`、`--rerun-open` 都是全局参数。

```bash
.venv/bin/dimos --replay run bbox-distance-follow
```

或连接真实 Go2：

```bash
.venv/bin/dimos --robot-ip 192.168.123.161 --rerun-open native run bbox-distance-follow
```

如需后台运行：

```bash
.venv/bin/dimos --replay run bbox-distance-follow --daemon
```

### 命令行 RPC 选择 bbox

列出候选：

```bash
.venv/bin/python -c 'from dimos.core.rpc_client import RPCClient; from dimos.robot.custom.bbox_distance_follow import BBoxSelectionModule; c=RPCClient.remote(BBoxSelectionModule); print(c.list_candidates()); c.stop_rpc_client()'
```

选择第 0 个 bbox：

```bash
.venv/bin/python -c 'from dimos.core.rpc_client import RPCClient; from dimos.robot.custom.bbox_distance_follow import BBoxSelectionModule; c=RPCClient.remote(BBoxSelectionModule); print(c.select_bbox(index=0)); c.stop_rpc_client()'
```

按 id 选择 bbox：

```bash
.venv/bin/python -c 'from dimos.core.rpc_client import RPCClient; from dimos.robot.custom.bbox_distance_follow import BBoxSelectionModule; c=RPCClient.remote(BBoxSelectionModule); print(c.select_bbox(id="0")); c.stop_rpc_client()'
```

清除选择：

```bash
.venv/bin/python -c 'from dimos.core.rpc_client import RPCClient; from dimos.robot.custom.bbox_distance_follow import BBoxSelectionModule; c=RPCClient.remote(BBoxSelectionModule); print(c.clear_selection()); c.stop_rpc_client()'
```

启动距离行为：

```bash
.venv/bin/python -c 'from dimos.core.rpc_client import RPCClient; from dimos.robot.custom.bbox_distance_follow import BBoxDistanceBehaviorModule; c=RPCClient.remote(BBoxDistanceBehaviorModule); print(c.start_bbox_distance_behavior()); c.stop_rpc_client()'
```

停止距离行为：

```bash
.venv/bin/python -c 'from dimos.core.rpc_client import RPCClient; from dimos.robot.custom.bbox_distance_follow import BBoxDistanceBehaviorModule; c=RPCClient.remote(BBoxDistanceBehaviorModule); print(c.stop_bbox_distance_behavior()); c.stop_rpc_client()'
```

### Viewer 点击选择

启动 replay + viewer 后，Camera 视图里会显示 `world/color_image`。`Detection2DModule` 发布的全部 YOLO 候选框会显示为黄色 bbox。通过命令行 RPC 调用 `select_bbox(index=...)` 后，`BBoxSelectionModule` 会发布 `/color_image/selected_bbox`，本 blueprint 的专用 visual override 会让 RerunBridge 在 `world/color_image/selected_bbox` 上显示绿色 bbox。

也可以直接在 dimos-viewer 的 Camera 视图里点击黄色 bbox。点击会沿用 viewer 已有的 WebSocket 回传链路变成 `clicked_point`，`BBoxSelectionModule` 会用点击像素坐标命中最新一帧候选 bbox，并立即发布对应的 `selected_bbox`。点击 Camera 视图里没有 bbox 的位置会清空当前选择，让行为模块收到空 selected bbox 后停止输出运动命令。

当 selected bbox 在后续帧中移动，overlay 会跟随当前帧 detection 更新；当选择被清除或当前帧找不到选中 bbox，模块会发布空 `Detection2DArray`，viewer 中旧框会消失。

### 非目标

- 不新增通用 `dimos rpc` CLI。
- 不接 Agent、Prompt、MCP、McpServer 或 McpClient。
- 不新增 `@skill`。
- 不在 `BBoxSelectionModule` 内实现 YOLO、ReID、EdgeTAM 或目标丢失恢复。
- 不在 `BBoxDistanceBehaviorModule` 内做人识别或 bbox 选择。

后续如果 lidar 和 camera 坐标系不一致，可在点云投影前补 TF 对齐。
