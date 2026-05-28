# DimOS 人物标记与持续重识别 — 调研 Summary

> 调研目标：在一个空间中，狗识别画面里的人并画框 → 用户在 UI 上标记选中某个人 → 狗后续运动中持续跟踪该人，最好能靠身体局部要素定位（不依赖完整人形画面）。
>
> 日期：2026-05-27 · 仓库：dimensionalOS/dimos

---

## 总体结论

| 能力 | DimOS 现状 | 可行性 |
|---|---|---|
| **1. 检测画面中的人并画框** | 组件齐全，开箱即用 | 🟢 直接可用 |
| **2. UI 上标记/选中某个人** | 底层能力齐全，缺前端 | 🟡 需自建 web 视图 |
| **3. 运动中持续跟踪选中的人** | 两套机制可组合，但"靠身体局部定位"是缺口 | 🟡 部分满足，需补强 |

---

## 能力 1：检测画面中的人，标记为不同的框

**现状：完全现成。**

- **检测器**：`dimos/perception/detection/module2D.py`（`Detection2DModule`）/ `module3D.py`，底层 YOLO，输出 COCO 类别（人 = class 0）
- **短期跟踪 → track_id**：`dimos/perception/object_tracker.py`（`ObjectTracker`），给每个连续出现的目标分配 `track_id`（同一个人在连续帧里 track_id 不变）
- **数据格式**：`Detection2DBBox`，字段 `bbox / track_id / class_id / confidence`
- **现成 blueprint**：`unitree-go2-detection`，已经在发 `/detector3d/detections` 和带框的图像 `/detector3d/image/N`
- **可视化**：当前在 Rerun 里看（只读）

> 一句话：狗看一帧 → YOLO 检测出所有人 → ObjectTracker 给每个人一个 track_id → 框 + ID 都有了。

---

## 能力 2：UI 上标记/选中某一个人

**现状：选中的"后端能力"齐全，缺的是"前端交互页面"。**

- **选中入口（已支持喂框）**：
  `follow_person(query, initial_bbox=[x1,y1,x2,y2], initial_image)`（`dimos/agents/skills/person_follow.py`）。给了 bbox 就跳过"文字找人"，直接用你选的框。
- **底层锁定能力**：`EdgeTAMProcessor.init_track(box=[x1,y1,x2,y2])`（`dimos/models/segmentation/edge_tam.py:131`），EdgeTAM（SAM2 系）支持用**框**或**点**作为 prompt 来锁定目标。

**缺口（需自建）**：
- 命令中心是**编译好的 React 黑盒**（源码不在仓库，只发了 `command_center.html` 构建产物）→ 改不了
- 而且命令中心是**2D 地图视图，没有摄像头画面**
- → 需要在同一个 web server（`WebsocketVisModule`）上**新增一个专用页面**：canvas 显示摄像头帧 + 叠加可点击的检测框 + 点击 emit `select_target(bbox)` → 后端 RPC 调 `follow_person`

> 一句话：DimOS 知道"拿到框之后怎么锁定"，但没有"让你在画面上点框"的界面，这块要自己写（纯 HTML/JS + 加几个 socket 事件）。

---

## 能力 3：运动中持续跟踪选中的人（关键：靠身体局部定位，不依赖完整画面）

**现状：DimOS 有两套互补机制，但"局部特征定位"是最大缺口。**

DimOS 里"持续跟踪"实际由**两层**协作：

### 第一层：EdgeTAM 连续掩码跟踪（帧间）
- `dimos/models/segmentation/edge_tam.py`，SAM2 系**记忆传播**模型
- 一旦用框锁定，后续每帧靠 mask 传播跟踪 —— **对部分遮挡 / 部分出画面天然鲁棒**（人只露半个身子也能跟）
- ✅ **这一层部分满足"靠局部也能跟"的诉求**——连续跟踪时不需要完整人形

### 第二层：EmbeddingIDSystem 外观重识别（丢失后重锁）
- `dimos/perception/detection/reid/embedding_id_system.py`
- 机制：每个 track 存**多个 embedding**（最多 500 个），cosine 相似度聚合（默认 top-30 均值），阈值 0.63；同帧共现的人自动记为"不可能是同一人"（负约束）
- 模型：`TorchReIDModel`（osnet_x1_0，行人重识别专用）
- ⚠️ **关键限制：embedding 是对整个人物框裁剪做的整体外观特征**（`cropped_image` 裁的是完整 bbox），**不是部件级**。CLIP / MobileCLIP 同样是整图特征。

### 🔴 缺口：你要的"靠身体一部分要素定位"
DimOS **原生不支持部件级 / 局部特征 reID**。现有方案是：
- 连续跟踪靠 EdgeTAM mask 传播（对局部可见鲁棒）✅
- 但**彻底丢失后的重新锁定**靠的是整体外观 embedding ❌（人转身、换姿态、被长时间遮挡后，靠整体特征容易失配）

**若要真正做到"靠局部要素定位"，需要补强（按成本排序）：**
1. **加大 padding + 多 embedding 库**（最省事）：现有 `EmbeddingIDSystem` 已存多视角 embedding，调 `padding`、`max_embeddings_per_track`、`comparison_mode=max` 能提升对姿态变化的容忍度——但仍是整体特征
2. **接部件级 reID 模型**（中等）：换成 part-based reID（如 PCB / 部件分块 osnet 变体），把人切成上/中/下若干条带分别 embed，匹配时按部件投票 → 半遮挡也能靠可见部件命中。需要新写一个 `EmbeddingModel` 实现，接口 `embed(image)` 已标准化，改动可控
3. **局部特征点 / 衣服图案锚定**（重）：对选中目标提取显著局部 patch（如背包、logo、鞋），单独建特征库做匹配。DimOS 无现成模块，需从头写

---

## 落地建议（分阶段）

| 阶段 | 内容 | 依赖现成能力 | 需新建 |
|---|---|---|---|
| **P1** | 检测 + 画框 + web 页面显示实时画面和框 | 检测/跟踪/blueprint | `/reid` web 页面 + 后端推 image/detection |
| **P2** | 点击框选中 → EdgeTAM 锁定 → 连续跟踪 + 狗跟随 | `follow_person` / EdgeTAM | `select_target` 事件 + RPC 接线 |
| **P3** | 部件级 reID（满足"靠局部定位"） | `EmbeddingIDSystem` 框架 | 新 part-based `EmbeddingModel` |

## 关键风险 / 注意

1. **命令中心 React 无源码**：能力 2 的 UI 必须新建专用页面，不能改现有命令中心
2. **macOS CPU 跑 EdgeTAM/torchreid 很慢**：本机能验证流程，流畅跟踪需上 CUDA Linux 机器（如现场有）
3. **"靠局部定位"是当前最大技术缺口**：连续跟踪层（EdgeTAM）能扛部分遮挡，但重识别层是整体特征，要满足明确诉求需要 P3 的部件级改造
4. **osnet 权重首次需联网下载**到 `dimos/data/models_torchreid/`

---

## 关键代码索引

```
检测:        dimos/perception/detection/module2D.py / module3D.py
跟踪:        dimos/perception/object_tracker.py             (track_id)
选中入口:     dimos/agents/skills/person_follow.py           (follow_person, 支持 initial_bbox)
锁定:        dimos/models/segmentation/edge_tam.py:131      (init_track, box/point prompt)
重识别:       dimos/perception/detection/reid/embedding_id_system.py
reID 模型:    dimos/models/embedding/treid.py                (osnet, 整体特征)
现成 blueprint: dimos/robot/unitree/go2/blueprints/smart/unitree_go2_detection.py
web 后端:     dimos/web/websocket_vis/websocket_vis_module.py
web 页面壳:    dimos/web/templates/rerun_dashboard.html       (纯 HTML 参考)
```
