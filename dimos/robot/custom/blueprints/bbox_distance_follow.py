from __future__ import annotations  # 允许类型注解延迟解析，减少循环导入风险

from pathlib import Path
from typing import Any

from dimos.core.coordination.blueprints import autoconnect  # 导入蓝图组合函数
from dimos.core.coordination.module_coordinator import ModuleCoordinator  # 导入直接运行蓝图所需协调器
from dimos.core.global_config import global_config  # 导入全局配置，用于复用 viewer backend 选择
from dimos.core.transport import LCMTransport  # 导入 LCM transport，用于固定 topic 名称
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray  # 导入 2D 检测数组消息类型
from dimos.perception.detection.module2D import Detection2DModule  # 导入多 bbox 检测模块
from dimos.robot.custom.modules.bbox_distance_behavior_module import BBoxDistanceBehaviorModule  # 导入距离行为模块
from dimos.robot.custom.modules.bbox_selection_module import BBoxSelectionModule  # 导入 bbox 选择模块
from dimos.robot.custom.visualization.detection2d_overlay import (
    detections_overlay,  # 黄色候选 bbox overlay
    selected_bbox_overlay,  # 绿色 selected bbox overlay
)
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import (
    rerun_config as go2_rerun_config,  # 导入 Go2 默认 rerun 配置，用于局部扩展
    unitree_go2_basic,  # 导入 Go2 基础蓝图
)
from dimos.robot.unitree.go2.connection import GO2Connection  # 导入 Go2 连接类，复用静态 camera_info
from dimos.utils.data import get_data_dir  # 导入数据目录解析函数，用于定位本地模型文件
from dimos.visualization.vis_module import vis_module  # 导入 viewer 模块工厂

_YOLO_MODEL_NAME = "yolo11n.pt"  # Detection2DModule 默认需要的 YOLO 模型文件名
_YOLO_MODEL_PATH = get_data_dir("models_yolo") / _YOLO_MODEL_NAME  # 本地预下载模型路径
_YOLO_MODEL_URL = "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n.pt"  # 官方权重下载地址


def _require_yolo11n_model() -> str | None:  # 定义 blueprint 启动前的本地 YOLO 权重检查
    if _YOLO_MODEL_PATH.exists():  # 如果本地权重已经预下载完成
        return None  # 返回 None 表示 requirement 通过
    return _format_missing_yolo_model_error(_YOLO_MODEL_PATH)  # 返回明确的预下载指令


def _format_missing_yolo_model_error(model_path: Path) -> str:  # 定义缺失模型时的错误消息构造函数
    return (
        f"Missing local YOLO model: {model_path}. "
        "Real Go2 tests are usually offline, so pre-download it while online: "
        f"mkdir -p {model_path.parent} && curl -L -o {model_path} {_YOLO_MODEL_URL}"
    )


def _bbox_distance_rerun_blueprint() -> Any:  # 定义 bbox-distance-follow 专用 Rerun 布局
    import rerun as rr  # 延迟导入 Rerun，避免非 viewer 路径承担额外导入成本
    import rerun.blueprint as rrb  # 延迟导入 Rerun blueprint API

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(
                origin="world/color_image",
                contents=["world/color_image/**"],
                name="Camera",
            ),
            rrb.Spatial3DView(
                origin="world",
                contents=[
                    "world/**",
                    "-world/color_image/detections",  # 在 3D 视图中隐藏 YOLO 2D bbox overlay
                    "-world/color_image/selected_bbox",  # 在 3D 视图中隐藏 selected 2D bbox overlay
                ],
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
                overrides={
                    "world/lidar": rrb.EntityBehavior(visible=False),
                },
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


_bbox_distance_rerun_config = {  # bbox-distance-follow 专用 rerun 配置
    **go2_rerun_config,  # 继承 Go2 默认 Camera/3D 布局和限频设置
    "blueprint": _bbox_distance_rerun_blueprint,  # 使用专用布局，避免 3D view 渲染 2D selected bbox
    "visual_override": {
        **go2_rerun_config["visual_override"],  # 保留 Go2 默认 camera_info、map 和 costmap 转换逻辑
        "world/color_image/detections": detections_overlay,  # 对 YOLO 多 bbox topic 做黄色 Camera overlay
        "world/color_image/selected_bbox": selected_bbox_overlay,  # 对 selected bbox topic 做绿色 2D overlay
    },
}

_bbox_distance_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config=_bbox_distance_rerun_config,
)


bbox_distance_follow = autoconnect(  # 定义 CLI 可运行的 bbox-distance-follow 蓝图
    unitree_go2_basic,  # Go2 基础连接和 viewer
    _bbox_distance_vis,  # 替换 Go2 默认 viewer 配置，增加 bbox overlay
    Detection2DModule.blueprint(
        camera_info=GO2Connection.camera_info_static,  # 复用 Go2 静态相机内参
        publish_detection_images=False,  # 关闭 cropped detected_image，避免 3D view 无 Pinhole 警告
    ),
    BBoxSelectionModule.blueprint(),  # 从多 bbox 中选择单个 bbox
    BBoxDistanceBehaviorModule.blueprint(),  # 根据 selected bbox、lidar 和 camera_info 控制距离
).global_config(
    n_workers=6,  # 给 Go2、viewer、detector、selection 和 behavior 留足 worker
    robot_model="unitree_go2",
).transports(
    {
        ("detections", Detection2DArray): LCMTransport("/color_image/detections", Detection2DArray),  # 固定 YOLO bbox topic
        ("selected_bbox", Detection2DArray): LCMTransport("/color_image/selected_bbox", Detection2DArray),  # 固定 selected bbox topic
    }
).requirements(
    _require_yolo11n_model,  # 检查 YOLO 权重是否已经预下载，避免真机断网时隐式下载失败
)


__all__ = [
    "bbox_distance_follow",
]


if __name__ == "__main__":
    ModuleCoordinator.build(bbox_distance_follow).loop()
