from __future__ import annotations  # 允许类型注解延迟解析，减少循环导入风险

from typing import Any

from dimos.core.coordination.blueprints import autoconnect  # 导入蓝图组合函数
from dimos.core.coordination.module_coordinator import ModuleCoordinator  # 导入直接运行蓝图所需协调器
from dimos.core.global_config import global_config  # 导入全局配置，用于复用 viewer backend 选择
from dimos.core.transport import LCMTransport  # 导入 LCM transport，用于固定 topic 名称
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray  # 导入 2D 检测数组消息类型
from dimos.robot.custom.modules.yoloe_tracking_module import (
    YoloeTrackingModule,  # 导入 YOLOE tracking 模块
    _require_yoloe_lrpc_model,  # 导入模型文件预检查函数
)
from dimos.robot.custom.visualization.detection2d_overlay import yoloe_overlay  # 导入青色 YOLOE overlay
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import (
    rerun_config as go2_rerun_config,  # 导入 Go2 默认 rerun 配置，用于局部扩展
    unitree_go2_basic,  # 导入 Go2 基础蓝图
)
from dimos.visualization.vis_module import vis_module  # 导入 viewer 模块工厂

_YOLOE_DETECTIONS_TOPIC = "/color_image/yoloe_detections"  # YOLOE detections LCM topic
_YOLOE_DETECTIONS_ENTITY = "world/color_image/yoloe_detections"  # Rerun entity path


def _yoloe_tracking_rerun_blueprint() -> Any:  # 定义 yoloe-tracking-test 专用 Rerun 布局
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
                    f"-{_YOLOE_DETECTIONS_ENTITY}",  # 在 3D 视图中隐藏 YOLOE 2D bbox overlay
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


_yoloe_tracking_rerun_config = {  # yoloe-tracking-test 专用 rerun 配置
    **go2_rerun_config,  # 继承 Go2 默认 Camera/3D 布局和限频设置
    "blueprint": _yoloe_tracking_rerun_blueprint,
    "visual_override": {
        **go2_rerun_config["visual_override"],
        _YOLOE_DETECTIONS_ENTITY: yoloe_overlay,  # 对 YOLOE detections 做青色 Camera overlay
    },
}

_yoloe_tracking_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config=_yoloe_tracking_rerun_config,
)


yoloe_tracking_test = (
    autoconnect(
        unitree_go2_basic,
        _yoloe_tracking_vis,
        YoloeTrackingModule.blueprint(),
    )
    .global_config(
        n_workers=6,
        robot_model="unitree_go2",
    )
    .transports(
        {
            ("detections", Detection2DArray): LCMTransport(
                _YOLOE_DETECTIONS_TOPIC,
                Detection2DArray,
            ),
        }
    )
    .requirements(
        _require_yoloe_lrpc_model,  # 检查 YOLOE 模型文件是否已经预下载
    )
)


__all__ = [
    "yoloe_tracking_test",
]


if __name__ == "__main__":
    ModuleCoordinator.build(yoloe_tracking_test).loop()
