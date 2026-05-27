from __future__ import annotations  # 允许类型注解延迟解析，减少循环导入风险

import json  # 导入 JSON 工具，用于发布结构化行为状态
import math  # 导入数学工具，用于检查有限数值
from pathlib import Path  # 导入路径类型，用于检查本地 YOLO 权重文件
import threading  # 导入线程工具，用后台循环发布速度命令
import time  # 导入时间工具，用于状态机计时和消息时间戳
from typing import Any, Literal  # 导入通用类型和状态字面量类型

from dimos_lcm.std_msgs import String  # 导入 LCM 字符串消息，用于状态输出
import numpy as np  # 导入 numpy，用于点云投影和百分位距离计算
from numpy.typing import NDArray  # 导入 numpy 数组类型，便于标注点云数组
from reactivex.disposable import Disposable  # 导入 Disposable，用于注册输入流订阅

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT  # 导入线程停止等待的默认超时时间
from dimos.core.coordination.blueprints import autoconnect  # 导入蓝图组合函数
from dimos.core.coordination.module_coordinator import (  # 导入直接运行蓝图所需协调器
    ModuleCoordinator,  # 导入直接运行蓝图所需协调器
)  # 结束多行 import
from dimos.core.core import rpc  # 导入 rpc 装饰器，让方法可通过 DimOS RPC 调用
from dimos.core.global_config import global_config  # 导入全局配置，用于复用 viewer backend 选择
from dimos.core.module import Module, ModuleConfig  # 导入模块基类和模块配置基类
from dimos.core.stream import In, Out  # 导入输入输出流类型
from dimos.core.transport import LCMTransport  # 导入 LCM transport，用于固定 selected_bbox topic
from dimos.msgs.geometry_msgs.Twist import Twist  # 导入速度命令消息类型
from dimos.msgs.geometry_msgs.Vector3 import Vector3  # 导入三维向量类型，用于构造 Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo  # 导入相机内参消息类型
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2  # 导入 lidar 点云消息类型
from dimos.msgs.std_msgs.Header import Header  # 导入 DimOS Header 便捷构造，用于空检测头
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray  # 导入 2D 检测数组消息类型
from dimos.perception.detection.module2D import Detection2DModule  # 导入现有多 bbox 检测模块
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import (  # 导入 Go2 基础蓝图
    rerun_config as go2_rerun_config,  # 导入 Go2 默认 rerun 配置，用于局部扩展 selected bbox overlay
    unitree_go2_basic,  # 导入 Go2 基础蓝图
)  # 结束多行 import
from dimos.robot.unitree.go2.connection import (  # 导入 Go2 连接类，用于复用静态 camera_info
    GO2Connection,  # 导入 Go2 连接类，用于复用静态 camera_info
)  # 结束多行 import
from dimos.utils.data import get_data_dir  # 导入数据目录解析函数，用于定位本地模型文件
from dimos.utils.logging_config import setup_logger  # 导入日志初始化函数
from dimos.visualization.vis_module import (  # 导入 viewer 模块工厂，用于替换 Go2 默认 rerun 配置
    vis_module,  # 导入 viewer 模块工厂，用于替换 Go2 默认 rerun 配置
)  # 结束多行 import

logger = setup_logger()  # 创建当前文件使用的日志对象

BehaviorState = Literal["idle", "holding_distance", "approaching", "done"]  # 定义行为状态机的合法状态

_LINEAR_GAIN = 0.8  # 定义距离误差到线速度的简单比例增益
_ANGULAR_GAIN = 1.0  # 定义横向像素误差到角速度的简单比例增益
_DISTANCE_TOLERANCE_M = 0.05  # 定义靠近完成时使用的距离容差
_DEFAULT_FRAME_ID = "camera_optical"  # 定义空 Detection2DArray 使用的默认坐标系
_YOLO_MODEL_NAME = "yolo11n.pt"  # 定义 Detection2DModule 默认需要的 YOLO 模型文件名
_YOLO_MODEL_PATH = get_data_dir("models_yolo") / _YOLO_MODEL_NAME  # 定义本地预下载模型路径
_YOLO_MODEL_URL = "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n.pt"  # 定义官方权重下载地址


def _detection_array_to_rerun(  # 定义 Detection2DArray 到 Rerun 2D overlay 的通用转换
    detections: Detection2DArray,  # 输入需要显示的 Detection2DArray 消息
    color: tuple[int, int, int, int],  # 输入每个 bbox 使用的 RGBA 颜色
    draw_order: float,  # 输入 Rerun 绘制层级，数值越大越靠上
) -> Any:  # 返回 Rerun 2D overlay
    import rerun as rr  # 延迟导入 Rerun，避免非 viewer 路径承担额外导入成本

    boxes: list[list[float]] = []  # 保存 Rerun 需要的 xywh bbox 列表
    labels: list[str] = []  # 保存每个 bbox 的显示标签
    colors: list[list[int]] = []  # 保存每个 bbox 的颜色
    for index, detection in enumerate(detections.detections):  # 遍历 Detection2DArray 消息里的全部 detection
        bbox = detection.bbox  # 读取 Detection2D 的中心点 bbox
        center = bbox.center.position  # 读取 bbox 中心点
        x = float(center.x - bbox.size_x / 2.0)  # 计算左上角 x
        y = float(center.y - bbox.size_y / 2.0)  # 计算左上角 y
        width = float(bbox.size_x)  # 读取 bbox 宽度
        height = float(bbox.size_y)  # 读取 bbox 高度
        boxes.append([x, y, width, height])  # 添加 Rerun 使用的 xywh bbox
        labels.append(str(detection.id or index))  # 优先显示 detection.id，没有 id 时显示当前序号
        colors.append(list(color))  # 添加当前 overlay 指定的 RGBA 颜色

    return rr.Boxes2D(  # 返回 Rerun 2D 框；空 boxes 会清除 viewer 里的旧框
        array=boxes,  # 传入所有 xywh bbox，允许为空列表
        array_format=rr.Box2DFormat.XYWH,  # 声明 bbox 数组格式为 xywh
        colors=colors,  # 传入每个框的颜色，允许为空列表
        labels=labels,  # 传入每个框的标签，允许为空列表
        show_labels=True,  # 在 viewer 中显示 bbox 标签
        draw_order=draw_order,  # 使用调用方指定的绘制层级
    )  # 结束 Rerun Boxes2D 构造


def _detections_to_rerun(detections: Detection2DArray) -> Any:  # 把 YOLO 多 bbox detections 转成 Camera 视图 overlay
    return _detection_array_to_rerun(detections, (255, 190, 0, 180), 90.0)  # 用黄色半透明框显示所有候选 bbox


def _selected_bbox_to_rerun(detections: Detection2DArray) -> Any:  # 只把 selected bbox topic 转换成 Rerun 2D overlay
    return _detection_array_to_rerun(detections, (0, 255, 0, 255), 100.0)  # 用绿色不透明框高亮命令行 RPC 选中的 bbox


def _bbox_distance_rerun_blueprint() -> Any:  # 定义 bbox-distance-follow 专用 Rerun 布局
    import rerun as rr  # 延迟导入 Rerun，避免非 viewer 路径承担额外导入成本
    import rerun.blueprint as rrb  # 延迟导入 Rerun blueprint API，用于构造 viewer 布局

    return rrb.Blueprint(  # 返回和 Go2 默认布局一致但修正 selected bbox 3D 可见性的 blueprint
        rrb.Horizontal(  # 使用左右分栏布局
            rrb.Spatial2DView(  # Camera 视图显示 color image、YOLO 候选框和 selected bbox overlay
                origin="world/color_image",  # 以实时相机图像实体作为 2D view 根节点
                contents=["world/color_image/**"],  # 显式纳入相机图像实体及其所有 overlay 子实体
                name="Camera",  # 保持 Go2 默认 Camera 视图名称
            ),  # 结束 Camera 视图配置
            rrb.Spatial3DView(  # 3D 视图显示世界、点云和机器人相关实体
                origin="world",  # 3D 视图以 world 为根
                contents=[  # 显式排除 2D bbox overlay，避免 3D view 报 Pinhole ancestor 警告
                    "world/**",  # 3D 视图默认显示 world 下的大部分实体
                    "-world/color_image/detections",  # 在 3D 视图中隐藏 YOLO 2D bbox overlay
                    "-world/color_image/selected_bbox",  # 在 3D 视图中隐藏 selected 2D bbox overlay
                ],  # 结束 3D 视图 contents 配置
                name="3D",  # 3D 视图名称保持和 Go2 默认一致
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),  # 使用黑色背景
                line_grid=rrb.LineGrid3D(  # 配置地面网格
                    plane=rr.components.Plane3D.XY.with_distance(0.5),  # 在 XY 平面显示 0.5 米间距网格
                ),  # 结束地面网格配置
                overrides={  # 配置 3D 视图中的实体可见性
                    "world/lidar": rrb.EntityBehavior(visible=False),  # 沿用 Go2 默认设置，隐藏原始 lidar entity
                },  # 结束 3D 视图 overrides
            ),  # 结束 3D 视图
            column_shares=[1, 2],  # 保持 Go2 默认的 Camera/3D 宽度比例
        ),  # 结束左右分栏布局
        rrb.TimePanel(state="hidden"),  # 隐藏时间面板，保持 Go2 默认布局
        rrb.SelectionPanel(state="hidden"),  # 隐藏选择面板，保持 Go2 默认布局
    )  # 结束 Rerun blueprint 构造


_bbox_distance_rerun_config = {  # 定义只属于 bbox-distance-follow 的 rerun 配置
    **go2_rerun_config,  # 继承 Go2 默认 Camera/3D 布局、静态实体和限频设置
    "blueprint": _bbox_distance_rerun_blueprint,  # 使用专用布局，避免 3D view 渲染 2D selected bbox
    "visual_override": {  # 覆盖并扩展 Go2 默认 visual_override
        **go2_rerun_config["visual_override"],  # 保留 Go2 默认 camera_info、map 和 costmap 转换逻辑
        "world/color_image/detections": _detections_to_rerun,  # 对 YOLO 多 bbox topic 做 Camera overlay
        "world/color_image/selected_bbox": _selected_bbox_to_rerun,  # 只对 selected bbox topic 做 2D overlay
    },  # 结束 visual_override 配置
}  # 结束 rerun 配置

_bbox_distance_vis = vis_module(  # 定义 bbox-distance-follow 专用 viewer 蓝图
    viewer_backend=global_config.viewer,  # 复用用户当前选择的 viewer backend
    rerun_config=_bbox_distance_rerun_config,  # 使用带 selected bbox overlay 的 rerun 配置
)  # 结束 viewer 蓝图创建


def _require_yolo11n_model() -> str | None:  # 定义 blueprint 启动前的本地 YOLO 权重检查
    if _YOLO_MODEL_PATH.exists():  # 如果本地权重已经预下载完成
        return None  # 返回 None 表示 requirement 通过

    return _format_missing_yolo_model_error(_YOLO_MODEL_PATH)  # 返回明确的预下载指令，避免运行时隐式联网下载


def _format_missing_yolo_model_error(model_path: Path) -> str:  # 定义缺失模型时的错误消息构造函数
    return (  # 返回一条能直接复制执行的修复建议
        f"Missing local YOLO model: {model_path}. "  # 说明缺失的本地模型路径
        "Real Go2 tests are usually offline, so pre-download it while online: "  # 说明真机断网场景需要提前准备
        f"mkdir -p {model_path.parent} && curl -L -o {model_path} {_YOLO_MODEL_URL}"  # 给出预下载命令
    )  # 结束错误消息构造


class BBoxSelectionConfig(ModuleConfig):  # 定义 bbox 选择模块配置，当前 MVP 暂无额外参数
    pass  # 当前选择逻辑全部由 RPC 输入驱动，不需要额外配置项


class BBoxSelectionModule(Module):  # 定义 bbox 选择模块，只负责从多 bbox 中转发单个 bbox
    config: BBoxSelectionConfig  # 声明本模块使用的配置类型
    detections: In[Detection2DArray]  # 输入现有检测模块发布的多 bbox Detection2DArray
    selected_bbox: Out[Detection2DArray]  # 输出只包含当前选中 bbox 的 Detection2DArray

    def __init__(self, **kwargs: Any) -> None:  # 定义构造函数，接收框架传入的配置参数
        super().__init__(**kwargs)  # 调用父类构造函数，让 DimOS 初始化模块和流
        self._lock = threading.RLock()  # 创建递归锁，保护最新 detections 和选择状态
        self._latest_detections: Detection2DArray | None = None  # 保存最新一帧多 bbox 检测结果
        self._selected_index: int | None = None  # 保存通过 index 选择的目标序号
        self._selected_id: str | None = None  # 保存通过 id 选择的目标 id

    @rpc  # 标记 start() 是框架生命周期 RPC
    def start(self) -> None:  # 定义模块启动逻辑
        super().start()  # 启动父类逻辑，包括 RPC 和自动绑定
        self.register_disposable(Disposable(self.detections.subscribe(self._on_detections)))  # 订阅检测流

    @rpc  # 标记 list_candidates() 可通过 DimOS RPC 调用
    def list_candidates(self) -> list[dict[str, Any]]:  # 返回最新一帧候选 bbox 列表
        with self._lock:  # 加锁读取最新 detections，避免和订阅回调并发冲突
            detections = self._latest_detections  # 复制引用，缩短锁内逻辑

        if detections is None:  # 如果还没有收到任何检测帧
            return []  # 返回空候选列表

        return [self._candidate_to_dict(index, detection) for index, detection in enumerate(detections.detections)]  # 转换每个检测为 RPC 友好的字典

    @rpc  # 标记 select_bbox() 可通过 DimOS RPC 调用
    def select_bbox(self, index: int | None = None, id: str | None = None) -> str:  # 保存用户选择的 bbox 条件
        if index is None and id is None:  # 如果调用方没有提供 index 或 id
            return "select_bbox requires index or id"  # 返回可读错误，不改变当前选择

        if index is not None and id is not None:  # 如果调用方同时提供两种选择条件
            return "select_bbox accepts only one of index or id"  # 返回可读错误，避免选择语义不明确

        with self._lock:  # 加锁更新选择状态
            self._selected_index = index  # 保存 index 选择，未使用时为 None
            self._selected_id = str(id) if id is not None else None  # 保存 id 选择，统一转为字符串
            latest = self._latest_detections  # 取出最新 detections，用于立即刷新 viewer

        if latest is not None:  # 如果已经收到过检测帧
            self._publish_selected(latest)  # 立即按当前选择发布 selected_bbox，方便 viewer 立刻反馈

        if index is not None:  # 如果用户按 index 选择
            return f"selected bbox index={index}"  # 返回确认信息

        return f"selected bbox id={id}"  # 返回按 id 选择的确认信息

    @rpc  # 标记 clear_selection() 可通过 DimOS RPC 调用
    def clear_selection(self) -> str:  # 清除当前 bbox 选择
        with self._lock:  # 加锁更新选择状态
            self._selected_index = None  # 清空 index 选择
            self._selected_id = None  # 清空 id 选择
            latest = self._latest_detections  # 取出最新 detections，用于保持 header

        self.selected_bbox.publish(self._empty_detection_array(latest))  # 发布空数组，清除 viewer 中的旧选框
        return "cleared bbox selection"  # 返回确认信息

    def _on_detections(self, detections: Detection2DArray) -> None:  # 处理检测模块发布的新一帧多 bbox
        with self._lock:  # 加锁更新最新检测结果
            self._latest_detections = detections  # 保存最新一帧 detections

        self._publish_selected(detections)  # 每帧根据当前选择转发单个 bbox 或空 bbox

    def _publish_selected(self, detections: Detection2DArray) -> None:  # 根据当前选择发布 selected_bbox
        selected = self._find_selected_detection(detections)  # 在当前帧里查找被选中的检测
        if selected is None:  # 如果当前帧没有选中目标或选择还不存在
            self.selected_bbox.publish(self._empty_detection_array(detections))  # 发布空数组，避免下游复用旧 bbox
            return  # 结束本帧处理

        msg = Detection2DArray(  # 构造只包含一个 detection 的消息
            detections_length=1,  # 设置检测数量为 1
            header=detections.header,  # 复用当前帧 header，保持时间和坐标系一致
            detections=[selected],  # 只放入当前帧匹配到的 selected detection
        )  # 结束 Detection2DArray 构造
        self.selected_bbox.publish(msg)  # 发布 selected_bbox 给行为模块和 viewer

    def _find_selected_detection(self, detections: Detection2DArray) -> Any | None:  # 在当前帧中查找选中的 detection
        with self._lock:  # 加锁读取选择条件
            selected_index = self._selected_index  # 复制 index 选择
            selected_id = self._selected_id  # 复制 id 选择

        if selected_id is not None:  # 如果当前使用 id 选择
            for index, detection in enumerate(detections.detections):  # 遍历当前帧所有 detection
                if self._detection_id(detection, index) == selected_id:  # 比较真实 id 或 index fallback
                    return detection  # 找到匹配 id 的 detection
            return None  # 当前帧没有匹配 id 时返回空

        if selected_index is not None:  # 如果当前使用 index 选择
            if 0 <= selected_index < len(detections.detections):  # 如果 index 在当前帧范围内
                return detections.detections[selected_index]  # 返回当前帧对应序号的 detection
            return None  # index 越界时返回空

        return None  # 没有选择时返回空

    @staticmethod  # 声明这是不依赖实例状态的工具函数
    def _empty_detection_array(source: Detection2DArray | None) -> Detection2DArray:  # 构造空 Detection2DArray
        header = source.header if source is not None else Header(time.time(), _DEFAULT_FRAME_ID)  # 优先复用来源 header
        return Detection2DArray(detections_length=0, header=header, detections=[])  # 返回空检测数组

    @classmethod  # 声明候选转换需要复用类级工具函数
    def _candidate_to_dict(cls, index: int, detection: Any) -> dict[str, Any]:  # 把 detection 转成 RPC 字典
        x1, y1, x2, y2 = cls._bbox_corners(detection)  # 计算 bbox 的左上和右下坐标
        confidence, class_id = cls._best_result(detection)  # 读取第一条 hypothesis 的置信度和类别
        return {  # 返回用户可读且 JSON 友好的候选结构
            "index": index,  # 返回当前帧中的候选序号
            "id": cls._detection_id(detection, index),  # 返回 detection.id，没有时回退为 index 字符串
            "bbox": [x1, y1, x2, y2],  # 返回 xyxy 格式 bbox
            "confidence": confidence,  # 返回置信度，缺失时为 0.0
            "class_id": class_id,  # 返回类别 id，缺失时为 None
        }  # 结束候选字典

    @staticmethod  # 声明这是不依赖实例状态的工具函数
    def _detection_id(detection: Any, index: int) -> str:  # 读取 detection id，并在缺失时回退到 index
        detection_id = getattr(detection, "id", "")  # 读取 detection.id，缺失时使用空字符串
        return str(detection_id) if detection_id else str(index)  # 返回真实 id 或 index 字符串

    @staticmethod  # 声明这是不依赖实例状态的工具函数
    def _bbox_corners(detection: Any) -> tuple[float, float, float, float]:  # 把中心点 bbox 转成 xyxy
        bbox = detection.bbox  # 读取 Detection2D 的 bbox 字段
        center = bbox.center.position  # 读取 bbox 中心点位置
        half_width = float(bbox.size_x) / 2.0  # 计算 bbox 半宽
        half_height = float(bbox.size_y) / 2.0  # 计算 bbox 半高
        x1 = float(center.x) - half_width  # 计算左上角 x
        y1 = float(center.y) - half_height  # 计算左上角 y
        x2 = float(center.x) + half_width  # 计算右下角 x
        y2 = float(center.y) + half_height  # 计算右下角 y
        return x1, y1, x2, y2  # 返回 xyxy 四元组

    @staticmethod  # 声明这是不依赖实例状态的工具函数
    def _best_result(detection: Any) -> tuple[float, str | None]:  # 读取 detection 的首个分类结果
        results = getattr(detection, "results", [])  # 读取 detection.results，缺失时使用空列表
        if not results:  # 如果没有任何 hypothesis
            return 0.0, None  # 返回默认置信度和空类别

        hypothesis = results[0].hypothesis  # 读取第一条 ObjectHypothesis
        confidence = float(getattr(hypothesis, "score", 0.0))  # 读取置信度，缺失时为 0.0
        class_id = getattr(hypothesis, "class_id", None)  # 读取类别 id，缺失时为 None
        return confidence, class_id  # 返回置信度和类别 id


class BBoxDistanceBehaviorConfig(ModuleConfig):  # 定义 bbox 距离行为模块配置
    command_hz: float = 20.0  # 速度命令发布频率，单位 Hz
    hold_seconds: float = 3.0  # 保持 hold_distance 的默认时间，单位秒
    hold_distance: float = 1.5  # 保持阶段目标距离，单位米
    approach_distance: float = 0.8  # 靠近阶段目标距离，单位米
    depth_percentile: float = 25.0  # bbox 内点云深度百分位，降低远处离群点影响
    max_linear_speed: float = 0.45  # 最大线速度，单位 m/s
    max_angular_speed: float = 0.8  # 最大角速度，单位 rad/s


class BBoxDistanceBehaviorModule(Module):  # 定义距离行为模块，只消费 selected bbox、lidar 和 camera_info
    config: BBoxDistanceBehaviorConfig  # 声明本模块使用的配置类型
    selected_bbox: In[Detection2DArray]  # 输入选择模块发布的单 bbox Detection2DArray
    lidar: In[PointCloud2]  # 输入 Go2 lidar 点云
    camera_info: In[CameraInfo]  # 输入 Go2 相机内参
    cmd_vel: Out[Twist]  # 输出 Go2 速度命令
    behavior_status: Out[String]  # 输出行为状态，便于日志和调试

    def __init__(self, **kwargs: Any) -> None:  # 定义构造函数，接收框架传入的配置参数
        super().__init__(**kwargs)  # 调用父类构造函数，让 DimOS 初始化模块和流
        self._lock = threading.RLock()  # 创建递归锁，保护传感器缓存和状态机
        self._stop_event = threading.Event()  # 创建后台命令循环停止事件
        self._thread: threading.Thread | None = None  # 保存后台命令循环线程
        self._state: BehaviorState = "idle"  # 初始化行为状态为 idle
        self._latest_selected_bbox: Detection2DArray | None = None  # 保存最新 selected bbox
        self._latest_lidar: PointCloud2 | None = None  # 保存最新 lidar 点云
        self._latest_camera_info: CameraInfo | None = None  # 保存最新 camera_info
        self._active_hold_seconds = self.config.hold_seconds  # 保存本次行为使用的保持时间
        self._active_hold_distance = self.config.hold_distance  # 保存本次行为使用的保持距离
        self._active_approach_distance = self.config.approach_distance  # 保存本次行为使用的靠近距离
        self._hold_started_at: float | None = None  # 保存第一次获得有效距离后的保持阶段起始时间

    @rpc  # 标记 start() 是框架生命周期 RPC
    def start(self) -> None:  # 定义模块启动逻辑
        super().start()  # 启动父类逻辑，包括 RPC 和自动绑定
        self.register_disposable(Disposable(self.selected_bbox.subscribe(self._on_selected_bbox)))  # 订阅 selected bbox
        self.register_disposable(Disposable(self.lidar.subscribe(self._on_lidar)))  # 订阅 lidar 点云
        self.register_disposable(Disposable(self.camera_info.subscribe(self._on_camera_info)))  # 订阅 camera_info
        self._stop_event.clear()  # 清除停止事件，允许后台线程运行
        self._thread = threading.Thread(  # 创建后台命令发布线程
            target=self._command_loop,  # 指定线程执行固定频率控制循环
            name="BBoxDistanceBehaviorModule",  # 给线程命名，方便日志和调试
            daemon=True,  # 设置守护线程，进程退出时不会被它卡住
        )  # 结束线程参数
        self._thread.start()  # 启动后台命令发布线程
        self._publish_status("idle")  # 发布初始状态

    @rpc  # 标记 stop() 是框架生命周期 RPC
    def stop(self) -> None:  # 定义模块停止逻辑
        self._stop_event.set()  # 通知后台线程退出
        self.cmd_vel.publish(Twist.zero())  # 停止时立即发布零速度
        if self._thread is not None and self._thread.is_alive():  # 如果后台线程存在且仍在运行
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)  # 等待后台线程退出，但最多等默认超时
        super().stop()  # 调用父类停止逻辑，释放订阅和 transport

    @rpc  # 标记 start_bbox_distance_behavior() 可通过 DimOS RPC 调用
    def start_bbox_distance_behavior(  # 定义行为启动 RPC
        self,  # 传入模块实例
        hold_seconds: float | None = None,  # 可选覆盖保持时间
        hold_distance: float | None = None,  # 可选覆盖保持阶段目标距离
        approach_distance: float | None = None,  # 可选覆盖靠近阶段目标距离
    ) -> str:  # 返回可读启动结果
        with self._lock:  # 加锁重置状态机参数
            self._active_hold_seconds = self.config.hold_seconds if hold_seconds is None else float(hold_seconds)  # 设置本次保持时间
            self._active_hold_distance = self.config.hold_distance if hold_distance is None else float(hold_distance)  # 设置本次保持距离
            self._active_approach_distance = self.config.approach_distance if approach_distance is None else float(approach_distance)  # 设置本次靠近距离
            self._hold_started_at = None  # 清空保持阶段起点，等待首次有效 bbox+lidar
            self._state = "holding_distance"  # 进入保持距离阶段

        self._publish_status("holding_distance")  # 发布状态变化
        return "bbox distance behavior started"  # 返回启动确认

    @rpc  # 标记 stop_bbox_distance_behavior() 可通过 DimOS RPC 调用
    def stop_bbox_distance_behavior(self) -> str:  # 定义行为停止 RPC
        with self._lock:  # 加锁更新状态机
            self._state = "idle"  # 回到 idle 状态
            self._hold_started_at = None  # 清空保持阶段计时

        self.cmd_vel.publish(Twist.zero())  # 行为停止时发布零速度
        self._publish_status("idle")  # 发布 idle 状态
        return "bbox distance behavior stopped"  # 返回停止确认

    def _on_selected_bbox(self, selected_bbox: Detection2DArray) -> None:  # 处理新的 selected bbox
        with self._lock:  # 加锁更新缓存
            self._latest_selected_bbox = selected_bbox  # 保存最新 selected bbox

    def _on_lidar(self, lidar: PointCloud2) -> None:  # 处理新的 lidar 点云
        with self._lock:  # 加锁更新缓存
            self._latest_lidar = lidar  # 保存最新 lidar 点云

    def _on_camera_info(self, camera_info: CameraInfo) -> None:  # 处理新的 camera_info
        with self._lock:  # 加锁更新缓存
            self._latest_camera_info = camera_info  # 保存最新 camera_info

    def _command_loop(self) -> None:  # 固定频率控制循环
        period_sec = 1.0 / max(self.config.command_hz, 1.0)  # 根据配置计算循环周期，并防止除零
        while not self._stop_event.wait(period_sec):  # 按周期运行，收到停止事件后退出
            twist = self._compute_next_twist()  # 根据当前状态和传感器缓存计算下一条速度命令
            if twist is not None:  # idle 状态会返回 None，表示无需重复发布
                self.cmd_vel.publish(twist)  # 发布速度命令

        self.cmd_vel.publish(Twist.zero())  # 线程退出前再发布一次零速度

    def _compute_next_twist(self) -> Twist | None:  # 计算当前周期应该发布的速度
        with self._lock:  # 加锁读取状态和传感器缓存
            state = self._state  # 复制当前状态
            selected_bbox = self._latest_selected_bbox  # 复制 latest selected bbox
            lidar = self._latest_lidar  # 复制 latest lidar
            camera_info = self._latest_camera_info  # 复制 latest camera_info
            hold_distance = self._active_hold_distance  # 复制本次保持距离
            approach_distance = self._active_approach_distance  # 复制本次靠近距离
            hold_seconds = self._active_hold_seconds  # 复制本次保持时间
            hold_started_at = self._hold_started_at  # 复制保持阶段起点

        if state == "idle":  # 如果行为未启动
            return None  # 不重复发布速度命令

        if state == "done":  # 如果行为已经完成
            return Twist.zero()  # 持续发布零速度，确保完成后保持停止

        detection = self._extract_single_detection(selected_bbox)  # 从 selected_bbox 中取出单个 detection
        if detection is None or lidar is None or camera_info is None:  # 如果 bbox、lidar 或 camera_info 任一缺失
            return Twist.zero()  # 发布零速度并等待数据补齐

        distance = self._estimate_bbox_distance(detection, lidar, camera_info)  # 估计 bbox 内点云距离
        if distance is None:  # 如果当前点云无法在 bbox 内给出有效距离
            return Twist.zero()  # 发布零速度并等待下一帧

        bbox_center_x = float(detection.bbox.center.position.x)  # 读取 bbox 中心 x 像素坐标
        target_distance = hold_distance if state == "holding_distance" else approach_distance  # 根据状态选择目标距离
        twist = self._make_twist(distance, target_distance, bbox_center_x, camera_info)  # 生成线速度和角速度命令

        if state == "holding_distance":  # 如果当前处于保持距离阶段
            now = time.monotonic()  # 读取单调时钟，用于保持时间计时
            if hold_started_at is None:  # 如果还没有开始保持阶段计时
                with self._lock:  # 加锁写入保持阶段起点
                    self._hold_started_at = now  # 从第一次有效 bbox+lidar 距离开始计时
                self._publish_status("holding_distance", distance=distance)  # 发布首次有效距离状态
            elif now - hold_started_at >= hold_seconds:  # 如果保持时间已经满足
                with self._lock:  # 加锁切换状态
                    self._state = "approaching"  # 进入靠近阶段
                self._publish_status("approaching", distance=distance)  # 发布状态变化

        if state == "approaching" and distance <= approach_distance + _DISTANCE_TOLERANCE_M:  # 如果已经到达靠近目标距离
            with self._lock:  # 加锁切换完成状态
                self._state = "done"  # 标记行为完成
            self._publish_status("done", distance=distance)  # 发布完成状态
            return Twist.zero()  # 完成时发布零速度

        return twist  # 返回当前控制周期的速度命令

    def _make_twist(  # 定义根据距离和 bbox 横向位置生成 Twist 的函数
        self,  # 传入模块实例
        distance: float,  # 当前估计距离
        target_distance: float,  # 当前阶段目标距离
        bbox_center_x: float,  # bbox 中心 x 像素坐标
        camera_info: CameraInfo,  # 相机内参
    ) -> Twist:  # 返回速度命令
        distance_error = distance - target_distance  # 计算目标距离误差，正数表示目标太远
        linear_x = self._clamp(distance_error * _LINEAR_GAIN, -self.config.max_linear_speed, self.config.max_linear_speed)  # 计算并限幅线速度
        fx = self._intrinsic_value(camera_info, 0, 0)  # 读取相机 fx
        cx = self._intrinsic_value(camera_info, 0, 2)  # 读取相机 cx
        angular_z = 0.0 if fx <= 0.0 else -((bbox_center_x - cx) / fx) * _ANGULAR_GAIN  # 根据 bbox 横向误差计算转向速度
        angular_z = self._clamp(angular_z, -self.config.max_angular_speed, self.config.max_angular_speed)  # 对角速度限幅
        return Twist(  # 构造 Twist 命令
            linear=Vector3(linear_x, 0.0, 0.0),  # 设置 x 方向线速度
            angular=Vector3(0.0, 0.0, angular_z),  # 设置 yaw 角速度
        )  # 结束 Twist 构造

    def _estimate_bbox_distance(  # 定义 bbox 内点云距离估计函数
        self,  # 传入模块实例
        detection: Any,  # 当前选中的 Detection2D
        lidar: PointCloud2,  # 最新 lidar 点云
        camera_info: CameraInfo,  # 最新 camera_info
    ) -> float | None:  # 返回米级距离，失败时返回 None
        x1, y1, x2, y2 = BBoxSelectionModule._bbox_corners(detection)  # 读取 bbox 的 xyxy 像素范围
        points = lidar.points_f32()  # 获取 float32 点云坐标，MVP 假设已经在相机坐标系或近似对齐
        if points.size == 0:  # 如果点云为空
            return None  # 返回无有效距离

        projected = self._project_points(points, camera_info)  # 把点云直接投影到相机像素平面
        if projected is None:  # 如果 camera_info 无效导致无法投影
            return None  # 返回无有效距离

        u, v, z = projected  # 解包投影后的像素坐标和 z 深度
        inside = (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)  # 计算落在 bbox 内的点
        depths = z[inside]  # 取出 bbox 内点的 z 深度
        if depths.size == 0:  # 如果 bbox 内没有有效点
            return None  # 返回无有效距离

        distance = float(np.percentile(depths, self.config.depth_percentile))  # 用配置的百分位作为目标距离
        return distance if math.isfinite(distance) and distance > 0.0 else None  # 只接受有限且正的距离

    def _project_points(  # 定义点云投影函数
        self,  # 传入模块实例
        points: NDArray[np.float32],  # 输入 Nx3 点云
        camera_info: CameraInfo,  # 输入相机内参
    ) -> tuple[NDArray[np.float32], NDArray[np.float32], NDArray[np.float32]] | None:  # 返回 u/v/z 或 None
        fx = self._intrinsic_value(camera_info, 0, 0)  # 读取相机 fx
        fy = self._intrinsic_value(camera_info, 1, 1)  # 读取相机 fy
        cx = self._intrinsic_value(camera_info, 0, 2)  # 读取相机 cx
        cy = self._intrinsic_value(camera_info, 1, 2)  # 读取相机 cy
        if fx <= 0.0 or fy <= 0.0:  # 如果内参无效
            return None  # 无法投影

        x = points[:, 0]  # 读取点云 x 坐标
        y = points[:, 1]  # 读取点云 y 坐标
        z = points[:, 2]  # 读取点云 z 坐标，MVP 直接当作相机前向距离
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z > 0.0)  # 过滤无效点和相机后方点
        if not np.any(valid):  # 如果没有任何有效点
            return None  # 无法投影

        x_valid = x[valid]  # 取出有效 x
        y_valid = y[valid]  # 取出有效 y
        z_valid = z[valid]  # 取出有效 z
        u = (fx * x_valid / z_valid + cx).astype(np.float32)  # 计算像素 u
        v = (fy * y_valid / z_valid + cy).astype(np.float32)  # 计算像素 v
        return u, v, z_valid.astype(np.float32)  # 返回投影坐标和对应深度

    @staticmethod  # 声明这是不依赖实例状态的工具函数
    def _intrinsic_value(camera_info: CameraInfo, row: int, col: int) -> float:  # 读取 K 矩阵中的一个值
        return float(camera_info.K[row * 3 + col]) if len(camera_info.K) >= 9 else 0.0  # K 缺失时返回 0.0

    @staticmethod  # 声明这是不依赖实例状态的工具函数
    def _extract_single_detection(selected_bbox: Detection2DArray | None) -> Any | None:  # 从 selected_bbox 中取单个 detection
        if selected_bbox is None or selected_bbox.detections_length == 0 or not selected_bbox.detections:  # 如果没有选中 bbox
            return None  # 返回空
        return selected_bbox.detections[0]  # 返回第一个 detection，选择模块保证最多一个

    @staticmethod  # 声明这是不依赖实例状态的工具函数
    def _clamp(value: float, lower: float, upper: float) -> float:  # 把数值限制在上下界之间
        return max(lower, min(upper, value))  # 返回限幅后的数值

    def _publish_status(self, state: BehaviorState, **fields: float) -> None:  # 发布行为状态消息
        payload: dict[str, Any] = {"state": state, **fields}  # 构造 JSON 状态载荷
        self.behavior_status.publish(String(json.dumps(payload)))  # 发布 JSON 字符串状态


bbox_distance_follow = autoconnect(  # 定义 CLI 可运行的 bbox-distance-follow 蓝图
    unitree_go2_basic,  # 第一部分：Go2 基础连接和 viewer
    _bbox_distance_vis,  # 第二部分：替换 Go2 默认 viewer 配置，增加 bbox overlay
    Detection2DModule.blueprint(  # 第三部分：现有多 bbox 检测模块
        camera_info=GO2Connection.camera_info_static,  # 复用 Go2 静态相机内参
        publish_detection_images=False,  # 关闭 cropped detected_image 输出，避免 3D view 无 Pinhole 警告
    ),  # 结束 Detection2DModule 配置
    BBoxSelectionModule.blueprint(),  # 第四部分：从多 bbox 中选择单个 bbox
    BBoxDistanceBehaviorModule.blueprint(),  # 第五部分：根据 selected bbox、lidar 和 camera_info 控制距离
).global_config(  # 设置该 blueprint 的全局配置
    n_workers=6,  # 给 Go2、viewer、detector、selection 和 behavior 留足 worker
    robot_model="unitree_go2",  # 标记机器人模型为 Go2
).transports(  # 覆盖 selected_bbox 的 transport，让 viewer 使用稳定 topic
    {  # 定义 transport 覆盖表
        ("detections", Detection2DArray): LCMTransport("/color_image/detections", Detection2DArray),  # 固定 YOLO bbox topic 到相机实体下面
        ("selected_bbox", Detection2DArray): LCMTransport("/color_image/selected_bbox", Detection2DArray),  # 固定 selected bbox topic
    }  # 结束 transport 覆盖表
).requirements(  # 添加 blueprint 启动前检查，避免真机断网时才由 Ultralytics 隐式下载失败
    _require_yolo11n_model,  # 检查 Detection2DModule 默认 YOLO 权重是否已经预下载
)  # 结束蓝图定义


__all__ = [  # 声明这个文件希望对外暴露的名字
    "BBoxDistanceBehaviorConfig",  # 暴露行为模块配置
    "BBoxDistanceBehaviorModule",  # 暴露行为模块
    "BBoxSelectionConfig",  # 暴露选择模块配置
    "BBoxSelectionModule",  # 暴露选择模块
    "bbox_distance_follow",  # 暴露顶层 blueprint 变量
]  # 结束 __all__ 列表


if __name__ == "__main__":  # 支持直接 python 执行该 blueprint 文件
    ModuleCoordinator.build(bbox_distance_follow).loop()  # 构建并运行 bbox-distance-follow 蓝图
