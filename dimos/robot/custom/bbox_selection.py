from __future__ import annotations  # 允许类型注解延迟解析，减少循环导入风险

import math  # 导入数学工具，用于检查点击像素坐标是否有效
import threading  # 导入线程锁，保护最新 detections 和选择状态
import time  # 导入时间工具，用于空检测消息时间戳
from typing import Any  # 导入通用类型，兼容 LCM 生成消息字段

from reactivex.disposable import Disposable  # 导入 Disposable，用于注册输入流订阅

from dimos.core.core import rpc  # 导入 rpc 装饰器，让方法可通过 DimOS RPC 调用
from dimos.core.module import Module, ModuleConfig  # 导入模块基类和模块配置基类
from dimos.core.stream import In, Out  # 导入输入输出流类型
from dimos.msgs.geometry_msgs.PointStamped import (
    PointStamped,  # 导入 viewer 点击点类型，用于相机 bbox 点击选择
)
from dimos.msgs.std_msgs.Header import Header  # 导入 DimOS Header 便捷构造，用于空检测头
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray  # 导入 2D 检测数组消息类型

_DEFAULT_FRAME_ID = "camera_optical"  # 定义空 Detection2DArray 使用的默认坐标系


class BBoxSelectionConfig(ModuleConfig):  # 定义 bbox 选择模块配置，当前 MVP 暂无额外参数
    pass  # 当前选择逻辑全部由 RPC 或 viewer 点击驱动，不需要额外配置项


class BBoxSelectionModule(Module):  # 定义 bbox 选择模块，只负责从多 bbox 中转发单个 bbox
    config: BBoxSelectionConfig  # 声明本模块使用的配置类型
    detections: In[Detection2DArray]  # 输入现有检测模块发布的多 bbox Detection2DArray
    clicked_point: In[PointStamped]  # 输入 dimos-viewer 点击事件，用于把相机像素点击映射到 bbox
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
        self.register_disposable(Disposable(self.clicked_point.subscribe(self._on_clicked_point)))  # 订阅 viewer 点击流

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

    def _on_clicked_point(self, point: PointStamped) -> None:  # 处理 dimos-viewer 发回来的点击点
        if not self._is_color_image_click(point):  # 只接受 Camera/color_image 视图里的点击，避免误吃 3D 点击
            return  # 非相机点击不改变当前 bbox 选择

        if not (math.isfinite(point.x) and math.isfinite(point.y)):  # 检查点击像素坐标是否可用
            return  # 无效点击直接忽略，不清除当前选择

        with self._lock:  # 加锁读取最新检测帧
            detections = self._latest_detections  # 复制最新 detections，后续命中测试不持锁

        if detections is None:  # 如果 detector 还没有发布过任何候选 bbox
            return  # 暂时无法选择，等待下一帧 detections

        selected_index = self._find_clicked_detection_index(  # 在最新 detections 中查找被点击命中的 bbox
            detections,  # 传入最新一帧候选 bbox
            float(point.x),  # 传入 viewer 点击的 x 像素坐标
            float(point.y),  # 传入 viewer 点击的 y 像素坐标
        )  # 结束命中测试

        with self._lock:  # 加锁更新选择状态
            self._selected_index = selected_index  # 命中则保存 index，未命中则清空 index
            self._selected_id = None  # viewer 点击按当前帧 index 选择，不沿用旧 id 选择

        self._publish_selected(detections)  # 立即刷新 selected_bbox，让机器人和 viewer 同步看到选择结果

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
    def _is_color_image_click(point: PointStamped) -> bool:  # 判断点击是否来自相机图像或其 bbox overlay
        frame_parts = point.frame_id.strip("/").split("/")  # 把 entity_path 拆成路径片段，兼容有无前导斜杠
        return "color_image" in frame_parts  # 只让 color_image 视图点击驱动 bbox 选择

    @classmethod  # 声明命中测试需要复用 bbox 坐标转换工具
    def _find_clicked_detection_index(  # 查找包含点击像素的 detection index
        cls,  # 传入类本身，方便调用类方法
        detections: Detection2DArray,  # 输入最新一帧候选 bbox
        x: float,  # 输入点击的 x 像素坐标
        y: float,  # 输入点击的 y 像素坐标
    ) -> int | None:  # 返回命中的 bbox index，未命中时返回 None
        hits: list[tuple[float, int]] = []  # 保存命中的 bbox 面积和 index，用于重叠时选更小框
        for index, detection in enumerate(detections.detections):  # 遍历当前帧所有候选 bbox
            x1, y1, x2, y2 = cls._bbox_corners(detection)  # 读取当前 bbox 的 xyxy 像素范围
            left, right = sorted((x1, x2))  # 归一化左右边界，防止异常 bbox 坐标反向
            top, bottom = sorted((y1, y2))  # 归一化上下边界，防止异常 bbox 坐标反向
            if left <= x <= right and top <= y <= bottom:  # 如果点击点落在当前 bbox 内
                area = max((right - left) * (bottom - top), 0.0)  # 计算 bbox 面积，重叠时优先小框
                hits.append((area, index))  # 记录命中的候选 bbox

        if not hits:  # 如果没有任何 bbox 包含点击点
            return None  # 返回 None，调用方会清空当前选择

        return min(hits)[1]  # 多个 bbox 重叠时选择面积最小的那个

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


__all__ = [  # 声明这个文件希望对外暴露的名字
    "BBoxSelectionConfig",  # 暴露选择模块配置
    "BBoxSelectionModule",  # 暴露选择模块
]  # 结束 __all__ 列表
