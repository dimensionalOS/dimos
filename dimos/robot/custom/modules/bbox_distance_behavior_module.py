from __future__ import annotations  # 允许类型注解延迟解析，减少循环导入风险

import json  # 导入 JSON 工具，用于发布结构化行为状态
import math  # 导入数学工具，用于检查有限数值
import threading  # 导入线程工具，用后台循环发布速度命令
import time  # 导入时间工具，用于状态机计时和消息时间戳
from typing import Any, Literal  # 导入通用类型和状态字面量类型

from dimos_lcm.std_msgs import String  # 导入 LCM 字符串消息，用于状态输出
import numpy as np  # 导入 numpy，用于点云投影和百分位距离计算
from numpy.typing import NDArray  # 导入 numpy 数组类型，便于标注点云数组
from reactivex.disposable import Disposable  # 导入 Disposable，用于注册输入流订阅

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT  # 导入线程停止等待的默认超时时间
from dimos.core.core import rpc  # 导入 rpc 装饰器，让方法可通过 DimOS RPC 调用
from dimos.core.module import Module, ModuleConfig  # 导入模块基类和模块配置基类
from dimos.core.stream import In, Out  # 导入输入输出流类型
from dimos.msgs.geometry_msgs.Twist import Twist  # 导入速度命令消息类型
from dimos.msgs.geometry_msgs.Vector3 import Vector3  # 导入三维向量类型，用于构造 Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo  # 导入相机内参消息类型
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2  # 导入 lidar 点云消息类型
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray  # 导入 2D 检测数组消息类型
from dimos.robot.custom.modules.bbox_selection_module import BBoxSelectionModule  # 导入 bbox 选择模块，复用 _bbox_corners 工具
from dimos.utils.logging_config import setup_logger  # 导入日志初始化函数

logger = setup_logger()  # 创建当前文件使用的日志对象

BehaviorState = Literal["idle", "holding_distance", "approaching", "done"]  # 定义行为状态机的合法状态

_LINEAR_GAIN = 0.8  # 定义距离误差到线速度的简单比例增益
_ANGULAR_GAIN = 1.0  # 定义横向像素误差到角速度的简单比例增益
_DISTANCE_TOLERANCE_M = 0.05  # 定义靠近完成时使用的距离容差


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


__all__ = [  # 声明这个文件希望对外暴露的名字
    "BBoxDistanceBehaviorConfig",  # 暴露行为模块配置
    "BBoxDistanceBehaviorModule",  # 暴露行为模块
]  # 结束 __all__ 列表
