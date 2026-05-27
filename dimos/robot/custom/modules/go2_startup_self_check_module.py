from __future__ import annotations  # 允许类型注解延迟解析，减少循环导入风险

import threading  # 导入线程工具，用后台线程执行自检动作
import time  # 导入时间工具，用来计时和控制发布频率
from typing import Any  # 导入 Any，表示这里可以接收任意类型参数

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT  # 导入线程停止等待的默认超时时间
from dimos.core.core import rpc  # 导入 rpc 装饰器，让生命周期方法可被框架调用
from dimos.core.module import Module, ModuleConfig  # 导入模块基类和模块配置基类
from dimos.core.stream import Out  # 导入输出流类型，用来发布消息
from dimos.msgs.geometry_msgs.Twist import Twist  # 导入速度命令消息类型
from dimos.msgs.geometry_msgs.Vector3 import Vector3  # 导入三维向量类型
from dimos.utils.logging_config import setup_logger  # 导入日志初始化函数

logger = setup_logger()  # 创建当前文件使用的日志对象


class Go2StartupSelfCheckConfig(ModuleConfig):  # 定义自检模块的配置类
    """Runtime knobs for the one-shot Go2 startup movement check."""  # 说明这些配置控制一次性启动自检

    linear_speed_mps: float = 0.25  # 自检前进和后退速度，单位是米每秒
    forward_duration_sec: float = 2.0  # 前进持续时间，单位是秒
    backward_duration_sec: float = 2.0  # 后退持续时间，单位是秒
    command_publish_hz: float = 20.0  # 速度命令发布频率，单位是赫兹
    startup_delay_sec: float = 10.0  # 兜底等待时间，避免 Go2 start() 不返回时自检永远不启动


class Go2StartupSelfCheck(Module):  # 定义一个 DimOS 模块，用来发布启动自检速度命令
    """Publish one forward/stop/backward/stop sequence after all modules start."""  # 说明模块会在系统启动后执行前进/停止/后退/停止

    config: Go2StartupSelfCheckConfig  # 声明本模块使用上面的配置类型
    cmd_vel: Out[Twist]  # 声明输出流，向外发布 Twist 速度命令

    def __init__(self, **kwargs: Any) -> None:  # 定义构造函数，接收框架传入的配置参数
        super().__init__(**kwargs)  # 调用父类构造函数，让 DimOS 初始化模块和流
        # 这个事件用于通知后台线程停止，避免退出时还在发运动命令。
        self._stop_event = threading.Event()  # 创建停止事件，默认状态是未停止
        # 线程先保持为空，直到 on_system_modules() 才启动，所以 start() 不会触发运动。
        self._thread: threading.Thread | None = None  # 保存后台自检线程对象
        # 兜底线程用于在 on_system_modules() 没到达时延迟启动自检。
        self._fallback_thread: threading.Thread | None = None  # 保存延迟兜底线程对象
        # 这个锁防止 on_system_modules() 和兜底线程同时启动两个自检线程。
        self._start_lock = threading.Lock()  # 创建启动锁，保护自检启动状态
        # 这个标记防止 on_system_modules() 被重复调用时重复执行自检。
        self._started_self_check = False  # 记录自检是否已经启动过

    @rpc  # 标记 start() 是可被 DimOS 框架调用的 RPC 生命周期方法
    def start(self) -> None:  # 定义模块启动方法
        super().start()  # 调用父类启动逻辑，启动 RPC、自动绑定等框架功能
        # start() 只重置状态；真正运动要等所有系统模块都启动完。
        self._stop_event.clear()  # 清除停止事件，允许后续自检线程运行
        # 如果 GO2Connection.start() 长时间不返回，on_system_modules() 不会被调用，所以这里启动兜底等待线程。
        self._fallback_thread = threading.Thread(  # 创建延迟兜底线程
            target=self._run_delayed_self_check,  # 指定线程执行延迟启动函数
            name="Go2StartupSelfCheckFallback",  # 给兜底线程起名，方便日志和调试
            daemon=True,  # 设置为守护线程，进程退出时不会被它卡住
        )  # 结束兜底线程参数
        self._fallback_thread.start()  # 启动兜底线程，但它会先等待 startup_delay_sec

    @rpc  # 标记 on_system_modules() 是可被 DimOS 框架调用的 RPC 方法
    def on_system_modules(self, _modules: list[Any]) -> None:  # 所有模块启动后，框架会调用这里
        # ModuleCoordinator 会在每个模块 start() 都返回后调用这里。
        self._start_self_check("all modules started")  # 所有模块启动完成时，立即启动自检

    @rpc  # 标记 stop() 是可被 DimOS 框架调用的 RPC 生命周期方法
    def stop(self) -> None:  # 定义模块停止方法
        # 先通知线程停止，再发零速度，保证退出时机器人不会继续动。
        self._stop_event.set()  # 设置停止事件，让后台线程尽快退出循环
        self.cmd_vel.publish(Twist.zero())  # 发布零速度命令，让机器人停下

        if self._thread is not None and self._thread.is_alive():  # 如果线程存在且仍在运行
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)  # 等待线程结束，但最多等默认超时时间

        if self._fallback_thread is not None and self._fallback_thread.is_alive():  # 如果兜底线程还在等待
            self._fallback_thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)  # 等待兜底线程退出，但最多等默认超时时间

        super().stop()  # 调用父类停止逻辑，释放框架资源

    def _run_delayed_self_check(self) -> None:  # 定义延迟兜底启动逻辑
        logger.info("Go2 startup self-check fallback waiting")  # 记录兜底等待开始日志
        if self._stop_event.wait(self.config.startup_delay_sec):  # 等待配置的秒数，期间如果收到停止就返回 True
            return  # 如果停止事件已经触发，就不再启动自检

        self._start_self_check("startup delay elapsed")  # 等待结束后，如果还没启动过，就兜底启动自检

    def _start_self_check(self, reason: str) -> None:  # 定义统一的自检启动入口
        with self._start_lock:  # 加锁，避免两个线程同时进入启动逻辑
            if self._started_self_check:  # 如果自检已经启动过
                return  # 直接返回，避免重复启动第二个自检线程

            self._started_self_check = True  # 标记自检已经启动
            self._thread = threading.Thread(  # 创建后台线程，避免阻塞主流程
                target=self._run_self_check,  # 指定线程执行自检函数
                name="Go2StartupSelfCheck",  # 给线程起名，方便日志和调试
                daemon=True,  # 设置为守护线程，进程退出时不会被它卡住
            )  # 结束线程参数
            logger.info("Go2 startup self-check scheduled", reason=reason)  # 记录自检启动原因
            self._thread.start()  # 启动后台自检线程

    def _run_self_check(self) -> None:  # 定义后台线程实际执行的自检流程
        logger.info("Go2 startup self-check started")  # 记录自检开始日志
        try:  # 使用 try/finally，保证异常或停止时也会发零速度
            self._publish_for_duration(  # 发布一段时间的前进速度
                speed_mps=self.config.linear_speed_mps,  # 前进速度使用配置里的正速度
                duration_sec=self.config.forward_duration_sec,  # 前进时间使用配置里的前进时长
            )  # 结束前进发布调用
            self.cmd_vel.publish(Twist.zero())  # 前进结束后发布零速度，先停一下

            self._publish_for_duration(  # 发布一段时间的后退速度
                speed_mps=-self.config.linear_speed_mps,  # 后退速度使用配置速度的负数
                duration_sec=self.config.backward_duration_sec,  # 后退时间使用配置里的后退时长
            )  # 结束后退发布调用
            self.cmd_vel.publish(Twist.zero())  # 后退结束后发布零速度，让机器人停下
        finally:  # 不管上面是否正常结束，最后都执行这里
            # 最后再发一次零速度，确保中断或异常时最终状态也是停止。
            self.cmd_vel.publish(Twist.zero())  # 发布最终零速度命令
            logger.info("Go2 startup self-check finished")  # 记录自检结束日志

    def _publish_for_duration(self, speed_mps: float, duration_sec: float) -> None:  # 按固定时间持续发布速度
        period_sec = 1.0 / self.config.command_publish_hz  # 根据频率计算每次发布之间的间隔
        end_time = time.monotonic() + duration_sec  # 计算这段动作应该结束的时间点
        twist = Twist(  # 创建一个 Twist 速度命令
            linear=Vector3(speed_mps, 0.0, 0.0),  # 设置 x 方向线速度，y/z 为 0
            angular=Vector3(0.0, 0.0, 0.0),  # 设置角速度全为 0，不转向
        )  # 结束 Twist 创建

        while not self._stop_event.is_set() and time.monotonic() < end_time:  # 未停止且未超时时继续发布
            self.cmd_vel.publish(twist)  # 发布当前速度命令
            time.sleep(period_sec)  # 睡眠一个周期，控制发布频率


__all__ = [  # 声明这个文件希望对外暴露的名字
    "Go2StartupSelfCheck",  # 暴露自检模块类
    "Go2StartupSelfCheckConfig",  # 暴露自检配置类
]  # 结束 __all__ 列表
