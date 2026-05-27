from __future__ import annotations  # 允许类型注解延迟解析，减少循环导入风险

from dimos.core.coordination.blueprints import autoconnect  # 导入蓝图组合函数
from dimos.robot.custom.modules.go2_startup_self_check_module import Go2StartupSelfCheck  # 导入自检模块
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2  # 导入现有 Go2 完整蓝图

# 复用现有 Go2 完整蓝图，再额外加上一次性启动自检发布者。
unitree_go2_startup_self_check = autoconnect(  # 定义 CLI 可运行的组合蓝图
    unitree_go2,  # 第一部分：现有 Go2 连接、viewer、导航和 MovementManager
    Go2StartupSelfCheck.blueprint(),  # 第二部分：新增的启动自检模块
).remappings(  # 把自检速度命令接入现有 MovementManager 的手动控制入口
    [
        (Go2StartupSelfCheck, "cmd_vel", "tele_cmd_vel"),  # 自检发布到 tele_cmd_vel，由 MovementManager 转成 cmd_vel
    ]
)


__all__ = [
    "unitree_go2_startup_self_check",
]
