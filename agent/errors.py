"""全项目共用的异常类型。特意不 import rclpy —— 这样没装 ROS2 的机器
（比如 Mac）也能跑技能库的离线测试和参数校验。"""

from __future__ import annotations


class BridgeError(RuntimeError):
    """与 agent_bridge 通信相关的错误基类。"""


class StaleStateError(BridgeError):
    """状态数据太旧，不能用于决策。"""


class ToolForbidden(BridgeError):
    """试图调用被本项目禁用的工具（安全红线）。"""


class PreconditionFailed(BridgeError):
    """前置条件不满足，技能拒绝执行。"""


class BrainError(RuntimeError):
    """LLM 决策层的错误。"""
